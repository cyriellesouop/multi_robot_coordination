// DWAFP Stage I: single-edit repair candidates (Phase C3).
//
// ROS-free, built on the workflow model (C1) and the impact analysis (C2). Given an ImpactAnalysis
// that needs repair, generate_single_edits() implements Algorithm 2 of the paper:
//   for every policy o, every target T_k in the repair region, every eligible robot r:
//     pi = (o, T_k, r) -> apply it to a COPY of the plan and the state
//                      -> propagate it through its own validation horizon (incremental)
//                      -> keep it if it is structurally admissible (no new violation anywhere)
//                      -> score it: R (residual risk), D (disruption), C (added cost)
//   best = lexicographic minimum of [R, D, C] among the preventive candidates (R = 0, with the
//          dispatch latency included in the prediction); the admissible candidates with R > 0
//          are kept for Stage II (pairs).
// Response window: the edited task cannot start before t + T_decision + T_dispatch (its release
// time), and the propagation decides whether every threatened condition still holds. A latency
// therefore costs nothing when the task would wait for its inputs anyway, and makes an upstream
// repair too late when its successors need its output earlier. The incumbent window B_i^0 does
// NOT filter candidates: it is computed with the durations of the CURRENT assignment, so a
// candidate that moves a task to a faster robot can be preventive even when B_i^0 = 0 (every
// candidate is judged by its own activation-aware propagation, with its own robot's duration).
//
// Policies (the bounded library of the paper; hold and escalation are guards, not candidates):
//   retry        : the task's own robot restarts it (a failed task, or a running one that is
//                  cancelled and started again). Applicable only when the task is restartable
//                  (Subtask::retryable), its robot is operational (available), and the triggering
//                  condition is transient or has cleared (TaskRuntime::same_robot_retry_feasible). A
//                  persistent blockage excludes retry; substitution or escalation remain.
//   substitution : the task moves to another available robot. The task is unstarted, or executing /
//                  failed and retryable (it restarts there). If the receiving robot is busy, this
//                  is a NON-PREEMPTIVE NEXT-SLOT substitution: the robot finishes its current task,
//                  then runs T_k, then the rest of its queue in the same relative order;
//                  propagation decides whether that later start is still useful.
//   takeover     : the same edit as a substitution, when the receiving robot is idle because its
//                  next task waits for T_k (blocked successor): it runs T_k first, then resumes.
// Insertion rule (fixed): T_k goes at the front of the receiving robot's unstarted tasks, i.e.
// right after its executing task if it has one (non-preemptive, constraint (iii)); if that order
// would create a cycle, the next position, and so on. The relative order of every other task is
// preserved.
//
// This is a bounded repair of the incumbent schedule, not a new scheduling problem: only an
// affected task moves, only repair-eligible robots are considered, an executing task is never
// interrupted (except the affected task itself when it restarts), the insertion position follows
// the fixed rule above, and no start time or queue permutation is optimized.
//
// Units: integer nominal ms from mission start, as in workflow.hpp.

#ifndef ACTIONS__REPAIR_HPP_
#define ACTIONS__REPAIR_HPP_

#include <cstddef>
#include <optional>
#include <utility>  // std::move
#include <vector>

#include "actions/impact.hpp"
#include "actions/workflow.hpp"

namespace dwafp
{

enum class RepairPolicy
{
  kRetry,
  kSubstitution,
  kTakeover
};

// Which tasks a repair may edit. DWAFP uses the repair region; the two others exist for the
// ablation experiments (same policy library, different edit locations).
enum class RepairScope
{
  kRegion,         // DWAFP: tasks of the repair region V_R
  kSources,        // source-only repair: only the deviating tasks (closest to existing approaches)
  kAllUnfinished   // global single-edit oracle: every unfinished task, no localization
};

// Overheads of putting a repair in place. Their sum is the dispatch latency: the edited task is
// released (may start) at t + T_decision + T_dispatch, and propagation does the rest.
struct RepairConfig
{
  TimeMs decision_overhead_ms{0};  // T_decision: time to compute the repair
  TimeMs dispatch_overhead_ms{0};  // T_dispatch: time to cancel / send the goals
  RepairScope scope{RepairScope::kRegion};
};

// One primitive repair pi = (o, T_k, r), applied and evaluated.
struct RepairCandidate
{
  // A Plan can only exist validated (it has no default constructor), so a candidate is created
  // from its already-built edited plan.
  explicit RepairCandidate(Plan edited_plan) : plan(std::move(edited_plan)) {}

  RepairPolicy policy{RepairPolicy::kRetry};
  Index task{kNoIndex};   // T_k: the subtask the edit modifies
  Index robot{kNoIndex};  // r: original robot for a retry, receiving robot otherwise
  // Substitution into a busy robot: T_k waits for the robot's current task (next-slot insertion).
  bool after_current_task{false};

  Plan plan;               // edited copy of the incumbent plan
  RuntimeState state;      // edited copy of the state (T_k unstarted again if it restarts)
  Prediction prediction;   // prediction of (plan, state), obtained incrementally
  std::vector<Index> horizon;  // validation horizon: subtasks whose prediction the edit changed

  // Scores of the paper, lower is better, compared in this order:
  int residual_risk{0};    // R: violated predicates over the originally threatened set A_i
  int disruption{0};       // D: subtasks whose robot, queue predecessor or retry flag changed
  TimeMs cost_ms{0};       // C: discarded progress + delay added to any subtask by the edit
  // T_restoration: predicted finish of T_k - t, latency included (kUnreachable if it never ends).
  // Reported for the experiments; the window itself is checked through R (see above).
  TimeMs restoration_ms{0};

  // Preventive: with the edited task activated at t + T_decision + T_dispatch and its duration on
  // the candidate's robot, every original threat is removed (deadlines met, outcomes and
  // assignments restored) and, by admissibility, no new violation is created.
  bool preventive() const {return residual_risk == 0;}
};

struct StageOneResult
{
  // Tasks the engine holds during the repair search (see tasks_to_hold()).
  std::vector<Index> to_hold;
  std::size_t generated{0};              // primitives built (every policy x target x robot)
  std::size_t rejected_cycle{0};         // no insertion position keeps the plan acyclic
  std::size_t rejected_new_violation{0}; // they break a condition that held before the repair
  // Structurally admissible candidates, in generation order (policy, target, robot). Those with
  // R > 0 are kept: two of them may form a Stage II repair together.
  std::vector<RepairCandidate> admissible;
  // Index in `admissible` of the best preventive candidate (lexicographic [R, D, C], first one
  // generated on ties), or nullopt when Stage I finds none (Stage II, then escalation).
  std::optional<std::size_t> best;
};

// Hold guard: the unstarted tasks of A_i whose mandatory INPUT (outcome) or ROBOT (assignment) is
// threatened. They must not start with invalid conditions. A task threatened only by its deadline
// keeps its normal dispatch (it would be late, not invalid), and unaffected branches continue.
std::vector<Index> tasks_to_hold(const ImpactAnalysis & impact, const RuntimeState & actual);

// LocalPreventionInfeasible: the threatened tasks whose conditions NO permitted local repair can
// protect, proved with a lower bound (empty: local prevention may be feasible, run the search).
// For every task it computes the earliest finish and whether its output can exist under ANY
// combination of permitted edits, relaxing everything that can only delay:
//   completed  : its actual finish, output exists
//   executing  : min(continue: t + r_hat if its robot is available,
//                    restart if retryable: release + p on the fastest PERMITTED robot), where
//                permitted = available robots, the task's own robot only if same-robot retry is
//                feasible, and release = t + T_decision + T_dispatch
//   failed     : restart if retryable (as above), otherwise its output is lost for good
//   unstarted  : start >= max(t, earliest finish of its dependency predecessors), plus p on the
//                fastest available robot (queue edges ignored: they can only delay)
//   output possible = own output possible AND every dependency predecessor's output possible.
// Every Stage I or Stage II candidate finishes each task no earlier than this bound and needs its
// outputs, so a task of A_i whose bound misses its deadline, or whose output cannot exist, is
// unprotectable: complete local prevention is infeasible. Cost: one pass over the graph.
std::vector<Index> unprotectable_threats(
  const Mission & mission, const Plan & incumbent, const RuntimeState & actual,
  const DurationSnapshot & durations, const ImpactAnalysis & impact,
  const RepairConfig & config = {});

// Algorithm 2 (GenerateSingleEdits). `incumbent` and `actual` are the plan and state at the event
// time (the ones `impact` was computed with); `durations` is the snapshot frozen for this event.
// Returns an empty result when the impact analysis needs no repair.
StageOneResult generate_single_edits(
  const Mission & mission, const Plan & incumbent, const RuntimeState & actual,
  const DurationSnapshot & durations, const ImpactAnalysis & impact,
  const RepairConfig & config = {});

// Outcome of one DWAFP event (Algorithm 1, up to Stage I).
enum class DecisionStatus
{
  kContinue,               // no condition newly threatened: continue and monitor
  kRepair,                 // a preventive single edit was found (stage1.best)
  kNoFeasibleLocalRepair,  // provably no permitted local repair protects every threat: hold the
                           // tasks of to_hold, skip the search, report to mission-level recovery
  kNoSingleEdit            // local prevention not ruled out, but no single edit works: Stage II
};

struct Decision
{
  DecisionStatus status{DecisionStatus::kContinue};
  ImpactAnalysis impact;
  std::vector<Index> unprotectable;  // proof of kNoFeasibleLocalRepair (tasks of A_i)
  std::vector<Index> to_hold;        // tasks the engine must not dispatch for now
  StageOneResult stage1;             // empty unless the single-edit search ran
  double impact_us{0};               // timings (wall clock, microseconds), for the experiments
  double check_us{0};
  double stage1_us{0};
};

// Algorithm 1 up to Stage I:
//   AnalyzeImpact -> nothing threatened: kContinue
//   -> LocalPreventionInfeasible: hold, kNoFeasibleLocalRepair (no search, no Stage II)
//   -> GenerateSingleEdits -> best found: kRepair, otherwise kNoSingleEdit (Stage II comes next).
// The whole mission is never declared unrecoverable: only complete LOCAL prevention of this event.
Decision decide(
  const Mission & mission, const Plan & incumbent, const RuntimeState & expected,
  const RuntimeState & actual, const DurationSnapshot & durations, const RepairConfig & config = {});

}  // namespace dwafp

#endif  // ACTIONS__REPAIR_HPP_
