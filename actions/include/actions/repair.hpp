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
// repair too late when its successors need its output earlier. When B_i = 0 the threatened
// condition is already due: candidates are still evaluated, but none is reported as preventive.
//
// Policies (the bounded library of the paper; hold and escalation are guards, not candidates):
//   retry        : the task's own robot restarts it (a failed task, or a running one that is
//                  cancelled and started again). Applicable only when the task is restartable
//                  (Subtask::retryable), its robot is operational (available), and the triggering
//                  condition is transient or has cleared (TaskRuntime::retry_available). A
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

// Overheads of putting a repair in place. Their sum is the dispatch latency: the edited task is
// released (may start) at t + T_decision + T_dispatch, and propagation does the rest.
struct RepairConfig
{
  TimeMs decision_overhead_ms{0};  // T_decision: time to compute the repair
  TimeMs dispatch_overhead_ms{0};  // T_dispatch: time to cancel / send the goals
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

  // Removes every threatened condition, with the dispatch latency included in the prediction.
  // (It is a PREVENTIVE repair only if B_i > 0: see StageOneResult::best.)
  bool preventive() const {return residual_risk == 0;}
};

struct StageOneResult
{
  // Unstarted subtasks of A_i: held by the engine during the repair search (dependency guard).
  std::vector<Index> to_hold;
  std::size_t generated{0};              // primitives built (every policy x target x robot)
  std::size_t rejected_cycle{0};         // no insertion position keeps the plan acyclic
  std::size_t rejected_new_violation{0}; // they break a condition that held before the repair
  // Structurally admissible candidates, in generation order (policy, target, robot). Those with
  // R > 0 are kept: two of them may form a Stage II repair together.
  std::vector<RepairCandidate> admissible;
  // Index in `admissible` of the best preventive candidate (lexicographic [R, D, C], first one
  // generated on ties), or nullopt when Stage I finds none (Stage II, then escalation) or when
  // B_i = 0 (the condition is already due: DWAFP holds and escalates, it cannot prevent it).
  std::optional<std::size_t> best;
};

// Algorithm 2 (GenerateSingleEdits). `incumbent` and `actual` are the plan and state at the event
// time (the ones `impact` was computed with); `durations` is the snapshot frozen for this event.
// Returns an empty result when the impact analysis needs no repair.
StageOneResult generate_single_edits(
  const Mission & mission, const Plan & incumbent, const RuntimeState & actual,
  const DurationSnapshot & durations, const ImpactAnalysis & impact,
  const RepairConfig & config = {});

}  // namespace dwafp

#endif  // ACTIONS__REPAIR_HPP_
