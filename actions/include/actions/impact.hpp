// DWAFP deviation trigger and impact analysis (Phase C2).
//
// ROS-free, built on the workflow model of C1 (actions/workflow.hpp). It answers, in order:
//   1. WHEN to look   : DeviationTrigger turns robot feedback into a deviation event when the
//                       execution differs from the ACCEPTED prediction: the deviation
//                       Delta_i(t) = (t + r_hat_i(t)) - f_ref_i exceeds a tolerance epsilon for k
//                       consecutive feedback updates, or the robot stays blocked for at least tau.
//                       The trigger is local (it only looks at the executing subtask) and does not
//                       use any deadline: a subtask that still meets its own deadline can delay a
//                       tighter subtask downstream. (Outcome failures and robot unavailability are
//                       events by themselves; the reactive baseline instead waits for the actual
//                       failure or timeout.)
//   2. WHAT changed   : expected_state() rebuilds the state "as if the deviation had not
//                       happened", at the same time t, so the deviation can be isolated.
//   3. WHAT is at risk: analyze_impact() propagates the deviation (full baseline + incremental
//                       update) and returns the threatened set A_i (subtasks whose predicate went
//                       from satisfied to violated), the initial repair region V_R^(0), and the
//                       response window B_i.
// An event is not a repair: when analyze_impact() finds no threatened condition, DWAFP only
// continues monitoring. Repair generation (C3) starts from an ImpactAnalysis that needs repair.
//
// Units: integer nominal ms from mission start, as in workflow.hpp.

#ifndef ACTIONS__IMPACT_HPP_
#define ACTIONS__IMPACT_HPP_

#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include "actions/workflow.hpp"

namespace dwafp
{

// ----------------------------------------------------------------------------------------------
// 1. Persistence trigger
// ----------------------------------------------------------------------------------------------

// Parameters of the DWAFP trigger (all three are experiment parameters of the paper).
struct TriggerConfig
{
  int persistence_updates{3};         // k: consecutive feedback updates with Delta > epsilon
  TimeMs deviation_tolerance_ms{0};   // epsilon: deviation ignored as noise (>= 0)
  TimeMs blocked_threshold_ms{2000};  // tau: minimum continuous blocked time
};

// One feedback message of an executing subtask, already converted to nominal ms by the adapter.
struct FeedbackSample
{
  Index task{kNoIndex};
  TimeMs time_ms{0};       // t: when the feedback was received
  TimeMs remaining_ms{0};  // r_hat(t): the robot's estimate of the time it still needs
  bool blocked{false};     // the robot reports state BLOCKED
};

// Why a deviation event was raised.
enum class DeviationKind
{
  kPersistentDeviation,  // Delta > epsilon for k consecutive updates
  kBlocked               // blocked for at least tau
};

struct DeviationEvent
{
  DeviationKind kind{DeviationKind::kPersistentDeviation};
  Index task{kNoIndex};
  TimeMs time_ms{0};
  TimeMs deviation_ms{0};  // Delta_i(t) = (t + r_hat_i(t)) - f_ref_i at the time of the event
};

// Deviation of an executing subtask from the accepted prediction:
//   Delta_i(t) = (t + r_hat_i(t)) - f_ref_i.
// Positive = later than expected, negative = earlier. Connecting-path example: reference finish of
// T1 = 5 s, new estimate 9 s -> Delta = +4 s, although T1 still meets its own 20 s deadline.
TimeMs deviation_ms(TimeMs reference_finish_ms, TimeMs time_ms, TimeMs remaining_ms);

// Turns a stream of feedback samples into deviation events, one subtask at a time.
// An event is raised ONCE per deviation episode: after it fires, the subtask is "latched" and
// raises nothing more until both conditions have cleared (Delta <= epsilon and not blocked), so
// one long blockage does not raise an event at every feedback message.
class DeviationTrigger
{
public:
  // `reference` is the accepted prediction (mission start, or after the last accepted repair);
  // only its finish times are kept (copied). Throws std::invalid_argument if k < 1, epsilon < 0,
  // tau < 0, or the reference does not have one entry per subtask.
  DeviationTrigger(const Mission & mission, const Prediction & reference, TriggerConfig config);

  // Replaces the accepted prediction (after a repair is executed) and clears every history, so
  // deviations are measured against the new plan from now on.
  void set_reference(const Prediction & reference);

  // Processes one feedback sample. Returns an event when this sample makes the deviation
  // persistent (and the subtask is not latched), otherwise std::nullopt. A subtask whose
  // reference finish is kUnreachable has no expected completion: only the blocked condition
  // applies to it. Throws std::invalid_argument for an unknown task or a sample older than the
  // previous one of the same subtask.
  std::optional<DeviationEvent> on_feedback(const FeedbackSample & sample);

  // Forgets the history of a subtask (it finished, or a new attempt starts after a repair).
  void reset(Index task);

private:
  struct TaskHistory
  {
    int deviating_updates{0};              // consecutive samples with Delta > epsilon
    TimeMs blocked_since_ms{kUnreachable};  // start of the current blocked period, or none
    TimeMs last_time_ms{-1};               // time of the last sample (to reject old samples)
    bool latched{false};                   // an event was raised for the current episode
  };

  TriggerConfig config_;
  std::vector<TimeMs> reference_finish_ms_;  // f_ref_i, one entry per subtask
  std::vector<TaskHistory> history_;         // one entry per subtask
};

// ----------------------------------------------------------------------------------------------
// 2. Expected state at the event time
// ----------------------------------------------------------------------------------------------

// The state "as if the deviation had not happened", at the same time t as `actual`, rebuilt from
// the REFERENCE prediction (the last accepted one: mission start, or after the last repair).
// A task is put where the reference says it should be at time t, from its start s_ref and
// finish f_ref:
//   t <  s_ref         -> unstarted
//   s_ref <= t < f_ref -> executing since s_ref, remaining = f_ref - t
//   t >= f_ref         -> completed at f_ref
// This is applied to
//   1. every task of `deviating_tasks` that is executing or failed in `actual`;
//   2. every OVERDUE descendant (combined graph) of a deviating task or of a task of a deviating
//      robot: still unstarted, reference start already past, and every predecessor completed in
//      the expected state. It waits only because of the deviation, so in the expected world it
//      would already be running or done.
// The "completed at f_ref" case matters for a deviation observed AFTER the nominal completion:
// "executing with 0 s left" would make the expected task finish at t instead of f_ref, and its
// successors start at t, which hides part of the impact.
// Every robot of `deviating_robots` is available again (same a_r). Everything else is copied.
// A deviating task without finite reference times is kept as observed if executing, and treated
// as executing with 0 s left if failed.
// Throws std::invalid_argument for invalid indices or sizes.
RuntimeState expected_state(
  const Mission & mission, const Plan & plan, const RuntimeState & actual,
  const Prediction & reference, const std::vector<Index> & deviating_tasks,
  const std::vector<Index> & deviating_robots);

// ----------------------------------------------------------------------------------------------
// 3 + 4. Impact analysis, repair region, response window
// ----------------------------------------------------------------------------------------------

struct ImpactConfig
{
  // Largest repair region DWAFP accepts. A larger region means the deviation is not local:
  // ImpactAnalysis::region_too_large is set and the engine escalates instead of repairing.
  std::size_t max_region_size{std::numeric_limits<std::size_t>::max()};
};

// One newly threatened task: which of its predicates went from satisfied (expected) to violated
// (updated). It may be the deviating source itself, a downstream task, or both kinds appear.
struct Threat
{
  Index task{kNoIndex};
  bool deadline{false};
  bool outcome{false};
  bool assignment{false};
  // t_j^viol (violation time): the earliest moment one of its violated conditions actually happens
  // (see analyze_impact); kUnreachable when the violation is already realized.
  TimeMs violation_ms{0};
};

struct ImpactAnalysis
{
  Prediction expected;          // prediction of the expected state (baseline, full propagation)
  Prediction updated;           // prediction of the actual state (incremental from `expected`)
  // Subtasks whose own inputs differ between the two states: the EVENT SOURCES (the tasks whose
  // deviation generated the event) plus the OVERDUE descendants that expected_state() rebuilds
  // (unstarted although the reference says they should run). So this is not the event-source
  // set: trigger-task-only repair takes the event sources from RepairConfig::deviating_tasks.
  // Terminology: event source = generated the event; affected task = its prediction changed
  // through propagation (horizon); threatened task = acquired a violated condition (A_i).
  std::vector<Index> sources;
  std::vector<Index> horizon;   // subtasks whose prediction changed (deviation horizon)
  std::vector<Threat> threats;  // A_i (newly threatened tasks) with details, topological order
  std::vector<Index> threatened;  // A_i: the subtasks of `threats`
  std::vector<Index> region;    // V_R^(0), in topological order (empty when A_i is empty)
  bool region_too_large{false}; // region.size() > max_region_size: escalate
  TimeMs earliest_violation_ms{kUnreachable};  // min t_j^viol over A_i
  TimeMs response_window_ms{kUnreachable};     // B_i = max(0, min t_j^viol - t_detect)

  // false: no condition is newly threatened, DWAFP continues and monitors (an event is not a repair)
  bool needs_repair() const {return !threatened.empty();}

  // Incumbent response window B_i^0 > 0: the threatened conditions can still be protected under
  // the CURRENT assignment (the latest starts use the incumbent robots' durations). It measures
  // the urgency of the current workflow state. B_i^0 = 0 means they cannot be protected without
  // changing the assignment; it is NOT a proof that no repair exists (a faster robot may still
  // make it), so it never rejects a candidate: each candidate is judged by its own propagation.
  bool incumbent_window_open() const {return needs_repair() && response_window_ms > 0;}
};

// AnalyzeImpact. `expected` and `actual` must describe the same time t (now_ms) and differ only
// by the deviation; both are checked against `plan` by the Predictor.
//   sources   = subtasks whose runtime entry, or whose robot's entry, differs between the states
//   horizon   = incremental propagation of the sources from the expected (baseline) prediction
//   A_i       = NEWLY THREATENED TASKS: unfinished subtasks of the horizon with at least one
//               predicate that was satisfied in the expected prediction (the reference rebuilt at
//               t) and is violated in the updated one. It may contain the deviating source (its
//               own deadline), downstream tasks, or both; pre-existing violations are not caused
//               by this deviation and are not counted.
//   V_R^(0)   = sources u A_i u every subtask on a path of the combined graph from a source to a
//               subtask of A_i; completed subtasks are left out (they cannot be edited)
//   t_j^viol  = violation time: the last moment at which the violated condition can still be
//               protected:
//               deadline threat                                  -> d_j (done by then)
//               missing input / infeasible assignment, unstarted -> latest start
//                                                   l_j = d_j - p_{j,alpha_j} (input / robot
//                                                   there by then, on its current robot)
//               missing output / infeasible assignment of the executing source -> already
//               realized, does not set the window (otherwise a robot breakdown would always give
//               B_i = 0; the next task's latest start is what counts)
//               These are INCUMBENT values (t_j^viol,0, on the current assignment alpha^0).
//   B_i       = incumbent response window B_i^0 = max(0, min t_j^viol,0 - t_detect),
//               kUnreachable when A_i is empty. B_i^0 = 0 means the threatened conditions cannot
//               be protected WITHOUT changing the assignment, not that no repair exists.
//               Examples: deadline 6 s at t = 4 s -> 2 s; T1 fails at 3 s, T2 (d 14 s, 7.5 s)
//               needs it by 6.5 s -> 3.5 s; robot1 breaks down at 2 s, its next task B (d 11 s,
//               5 s) needs a robot by 6 s -> 4 s.
// Throws std::invalid_argument if the two states are at different times or have different sizes.
ImpactAnalysis analyze_impact(
  const Mission & mission, const Plan & plan, const RuntimeState & expected,
  const RuntimeState & actual, const DurationSnapshot & durations, const ImpactConfig & config = {});

}  // namespace dwafp

#endif  // ACTIONS__IMPACT_HPP_
