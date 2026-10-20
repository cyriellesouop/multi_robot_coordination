// DWAFP deviation trigger and impact analysis (Phase C2). See include/actions/impact.hpp.
//
// Running example in the comments: config/missions/chain.yaml
//   T1 (robot1, 5 s, deadline 6 s) --> T2 (robot2, 7.5 s, deadline 14 s)
// At t = 4 s robot1 reports 4 s left for T1 instead of the expected 1 s.

#include "actions/impact.hpp"

#include <algorithm>  // std::max, std::min, std::sort
#include <stdexcept>  // std::invalid_argument
#include <string>

namespace dwafp
{

namespace
{

// Two runtime entries are "the same input" for the predictor when every field it reads is equal.
// Used to find the sources of a deviation automatically, so the caller cannot forget one.
bool same(const TaskRuntime & a, const TaskRuntime & b)
{
  return a.status == b.status && a.start_ms == b.start_ms && a.remaining_ms == b.remaining_ms &&
         a.end_ms == b.end_ms && a.release_ms == b.release_ms;
}

bool same(const RobotRuntime & a, const RobotRuntime & b)
{
  return a.available == b.available && a.available_from_ms == b.available_from_ms &&
         a.unavailable_since_ms == b.unavailable_since_ms;
}

}  // namespace

// ----------------------------------------------------------------------------------------------
// 1. Persistence trigger
// ----------------------------------------------------------------------------------------------

// Delta_i(t) = (t + r_hat_i(t)) - f_ref_i: how much later than accepted the robot now expects to
// finish. Chain example: (4 s + 4 s) - 5 s = +3 s.
TimeMs deviation_ms(TimeMs reference_finish_ms, TimeMs time_ms, TimeMs remaining_ms)
{
  return (time_ms + remaining_ms) - reference_finish_ms;
}

DeviationTrigger::DeviationTrigger(
  const Mission & mission, const Prediction & reference, TriggerConfig config)
: config_(config), history_(mission.task_count())
{
  if (config_.persistence_updates < 1) {
    throw std::invalid_argument("trigger: persistence_updates (k) must be >= 1");
  }
  if (config_.deviation_tolerance_ms < 0) {
    throw std::invalid_argument("trigger: deviation_tolerance_ms (epsilon) must be >= 0");
  }
  if (config_.blocked_threshold_ms < 0) {
    throw std::invalid_argument("trigger: blocked_threshold_ms (tau) must be >= 0");
  }
  set_reference(reference);
}

void DeviationTrigger::set_reference(const Prediction & reference)
{
  if (reference.tasks.size() != history_.size()) {
    throw std::invalid_argument("trigger: the reference prediction has the wrong size");
  }
  // Only the finish times are needed; copying them means the trigger never depends on the
  // lifetime of the caller's Prediction object.
  reference_finish_ms_.clear();
  for (const TaskPrediction & p : reference.tasks) {reference_finish_ms_.push_back(p.finish_ms);}
  // A new accepted plan starts every subtask with a clean history.
  for (TaskHistory & h : history_) {h = TaskHistory{};}
}

std::optional<DeviationEvent> DeviationTrigger::on_feedback(const FeedbackSample & sample)
{
  if (sample.task >= history_.size()) {
    throw std::invalid_argument("trigger: unknown task index " + std::to_string(sample.task));
  }
  TaskHistory & h = history_[sample.task];
  // Feedback of one goal arrives in order; an older sample would corrupt the counters.
  if (sample.time_ms < h.last_time_ms) {
    throw std::invalid_argument("trigger: feedback sample older than the previous one");
  }
  h.last_time_ms = sample.time_ms;

  // Condition 1: the predicted completion is later than accepted by more than epsilon, counted
  // over CONSECUTIVE updates (one sample back within tolerance resets the count, so a single
  // noisy estimate never raises an event). No reference finish -> nothing to compare with.
  const TimeMs f_ref = reference_finish_ms_[sample.task];
  const bool has_reference = is_reachable(f_ref);
  const TimeMs delta = has_reference ? deviation_ms(f_ref, sample.time_ms, sample.remaining_ms) : 0;
  const bool deviating = has_reference && delta > config_.deviation_tolerance_ms;
  h.deviating_updates = deviating ? h.deviating_updates + 1 : 0;

  // Condition 2: blocked, measured as continuous time since the first BLOCKED sample of the
  // current blocked period (one non-blocked sample ends the period).
  if (sample.blocked) {
    if (h.blocked_since_ms == kUnreachable) {h.blocked_since_ms = sample.time_ms;}
  } else {
    h.blocked_since_ms = kUnreachable;
  }

  const bool persistent = h.deviating_updates >= config_.persistence_updates;
  const bool long_block = h.blocked_since_ms != kUnreachable &&
    sample.time_ms - h.blocked_since_ms >= config_.blocked_threshold_ms;

  if (!persistent && !long_block) {
    // Re-arm only when the deviation has fully cleared (within tolerance and not blocked);
    // a deviation that is still there but below k updates keeps the episode open.
    if (!deviating && !sample.blocked) {h.latched = false;}
    return std::nullopt;
  }
  if (h.latched) {return std::nullopt;}  // already reported for this episode
  h.latched = true;

  DeviationEvent event;
  event.kind = persistent ? DeviationKind::kPersistentDeviation : DeviationKind::kBlocked;
  event.task = sample.task;
  event.time_ms = sample.time_ms;
  event.deviation_ms = delta;
  return event;
}

void DeviationTrigger::reset(Index task)
{
  history_.at(task) = TaskHistory{};
}

// ----------------------------------------------------------------------------------------------
// 2. Expected state at the event time
// ----------------------------------------------------------------------------------------------

namespace
{

// Puts one task where the reference (s_ref, f_ref) says it should be at time `now`.
TaskRuntime reference_runtime(TimeMs s_ref, TimeMs f_ref, TimeMs now)
{
  TaskRuntime task;
  if (now < s_ref) {
    task.status = TaskStatus::kUnstarted;  // not expected to have started yet
  } else if (now < f_ref) {
    task.status = TaskStatus::kExecuting;  // expected to be running
    task.start_ms = s_ref;
    task.remaining_ms = f_ref - now;
  } else {
    task.status = TaskStatus::kCompleted;  // expected to be done already
    task.start_ms = s_ref;
    task.end_ms = f_ref;
  }
  return task;
}

}  // namespace

// Chain example: actual = "T1 executing, 4 s left at t = 4 s"; the reference says T1 runs 0-5 s,
// so the expected state is "T1 executing since 0 s, 1 s left at t = 4 s".
// Late example: at t = 7 s T1 is still running (or fails); the reference says T1 completed at 5 s
// and T2 started at 5 s, so the expected state is "T1 completed at 5 s, T2 executing since 5 s".
RuntimeState expected_state(
  const Mission & mission, const Plan & plan, const RuntimeState & actual,
  const Prediction & reference, const std::vector<Index> & deviating_tasks,
  const std::vector<Index> & deviating_robots)
{
  const std::size_t n = mission.task_count();
  if (reference.tasks.size() != n || actual.tasks.size() != n ||
    actual.robots.size() != mission.robot_count() || plan.task_count() != n)
  {
    throw std::invalid_argument("expected_state: mission, plan, state and reference sizes differ");
  }
  RuntimeState expected = actual;  // copy: everything not deviating stays as observed
  const TimeMs now = actual.now_ms;

  // Step 1: the deviating tasks themselves.
  std::vector<bool> deviating(n, false);
  for (const Index j : deviating_tasks) {
    if (j >= n) {
      throw std::invalid_argument("expected_state: task index " + std::to_string(j) + " out of range");
    }
    deviating[j] = true;
    TaskRuntime & task = expected.tasks[j];
    // Only a task that is (or was until its failure) running can have deviated.
    if (task.status != TaskStatus::kExecuting && task.status != TaskStatus::kFailed) {continue;}
    const TimeMs s_ref = reference.tasks[j].start_ms;
    const TimeMs f_ref = reference.tasks[j].finish_ms;
    if (is_reachable(s_ref) && is_reachable(f_ref)) {
      task = reference_runtime(s_ref, f_ref, now);
    } else if (task.status == TaskStatus::kFailed) {
      // No expectation available: treat the failed task as if it had just completed its run.
      task.status = TaskStatus::kExecuting;
      task.remaining_ms = 0;
      task.end_ms = 0;
    }  // an executing task without expectation is kept as observed
  }

  for (const Index r : deviating_robots) {
    if (r >= expected.robots.size()) {
      throw std::invalid_argument("expected_state: robot index " + std::to_string(r) + " out of range");
    }
    expected.robots[r].available = true;  // as if it had not broken down
    expected.robots[r].unavailable_since_ms = 0;
    for (const Index j : plan.queue(r)) {deviating[j] = true;}  // its tasks are affected sources
  }

  // Step 2: overdue descendants. A task downstream of a deviating one (combined graph) that is
  // still unstarted, although the reference says it should have started by now, is waiting
  // BECAUSE of the deviation: in the expected world it would already run (or be done). It is
  // reconstructed from the reference too, but only when everything it waits for (dependency
  // predecessors and queue predecessor) is completed in the expected state; otherwise its lateness
  // has another cause and it is kept as observed. Topological order guarantees that its
  // predecessors have already been reconstructed when it is examined.
  std::vector<bool> downstream(n, false);
  for (const Index j : plan.topological_order()) {
    // j is downstream if one of its predecessors is deviating or downstream.
    for (const Index h : mission.dependency_predecessors(j)) {
      if (deviating[h] || downstream[h]) {downstream[j] = true;}
    }
    const Index prev = plan.queue_predecessor(j);
    if (prev != kNoIndex && (deviating[prev] || downstream[prev])) {downstream[j] = true;}
    if (!downstream[j] || deviating[j]) {continue;}

    TaskRuntime & task = expected.tasks[j];
    const TimeMs s_ref = reference.tasks[j].start_ms;
    const TimeMs f_ref = reference.tasks[j].finish_ms;
    if (task.status != TaskStatus::kUnstarted || !is_reachable(s_ref) || !is_reachable(f_ref) ||
      now <= s_ref)
    {
      continue;  // not overdue (or already running / finished as observed)
    }
    bool inputs_done = true;
    for (const Index h : mission.dependency_predecessors(j)) {
      inputs_done = inputs_done && expected.tasks[h].status == TaskStatus::kCompleted;
    }
    if (prev != kNoIndex) {
      inputs_done = inputs_done && expected.tasks[prev].status == TaskStatus::kCompleted;
    }
    if (inputs_done) {task = reference_runtime(s_ref, f_ref, now);}
  }
  return expected;
}

// ----------------------------------------------------------------------------------------------
// 3 + 4. Impact analysis, repair region, response window
// ----------------------------------------------------------------------------------------------

ImpactAnalysis analyze_impact(
  const Mission & mission, const Plan & plan, const RuntimeState & expected,
  const RuntimeState & actual, const DurationSnapshot & durations, const ImpactConfig & config)
{
  if (expected.now_ms != actual.now_ms) {
    throw std::invalid_argument("analyze_impact: the two states are at different times");
  }
  if (expected.tasks.size() != actual.tasks.size() || expected.robots.size() != actual.robots.size()) {
    throw std::invalid_argument("analyze_impact: the two states have different sizes");
  }
  const std::size_t n = mission.task_count();
  const TimeMs now = actual.now_ms;

  // The two predictors also validate each state against the mission and the plan.
  const Predictor expected_predictor(mission, plan, expected, durations);
  const Predictor actual_predictor(mission, plan, actual, durations);

  ImpactAnalysis result;

  // Step 1: baseline = full propagation of the expected state (one full pass per event).
  result.expected = expected_predictor.full();

  // Step 2: sources = subtasks whose OWN inputs differ: their runtime entry, or the entry of the
  // robot that executes them (e.g. robot1 unavailable -> every task of robot1's queue).
  for (const Index j : plan.topological_order()) {
    const Index r = plan.robot_of(j);
    if (!same(expected.tasks[j], actual.tasks[j]) || !same(expected.robots[r], actual.robots[r])) {
      result.sources.push_back(j);
    }
  }

  // Step 3: deviation horizon = incremental propagation of the sources from the baseline.
  // Chain example: T1 5 s -> 8 s changes, so T2 5-12.5 s -> 8-15.5 s changes: horizon {T1, T2}.
  result.updated = result.expected;
  result.horizon = actual_predictor.incremental(result.updated, result.sources);

  // Step 4: A_i = unfinished subtasks of the horizon with a predicate that was satisfied in the
  // baseline and is violated now. Tasks outside the horizon cannot have changed, so only the
  // horizon is scanned. Each threat also gets the time at which its violation would happen.
  for (const Index j : result.horizon) {
    const TaskStatus status = actual.tasks[j].status;
    if (status != TaskStatus::kUnstarted && status != TaskStatus::kExecuting) {continue;}
    const TaskPrediction & e = result.expected.tasks[j];
    const TaskPrediction & u = result.updated.tasks[j];
    Threat threat;
    threat.task = j;
    threat.deadline = e.deadline_ok && !u.deadline_ok;
    threat.outcome = e.outcome_ok && !u.outcome_ok;
    threat.assignment = e.assignment_ok && !u.assignment_ok;
    if (!threat.deadline && !threat.outcome && !threat.assignment) {continue;}

    // Violation time t_j^viol (input of the response window):
    //   deadline                         -> d_j (the subtask must be done by then)
    //   outcome / assignment, unstarted  -> its LATEST START l_j = d_j - p_{j,alpha_j}: the last
    //                                       moment its input (or a robot) can be there without
    //                                       making it miss its deadline. Before l_j, holding it
    //                                       only uses slack; after l_j the CURRENT assignment can
    //                                       no longer meet it (a faster robot still might).
    //   outcome / assignment, executing  -> already realized: does not set the window
    // Chain example: T1 fails at 3 s, T2 (d 14 s, 7.5 s on robot2) needs T1's output by 6.5 s.
    TimeMs violation = kUnreachable;
    const TimeMs deadline = mission.task(j).deadline_ms;
    if (threat.deadline) {violation = std::min(violation, deadline);}
    if ((threat.outcome || threat.assignment) && status == TaskStatus::kUnstarted) {
      // Duration on its CURRENT robot: the latest start of the task as planned. (A repair that
      // moves it to a faster robot gives it more time; propagation accounts for that.)
      const TimeMs latest_start = deadline - durations.at(j, plan.robot_of(j));
      violation = std::min(violation, latest_start);
    }
    threat.violation_ms = violation;
    result.earliest_violation_ms = std::min(result.earliest_violation_ms, violation);
    result.threats.push_back(threat);
    result.threatened.push_back(j);
  }

  // Response window B_i: time left before the earliest violation (0 if it is already due).
  if (is_reachable(result.earliest_violation_ms)) {
    result.response_window_ms = std::max<TimeMs>(0, result.earliest_violation_ms - now);
  }

  // No threatened condition: DWAFP continues and monitors, no region is built.
  if (result.threatened.empty()) {return result;}

  // Step 5: repair region V_R^(0) = sources u A_i u Path(source, T_j) for every T_j in A_i.
  // A subtask lies on such a path exactly when it is reachable FROM a source (forward search)
  // AND can reach a threatened subtask (backward search). Both searches use the combined graph
  // (dependency edges and queue edges), as every other part of DWAFP.
  std::vector<bool> from_source(n, false);
  std::vector<Index> stack(result.sources.begin(), result.sources.end());
  for (const Index j : stack) {from_source[j] = true;}
  while (!stack.empty()) {  // forward: descendants of the sources
    const Index h = stack.back();
    stack.pop_back();
    const auto visit = [&](Index s) {
        if (s != kNoIndex && !from_source[s]) {from_source[s] = true; stack.push_back(s);}
      };
    for (const Index s : mission.dependency_successors(h)) {visit(s);}
    visit(plan.queue_successor(h));
  }

  std::vector<bool> to_threat(n, false);
  stack.assign(result.threatened.begin(), result.threatened.end());
  for (const Index j : stack) {to_threat[j] = true;}
  while (!stack.empty()) {  // backward: ancestors of the threatened subtasks, among descendants
    const Index h = stack.back();
    stack.pop_back();
    const auto visit = [&](Index p) {
        // Staying inside the descendants of the sources keeps unrelated branches out (fork-join:
        // T2 is an ancestor of T3 but not reachable from T1, so it is not on a T1 -> T3 path).
        if (p != kNoIndex && from_source[p] && !to_threat[p]) {to_threat[p] = true; stack.push_back(p);}
      };
    for (const Index p : mission.dependency_predecessors(h)) {visit(p);}
    visit(plan.queue_predecessor(h));
  }

  std::vector<bool> is_source(n, false);
  for (const Index j : result.sources) {is_source[j] = true;}
  for (const Index j : plan.topological_order()) {
    const bool on_path = from_source[j] && to_threat[j];  // includes A_i and the sources leading to it
    if (!on_path && !is_source[j]) {continue;}
    // Completed subtasks cannot be edited, so they are never part of the region.
    if (actual.tasks[j].status == TaskStatus::kCompleted) {continue;}
    result.region.push_back(j);
  }

  // A region larger than allowed means the deviation is not local: escalate (Algorithm 1).
  result.region_too_large = result.region.size() > config.max_region_size;
  return result;
}

}  // namespace dwafp
