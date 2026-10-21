// The compared methods as controllers of the closed-loop simulator. See controllers.hpp.

#include "actions/controllers.hpp"

#include <algorithm>  // std::find, std::max
#include <chrono>
#include <stdexcept>
#include <tuple>      // std::tie

namespace dwafp
{

namespace
{

// The prediction of the initial plan at mission start: the first accepted reference.
Prediction initial_reference(const Mission & mission, const Plan & plan)
{
  const RuntimeState s0 = RuntimeState::initial(mission);
  const DurationSnapshot p = DurationSnapshot::capture(mission, ConfigDurationModel{}, s0);
  return Predictor(mission, plan, s0, p).full();
}

void push_unique(std::vector<Index> & list, Index j)
{
  if (std::find(list.begin(), list.end(), j) == list.end()) {list.push_back(j);}
}

// Fallback objective, lower is better: (tasks that would never complete, total tardiness).
// Only tasks not completed yet count; a task "never completes" when its output is predicted
// missing or its finish is unreachable.
std::pair<int, TimeMs> recovery_objective(const Mission & m, const RuntimeState & s, const Prediction & p)
{
  int never = 0;
  TimeMs tardiness = 0;
  for (Index j = 0; j < m.task_count(); ++j) {
    if (s.tasks[j].status == TaskStatus::kCompleted) {continue;}
    const TaskPrediction & q = p.tasks[j];
    if (!q.outcome_ok || !is_reachable(q.finish_ms)) {
      ++never;
    } else {
      tardiness += std::max<TimeMs>(0, q.finish_ms - m.task(j).deadline_ms);
    }
  }
  return {never, tardiness};
}

}  // namespace

const char * method_name(Method m)
{
  switch (m) {
    case Method::kNoIntervention: return "no intervention";
    case Method::kReactive: return "reactive";
    case Method::kSourceOnly: return "source-only";
    case Method::kDwafp: return "DWAFP";
    case Method::kMissionWide: return "mission-wide";
  }
  return "?";
}

MethodController::MethodController(const Mission & mission, const Plan & initial_plan, ControllerConfig config)
: mission_(mission), config_(config), reference_(initial_reference(mission, initial_plan)),
  trigger_(mission, reference_, config.trigger),
  blocked_since_(mission.task_count(), kUnreachable), last_feedback_(mission.task_count(), kUnreachable)
{
}

bool MethodController::preventive() const
{
  return config_.method == Method::kSourceOnly || config_.method == Method::kDwafp ||
         config_.method == Method::kMissionWide;
}

RepairConfig MethodController::repair_config() const
{
  RepairConfig rc;
  rc.decision_overhead_ms = config_.decision_ms;
  rc.dispatch_overhead_ms = config_.dispatch_ms;
  switch (config_.method) {
    case Method::kSourceOnly: rc.scope = RepairScope::kSources; break;
    case Method::kMissionWide: rc.scope = RepairScope::kAllUnfinished; break;
    default: rc.scope = RepairScope::kRegion; break;
  }
  return rc;
}

void MethodController::timed(double us)
{
  stats_.decision_us += us;
  stats_.max_decision_us = std::max(stats_.max_decision_us, us);
  stats_.decision_times_us.push_back(us);
}

// ----------------------------------------------------------------------------------------------
// Notifications: only record what happened; decisions are taken in act()
// ----------------------------------------------------------------------------------------------

void MethodController::on_feedback(const FeedbackSample & sample)
{
  last_feedback_[sample.task] = sample.time_ms;
  if (sample.blocked) {
    if (!is_reachable(blocked_since_[sample.task])) {blocked_since_[sample.task] = sample.time_ms;}
  } else {
    blocked_since_[sample.task] = kUnreachable;
  }
  if (!preventive()) {return;}
  if (trigger_.on_feedback(sample)) {
    ++stats_.trigger_events;
    push_unique(pending_trigger_, sample.task);
  }
}

void MethodController::on_task_finished(Index task, bool success, TimeMs /*time_ms*/)
{
  blocked_since_[task] = kUnreachable;
  trigger_.reset(task);
  if (!success) {push_unique(pending_recovery_, task);}  // actual failure -> common recovery
}

void MethodController::on_robot_unavailable(Index robot, TimeMs /*time_ms*/)
{
  // Its unfinished tasks are handed to the recovery in act(), where the plan is known.
  push_unique(broken_robots_, robot);
}

void MethodController::on_deadline_missed(Index task, TimeMs /*time_ms*/)
{
  push_unique(pending_recovery_, task);  // timeout of an unfinished task -> common recovery
}

void MethodController::on_edit_installed(bool installed, TimeMs /*time_ms*/)
{
  edit_in_flight_ = false;
  if (!installed) {refresh_reference_ = true;}  // the accepted plan did not happen
}

// ----------------------------------------------------------------------------------------------
// Observation and prediction
// ----------------------------------------------------------------------------------------------

RuntimeState MethodController::observed(const Simulator & sim) const
{
  RuntimeState s = sim.state();
  const TimeMs now = sim.now();
  for (Index j = 0; j < s.tasks.size(); ++j) {
    TaskRuntime & task = s.tasks[j];
    if (task.status != TaskStatus::kExecuting) {continue;}
    // Only feedback of the CURRENT attempt counts.
    const bool fresh = is_reachable(last_feedback_[j]) && last_feedback_[j] >= task.start_ms;
    const bool blocked = fresh && is_reachable(blocked_since_[j]) && blocked_since_[j] >= task.start_ms;
    if (blocked) {
      if (!task.same_robot_retry_feasible) {
        // A persistent obstruction REPORTED by the robot: it will not get past it on this robot.
        task.remaining_ms = kUnreachable;
      } else if (config_.estimate == BlockageEstimate::kConservative) {
        task.remaining_ms += now - blocked_since_[j];  // heuristic: at least as long again
      }
      // nominal: r_hat as reported (t + r_hat already includes the delay so far)
    } else {
      // r_hat was given at the last feedback (or is the nominal duration since the start).
      const TimeMs since = now - (fresh ? last_feedback_[j] : task.start_ms);
      task.remaining_ms = std::max<TimeMs>(0, task.remaining_ms - since);
    }
  }
  return s;
}

Prediction MethodController::predict(const Simulator & sim, const RuntimeState & state) const
{
  const DurationSnapshot p = DurationSnapshot::capture(mission_, ConfigDurationModel{}, state);
  return Predictor(mission_, sim.plan(), state, p).full();
}

// ----------------------------------------------------------------------------------------------
// Decisions
// ----------------------------------------------------------------------------------------------

void MethodController::schedule(Simulator & sim, const RepairCandidate & candidate)
{
  const TimeMs activation = sim.now() + config_.decision_ms + config_.dispatch_ms;
  try {
    sim.schedule_edit(candidate.plan, {candidate.task}, activation);
  } catch (const std::invalid_argument &) {
    ++stats_.errors;
    refresh_reference_ = true;
    return;
  }
  edit_in_flight_ = activation > sim.now();
  // The candidate's prediction is the newly accepted plan: the trigger measures against it.
  reference_ = candidate.prediction;
  trigger_.set_reference(reference_);
  blocked_since_[candidate.task] = kUnreachable;
}

bool MethodController::try_preventive(Simulator & sim, const std::vector<Index> & tasks)
{
  ++stats_.analyses;
  const auto t0 = std::chrono::steady_clock::now();
  Decision d;
  try {
    const RuntimeState actual = observed(sim);
    const DurationSnapshot p = DurationSnapshot::capture(mission_, ConfigDurationModel{}, actual);
    const RuntimeState expected = expected_state(mission_, sim.plan(), actual, reference_, tasks, {});
    d = decide(mission_, sim.plan(), expected, actual, p, repair_config());
  } catch (const std::invalid_argument &) {
    ++stats_.errors;
    return false;
  }
  timed(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
  switch (d.status) {
    case DecisionStatus::kContinue: ++stats_.continued; return false;
    case DecisionStatus::kNoFeasibleLocalRepair: ++stats_.no_feasible_local; return false;
    case DecisionStatus::kNoSingleEdit: ++stats_.no_single_edit; return false;
    case DecisionStatus::kRepair: break;
  }
  ++stats_.preventive_repairs;
  schedule(sim, d.stage1.admissible[*d.stage1.best]);
  return true;
}

bool MethodController::try_recover(Simulator & sim, Index k)
{
  if (sim.state().tasks[k].status == TaskStatus::kCompleted) {return false;}
  ++stats_.fallback_events;
  const auto t0 = std::chrono::steady_clock::now();
  try {
    const RuntimeState actual = observed(sim);
    const DurationSnapshot p = DurationSnapshot::capture(mission_, ConfigDurationModel{}, actual);
    const Prediction current = Predictor(mission_, sim.plan(), actual, p).full();
    // Best-effort recovery of task k: the policy library applied to k only, judged by a global
    // objective (never-completing tasks, then tardiness). Every unfinished task is "threatened",
    // so no candidate is rejected for touching another one; the objective decides.
    ImpactAnalysis impact;
    impact.expected = current;
    impact.updated = current;
    impact.sources = {k};
    impact.region = {k};
    for (Index j = 0; j < mission_.task_count(); ++j) {
      if (actual.tasks[j].status == TaskStatus::kCompleted) {continue;}
      Threat t;
      t.task = j;
      impact.threats.push_back(t);
      impact.threatened.push_back(j);
    }
    RepairConfig rc = repair_config();
    rc.scope = RepairScope::kRegion;  // the region is {k}
    const StageOneResult r = generate_single_edits(mission_, sim.plan(), actual, p, impact, rc);
    const auto base = recovery_objective(mission_, actual, current);
    const RepairCandidate * best = nullptr;
    std::pair<int, TimeMs> best_obj = base;
    for (const RepairCandidate & c : r.admissible) {
      const auto obj = recovery_objective(mission_, c.state, c.prediction);
      if (obj < best_obj || (best && obj == best_obj &&
        std::tie(c.disruption, c.cost_ms) < std::tie(best->disruption, best->cost_ms)))
      {
        best = &c;
        best_obj = obj;
      }
    }
    timed(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
    if (!best) {return false;}
    ++stats_.fallback_repairs;
    schedule(sim, *best);
    return true;
  } catch (const std::invalid_argument &) {
    ++stats_.errors;
    return false;
  }
}

void MethodController::act(Simulator & sim)
{
  if (config_.method == Method::kNoIntervention) {return;}
  if (refresh_reference_ && !edit_in_flight_) {
    reference_ = predict(sim, observed(sim));
    trigger_.set_reference(reference_);
    refresh_reference_ = false;
  }
  // A breakdown hands every unfinished task of the robot to the recovery (its running task has
  // already been reported as failed), in queue order.
  for (const Index r : broken_robots_) {
    for (const Index j : sim.plan().queue(r)) {
      if (sim.state().tasks[j].status != TaskStatus::kCompleted) {push_unique(pending_recovery_, j);}
    }
  }
  broken_robots_.clear();
  // One edit at a time: with no latency an edit is installed at once and the loop goes on with
  // the next pending item; with latency it waits for the activation.
  for (int guard = 0; guard < 64 && !edit_in_flight_; ++guard) {
    // 1. Common recovery of actual failures, breakdowns and deadline misses (every
    //    repair-enabled method, identically).
    if (!pending_recovery_.empty()) {
      const Index k = pending_recovery_.front();
      pending_recovery_.erase(pending_recovery_.begin());
      try_recover(sim, k);
      continue;
    }
    // 2. Prevention on trigger events (source-only, DWAFP, mission-wide). Without a repair, the
    //    trigger of each task is re-armed so the situation is re-assessed while it grows.
    if (preventive() && !pending_trigger_.empty()) {
      const std::vector<Index> tasks = pending_trigger_;
      pending_trigger_.clear();
      if (!try_preventive(sim, tasks)) {
        for (const Index k : tasks) {trigger_.reset(k);}
      }
      continue;
    }
    break;
  }
}

}  // namespace dwafp
