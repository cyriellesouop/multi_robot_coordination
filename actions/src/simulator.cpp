// DWAFP closed-loop simulator, Step 1: the "world". See include/actions/simulator.hpp.
//
// How one attempt is simulated (all times absolute, nominal ms):
//   start s, real working time D = p_jr x noise factor
//   blocked attempt : works until p = s + f*D, then waits until the obstruction clears
//                     (transient: the absolute clearing time; persistent: never), then works the
//                     rest -> ends at clear + (1 - f)*D, or never
//   failing attempt : works until s + f*D and ends there WITHOUT its output
//   robot breakdown : the running attempt stops at the breakdown time, without its output
// Every event time is computed when the attempt starts, so the loop only has to jump to the
// earliest pending one.

#include "actions/simulator.hpp"

#include <algorithm>  // std::min, std::max
#include <cmath>      // std::llround
#include <stdexcept>  // std::invalid_argument
#include <string>

namespace dwafp
{

namespace
{

// SplitMix64: a small, good-quality hash. Used to draw the noise of one attempt from
// (seed, task, robot, attempt) only, so the same attempt gets the same noise in every run and for
// every method, whatever order things happen in (common random numbers).
std::uint64_t mix(std::uint64_t x)
{
  x += 0x9E3779B97F4A7C15ULL;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
  return x ^ (x >> 31);
}

// Uniform number in [0, 1) from the hash.
double unit(std::uint64_t h) {return static_cast<double>(h >> 11) * (1.0 / 9007199254740992.0);}

}  // namespace

// ----------------------------------------------------------------------------------------------
// Construction
// ----------------------------------------------------------------------------------------------

Simulator::Simulator(const Mission & mission, Plan plan, WorldConfig config)
: mission_(mission), plan_(std::move(plan)), config_(std::move(config))
{
  const std::size_t n = mission_.task_count();
  if (plan_.task_count() != n) {throw std::invalid_argument("simulator: plan of another mission");}
  if (!(config_.noise >= 0.0 && config_.noise < 1.0)) {
    throw std::invalid_argument("simulator: noise must be in [0, 1)");
  }
  if (config_.feedback_period_ms <= 0) {throw std::invalid_argument("simulator: feedback period must be > 0");}
  if (!(config_.timeout_multiplier == 0.0 || config_.timeout_multiplier >= 1.0)) {
    throw std::invalid_argument("simulator: timeout multiplier must be 0 (none) or >= 1");
  }
  for (const InjectedDeviation & d : config_.deviations) {
    if (d.type == DeviationType::kRobotBreakdown) {
      if (d.robot >= mission_.robot_count() || d.at_time_ms < 0) {
        throw std::invalid_argument("simulator: invalid robot breakdown");
      }
    } else if (d.task >= n || !(d.at_fraction >= 0.0 && d.at_fraction <= 1.0) || d.delay_ms < 0) {
      throw std::invalid_argument("simulator: invalid delay or failure deviation");
    }
  }
  state_ = RuntimeState::initial(mission_);
  records_.assign(n, TaskRecord{});
  attempts_.assign(n, Attempt{});
  held_.assign(n, false);
  first_attempt_robot_.assign(n, kNoIndex);
  breakdown_done_.assign(config_.deviations.size(), false);
  clear_ms_.assign(config_.deviations.size(), kUnreachable);
  deadline_reported_.assign(n, false);
  persistent_pair_.assign(n * mission_.robot_count(), false);
}

// ----------------------------------------------------------------------------------------------
// Ground truth of one attempt
// ----------------------------------------------------------------------------------------------

TimeMs Simulator::true_duration(Index j, Index r, int attempt) const
{
  const TimeMs nominal = ConfigDurationModel{}.estimate(mission_, j, r, state_);
  if (config_.noise == 0.0) {return nominal;}
  const std::uint64_t h = mix(config_.seed ^ mix(j * 1000003ULL ^ mix(r * 7919ULL ^ mix(
          static_cast<std::uint64_t>(attempt)))));
  const double factor = 1.0 - config_.noise + 2.0 * config_.noise * unit(h);
  return std::max<TimeMs>(1, std::llround(static_cast<double>(nominal) * factor));
}

void Simulator::start_attempt(Index j, TimeMs t)
{
  const Index r = plan_.robot_of(j);
  TaskRecord & rec = records_[j];
  ++rec.attempts;
  if (rec.attempts == 1) {first_attempt_robot_[j] = r;}

  Attempt a;
  a.active = true;
  a.start_ms = t;
  a.true_duration_ms = true_duration(j, r, rec.attempts);
  a.nominal_duration_ms = ConfigDurationModel{}.estimate(mission_, j, r, state_);

  // Which injected deviations hit THIS attempt: the first attempt always; a later one only if it
  // runs on the robot of the first attempt (the obstruction concerns that robot-task pair), and,
  // for a failure, only if the cause is persistent.
  const bool same_pair = rec.attempts == 1 || r == first_attempt_robot_[j];
  TimeMs work_end = a.true_duration_ms;  // working time after which the attempt ends
  for (const InjectedDeviation & d : config_.deviations) {
    if (d.type == DeviationType::kOutcomeFailure && d.task == j && (rec.attempts == 1 ||
      (d.persistent && same_pair)))
    {
      a.fails = true;
      work_end = std::min<TimeMs>(work_end, std::llround(d.at_fraction * static_cast<double>(a.true_duration_ms)));
    }
  }
  for (std::size_t i = 0; i < config_.deviations.size(); ++i) {
    const InjectedDeviation & d = config_.deviations[i];
    if (d.type != DeviationType::kDelay || d.task != j || !same_pair) {continue;}
    const TimeMs point = std::llround(d.at_fraction * static_cast<double>(a.true_duration_ms));
    if (point > work_end) {continue;}  // the attempt fails before reaching the obstruction
    const TimeMs reach = t + point;     // when this attempt reaches the obstruction
    TimeMs clear = kUnreachable;        // persistent: it never clears
    if (!d.persistent) {
      // Transient: the clearing time is fixed by the FIRST arrival; a later attempt waits only
      // until that same absolute time, or passes freely if it arrives after it.
      if (!is_reachable(clear_ms_[i])) {clear_ms_[i] = reach + d.delay_ms;}
      clear = clear_ms_[i];
      if (reach >= clear) {continue;}
    }
    a.blocks = true;
    a.block_start_ms = reach;
    a.block_end_ms = clear;
    break;  // one obstruction per attempt
  }
  // End: after the blockage (if any), the rest of the work.
  if (a.blocks) {
    a.end_ms = is_reachable(a.block_end_ms) ? a.block_end_ms + (work_end - (a.block_start_ms - t)) : kUnreachable;
  } else {
    a.end_ms = t + work_end;
  }
  // Execution timeout: the robot gives up after q = multiplier x nominal duration and reports a
  // failure (no output) at that time, whatever would have happened later.
  if (config_.timeout_multiplier > 0.0) {
    const TimeMs budget = std::llround(config_.timeout_multiplier * static_cast<double>(a.nominal_duration_ms));
    if (!is_reachable(a.end_ms) || a.end_ms > t + budget) {
      a.fails = true;
      a.times_out = true;
      a.end_ms = t + budget;
    }
  }
  attempts_[j] = a;

  // Observable state: executing since t; until the first feedback, r_hat = nominal duration.
  TaskRuntime & task = state_.tasks[j];
  task.status = TaskStatus::kExecuting;
  task.start_ms = t;
  task.remaining_ms = a.nominal_duration_ms;
  task.end_ms = 0;
  rec.robot = r;
  rec.start_ms = t;
}

TimeMs Simulator::worked_ms(const Attempt & a, TimeMs t) const
{
  TimeMs worked = t - a.start_ms;
  if (a.blocks && t > a.block_start_ms) {
    if (is_reachable(a.block_end_ms) && t >= a.block_end_ms) {
      worked -= a.block_end_ms - a.block_start_ms;  // the whole blockage is behind
    } else {
      worked = a.block_start_ms - a.start_ms;       // blocked right now: no progress
    }
  }
  return std::clamp<TimeMs>(worked, 0, a.true_duration_ms);
}

void Simulator::finish_attempt(Index j, bool success, TimeMs t, SimulationObserver * observer)
{
  Attempt & a = attempts_[j];
  a.active = false;
  TaskRecord & rec = records_[j];
  rec.end_ms = t;
  rec.status = success ? FinalStatus::kCompleted : FinalStatus::kFailed;
  rec.deadline_ok = success && t <= mission_.task(j).deadline_ms;
  if (a.times_out) {++rec.timeouts;}
  if (!success) {
    rec.discarded_work_ms += worked_ms(a, t);  // progress lost
    rec.wasted_elapsed_ms += t - a.start_ms;   // time lost, blocked time included
  }

  TaskRuntime & task = state_.tasks[j];
  task.status = success ? TaskStatus::kCompleted : TaskStatus::kFailed;
  task.end_ms = t;
  task.remaining_ms = 0;
  if (observer) {observer->on_task_finished(j, success, t);}
}

// ----------------------------------------------------------------------------------------------
// Dispatcher
// ----------------------------------------------------------------------------------------------

void Simulator::dispatch(TimeMs t)
{
  for (Index r = 0; r < mission_.robot_count(); ++r) {
    if (!state_.robots[r].available) {continue;}
    // The robot's next unfinished task in its fixed queue; if it is running, the robot is busy.
    Index next = kNoIndex;
    for (const Index j : plan_.queue(r)) {
      const TaskStatus s = state_.tasks[j].status;
      if (s == TaskStatus::kCompleted || s == TaskStatus::kFailed) {continue;}
      next = j;
      break;
    }
    if (next == kNoIndex || state_.tasks[next].status != TaskStatus::kUnstarted) {continue;}
    if (held_[next] || state_.tasks[next].release_ms > t) {continue;}
    bool inputs_ready = true;
    for (const Index h : mission_.dependency_predecessors(next)) {
      inputs_ready = inputs_ready && state_.tasks[h].status == TaskStatus::kCompleted;
    }
    if (inputs_ready) {start_attempt(next, t);}
  }
}

// ----------------------------------------------------------------------------------------------
// Event loop
// ----------------------------------------------------------------------------------------------

TimeMs Simulator::next_event_time() const
{
  const TimeMs now = state_.now_ms;
  TimeMs next = kUnreachable;
  bool any_active = false;
  for (Index j = 0; j < attempts_.size(); ++j) {
    const Attempt & a = attempts_[j];
    if (!a.active) {continue;}
    any_active = true;
    next = std::min(next, a.end_ms);  // kUnreachable for a persistent blockage
    if (a.blocks && a.block_start_ms > now) {next = std::min(next, a.block_start_ms);}
    if (a.blocks && is_reachable(a.block_end_ms) && a.block_end_ms > now) {next = std::min(next, a.block_end_ms);}
  }
  // Feedback ticks only matter while something is running.
  if (any_active) {
    next = std::min(next, (now / config_.feedback_period_ms + 1) * config_.feedback_period_ms);
  }
  for (std::size_t i = 0; i < config_.deviations.size(); ++i) {
    const InjectedDeviation & d = config_.deviations[i];
    if (d.type == DeviationType::kRobotBreakdown && !breakdown_done_[i]) {
      next = std::min(next, std::max(d.at_time_ms, now + 1));
    }
  }
  // A task that only waits for its release time.
  for (Index j = 0; j < state_.tasks.size(); ++j) {
    const TaskRuntime & task = state_.tasks[j];
    if (task.status == TaskStatus::kUnstarted && task.release_ms > now) {next = std::min(next, task.release_ms);}
  }
  // An edit to install.
  for (const PendingEdit & e : pending_edits_) {next = std::min(next, std::max(e.activation_ms, now + 1));}
  // A wake-up requested by the controller.
  for (const TimeMs w : wakeups_) {
    if (w > now) {next = std::min(next, w);}
  }
  // A deadline still to be reported (a task not completed by it).
  for (Index j = 0; j < state_.tasks.size(); ++j) {
    const TimeMs d = mission_.task(j).deadline_ms;
    if (!deadline_reported_[j] && state_.tasks[j].status != TaskStatus::kCompleted && d > now) {
      next = std::min(next, d);
    }
  }
  return next;
}

bool Simulator::step(SimulationObserver * observer)
{
  if (finished_) {return false;}
  TimeMs t = 0;
  if (started_) {
    t = next_event_time();
    if (!is_reachable(t) || t > config_.horizon_ms) {
      // The run is over: an attempt still running or blocked is incomplete; a task that had
      // attempts but is unstarted again (cancelled or reset by a repair) is unstarted at the end.
      for (Index j = 0; j < attempts_.size(); ++j) {
        if (attempts_[j].active) {
          records_[j].status = FinalStatus::kIncomplete;
        } else if (state_.tasks[j].status == TaskStatus::kUnstarted && records_[j].attempts > 0) {
          records_[j].status = FinalStatus::kUnstartedAtEnd;
        }
      }
      finished_ = true;
      return false;
    }
  }
  started_ = true;
  state_.now_ms = t;
  // Wake-ups up to now are consumed by this step.
  wakeups_.erase(std::remove_if(wakeups_.begin(), wakeups_.end(), [t](TimeMs w) {return w <= t;}),
    wakeups_.end());

  // 1. Attempts that end now (completions before breakdowns: a robot that finishes exactly when
  //    it breaks down has finished).
  for (Index j = 0; j < attempts_.size(); ++j) {
    if (attempts_[j].active && attempts_[j].end_ms == t) {finish_attempt(j, !attempts_[j].fails, t, observer);}
  }

  // 2. Robot breakdowns: the robot is unavailable from now on; its running attempt stops
  //    without its output.
  for (std::size_t i = 0; i < config_.deviations.size(); ++i) {
    const InjectedDeviation & d = config_.deviations[i];
    if (d.type != DeviationType::kRobotBreakdown || breakdown_done_[i] || d.at_time_ms > t) {continue;}
    breakdown_done_[i] = true;
    state_.robots[d.robot].available = false;
    state_.robots[d.robot].unavailable_since_ms = t;
    if (observer) {observer->on_robot_unavailable(d.robot, t);}
    for (const Index j : plan_.queue(d.robot)) {
      if (attempts_[j].active) {finish_attempt(j, false, t, observer);}
    }
  }

  // 3. Deadline misses: every task not completed by its deadline, reported once (it may still be
  //    running, blocked, failed or not started).
  for (Index j = 0; j < state_.tasks.size(); ++j) {
    if (deadline_reported_[j] || state_.tasks[j].status == TaskStatus::kCompleted) {continue;}
    if (mission_.task(j).deadline_ms > t) {continue;}
    deadline_reported_[j] = true;
    records_[j].deadline_missed_ms = t;
    if (observer) {observer->on_deadline_missed(j, t);}
  }

  // 4. Feedback of every running attempt, at the feedback period.
  if (t % config_.feedback_period_ms == 0) {
    for (Index j = 0; j < attempts_.size(); ++j) {
      const Attempt & a = attempts_[j];
      if (!a.active) {continue;}
      const double progress = static_cast<double>(worked_ms(a, t)) / static_cast<double>(a.true_duration_ms);
      FeedbackSample sample;
      sample.task = j;
      sample.time_ms = t;
      sample.remaining_ms = std::llround(static_cast<double>(a.nominal_duration_ms) * (1.0 - progress));
      sample.blocked = a.blocks && a.block_start_ms <= t && t < a.block_end_ms;
      state_.tasks[j].remaining_ms = sample.remaining_ms;
      if (sample.blocked) {
        // The robot reports whether its own retry could work (transient vs persistent cause).
        // The diagnostic belongs to this (task, robot) pair and is remembered for it.
        const Index r = plan_.robot_of(j);
        for (const InjectedDeviation & d : config_.deviations) {
          if (d.type == DeviationType::kDelay && d.task == j) {
            state_.tasks[j].same_robot_retry_feasible = !d.persistent;
            if (d.persistent) {persistent_pair_[j * mission_.robot_count() + r] = true;}
          }
        }
      }
      if (observer) {observer->on_feedback(sample);}
    }
  }

  // 5. Install the scheduled edits whose activation time has come (in scheduling order). An edit
  //    is rejected if one of its tasks completed before activation, or if it no longer fits.
  std::vector<PendingEdit> due;
  for (auto it = pending_edits_.begin(); it != pending_edits_.end(); ) {
    if (it->activation_ms <= t) {
      due.push_back(std::move(*it));
      it = pending_edits_.erase(it);
    } else {
      ++it;
    }
  }
  for (const PendingEdit & e : due) {
    const bool installed = install(e.plan, e.edited, t, nullptr) == EditOutcome::kInstalled;
    if (observer) {observer->on_edit_installed(installed, t);}
  }

  // 6. The controller decides (hold, edit the plan) with everything it was told at t.
  if (observer) {observer->act(*this);}

  // 7. Start everything that can start now.
  dispatch(t);
  return true;
}

const std::vector<TaskRecord> & Simulator::run(SimulationObserver * observer)
{
  while (step(observer)) {}
  return records_;
}

void Simulator::set_held(const std::vector<Index> & tasks)
{
  held_.assign(held_.size(), false);
  for (const Index j : tasks) {held_.at(j) = true;}
}

void Simulator::request_wakeup(TimeMs t)
{
  if (t > state_.now_ms) {wakeups_.push_back(t);}
}

void Simulator::schedule_edit(const Plan & new_plan, const std::vector<Index> & edited, TimeMs activation_ms)
{
  ++edit_counts_.scheduled;
  if (activation_ms <= state_.now_ms) {
    apply_edit(new_plan, edited, state_.now_ms);  // no latency: install now (throws if invalid)
    return;
  }
  pending_edits_.push_back({new_plan, edited, activation_ms});
}

void Simulator::apply_edit(const Plan & new_plan, const std::vector<Index> & edited, TimeMs release_ms)
{
  std::string error;
  if (install(new_plan, edited, release_ms, &error) != EditOutcome::kInstalled) {
    throw std::invalid_argument(error);
  }
}

Simulator::EditOutcome Simulator::install(
  const Plan & new_plan, const std::vector<Index> & edited, TimeMs release_ms, std::string * error)
{
  // Reason 1: an edited task has completed (e.g. before the activation of a scheduled edit).
  for (const Index j : edited) {
    if (j < state_.tasks.size() && state_.tasks[j].status == TaskStatus::kCompleted) {
      ++edit_counts_.rejected_task_completed;
      if (error) {*error = "edit: '" + mission_.task(j).id + "' is already completed";}
      return EditOutcome::kTaskCompleted;
    }
  }
  // Reason 2: anything else that makes the edit invalid.
  try {
    install_or_throw(new_plan, edited, release_ms);
  } catch (const std::invalid_argument & e) {
    ++edit_counts_.rejected_invalid;
    if (error) {*error = e.what();}
    return EditOutcome::kInvalid;
  }
  ++edit_counts_.installed;
  return EditOutcome::kInstalled;
}

void Simulator::install_or_throw(const Plan & new_plan, const std::vector<Index> & edited, TimeMs release_ms)
{
  const std::size_t n = mission_.task_count();
  const TimeMs now = state_.now_ms;
  if (new_plan.task_count() != n) {throw std::invalid_argument("apply_edit: plan of another mission");}
  release_ms = std::max(release_ms, now);

  // 1. Build the new observable state on a COPY, so an invalid edit changes nothing.
  std::vector<bool> is_edited(n, false);
  RuntimeState next = state_;
  for (const Index j : edited) {
    if (j >= n) {throw std::invalid_argument("apply_edit: task index out of range");}
    if (state_.tasks[j].status == TaskStatus::kCompleted) {
      throw std::invalid_argument("apply_edit: '" + mission_.task(j).id + "' is already completed");
    }
    is_edited[j] = true;
    // Retry diagnostic: same robot -> keep it; another robot -> that pair's own diagnostic.
    const Index new_robot = new_plan.robot_of(j);
    const bool diagnostic = new_robot == plan_.robot_of(j) ? state_.tasks[j].same_robot_retry_feasible :
      !persistent_pair_[j * mission_.robot_count() + new_robot];
    next.tasks[j] = TaskRuntime{};  // unstarted again (cancelled, retried or just moved)
    next.tasks[j].release_ms = release_ms;
    next.tasks[j].same_robot_retry_feasible = diagnostic;
  }
  for (Index j = 0; j < n; ++j) {
    if (attempts_[j].active && !is_edited[j] && new_plan.robot_of(j) != plan_.robot_of(j)) {
      throw std::invalid_argument(
              "apply_edit: running task '" + mission_.task(j).id + "' cannot change robot without restarting");
    }
  }
  // 2. The new queues must fit the state (finished tasks first, at most one running task, then
  //    unstarted ones): the Predictor checks exactly that and throws otherwise.
  const DurationSnapshot durations = DurationSnapshot::capture(mission_, ConfigDurationModel{}, next);
  const Predictor check(mission_, new_plan, next, durations);
  (void)check;

  // 3. Commit: cancel the running edited attempts (their progress is lost), then switch.
  for (const Index j : edited) {
    Attempt & a = attempts_[j];
    if (!a.active) {continue;}
    a.active = false;
    records_[j].discarded_work_ms += worked_ms(a, now);
    records_[j].wasted_elapsed_ms += now - a.start_ms;
    records_[j].status = FinalStatus::kUnstartedAtEnd;  // until it is dispatched again
  }
  state_ = std::move(next);
  plan_ = new_plan;
}

}  // namespace dwafp
