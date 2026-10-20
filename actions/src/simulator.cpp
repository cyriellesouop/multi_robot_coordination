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
      // The run is over: an attempt still running or blocked is incomplete.
      for (Index j = 0; j < attempts_.size(); ++j) {
        if (attempts_[j].active) {records_[j].status = FinalStatus::kIncomplete;}
      }
      finished_ = true;
      return false;
    }
  }
  started_ = true;
  state_.now_ms = t;

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
        for (const InjectedDeviation & d : config_.deviations) {
          if (d.type == DeviationType::kDelay && d.task == j) {
            state_.tasks[j].same_robot_retry_feasible = !d.persistent;
          }
        }
      }
      if (observer) {observer->on_feedback(sample);}
    }
  }

  // 5. Start everything that can start now.
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

}  // namespace dwafp
