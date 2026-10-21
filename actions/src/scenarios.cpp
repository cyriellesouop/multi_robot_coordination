// Scenario generator for the closed-loop experiments. See include/actions/scenarios.hpp.

#include "actions/scenarios.hpp"

#include <algorithm>  // std::max, std::min
#include <cmath>      // std::llround, std::sqrt
#include <optional>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dwafp
{

const char * shape_name(WorkflowShape s)
{
  switch (s) {
    case WorkflowShape::kRandomDag: return "random DAG";
    case WorkflowShape::kChains: return "chains";
    case WorkflowShape::kForkJoin: return "fork-join";
    case WorkflowShape::kRendezvous: return "rendezvous";
  }
  return "?";
}

const char * source_class_name(SourceClass c)
{
  switch (c) {
    case SourceClass::kSourceOnly: return "source only";
    case SourceClass::kSameRobotQueue: return "same-robot queue";
    case SourceClass::kDownstreamDependency: return "downstream dependency";
    case SourceClass::kCrossRobotHandoff: return "cross-robot handoff";
    case SourceClass::kNotApplicable: return "n/a";
  }
  return "?";
}

const char * deadline_structure_name(DeadlineStructure d)
{
  switch (d) {
    case DeadlineStructure::kEveryTask: return "every task";
    case DeadlineStructure::kHandoffAndFinal: return "handoff+final";
    case DeadlineStructure::kFinalOnly: return "final only";
  }
  return "?";
}

namespace
{

std::string id(int j) {return "T" + std::to_string(j);}

// Dependencies (predecessor indices per task) and robots of the chosen shape. Every predecessor
// index is smaller than the task's own index (a topological order).
void build_structure(const ScenarioConfig & c, std::mt19937_64 & rng,
  std::vector<std::vector<int>> & preds, std::vector<int> & robot)
{
  const int n = c.tasks;
  const int r = c.robots;
  preds.assign(static_cast<std::size_t>(n), {});
  robot.assign(static_cast<std::size_t>(n), 0);
  std::uniform_int_distribution<int> any_robot(0, r - 1);
  std::uniform_real_distribution<double> coin(0.0, 1.0);
  // Locality: stay on the robot of the first predecessor with probability 0.7.
  const auto sticky = [&](int j) {
      const auto & p = preds[static_cast<std::size_t>(j)];
      const int random_robot = any_robot(rng);  // always drawn: keeps the random stream aligned
      const bool stay = coin(rng) < 0.7;
      robot[static_cast<std::size_t>(j)] = (!p.empty() && stay) ? robot[static_cast<std::size_t>(p.front())] : random_robot;
    };

  switch (c.shape) {
    case WorkflowShape::kRandomDag: {
        const double p = std::min(0.5, 3.0 / n);
        for (int j = 0; j < n; ++j) {
          for (int h = 0; h < j; ++h) {
            if (coin(rng) < p) {preds[static_cast<std::size_t>(j)].push_back(h);}
          }
          sticky(j);
        }
        break;
      }
    case WorkflowShape::kChains: {
        const int chains = std::max(2, r / 2);
        for (int j = 0; j < n; ++j) {
          if (j >= chains) {preds[static_cast<std::size_t>(j)].push_back(j - chains);}
          sticky(j);
        }
        break;
      }
    case WorkflowShape::kForkJoin: {
        // fork, then repeated stages: w branches (after the previous fork/join), then a join.
        const int w = std::min(3, std::max(2, r));
        int anchor = 0;  // task 0 is the first fork
        robot[0] = any_robot(rng);
        int j = 1;
        while (j < n) {
          std::vector<int> branches;
          for (int b = 0; b < w && j < n; ++b, ++j) {
            preds[static_cast<std::size_t>(j)].push_back(anchor);
            robot[static_cast<std::size_t>(j)] = (robot[static_cast<std::size_t>(anchor)] + b) % r;
            branches.push_back(j);
          }
          if (j < n) {
            preds[static_cast<std::size_t>(j)] = branches;  // the join needs every branch
            robot[static_cast<std::size_t>(j)] = robot[static_cast<std::size_t>(branches.front())];
            anchor = j++;
          }
        }
        break;
      }
    case WorkflowShape::kRendezvous: {
        const int lines = std::min(4, std::max(2, r));
        for (int j = 0; j < n; ++j) {
          const int line = j % lines;
          const int round = j / lines;
          robot[static_cast<std::size_t>(j)] = line % r;  // each line is one robot's work
          if (round == 0) {continue;}
          preds[static_cast<std::size_t>(j)].push_back(j - lines);  // previous task of the line
          if (line == 0 && round % 3 == 0) {
            // Meeting point: also needs the previous task of every other line.
            for (int other = 1; other < lines; ++other) {
              preds[static_cast<std::size_t>(j)].push_back((round - 1) * lines + other);
            }
          }
        }
        break;
      }
  }
}

}  // namespace

namespace
{

// One generation attempt. Returns nullopt when the primary source selection finds no eligible
// source (no task with a dependency successor starts in the window): the caller then draws a new
// graph. It never falls back to another selection rule.
std::optional<Scenario> try_make_scenario(const ScenarioConfig & c, std::uint64_t seed, std::uint64_t attempt)
{
  if (c.robots < 1 || c.tasks < 2 || c.work_min < 1 || c.work_max < c.work_min ||
    c.slack_lo < 0.0 || c.slack_hi < c.slack_lo || c.delay_lo < 0.0 || c.delay_hi < c.delay_lo ||
    !(c.window > 0.0 && c.window <= 1.0) ||
    // progress: a share of the work in [0, 1) (at 1 the task would already be done)
    !(c.progress_lo >= 0.0 && c.progress_hi >= c.progress_lo && c.progress_hi < 1.0) ||
    (c.execution_time_ratio != 0.0 && !(c.execution_time_ratio >= 1.0)) || c.spare_robots < 0 ||
    // busy_fleet checks the fleet at ONE deviation time, so the progress must be fixed
    (c.busy_fleet && c.progress_lo != c.progress_hi))
  {
    throw std::invalid_argument("make_scenario: invalid configuration");
  }
  const double mix_total = c.mix.transient_delay + c.mix.persistent_delay + c.mix.outcome_failure +
    c.mix.robot_breakdown;
  if (!(mix_total > 0.0)) {throw std::invalid_argument("make_scenario: empty deviation mix");}

  // Attempt 0 uses the seed itself; a regeneration uses a stream derived from (seed, attempt).
  std::mt19937_64 rng(attempt == 0 ? seed : seed ^ (0x9E3779B97F4A7C15ULL * attempt));
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  // Robots: durations per unit of work cycle through 1.0 / 1.5 / 0.8 s (default), or
  // 1.0 / sqrt(rho) / rho with an execution-time ratio rho, or all 1.0 (homogeneous_rates). Spares come after the workflow robots
  // and continue the same cycle; build_structure() only assigns tasks to the first c.robots
  // robots, and no random number is drawn here, so spares never change the drawn workflow.
  const double legacy[] = {1000, 1500, 800};
  const double rho = c.execution_time_ratio;
  const double ratio_rates[] = {1000, 1000 * std::sqrt(rho), 1000 * rho};
  std::vector<Robot> fleet;
  for (int r = 0; r < c.robots + c.spare_robots; ++r) {
    const double rate = c.homogeneous_rates ? 1000.0 : (rho >= 1.0 ? ratio_rates[r % 3] : legacy[r % 3]);
    fleet.push_back({"robot" + std::to_string(r + 1), rate});
  }

  // Structure and work.
  std::vector<std::vector<int>> preds;
  std::vector<int> robot;
  build_structure(c, rng, preds, robot);
  std::uniform_int_distribution<int> work(c.work_min, c.work_max);
  std::vector<Subtask> tasks(static_cast<std::size_t>(c.tasks));
  for (int j = 0; j < c.tasks; ++j) {
    Subtask & t = tasks[static_cast<std::size_t>(j)];
    t.id = id(j);
    t.robot = fleet[static_cast<std::size_t>(robot[static_cast<std::size_t>(j)])].name;
    t.work = work(rng);
    t.deadline_ms = kNoDeadline;  // set below from the nominal schedule
    for (const int h : preds[static_cast<std::size_t>(j)]) {t.predecessors.push_back(id(h));}
  }

  // Which tasks get a deadline: every task, handoff and final tasks, or final tasks only.
  std::vector<bool> gets_deadline(static_cast<std::size_t>(c.tasks), c.deadlines == DeadlineStructure::kEveryTask);
  if (c.deadlines == DeadlineStructure::kHandoffAndFinal) {
    std::vector<bool> has_successor(static_cast<std::size_t>(c.tasks), false);
    for (int j = 0; j < c.tasks; ++j) {
      for (const int h : preds[static_cast<std::size_t>(j)]) {
        has_successor[static_cast<std::size_t>(h)] = true;
        if (robot[static_cast<std::size_t>(h)] != robot[static_cast<std::size_t>(j)]) {
          gets_deadline[static_cast<std::size_t>(h)] = true;  // handoff to another robot
        }
      }
    }
    for (int j = 0; j < c.tasks; ++j) {
      if (!has_successor[static_cast<std::size_t>(j)]) {gets_deadline[static_cast<std::size_t>(j)] = true;}  // final
    }
  }
  if (c.deadlines == DeadlineStructure::kFinalOnly) {
    // Sinks only: a task that is the predecessor of another one has no deadline of its own.
    std::fill(gets_deadline.begin(), gets_deadline.end(), true);
    for (int j = 0; j < c.tasks; ++j) {
      for (const int h : preds[static_cast<std::size_t>(j)]) {gets_deadline[static_cast<std::size_t>(h)] = false;}
    }
  }

  // Nominal schedule (no deadline pressure), then deadlines with duration-normalized additive slack.
  TimeMs makespan = 0;
  {
    const Mission draft("draft", tasks, fleet);
    const Plan plan = Plan::initial(draft);
    const RuntimeState s0 = RuntimeState::initial(draft);
    const DurationSnapshot p = DurationSnapshot::capture(draft, ConfigDurationModel{}, s0);
    const Prediction nominal = Predictor(draft, plan, s0, p).full();
    std::uniform_real_distribution<double> slack(c.slack_lo, c.slack_hi);
    for (std::size_t j = 0; j < tasks.size(); ++j) {
      const TimeMs own = p.at(j, plan.robot_of(j));
      const double s = slack(rng);  // drawn for every task, so the stream does not depend on the
                                    // deadline structure
      makespan = std::max(makespan, nominal.tasks[j].finish_ms);
      if (gets_deadline[j]) {
        tasks[j].deadline_ms = nominal.tasks[j].finish_ms + static_cast<TimeMs>(s * static_cast<double>(own));
      }
    }
  }

  Mission mission("scenario", tasks, fleet);
  Plan plan = Plan::initial(mission);
  const RuntimeState s0 = RuntimeState::initial(mission);
  const DurationSnapshot p = DurationSnapshot::capture(mission, ConfigDurationModel{}, s0);
  const Prediction nominal = Predictor(mission, plan, s0, p).full();

  // The deviation.
  const double pick = unit(rng) * mix_total;
  InjectedDeviation d;
  const TimeMs window_end = std::max<TimeMs>(1, static_cast<TimeMs>(c.window * static_cast<double>(makespan)));
  std::string what;
  if (pick < c.mix.transient_delay + c.mix.persistent_delay + c.mix.outcome_failure) {
    // A task starting in the window (primary: one whose deviation can propagate through a
    // dependency; secondary: any task). No eligible task: this graph is rejected (regenerated).
    std::vector<Index> candidates;
    // busy_fleet: robot r (a workflow robot, not a spare) executes some task at time t in the
    // nominal plan.
    const auto executing_at = [&](Index r, TimeMs t) {
        for (const Index k : plan.queue(r)) {
          if (nominal.tasks[k].start_ms <= t && t < nominal.tasks[k].finish_ms) {return true;}
        }
        return false;
      };
    for (Index j = 0; j < mission.task_count(); ++j) {
      if (nominal.tasks[j].start_ms >= window_end) {continue;}
      if (c.source == SourceSelection::kWithDependents && mission.dependency_successors(j).empty()) {continue;}
      if (c.busy_fleet) {
        // Deviation time b with the fixed progress; every OTHER workflow robot must be executing
        // at b and at b + 2 s (the source's own robot is busy with the deviating source).
        const TimeMs b = nominal.tasks[j].start_ms +
          std::llround(c.progress_lo * static_cast<double>(p.at(j, plan.robot_of(j))));
        bool all_busy = true;
        for (Index r = 0; r < static_cast<Index>(c.robots) && all_busy; ++r) {
          if (r == plan.robot_of(j)) {continue;}
          all_busy = executing_at(r, b) && executing_at(r, b + 2000);
        }
        if (!all_busy) {continue;}
      }
      candidates.push_back(j);
    }
    if (candidates.empty()) {return std::nullopt;}
    d.task = candidates[std::uniform_int_distribution<std::size_t>(0, candidates.size() - 1)(rng)];
    // One draw in every case (the stream does not depend on the progress setting); with the
    // defaults 0.2 / 0.8 this is the original U[0.2, 0.8].
    d.at_fraction = c.progress_lo + (c.progress_hi - c.progress_lo) * unit(rng);
    if (pick < c.mix.transient_delay) {
      d.type = DeviationType::kDelay;
      d.persistent = false;
      const double own = static_cast<double>(p.at(d.task, plan.robot_of(d.task)));
      d.delay_ms = std::llround(std::uniform_real_distribution<double>(c.delay_lo, c.delay_hi)(rng) * own);
      what = "transient delay of " + std::to_string(d.delay_ms) + " ms";
    } else if (pick < c.mix.transient_delay + c.mix.persistent_delay) {
      d.type = DeviationType::kDelay;
      d.persistent = true;
      what = "persistent obstruction";
    } else {
      d.type = DeviationType::kOutcomeFailure;
      what = "outcome failure";
    }
    what += " of " + mission.task(d.task).id + " at " + std::to_string(static_cast<int>(100 * d.at_fraction)) + "%";
  } else {
    // A robot that still has unfinished work at a time in the window.
    d.type = DeviationType::kRobotBreakdown;
    for (int attempt = 0; attempt < 100; ++attempt) {
      const TimeMs t = std::uniform_int_distribution<TimeMs>(window_end / 10, window_end)(rng);
      std::vector<Index> busy;
      for (Index r = 0; r < mission.robot_count(); ++r) {
        for (const Index j : plan.queue(r)) {
          if (nominal.tasks[j].finish_ms > t) {busy.push_back(r); break;}
        }
      }
      if (busy.empty()) {continue;}
      d.robot = busy[std::uniform_int_distribution<std::size_t>(0, busy.size() - 1)(rng)];
      d.at_time_ms = t;
      break;
    }
    if (d.robot == kNoIndex) {  // degenerate case: break the robot of the last task at time 0
      d.robot = plan.robot_of(mission.task_count() - 1);
      d.at_time_ms = 0;
    }
    what = "breakdown of " + mission.robot(d.robot).name + " at " + std::to_string(d.at_time_ms) + " ms";
  }

  // Source class: the strongest kind of consequence the structure allows, over ALL dependency
  // descendants (T1@R1 -> T2@R1 -> T3@R2: T1's consequences cross robots through T2).
  SourceClass source_class = SourceClass::kNotApplicable;
  if (d.type != DeviationType::kRobotBreakdown) {
    source_class = SourceClass::kSourceOnly;
    if (plan.queue_successor(d.task) != kNoIndex) {source_class = SourceClass::kSameRobotQueue;}
    std::vector<bool> seen(mission.task_count(), false);
    std::vector<Index> stack(mission.dependency_successors(d.task).begin(), mission.dependency_successors(d.task).end());
    if (!stack.empty()) {source_class = SourceClass::kDownstreamDependency;}
    while (!stack.empty()) {
      const Index k = stack.back();
      stack.pop_back();
      if (seen[k]) {continue;}
      seen[k] = true;
      if (plan.robot_of(k) != plan.robot_of(d.task)) {
        source_class = SourceClass::kCrossRobotHandoff;
        break;
      }
      for (const Index s : mission.dependency_successors(k)) {stack.push_back(s);}
    }
  }

  WorldConfig world;
  world.seed = seed;
  world.noise = c.noise;
  world.timeout_multiplier = c.timeout_multiplier;
  world.horizon_ms = c.horizon_ms;
  world.deviations.push_back(d);

  const std::string description = std::string(shape_name(c.shape)) + ", " +
    deadline_structure_name(c.deadlines) + ", " + std::to_string(c.robots) + " robots, " +
    std::to_string(c.tasks) + " tasks" + (c.spare_robots ? " + " + std::to_string(c.spare_robots) + " spare" : std::string()) +
    (c.busy_fleet ? ", busy fleet" : "") + ": " + what + " (" + source_class_name(source_class) + ")";
  return Scenario{std::move(mission), std::move(plan), std::move(world), d, source_class, makespan, description};
}

}  // namespace

Scenario make_scenario(const ScenarioConfig & c, std::uint64_t seed)
{
  // Deterministic regeneration: the same seed always leads to the same accepted attempt.
  constexpr std::uint64_t kAttempts = 100;
  for (std::uint64_t attempt = 0; attempt < kAttempts; ++attempt) {
    if (auto s = try_make_scenario(c, seed, attempt)) {return std::move(*s);}
  }
  throw std::invalid_argument(
          "make_scenario: no eligible source (a task with a dependency successor starting in the "
          "injection window, with every workflow robot busy at its deviation if busy_fleet) after " +
          std::to_string(kAttempts) + " graphs; use SourceSelection::kAny, a denser workflow, or no busy_fleet");
}

}  // namespace dwafp
