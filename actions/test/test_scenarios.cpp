// Unit tests of the scenario generator (Phase C6, Step 4).

#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>
#include <vector>

#include "actions/scenarios.hpp"
#include "actions/simulator.hpp"

using dwafp::DeadlineStructure;
using dwafp::DeviationType;
using dwafp::FinalStatus;
using dwafp::Index;
using dwafp::Scenario;
using dwafp::ScenarioConfig;
using dwafp::Simulator;
using dwafp::WorkflowShape;

namespace
{

const WorkflowShape kShapes[] = {WorkflowShape::kRandomDag, WorkflowShape::kChains,
  WorkflowShape::kForkJoin, WorkflowShape::kRendezvous};
const DeadlineStructure kStructures[] = {DeadlineStructure::kEveryTask, DeadlineStructure::kHandoffAndFinal};

// The world of a scenario without its deviation (and without noise).
dwafp::WorldConfig nominal_world(const Scenario & s)
{
  dwafp::WorldConfig w = s.world;
  w.deviations.clear();
  w.noise = 0.0;
  return w;
}

}  // namespace

// Every shape, deadline structure and size gives a valid mission whose nominal execution (no
// deviation, no noise) completes every task and meets every deadline.
TEST(Scenarios, NominalExecutionMeetsEveryDeadline)
{
  for (const WorkflowShape shape : kShapes) {
    for (const DeadlineStructure structure : kStructures) {
      for (const auto & [robots, tasks] : std::vector<std::pair<int, int>>{{2, 10}, {4, 20}, {8, 40}}) {
        for (std::uint64_t seed = 1; seed <= 10; ++seed) {
          ScenarioConfig c;
          c.shape = shape;
          c.deadlines = structure;
          c.robots = robots;
          c.tasks = tasks;
          const Scenario s = dwafp::make_scenario(c, seed);
          SCOPED_TRACE(s.description + ", seed " + std::to_string(seed));
          ASSERT_EQ(static_cast<int>(s.mission.task_count()), tasks);
          ASSERT_EQ(static_cast<int>(s.mission.robot_count()), robots);
          Simulator sim(s.mission, s.plan, nominal_world(s));
          const auto & rec = sim.run();
          for (Index j = 0; j < s.mission.task_count(); ++j) {
            EXPECT_EQ(rec[j].status, FinalStatus::kCompleted) << s.mission.task(j).id;
            EXPECT_TRUE(rec[j].deadline_ok) << s.mission.task(j).id;
          }
        }
      }
    }
  }
}

// The same seed gives the same scenario in EVERY respect: dependencies, assignments, work,
// deadlines, robots, the deviation and the world configuration. Another seed gives another one.
TEST(Scenarios, SameSeedSameScenario)
{
  for (const WorkflowShape shape : kShapes) {
    ScenarioConfig c;
    c.shape = shape;
    c.noise = 0.1;
    const Scenario a = dwafp::make_scenario(c, 42);
    const Scenario b = dwafp::make_scenario(c, 42);
    const Scenario other = dwafp::make_scenario(c, 43);
    SCOPED_TRACE(a.description);
    ASSERT_EQ(a.mission.task_count(), b.mission.task_count());
    bool differs = a.description != other.description;
    for (Index j = 0; j < a.mission.task_count(); ++j) {
      const auto & ta = a.mission.task(j);
      const auto & tb = b.mission.task(j);
      EXPECT_EQ(ta.predecessors, tb.predecessors);
      EXPECT_EQ(ta.robot, tb.robot);
      EXPECT_EQ(ta.work, tb.work);
      EXPECT_EQ(ta.deadline_ms, tb.deadline_ms);
      EXPECT_EQ(a.plan.robot_of(j), b.plan.robot_of(j));
      differs = differs || ta.deadline_ms != other.mission.task(j).deadline_ms || ta.robot != other.mission.task(j).robot;
    }
    for (Index r = 0; r < a.mission.robot_count(); ++r) {
      EXPECT_EQ(a.mission.robot(r).ms_per_work_unit, b.mission.robot(r).ms_per_work_unit);
    }
    const auto & da = a.deviation;
    const auto & db = b.deviation;
    EXPECT_EQ(da.type, db.type);
    EXPECT_EQ(da.task, db.task);
    EXPECT_EQ(da.robot, db.robot);
    EXPECT_EQ(da.at_fraction, db.at_fraction);
    EXPECT_EQ(da.at_time_ms, db.at_time_ms);
    EXPECT_EQ(da.delay_ms, db.delay_ms);
    EXPECT_EQ(da.persistent, db.persistent);
    EXPECT_EQ(a.source_class, b.source_class);
    EXPECT_EQ(a.world.seed, b.world.seed);
    EXPECT_EQ(a.world.noise, b.world.noise);
    EXPECT_EQ(a.world.timeout_multiplier, b.world.timeout_multiplier);
    EXPECT_EQ(a.world.horizon_ms, b.world.horizon_ms);
    EXPECT_EQ(a.world.deviations.size(), 1u);
    EXPECT_TRUE(differs);
  }
}

// Handoff-and-final deadlines: a task has a deadline exactly when its output is needed on another
// robot or nothing depends on it.
TEST(Scenarios, HandoffAndFinalDeadlinesAreWhereTheyShouldBe)
{
  for (const WorkflowShape shape : kShapes) {
    ScenarioConfig c;
    c.shape = shape;
    c.deadlines = DeadlineStructure::kHandoffAndFinal;
    const Scenario s = dwafp::make_scenario(c, 7);
    const auto & m = s.mission;
    SCOPED_TRACE(s.description);
    int with_deadline = 0;
    for (Index j = 0; j < m.task_count(); ++j) {
      const auto & succ = m.dependency_successors(j);
      bool handoff = false;
      for (const Index k : succ) {handoff = handoff || s.plan.robot_of(k) != s.plan.robot_of(j);}
      const bool expected = handoff || succ.empty();
      EXPECT_EQ(dwafp::has_deadline(m, j), expected) << m.task(j).id;
      with_deadline += dwafp::has_deadline(m, j) ? 1 : 0;
    }
    EXPECT_GT(with_deadline, 0);
    EXPECT_LT(with_deadline, static_cast<int>(m.task_count()));  // not every task
  }
}

// Each shape has its structure: chains have at most one predecessor per task, fork-join and
// rendezvous have joins (several predecessors), rendezvous joins combine several robots.
TEST(Scenarios, ShapesHaveTheirStructure)
{
  ScenarioConfig c;
  c.robots = 4;
  c.tasks = 20;
  c.shape = WorkflowShape::kChains;
  const Scenario chains = dwafp::make_scenario(c, 3);
  for (Index j = 0; j < chains.mission.task_count(); ++j) {
    EXPECT_LE(chains.mission.dependency_predecessors(j).size(), 1u);
  }
  for (const WorkflowShape shape : {WorkflowShape::kForkJoin, WorkflowShape::kRendezvous}) {
    c.shape = shape;
    const Scenario s = dwafp::make_scenario(c, 3);
    bool multi_robot_join = false;
    for (Index j = 0; j < s.mission.task_count(); ++j) {
      std::set<Index> robots;
      for (const Index h : s.mission.dependency_predecessors(j)) {robots.insert(s.plan.robot_of(h));}
      multi_robot_join = multi_robot_join || (s.mission.dependency_predecessors(j).size() >= 2 && robots.size() >= 2);
    }
    EXPECT_TRUE(multi_robot_join) << dwafp::shape_name(shape);
  }
}

// The deviation mix is respected, each deviation is well formed and happens in the window:
//   delay / failure: the source's NOMINAL start is in the first 60 % of the nominal makespan, and
//                    (primary selection) the source has at least one dependency successor
//   breakdown      : the robot still has unfinished work (nominally) at the breakdown time
TEST(Scenarios, DeviationsFollowTheMixAndTheWindow)
{
  ScenarioConfig c;
  int transient = 0, persistent = 0, failure = 0, breakdown = 0;
  const int n = 400;
  for (int seed = 1; seed <= n; ++seed) {
    const Scenario s = dwafp::make_scenario(c, static_cast<std::uint64_t>(seed));
    ASSERT_EQ(s.world.deviations.size(), 1u);
    const auto & d = s.deviation;
    const auto window_end = static_cast<dwafp::TimeMs>(c.window * static_cast<double>(s.nominal_makespan_ms));
    const dwafp::RuntimeState s0 = dwafp::RuntimeState::initial(s.mission);
    const auto p = dwafp::DurationSnapshot::capture(s.mission, dwafp::ConfigDurationModel{}, s0);
    const dwafp::Prediction nominal = dwafp::Predictor(s.mission, s.plan, s0, p).full();
    if (d.type == DeviationType::kRobotBreakdown) {
      ++breakdown;
      ASSERT_LT(d.robot, s.mission.robot_count());
      EXPECT_LE(d.at_time_ms, window_end);
      bool unfinished = false;
      for (const Index j : s.plan.queue(d.robot)) {unfinished = unfinished || nominal.tasks[j].finish_ms > d.at_time_ms;}
      EXPECT_TRUE(unfinished) << "seed " << seed;
      EXPECT_EQ(s.source_class, dwafp::SourceClass::kNotApplicable);
    } else {
      ASSERT_LT(d.task, s.mission.task_count());
      EXPECT_LT(nominal.tasks[d.task].start_ms, window_end) << "seed " << seed;
      EXPECT_FALSE(s.mission.dependency_successors(d.task).empty()) << "seed " << seed;
      EXPECT_GE(d.at_fraction, 0.2);
      EXPECT_LE(d.at_fraction, 0.8);
      if (d.type == DeviationType::kOutcomeFailure) {
        ++failure;
      } else if (d.persistent) {
        ++persistent;
      } else {
        ++transient;
        // Relative to the source's own nominal duration.
        const auto own = static_cast<double>(p.at(d.task, s.plan.robot_of(d.task)));
        EXPECT_GE(d.delay_ms, std::llround(c.delay_lo * own) - 1);
        EXPECT_LE(d.delay_ms, std::llround(c.delay_hi * own) + 1);
      }
    }
  }
  // Default mix 35 / 25 / 25 / 15 %, with a generous tolerance for 400 draws.
  EXPECT_NEAR(transient / static_cast<double>(n), 0.35, 0.08);
  EXPECT_NEAR(persistent / static_cast<double>(n), 0.25, 0.08);
  EXPECT_NEAR(failure / static_cast<double>(n), 0.25, 0.08);
  EXPECT_NEAR(breakdown / static_cast<double>(n), 0.15, 0.08);
}

// An invalid configuration is refused.
TEST(Scenarios, InvalidConfigurationIsRefused)
{
  ScenarioConfig c;
  c.robots = 0;
  EXPECT_THROW(dwafp::make_scenario(c, 1), std::invalid_argument);
  c = ScenarioConfig{};
  c.mix = {0, 0, 0, 0};
  EXPECT_THROW(dwafp::make_scenario(c, 1), std::invalid_argument);
}

// Tasks without a deadline never produce a deadline event, even when the deviation makes other
// tasks miss theirs.
TEST(Scenarios, NoDeadlineTasksNeverProduceDeadlineEvents)
{
  struct Misses : dwafp::SimulationObserver
  {
    std::vector<Index> tasks;
    void on_deadline_missed(Index task, dwafp::TimeMs) override {tasks.push_back(task);}
  };
  int runs_with_misses = 0;
  for (std::uint64_t seed = 1; seed <= 60; ++seed) {
    ScenarioConfig c;
    c.shape = WorkflowShape::kChains;
    c.deadlines = DeadlineStructure::kHandoffAndFinal;
    const Scenario s = dwafp::make_scenario(c, seed);
    Simulator sim(s.mission, s.plan, s.world);
    Misses obs;
    sim.run(&obs);
    for (const Index j : obs.tasks) {EXPECT_TRUE(dwafp::has_deadline(s.mission, j)) << s.mission.task(j).id;}
    runs_with_misses += obs.tasks.empty() ? 0 : 1;
  }
  EXPECT_GT(runs_with_misses, 10);  // the property is exercised
}

// Source classes are consistent with the structure around the source, and the secondary
// selection (kAny) can also pick tasks without dependency successors.
TEST(Scenarios, SourceClassAndSelection)
{
  int any_without_dependents = 0;
  for (std::uint64_t seed = 1; seed <= 200; ++seed) {
    ScenarioConfig c;
    c.mix = {1, 0, 0, 0};  // transient delays only
    const Scenario s = dwafp::make_scenario(c, seed);
    const Index k = s.deviation.task;
    // Expected class from ALL dependency descendants (not only the direct successors).
    bool handoff = false;
    std::vector<Index> stack(s.mission.dependency_successors(k).begin(), s.mission.dependency_successors(k).end());
    std::set<Index> seen;
    while (!stack.empty()) {
      const Index j = stack.back();
      stack.pop_back();
      if (!seen.insert(j).second) {continue;}
      handoff = handoff || s.plan.robot_of(j) != s.plan.robot_of(k);
      for (const Index x : s.mission.dependency_successors(j)) {stack.push_back(x);}
    }
    const auto expected = handoff ? dwafp::SourceClass::kCrossRobotHandoff : dwafp::SourceClass::kDownstreamDependency;
    EXPECT_EQ(s.source_class, expected) << "seed " << seed;  // primary: always has dependents
    c.source = dwafp::SourceSelection::kAny;
    const Scenario any = dwafp::make_scenario(c, seed);
    if (any.mission.dependency_successors(any.deviation.task).empty()) {
      ++any_without_dependents;
      EXPECT_TRUE(any.source_class == dwafp::SourceClass::kSourceOnly ||
        any.source_class == dwafp::SourceClass::kSameRobotQueue);
    }
  }
  EXPECT_GT(any_without_dependents, 0);
}

// Homogeneous rates (scaling experiments): every robot needs 1.0 s per unit of work.
TEST(Scenarios, HomogeneousRates)
{
  ScenarioConfig c;
  c.robots = 6;
  c.homogeneous_rates = true;
  const Scenario s = dwafp::make_scenario(c, 1);
  for (Index r = 0; r < s.mission.robot_count(); ++r) {EXPECT_EQ(s.mission.robot(r).ms_per_work_unit, 1000.0);}
  EXPECT_EQ(s.world.timeout_multiplier, 2.0);  // the default execution budget reaches the world
}

// Primary source selection on sparse, small graphs: an eligible source (with a dependency
// successor) is always found by deterministic regeneration, never by falling back to any task;
// when no graph can have one (2 tasks in 2 independent chains), make_scenario throws.
TEST(Scenarios, EligibleSourceIsGuaranteedOrRefused)
{
  ScenarioConfig c;
  c.robots = 2;
  c.tasks = 3;
  c.mix = {1, 0, 0, 0};  // delays only: they need a source task
  for (std::uint64_t seed = 1; seed <= 100; ++seed) {
    const Scenario s = dwafp::make_scenario(c, seed);
    EXPECT_FALSE(s.mission.dependency_successors(s.deviation.task).empty()) << "seed " << seed;
    const Scenario again = dwafp::make_scenario(c, seed);  // regeneration is deterministic
    EXPECT_EQ(s.description, again.description);
  }
  ScenarioConfig impossible;
  impossible.shape = WorkflowShape::kChains;
  impossible.robots = 4;  // 2 chains
  impossible.tasks = 2;   // one task per chain: no dependency at all
  impossible.mix = {1, 0, 0, 0};
  EXPECT_THROW(dwafp::make_scenario(impossible, 1), std::invalid_argument);
}

// The source class looks through same-robot steps: in a chain T0@r -> T2@r -> T4@other, a
// deviation of T0 is a cross-robot handoff although its direct successor is on the same robot.
TEST(Scenarios, SourceClassUsesAllDescendants)
{
  bool found_indirect = false;
  for (std::uint64_t seed = 1; seed <= 300 && !found_indirect; ++seed) {
    ScenarioConfig c;
    c.shape = WorkflowShape::kChains;
    c.mix = {1, 0, 0, 0};
    const Scenario s = dwafp::make_scenario(c, seed);
    const Index k = s.deviation.task;
    bool direct_handoff = false;
    for (const Index j : s.mission.dependency_successors(k)) {direct_handoff = direct_handoff || s.plan.robot_of(j) != s.plan.robot_of(k);}
    if (!direct_handoff && s.source_class == dwafp::SourceClass::kCrossRobotHandoff) {found_indirect = true;}
  }
  EXPECT_TRUE(found_indirect);  // such sources exist and are classified as handoffs
}


// Final-only deadlines: exactly the tasks without a dependency successor have one.
TEST(Scenarios, FinalOnlyDeadlinesAreOnSinks)
{
  for (const WorkflowShape shape : kShapes) {
    ScenarioConfig c;
    c.shape = shape;
    c.deadlines = DeadlineStructure::kFinalOnly;
    const Scenario s = dwafp::make_scenario(c, 3);
    for (Index j = 0; j < s.mission.task_count(); ++j) {
      EXPECT_EQ(dwafp::has_deadline(s.mission, j), s.mission.dependency_successors(j).empty())
        << dwafp::shape_name(shape) << " " << s.mission.task(j).id;
    }
  }
}

// Fixed source progress and the execution-time ratio: progress_lo == progress_hi fixes the share of
// work done at the deviation; execution_time_ratio rho gives durations 1.0 / sqrt(rho) / rho s per
// unit of work, cyclically.
TEST(Scenarios, FixedProgressAndExecutionTimeRatio)
{
  ScenarioConfig c;
  c.mix = {1, 0, 0, 0};
  c.progress_lo = c.progress_hi = 0.8;
  c.execution_time_ratio = 3.0;
  for (std::uint64_t seed = 1; seed <= 20; ++seed) {
    const Scenario s = dwafp::make_scenario(c, seed);
    EXPECT_DOUBLE_EQ(s.deviation.at_fraction, 0.8);
  }
  const Scenario s = dwafp::make_scenario(c, 1);
  EXPECT_DOUBLE_EQ(s.mission.robot(0).ms_per_work_unit, 1000.0);
  EXPECT_DOUBLE_EQ(s.mission.robot(1).ms_per_work_unit, 1000.0 * std::sqrt(3.0));
  EXPECT_DOUBLE_EQ(s.mission.robot(2).ms_per_work_unit, 3000.0);
  EXPECT_DOUBLE_EQ(s.mission.robot(3).ms_per_work_unit, 1000.0);
  c.execution_time_ratio = 1.0;  // homogeneous
  const Scenario h = dwafp::make_scenario(c, 1);
  for (Index r = 0; r < h.mission.robot_count(); ++r) {EXPECT_DOUBLE_EQ(h.mission.robot(r).ms_per_work_unit, 1000.0);}
}

// A spare robot has no task and changes nothing else: the same seed gives the same tasks, robots,
// work, deadlines and deviation with or without it.
TEST(Scenarios, SpareRobotsDoNotChangeTheWorkflow)
{
  ScenarioConfig c;
  c.mix = {1, 0, 0, 0};
  ScenarioConfig with_spare = c;
  with_spare.spare_robots = 1;
  for (std::uint64_t seed = 1; seed <= 20; ++seed) {
    const Scenario a = dwafp::make_scenario(c, seed);
    const Scenario b = dwafp::make_scenario(with_spare, seed);
    ASSERT_EQ(b.mission.robot_count(), a.mission.robot_count() + 1);
    EXPECT_TRUE(b.plan.queue(a.mission.robot_count()).empty());  // the spare is idle
    ASSERT_EQ(a.mission.task_count(), b.mission.task_count());
    for (Index j = 0; j < a.mission.task_count(); ++j) {
      EXPECT_EQ(a.mission.task(j).robot, b.mission.task(j).robot);
      EXPECT_EQ(a.mission.task(j).work, b.mission.task(j).work);
      EXPECT_EQ(a.mission.task(j).deadline_ms, b.mission.task(j).deadline_ms);
      EXPECT_EQ(a.mission.task(j).predecessors, b.mission.task(j).predecessors);
    }
    EXPECT_EQ(a.deviation.task, b.deviation.task);
    EXPECT_EQ(a.deviation.delay_ms, b.deviation.delay_ms);
  }
}

// busy_fleet: at the deviation (and 2 s later) every other workflow robot executes a task in the
// nominal plan. It needs a fixed progress; an invalid rate ratio or spare count is refused.
TEST(Scenarios, BusyFleetSourceHasEveryRobotBusy)
{
  for (const WorkflowShape shape : {WorkflowShape::kRandomDag, WorkflowShape::kRendezvous}) {
    for (std::uint64_t seed = 1; seed <= 20; ++seed) {
      ScenarioConfig c;
      c.shape = shape;
      c.mix = {1, 0, 0, 0};
      c.busy_fleet = true;
      c.progress_lo = c.progress_hi = 0.5;
      const Scenario s = dwafp::make_scenario(c, seed);
      const Index src = s.deviation.task;
      const dwafp::RuntimeState s0 = dwafp::RuntimeState::initial(s.mission);
      const dwafp::DurationSnapshot p = dwafp::DurationSnapshot::capture(s.mission, dwafp::ConfigDurationModel{}, s0);
      const dwafp::Prediction nominal = dwafp::Predictor(s.mission, s.plan, s0, p).full();
      const dwafp::TimeMs b = nominal.tasks[src].start_ms + std::llround(0.5 * static_cast<double>(p.at(src, s.plan.robot_of(src))));
      for (Index r = 0; r < s.mission.robot_count(); ++r) {
        if (r == s.plan.robot_of(src)) {continue;}
        for (const dwafp::TimeMs t : {b, b + 2000}) {
          bool executing = false;
          for (const Index k : s.plan.queue(r)) {
            executing = executing || (nominal.tasks[k].start_ms <= t && t < nominal.tasks[k].finish_ms);
          }
          EXPECT_TRUE(executing) << dwafp::shape_name(shape) << " seed " << seed << " robot " << r << " at " << t;
        }
      }
    }
  }
  ScenarioConfig range;
  range.busy_fleet = true;  // progress range 0.2..0.8: refused
  EXPECT_THROW(dwafp::make_scenario(range, 1), std::invalid_argument);
  ScenarioConfig ratio;
  ratio.execution_time_ratio = 0.5;
  EXPECT_THROW(dwafp::make_scenario(ratio, 1), std::invalid_argument);
  ScenarioConfig spares;
  spares.spare_robots = -1;
  EXPECT_THROW(dwafp::make_scenario(spares, 1), std::invalid_argument);
}
