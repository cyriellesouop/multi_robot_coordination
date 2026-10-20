// Unit tests of the compared methods as simulator controllers (Phase C6, Step 3).
//
// Each test runs the SAME world (mission, deviation, noise) under several methods and compares
// what really happened. Durations: 5 s per task on robot1, 7.5 s on robot2, 4 s on robot3.
// Trigger: k = 3 consecutive late samples, epsilon = 0, tau = 2 s; feedback every 0.5 s.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "actions/controllers.hpp"
#include "actions/mission_loader.hpp"
#include "actions/simulator.hpp"

using dwafp::ControllerConfig;
using dwafp::DeviationType;
using dwafp::FinalStatus;
using dwafp::Index;
using dwafp::Method;
using dwafp::MethodController;
using dwafp::Mission;
using dwafp::Plan;
using dwafp::Simulator;
using dwafp::TaskRecord;
using dwafp::TimeMs;
using dwafp::WorldConfig;

namespace
{

Mission inline_mission(const std::string & yaml)
{
  return dwafp::mission_from_yaml(yaml, "robots: [{name: robot1, step_duration_sec: 1.0},"
                                        " {name: robot2, step_duration_sec: 1.5},"
                                        " {name: robot3, step_duration_sec: 0.8}]");
}

// T1 (robot1) --> T2 (robot2), deadlines in seconds.
Mission chain(int d1, int d2)
{
  return inline_mission(
    "{mission_id: chain, subtasks: [{id: T1, robot: robot1, deadline_sec: " + std::to_string(d1) + "},"
    " {id: T2, robot: robot2, deadline_sec: " + std::to_string(d2) + ", predecessors: [T1]}]}");
}

struct Outcome
{
  std::vector<TaskRecord> records;
  dwafp::ControllerStats stats;
  dwafp::EditCounts edits;
};

// Runs one method on one world.
Outcome run(const Mission & m, const WorldConfig & world, Method method, TimeMs latency_ms = 0)
{
  const Plan plan = Plan::initial(m);
  ControllerConfig config;
  config.method = method;
  config.dispatch_ms = latency_ms;
  MethodController controller(m, plan, config);
  Simulator sim(m, plan, world);
  Outcome o;
  o.records = sim.run(&controller);
  o.stats = controller.stats();
  o.edits = sim.edit_counts();
  return o;
}

// A PERSISTENT obstruction of robot1 on T1, at 40 % of its work (2 s in).
WorldConfig persistent_blockage(const Mission & m)
{
  WorldConfig w;
  w.horizon_ms = 120000;
  w.deviations.push_back({DeviationType::kDelay, m.task_index("T1"), dwafp::kNoIndex, 0.4, 0, 0, true});
  return w;
}

}  // namespace

// Chain T1 (d 30 s) --> T2 (d 16 s); robot1 is persistently obstructed on T1 from 2 s.
//   no intervention : T1 never ends, T2 never starts.
//   reactive        : acts only when T1 misses its deadline (30 s): T1 -> robot3, 30-34 s, then
//                     T2 34-41.5 s: both complete, both LATE. (At T2's own miss, 16 s, moving T2
//                     cannot help: its input is missing.)
//   DWAFP           : the trigger fires at 3.5 s (3 late samples after the blockage). With the
//                     persistent diagnostic, T1 cannot end on robot1: the only preventive single
//                     edit is T1 -> robot3, 3.5-7.5 s, then T2 7.5-15 s: BOTH ON TIME.
TEST(Controllers, DwafpPreventsWhatReactiveRepairsLate)
{
  const Mission m = chain(30, 16);
  const Index t1 = m.task_index("T1");
  const Index t2 = m.task_index("T2");
  const WorldConfig world = persistent_blockage(m);

  const Outcome none = run(m, world, Method::kNoIntervention);
  EXPECT_EQ(none.records[t1].status, FinalStatus::kIncomplete);
  EXPECT_EQ(none.records[t2].status, FinalStatus::kNeverStarted);

  const Outcome reactive = run(m, world, Method::kReactive);
  EXPECT_EQ(reactive.records[t1].status, FinalStatus::kCompleted);
  EXPECT_EQ(reactive.records[t1].robot, m.robot_index("robot3"));
  EXPECT_EQ(reactive.records[t1].end_ms, 34000);
  EXPECT_EQ(reactive.records[t2].end_ms, 41500);
  EXPECT_FALSE(reactive.records[t1].deadline_ok);
  EXPECT_FALSE(reactive.records[t2].deadline_ok);
  EXPECT_EQ(reactive.stats.fallback_repairs, 1);

  const Outcome dwafp = run(m, world, Method::kDwafp);
  EXPECT_EQ(dwafp.stats.trigger_events, 1);
  EXPECT_EQ(dwafp.stats.preventive_repairs, 1);
  EXPECT_EQ(dwafp.records[t1].robot, m.robot_index("robot3"));
  EXPECT_EQ(dwafp.records[t1].start_ms, 3500);
  EXPECT_EQ(dwafp.records[t1].end_ms, 7500);
  EXPECT_EQ(dwafp.records[t2].end_ms, 15000);
  EXPECT_TRUE(dwafp.records[t1].deadline_ok);
  EXPECT_TRUE(dwafp.records[t2].deadline_ok);
  EXPECT_EQ(dwafp.edits.installed, 1);
  EXPECT_EQ(dwafp.stats.errors, 0);
}

// Same world, 1 s of dispatch latency: DWAFP decides at 3.5 s, the move is activated at 4.5 s.
// T1 runs 4.5-8.5 s on robot3 and T2 8.5-16 s: still on time (exactly at its deadline).
TEST(Controllers, DwafpWithDispatchLatency)
{
  const Mission m = chain(30, 16);
  const Outcome dwafp = run(m, persistent_blockage(m), Method::kDwafp, 1000);
  EXPECT_EQ(dwafp.records[m.task_index("T1")].start_ms, 4500);
  EXPECT_EQ(dwafp.records[m.task_index("T2")].end_ms, 16000);
  EXPECT_TRUE(dwafp.records[m.task_index("T2")].deadline_ok);
  EXPECT_EQ(dwafp.edits.scheduled, 1);
  EXPECT_EQ(dwafp.edits.installed, 1);
}

// The source here IS the task to move, so source-only and mission-wide decide like DWAFP.
TEST(Controllers, SourceOnlyAndGlobalAgreeWhenTheSourceMustMove)
{
  const Mission m = chain(30, 16);
  const WorldConfig world = persistent_blockage(m);
  for (const Method method : {Method::kSourceOnly, Method::kMissionWide}) {
    const Outcome o = run(m, world, method);
    SCOPED_TRACE(dwafp::method_name(method));
    EXPECT_EQ(o.records[m.task_index("T1")].end_ms, 7500);
    EXPECT_EQ(o.records[m.task_index("T2")].end_ms, 15000);
  }
}

// Outcome failure: T1 (d 15 s) fails at 3 s; T2 (d 25 s) needs its output. A failure is an
// ACTUAL event: every repair-enabled method sends it to the SAME recovery (no preventive search),
// so reactive and DWAFP do exactly the same: T1 -> robot3 3-7 s (cheaper than a retry 3-8 s),
// T2 7-14.5 s. Without intervention T2 never starts.
TEST(Controllers, OutcomeFailureIsRecoveredByReactiveAndDwafpAlike)
{
  const Mission m = chain(15, 25);
  const Index t1 = m.task_index("T1");
  const Index t2 = m.task_index("T2");
  WorldConfig world;
  world.deviations.push_back({DeviationType::kOutcomeFailure, t1, dwafp::kNoIndex, 0.6, 0, 0, false});

  EXPECT_EQ(run(m, world, Method::kNoIntervention).records[t2].status, FinalStatus::kNeverStarted);
  for (const Method method : {Method::kReactive, Method::kDwafp}) {
    const Outcome o = run(m, world, method);
    SCOPED_TRACE(dwafp::method_name(method));
    EXPECT_EQ(o.records[t1].status, FinalStatus::kCompleted);
    EXPECT_EQ(o.records[t1].robot, m.robot_index("robot3"));
    EXPECT_EQ(o.records[t1].end_ms, 7000);
    EXPECT_EQ(o.records[t2].end_ms, 14500);
    EXPECT_TRUE(o.records[t2].deadline_ok);
    EXPECT_EQ(o.stats.analyses, 0);          // no preventive search for an actual failure
    EXPECT_EQ(o.stats.fallback_repairs, 1);
  }
}

// Robot breakdown: robot1 (A running, then B) breaks down at 2 s; robot2 runs D then C (C needs
// B). The common recovery handles the failed A and the stranded B, one edit each, identically for
// reactive and DWAFP:
//   A -> robot3 at 2 s (on time if alone); then B -> robot3 too, at the front of its unstarted
//   tasks: B 2-6 s, A 6-10 s (A late, but nothing is lost), C 7.5-15 s after D, D 0-7.5 s.
TEST(Controllers, BreakdownStrandedTasksGoToTheCommonRecovery)
{
  const Mission m = inline_mission(
    "{mission_id: queue, subtasks: ["
    " {id: A, robot: robot1, deadline_sec: 6},"
    " {id: D, robot: robot2, deadline_sec: 8},"
    " {id: B, robot: robot1, deadline_sec: 11},"
    " {id: C, robot: robot2, deadline_sec: 20, predecessors: [B]}]}");
  WorldConfig world;
  dwafp::InjectedDeviation breakdown;
  breakdown.type = DeviationType::kRobotBreakdown;
  breakdown.robot = m.robot_index("robot1");
  breakdown.at_time_ms = 2000;
  world.deviations.push_back(breakdown);

  const Outcome none = run(m, world, Method::kNoIntervention);
  EXPECT_EQ(none.records[m.task_index("C")].status, FinalStatus::kNeverStarted);

  const Outcome reactive = run(m, world, Method::kReactive);
  const Outcome dwafp = run(m, world, Method::kDwafp);
  for (const Outcome * o : {&reactive, &dwafp}) {
    EXPECT_EQ(o->stats.fallback_repairs, 2);
    EXPECT_EQ(o->records[m.task_index("B")].start_ms, 2000);
    EXPECT_EQ(o->records[m.task_index("B")].end_ms, 6000);
    EXPECT_EQ(o->records[m.task_index("A")].end_ms, 10000);
    EXPECT_EQ(o->records[m.task_index("C")].end_ms, 15000);
    for (Index j = 0; j < m.task_count(); ++j) {
      EXPECT_EQ(o->records[j].status, FinalStatus::kCompleted) << m.task(j).id;
    }
  }
}

// The estimator is a configuration shared by every method. Same persistent world as above: the
// robot's explicit persistent diagnosis makes T1 unreachable on robot1 in BOTH configurations,
// so DWAFP takes the same decision (T1 -> robot3 at 3.5 s) with the conservative estimator.
TEST(Controllers, PersistentDiagnosisIsUsedByBothEstimators)
{
  const Mission m = chain(30, 16);
  const Plan plan = Plan::initial(m);
  ControllerConfig config;
  config.method = Method::kDwafp;
  config.estimate = dwafp::BlockageEstimate::kConservative;
  MethodController controller(m, plan, config);
  Simulator sim(m, plan, persistent_blockage(m));
  const auto & rec = sim.run(&controller);
  EXPECT_EQ(rec[m.task_index("T1")].start_ms, 3500);
  EXPECT_EQ(rec[m.task_index("T2")].end_ms, 15000);
}

// A deviation absorbed by slack: T1 is blocked 1 s (transient) and T2 has plenty of slack.
// DWAFP is triggered, finds nothing threatened and does not touch the plan.
TEST(Controllers, AbsorbedDeviationTriggersNoRepair)
{
  const Mission m = chain(30, 40);
  WorldConfig world;
  world.deviations.push_back({DeviationType::kDelay, m.task_index("T1"), dwafp::kNoIndex, 0.4, 0, 1000, false});
  const Outcome o = run(m, world, Method::kDwafp);
  EXPECT_EQ(o.edits.scheduled, 0);
  EXPECT_EQ(o.stats.preventive_repairs, 0);
  EXPECT_EQ(o.records[m.task_index("T1")].end_ms, 6000);
  EXPECT_EQ(o.records[m.task_index("T2")].end_ms, 13500);
}
