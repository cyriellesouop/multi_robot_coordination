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

// A PERSISTENT obstruction of robot1 on T1, at 40 % of its work (2 s in). Every attempt has an
// execution budget of 2 x its nominal duration (the robot then reports a timeout failure).
WorldConfig persistent_blockage(const Mission & m)
{
  WorldConfig w;
  w.horizon_ms = 120000;
  w.timeout_multiplier = 2.0;
  w.deviations.push_back({DeviationType::kDelay, m.task_index("T1"), dwafp::kNoIndex, 0.4, 0, 0, true});
  return w;
}

}  // namespace

// Chain T1 (d 30 s) --> T2 (d 16 s); robot1 is persistently obstructed on T1 from 2 s.
//   no intervention : robot1 gives up T1 at its execution timeout (10 s = 2 x 5 s): T1 failed,
//                     T2 never starts.
//   reactive        : acts on the TIMEOUT result at 10 s (not on the 30 s deadline): T1 -> robot3
//                     10-14 s (retry on robot1 is excluded: persistent), then T2 14-21.5 s on
//                     robot2. When T2 misses its deadline at 16 s, the recovery acts again: robot3
//                     is idle and faster, so T2 restarts there 16-20 s (tardiness 4 s < 5.5 s).
//                     T1 on time, T2 LATE.
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
  EXPECT_EQ(none.records[t1].status, FinalStatus::kFailed);
  EXPECT_EQ(none.records[t1].timeouts, 1);
  EXPECT_EQ(none.records[t2].status, FinalStatus::kNeverStarted);

  const Outcome reactive = run(m, world, Method::kReactive);
  EXPECT_EQ(reactive.records[t1].status, FinalStatus::kCompleted);
  EXPECT_EQ(reactive.records[t1].robot, m.robot_index("robot3"));
  EXPECT_EQ(reactive.records[t1].start_ms, 10000);
  EXPECT_EQ(reactive.records[t1].end_ms, 14000);
  EXPECT_EQ(reactive.records[t2].robot, m.robot_index("robot3"));
  EXPECT_EQ(reactive.records[t2].end_ms, 20000);
  EXPECT_TRUE(reactive.records[t1].deadline_ok);
  EXPECT_FALSE(reactive.records[t2].deadline_ok);
  EXPECT_EQ(reactive.stats.fallback_repairs, 2);  // at T1's timeout and at T2's deadline miss

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

// Timeout -> the SAME recovery for every repair-enabled method. With prevention disabled (no
// trigger can fire), the persistent obstruction runs into the execution timeout at 10 s; the
// timeout is reported like any failure and reactive, source-only, DWAFP and mission-wide all
// apply the identical recovery: T1 -> robot3 10-14 s, then the same later edits.
TEST(Controllers, TimeoutGoesToTheIdenticalRecoveryForEveryMethod)
{
  const Mission m = chain(30, 16);
  const Plan plan = Plan::initial(m);
  std::vector<std::vector<TaskRecord>> results;
  for (const Method method : {Method::kReactive, Method::kSourceOnly, Method::kDwafp, Method::kMissionWide}) {
    ControllerConfig config;
    config.method = method;
    config.trigger.persistence_updates = 1000000;    // no trigger event: no prevention
    config.trigger.blocked_threshold_ms = 10000000;
    MethodController controller(m, plan, config);
    Simulator sim(m, plan, persistent_blockage(m));
    results.push_back(sim.run(&controller));
    SCOPED_TRACE(dwafp::method_name(method));
    EXPECT_EQ(controller.stats().preventive_repairs, 0);
    EXPECT_EQ(controller.stats().trigger_events, 0);
    EXPECT_EQ(results.back()[m.task_index("T1")].timeouts, 1);
    EXPECT_EQ(results.back()[m.task_index("T1")].start_ms, 10000);  // recovered at the timeout
  }
  for (std::size_t i = 1; i < results.size(); ++i) {
    for (Index j = 0; j < m.task_count(); ++j) {
      EXPECT_EQ(results[i][j].start_ms, results[0][j].start_ms);
      EXPECT_EQ(results[i][j].end_ms, results[0][j].end_ms);
      EXPECT_EQ(results[i][j].robot, results[0][j].robot);
    }
  }
}


// Downstream repair (repair-location study). T1 (robot1, no real deadline) -> T2 (robot2, d 13 s).
// T1 is held up for 3 s at 80 % of its work (4 s in, transient), so it really ends at 8 s and T2
// on robot2 (7.5 s) would end at 15.5 s: late.
//   source-only: when T1's trigger fires (about 5.5 s), no edit of T1 helps (restart on robot3
//   ends >= 9.5 s, then T2 on robot2 >= 17 s; a retry is later still). T2 starts late on robot2
//   at 8 s; once T2's own feedback deviates, source-only may edit T2 (now a deviating task), but
//   restarting it elsewhere ends at >= 13.5 s: too late. T2 misses its deadline.
//   DWAFP: at T1's trigger, T2 is downstream and unstarted: it moves T2 to robot1, right after T1
//   (5 s there, the first preventive candidate in generation order): T1 continues (no work thrown
//   away) and ends at 8 s, T2 runs 8-13 s on robot1, on time. It is counted as a downstream edit.
// Mission-wide makes the same choice (T1 first in its list, but no edit of T1 is preventive).
TEST(Controllers, DwafpRepairsDownstreamWhereSourceOnlyCannot)
{
  const Mission m = inline_mission(
    "{mission_id: downstream, subtasks: [{id: T1, robot: robot1, deadline_sec: 60},"
    " {id: T2, robot: robot2, deadline_sec: 13, predecessors: [T1]}]}");
  const Index t1 = m.task_index("T1");
  const Index t2 = m.task_index("T2");
  WorldConfig world;
  world.horizon_ms = 120000;
  world.timeout_multiplier = 2.0;
  world.deviations.push_back({DeviationType::kDelay, t1, dwafp::kNoIndex, 0.8, 0, 3000, false});

  const Outcome source_only = run(m, world, Method::kSourceOnly);
  EXPECT_EQ(source_only.stats.preventive_repairs, 0);    // no edit of a deviating task protects T2
  EXPECT_EQ(source_only.stats.preventive_downstream, 0);
  EXPECT_EQ(source_only.records[t1].end_ms, 8000);
  EXPECT_FALSE(source_only.records[t2].deadline_ok);

  for (const Method method : {Method::kDwafp, Method::kMissionWide}) {
    const Outcome o = run(m, world, method);
    SCOPED_TRACE(dwafp::method_name(method));
    EXPECT_EQ(o.stats.preventive_repairs, 1);
    EXPECT_EQ(o.stats.preventive_downstream, 1);         // the edit moved T2, not the source
    EXPECT_EQ(o.records[t1].robot, m.robot_index("robot1"));
    EXPECT_EQ(o.records[t1].end_ms, 8000);
    EXPECT_EQ(o.records[t1].discarded_work_ms, 0);        // the source's progress is kept
    EXPECT_EQ(o.records[t2].robot, m.robot_index("robot1"));
    EXPECT_EQ(o.records[t2].start_ms, 8000);
    EXPECT_EQ(o.records[t2].end_ms, 13000);
    EXPECT_TRUE(o.records[t2].deadline_ok);
  }
}
