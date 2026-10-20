// Unit tests of the DWAFP deviation trigger and impact analysis (Phase C2).
//
// Same conventions as test_workflow.cpp: times in comments are nominal seconds, in code nominal
// ms; durations from config/mock_fleet.yaml (robot1 5 s, robot2 7.5 s, robot3 4 s per subtask).
// Each impact test follows the event pipeline of the paper:
//   reference prediction (last accepted, here: mission start)
//   -> actual state at the event time t (with the deviation)
//   -> expected_state(): the same state without the deviation
//   -> analyze_impact(): sources, horizon, A_i, region, response window B_i

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "actions/impact.hpp"
#include "actions/mission_loader.hpp"
#include "actions/workflow.hpp"

using dwafp::DeviationKind;
using dwafp::DeviationTrigger;
using dwafp::DurationSnapshot;
using dwafp::FeedbackSample;
using dwafp::ImpactAnalysis;
using dwafp::Index;
using dwafp::Mission;
using dwafp::Plan;
using dwafp::Prediction;
using dwafp::Predictor;
using dwafp::RuntimeState;
using dwafp::TaskStatus;
using dwafp::TimeMs;

namespace
{

const std::string kConfigDir = ACTIONS_CONFIG_DIR;

Mission load(const std::string & mission_file)
{
  return dwafp::load_mission(kConfigDir + "/missions/" + mission_file, kConfigDir + "/mock_fleet.yaml");
}

// Mission written in the test, on the same three robots as mock_fleet.yaml.
Mission inline_mission(const std::string & yaml)
{
  return dwafp::mission_from_yaml(yaml, "robots: [{name: robot1, step_duration_sec: 1.0},"
                                        " {name: robot2, step_duration_sec: 1.5},"
                                        " {name: robot3, step_duration_sec: 0.8}]");
}

DurationSnapshot durations(const Mission & m)
{
  return DurationSnapshot::capture(m, dwafp::ConfigDurationModel{}, RuntimeState::initial(m));
}

// The reference prediction: the plan as accepted at mission start (t = 0, nothing started).
Prediction reference(const Mission & m, const Plan & plan, const DurationSnapshot & p)
{
  const RuntimeState s0 = RuntimeState::initial(m);
  return Predictor(m, plan, s0, p).full();
}

void set_executing(const Mission & m, RuntimeState & s, const std::string & id, TimeMs start, TimeMs remaining)
{
  auto & t = s.tasks[m.task_index(id)];
  t.status = TaskStatus::kExecuting;
  t.start_ms = start;
  t.remaining_ms = remaining;
}

void set_failed(const Mission & m, RuntimeState & s, const std::string & id, TimeMs start, TimeMs end)
{
  auto & t = s.tasks[m.task_index(id)];
  t.status = TaskStatus::kFailed;
  t.start_ms = start;
  t.end_ms = end;
}

// Subtask indices -> names, so failures print "T1" instead of numbers.
std::vector<std::string> names(const Mission & m, const std::vector<Index> & tasks)
{
  std::vector<std::string> out;
  for (const Index j : tasks) {out.push_back(m.task(j).id);}
  return out;
}

using Names = std::vector<std::string>;

}  // namespace

// =============================================================================================
// 1. Persistence trigger (deviation from the ACCEPTED prediction, no deadline involved)
// =============================================================================================

// Chain reference: T1 runs 0-5 s. k = 3, epsilon = 0. With remaining = 5.5 s - t the robot
// always predicts 5.5 s, i.e. Delta = +0.5 s. The event fires on the THIRD consecutive deviating
// sample, only once per episode, and again after the deviation has cleared (re-armed).
TEST(DeviationTrigger, PersistentDeviationFiresOncePerEpisode)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);
  const Index t1 = m.task_index("T1");
  DeviationTrigger trigger(m, reference(m, plan, durations(m)), {3, 0, 1000000});  // tau huge

  EXPECT_FALSE(trigger.on_feedback({t1, 1000, 4500, false}));  // Delta = +0.5 s, 1st
  EXPECT_FALSE(trigger.on_feedback({t1, 2000, 3500, false}));  // 2nd
  const auto event = trigger.on_feedback({t1, 3000, 2500, false});  // 3rd -> event
  ASSERT_TRUE(event);
  EXPECT_EQ(event->kind, DeviationKind::kPersistentDeviation);
  EXPECT_EQ(event->task, t1);
  EXPECT_EQ(event->time_ms, 3000);
  EXPECT_EQ(event->deviation_ms, 500);
  EXPECT_FALSE(trigger.on_feedback({t1, 3500, 2000, false}));  // same episode: no new event

  EXPECT_FALSE(trigger.on_feedback({t1, 4000, 1000, false}));  // Delta = 0: cleared, re-armed
  EXPECT_FALSE(trigger.on_feedback({t1, 4100, 1400, false}));  // new episode, 1st
  EXPECT_FALSE(trigger.on_feedback({t1, 4200, 1300, false}));  // 2nd
  EXPECT_TRUE(trigger.on_feedback({t1, 4300, 1200, false}));   // 3rd -> second event
}

// One sample back within tolerance resets the count; a deviation within epsilon is noise.
TEST(DeviationTrigger, ShortOrSmallDeviationsDoNotFire)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);
  const Prediction ref = reference(m, plan, durations(m));
  const Index t1 = m.task_index("T1");

  DeviationTrigger strict(m, ref, {3, 0, 1000000});
  EXPECT_FALSE(strict.on_feedback({t1, 1000, 4500, false}));  // +0.5 s
  EXPECT_FALSE(strict.on_feedback({t1, 2000, 3500, false}));  // +0.5 s
  EXPECT_FALSE(strict.on_feedback({t1, 3000, 2000, false}));  // on time: count back to 0
  EXPECT_FALSE(strict.on_feedback({t1, 3500, 2000, false}));  // +0.5 s, 1st again
  EXPECT_FALSE(strict.on_feedback({t1, 3600, 1900, false}));  // 2nd

  DeviationTrigger tolerant(m, ref, {1, 1000, 1000000});  // epsilon = 1 s, k = 1
  EXPECT_FALSE(tolerant.on_feedback({t1, 1000, 4500, false}));  // +0.5 s <= 1 s: noise
  EXPECT_TRUE(tolerant.on_feedback({t1, 2000, 4500, false}));   // +1.5 s > 1 s: event
}

// THE case the deadline-based trigger missed (connecting-path mission of the impact tests):
// T1's reference finish is 5 s, the robot now predicts 9 s. T1 still meets its own 20 s deadline
// (margin +11 s), but Delta = +4 s, so the event is raised; analyze_impact() then finds that T3,
// not T1, is threatened.
TEST(DeviationTrigger, FiresWhenTheSourceStillMeetsItsOwnDeadline)
{
  const Mission m = inline_mission(
    "{mission_id: path, subtasks: ["
    " {id: T1, robot: robot1, deadline_sec: 20},"
    " {id: U, robot: robot2, deadline_sec: 40},"
    " {id: T2, robot: robot2, deadline_sec: 30, predecessors: [T1]},"
    " {id: T3, robot: robot3, deadline_sec: 20, predecessors: [T2]}]}");
  const Plan plan = Plan::initial(m);
  const DurationSnapshot p = durations(m);
  const Prediction ref = reference(m, plan, p);
  const Index t1 = m.task_index("T1");

  DeviationTrigger trigger(m, ref, {3, 0, 1000000});
  EXPECT_FALSE(trigger.on_feedback({t1, 2000, 7000, false}));  // predicts 9 s: Delta +4 s
  EXPECT_FALSE(trigger.on_feedback({t1, 3000, 6000, false}));
  const auto event = trigger.on_feedback({t1, 4000, 5000, false});
  ASSERT_TRUE(event);
  EXPECT_EQ(event->deviation_ms, 4000);

  // Event -> impact analysis at the event time.
  RuntimeState actual = RuntimeState::initial(m);
  actual.now_ms = event->time_ms;
  set_executing(m, actual, "T1", 0, 5000);
  set_executing(m, actual, "U", 0, 3500);
  const RuntimeState expected = dwafp::expected_state(m, plan, actual, ref, {t1}, {});
  const ImpactAnalysis a = dwafp::analyze_impact(m, plan, expected, actual, p);
  EXPECT_EQ(names(m, a.threatened), (Names{"T3"}));  // downstream, not the source
}

// tau = 2 s: blocked from 1 s on, the event fires at 3 s (blocked for 2 s), even though the
// predicted completion is still on time (Delta = 0).
TEST(DeviationTrigger, LongBlockageFiresEvenWithoutPredictedDelay)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);
  const Index t1 = m.task_index("T1");
  DeviationTrigger trigger(m, reference(m, plan, durations(m)), {100, 0, 2000});  // k huge

  EXPECT_FALSE(trigger.on_feedback({t1, 1000, 4000, true}));   // blocked since 1 s, Delta 0
  EXPECT_FALSE(trigger.on_feedback({t1, 2000, 3000, true}));   // blocked for 1 s
  const auto event = trigger.on_feedback({t1, 3000, 2000, true});  // blocked for 2 s -> event
  ASSERT_TRUE(event);
  EXPECT_EQ(event->kind, DeviationKind::kBlocked);
  EXPECT_EQ(event->deviation_ms, 0);
  EXPECT_FALSE(trigger.on_feedback({t1, 3500, 1500, true}));   // same episode
}

// After an accepted repair the trigger measures against the new plan, with clean histories.
TEST(DeviationTrigger, NewReferenceClearsTheHistory)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);
  Prediction ref = reference(m, plan, durations(m));
  const Index t1 = m.task_index("T1");
  DeviationTrigger trigger(m, ref, {2, 0, 1000000});
  EXPECT_FALSE(trigger.on_feedback({t1, 1000, 6000, false}));  // predicts 7 s: Delta +2 s, 1st

  ref.tasks[t1].finish_ms = 7000;  // the repair accepted a finish at 7 s
  trigger.set_reference(ref);
  EXPECT_FALSE(trigger.on_feedback({t1, 2000, 5000, false}));  // 7 s: on time for the new plan
  EXPECT_FALSE(trigger.on_feedback({t1, 3000, 4000, false}));
}

TEST(DeviationTrigger, RejectsInvalidConfigurationAndOldSamples)
{
  const Mission m = load("chain.yaml");
  const Prediction ref = reference(m, Plan::initial(m), durations(m));
  EXPECT_THROW(DeviationTrigger(m, ref, {0, 0, 1000}), std::invalid_argument);   // k >= 1
  EXPECT_THROW(DeviationTrigger(m, ref, {3, -1, 1000}), std::invalid_argument);  // epsilon >= 0
  EXPECT_THROW(DeviationTrigger(m, ref, {3, 0, -1}), std::invalid_argument);     // tau >= 0
  DeviationTrigger trigger(m, ref, {3, 0, 1000});
  (void)trigger.on_feedback({0, 2000, 1000, false});
  EXPECT_THROW(trigger.on_feedback({0, 1000, 1000, false}), std::invalid_argument);  // older
  EXPECT_THROW(trigger.on_feedback({99, 3000, 1000, false}), std::invalid_argument);  // unknown task
}

// =============================================================================================
// 2. Expected state
// =============================================================================================

// Reference: T1 ends at 5 s. At t = 4 s the actual state says 4 s left; the expected state says
// 1 s left (5 s - 4 s). Everything else is copied unchanged.
TEST(ExpectedState, RestoresTheExpectedRemainingTime)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);
  const DurationSnapshot p = durations(m);
  RuntimeState actual = RuntimeState::initial(m);
  actual.now_ms = 4000;
  set_executing(m, actual, "T1", 0, 4000);

  const RuntimeState expected =
    dwafp::expected_state(m, plan, actual, reference(m, plan, p), {m.task_index("T1")}, {});
  EXPECT_EQ(expected.tasks[m.task_index("T1")].remaining_ms, 1000);
  EXPECT_EQ(expected.tasks[m.task_index("T1")].status, TaskStatus::kExecuting);
  EXPECT_EQ(expected.now_ms, 4000);
  EXPECT_EQ(expected.tasks[m.task_index("T2")].status, TaskStatus::kUnstarted);
}

// =============================================================================================
// 3 + 4. Impact analysis, region, response window
// =============================================================================================

// Chain, T1 3 s late at t = 4 s (ends 8 s instead of 5 s):
//   A_i = {T1 (8 > 6 s), T2 (15.5 > 14 s)}, region = {T1, T2},
//   earliest violation = T1's deadline 6 s -> B_i = 6 - 4 = 2 s.
TEST(ImpactAnalysis, ChainDelayThreatensBothAndGivesTheWindow)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);
  const DurationSnapshot p = durations(m);
  RuntimeState actual = RuntimeState::initial(m);
  actual.now_ms = 4000;
  set_executing(m, actual, "T1", 0, 4000);
  const RuntimeState expected =
    dwafp::expected_state(m, plan, actual, reference(m, plan, p), {m.task_index("T1")}, {});

  const ImpactAnalysis a = dwafp::analyze_impact(m, plan, expected, actual, p);
  EXPECT_EQ(names(m, a.sources), (Names{"T1"}));
  EXPECT_EQ(names(m, a.horizon), (Names{"T1", "T2"}));
  EXPECT_EQ(names(m, a.threatened), (Names{"T1", "T2"}));
  EXPECT_TRUE(a.threats[1].deadline);     // T2: deadline went from met to missed
  EXPECT_FALSE(a.threats[1].outcome);     // its input still arrives
  EXPECT_EQ(names(m, a.region), (Names{"T1", "T2"}));
  EXPECT_EQ(a.earliest_violation_ms, 6000);
  EXPECT_EQ(a.response_window_ms, 2000);
  EXPECT_TRUE(a.needs_repair());
  EXPECT_TRUE(a.incumbent_window_open());     // 2 s left to protect the deadline
  EXPECT_FALSE(a.region_too_large);
}

// Chain, T1 0.5 s late: times change (horizon {T1, T2}) but no predicate is violated:
// continue and monitor, no region, no window.
TEST(ImpactAnalysis, AbsorbedDelayNeedsNoRepair)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);
  const DurationSnapshot p = durations(m);
  RuntimeState actual = RuntimeState::initial(m);
  actual.now_ms = 4000;
  set_executing(m, actual, "T1", 0, 1500);
  const RuntimeState expected =
    dwafp::expected_state(m, plan, actual, reference(m, plan, p), {m.task_index("T1")}, {});

  const ImpactAnalysis a = dwafp::analyze_impact(m, plan, expected, actual, p);
  EXPECT_EQ(names(m, a.horizon), (Names{"T1", "T2"}));
  EXPECT_FALSE(a.needs_repair());
  EXPECT_FALSE(a.incumbent_window_open());    // no threat: continue and monitor
  EXPECT_TRUE(a.region.empty());
  EXPECT_EQ(a.response_window_ms, dwafp::kUnreachable);
}

// Fork-join at t = 4 s (T2 on schedule, ends 7.5 s). The region never contains branch T2:
//   T1 ends 9 s : A_i = {T1}       (T3 9-13 s still meets 14 s), region = {T1}
//   T1 ends 11 s: A_i = {T1, T3}   (T3 11-15 s misses 14 s),     region = {T1, T3}
// In both cases B_i = T1's deadline 6 s - 4 s = 2 s.
TEST(ImpactAnalysis, ForkJoinRegionContainsOnlyTheAffectedBranch)
{
  const Mission m = load("fork_join.yaml");
  const Plan plan = Plan::initial(m);
  const DurationSnapshot p = durations(m);
  const Prediction ref = reference(m, plan, p);

  for (const TimeMs t1_remaining : {5000, 7000}) {
    RuntimeState actual = RuntimeState::initial(m);
    actual.now_ms = 4000;
    set_executing(m, actual, "T1", 0, t1_remaining);
    set_executing(m, actual, "T2", 0, 3500);  // as expected
    const RuntimeState expected = dwafp::expected_state(m, plan, actual, ref, {m.task_index("T1")}, {});
    const ImpactAnalysis a = dwafp::analyze_impact(m, plan, expected, actual, p);

    SCOPED_TRACE("T1 remaining " + std::to_string(t1_remaining));
    EXPECT_EQ(names(m, a.sources), (Names{"T1"}));  // T2 did not deviate
    const Names expected_set = t1_remaining == 5000 ? Names{"T1"} : Names{"T1", "T3"};
    EXPECT_EQ(names(m, a.threatened), expected_set);
    EXPECT_EQ(names(m, a.region), expected_set);    // never T2
    EXPECT_EQ(a.response_window_ms, 2000);
  }
}

// The region includes the CONNECTING path, not only the endpoints, and nothing outside it.
//   robot1: T1 (d 20 s)
//   robot2: U (d 40 s, independent) --queue--> T2 (d 30 s, needs T1)
//   robot3: T3 (d 20 s, needs T2)
// Nominal: T1 0-5, U 0-7.5, T2 7.5-15, T3 15-19. T1 ends at 9 s instead of 5 s:
//   T1 9 <= 20 ok, T2 9-16.5 <= 30 ok, T3 16.5-20.5 > 20 -> A_i = {T3} only.
//   Region = {T1 (source), T2 (on the path T1 -> T3), T3}. U is an ancestor of T2 (queue edge)
//   but not reachable from T1, so it stays out. B_i = 20 s - 4 s = 16 s.
TEST(ImpactAnalysis, RegionIncludesConnectingPathButNotOtherAncestors)
{
  const Mission m = inline_mission(
    "{mission_id: path, subtasks: ["
    " {id: T1, robot: robot1, deadline_sec: 20},"
    " {id: U, robot: robot2, deadline_sec: 40},"
    " {id: T2, robot: robot2, deadline_sec: 30, predecessors: [T1]},"
    " {id: T3, robot: robot3, deadline_sec: 20, predecessors: [T2]}]}");
  const Plan plan = Plan::initial(m);
  const DurationSnapshot p = durations(m);
  RuntimeState actual = RuntimeState::initial(m);
  actual.now_ms = 4000;
  set_executing(m, actual, "T1", 0, 5000);  // ends 9 s
  set_executing(m, actual, "U", 0, 3500);   // ends 7.5 s, as expected
  const RuntimeState expected =
    dwafp::expected_state(m, plan, actual, reference(m, plan, p), {m.task_index("T1")}, {});

  const ImpactAnalysis a = dwafp::analyze_impact(m, plan, expected, actual, p);
  EXPECT_EQ(names(m, a.threatened), (Names{"T3"}));
  EXPECT_EQ(names(m, a.region), (Names{"T1", "T2", "T3"}));
  EXPECT_EQ(a.response_window_ms, 16000);
}

// Outcome failure: T1 fails at 3 s (it was expected to end at 5 s).
//   A_i = {T2}: its outcome is newly violated (its deadline would still hold: 3-10.5 s).
//   T1 is a realized failure, not a threat, but it is a source, so it is in the region.
//   T2 (d 14 s, 7.5 s on robot2) needs its input by its latest start 6.5 s -> B_i = 3.5 s.
TEST(ImpactAnalysis, OutcomeFailureWindowEndsAtTheSuccessorsLatestStart)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);
  const DurationSnapshot p = durations(m);
  RuntimeState actual = RuntimeState::initial(m);
  actual.now_ms = 3000;
  set_failed(m, actual, "T1", 0, 3000);
  const RuntimeState expected =
    dwafp::expected_state(m, plan, actual, reference(m, plan, p), {m.task_index("T1")}, {});
  EXPECT_EQ(expected.tasks[m.task_index("T1")].remaining_ms, 2000);  // as if still running

  const ImpactAnalysis a = dwafp::analyze_impact(m, plan, expected, actual, p);
  EXPECT_EQ(names(m, a.threatened), (Names{"T2"}));
  ASSERT_EQ(a.threats.size(), 1u);
  EXPECT_TRUE(a.threats[0].outcome);
  EXPECT_FALSE(a.threats[0].deadline);
  EXPECT_EQ(names(m, a.region), (Names{"T1", "T2"}));
  EXPECT_EQ(a.earliest_violation_ms, 6500);  // 14 s - 7.5 s
  EXPECT_EQ(a.response_window_ms, 3500);
}

// Failure observed AFTER the nominal completion: T1 (reference 0-5 s) fails at 7 s.
// The expected state is rebuilt from the reference: T1 completed at 5 s, and T2 (reference 5-12.5 s),
// still unstarted only because of T1, executing since 5 s. Then:
//   T2's deadline: expected 12.5 s (met) -> now 7-14.5 s (missed)  -> NEW deadline threat
//   T2's outcome : expected ok -> missing                          -> NEW outcome threat
// (With "T1 executing, 0 s left", T2 would start at 7 s in the expected state too, its deadline
// would look already missed, and the deadline threat caused by T1 would be lost.)
// T2 needed its input by its latest start 6.5 s (14 - 7.5), already past at 7 s: B_i^0 = 0. Under
// the current assignment (T2 on robot2, 7.5 s) T2's deadline can no longer be met; only a repair
// that changes the assignment could still save it.
TEST(ImpactAnalysis, FailureAfterNominalCompletionKeepsTheFullImpact)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);
  const DurationSnapshot p = durations(m);
  RuntimeState actual = RuntimeState::initial(m);
  actual.now_ms = 7000;
  set_failed(m, actual, "T1", 0, 7000);
  const RuntimeState expected =
    dwafp::expected_state(m, plan, actual, reference(m, plan, p), {m.task_index("T1")}, {});

  const auto & t1 = expected.tasks[m.task_index("T1")];
  EXPECT_EQ(t1.status, TaskStatus::kCompleted);  // done at its reference finish...
  EXPECT_EQ(t1.end_ms, 5000);                    // ...not at the observation time 7 s
  const auto & t2 = expected.tasks[m.task_index("T2")];
  EXPECT_EQ(t2.status, TaskStatus::kExecuting);  // overdue descendant, rebuilt from the reference
  EXPECT_EQ(t2.start_ms, 5000);
  EXPECT_EQ(t2.remaining_ms, 5500);              // 12.5 s - 7 s

  const ImpactAnalysis a = dwafp::analyze_impact(m, plan, expected, actual, p);
  EXPECT_EQ(names(m, a.threatened), (Names{"T2"}));
  ASSERT_EQ(a.threats.size(), 1u);
  EXPECT_TRUE(a.threats[0].deadline);
  EXPECT_TRUE(a.threats[0].outcome);
  EXPECT_EQ(a.earliest_violation_ms, 6500);
  EXPECT_EQ(a.response_window_ms, 0);
  EXPECT_TRUE(a.needs_repair());
  EXPECT_FALSE(a.incumbent_window_open());    // not protectable on the current assignment
}

// Delay observed late: at 7 s T1 is still running with 2 s left (ends 9 s; reference 5 s).
// T1 itself is newly threatened (expected: completed at 5 s <= 6 s; now 9 s > 6 s), and so is
// T2 (expected 5-12.5 s; now 9-16.5 s > 14 s).
TEST(ImpactAnalysis, DelayObservedAfterNominalCompletionThreatensSourceAndSuccessor)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);
  const DurationSnapshot p = durations(m);
  RuntimeState actual = RuntimeState::initial(m);
  actual.now_ms = 7000;
  set_executing(m, actual, "T1", 0, 2000);
  const RuntimeState expected =
    dwafp::expected_state(m, plan, actual, reference(m, plan, p), {m.task_index("T1")}, {});
  const ImpactAnalysis a = dwafp::analyze_impact(m, plan, expected, actual, p);
  EXPECT_EQ(names(m, a.threatened), (Names{"T1", "T2"}));
  EXPECT_EQ(names(m, a.region), (Names{"T1", "T2"}));
}

// Robot unavailability: robot1 (queue A, B) breaks down at 2 s. robot2 runs D then C (C needs B).
//   Sources = {A, B}: their robot's entry changed (found automatically).
//   A (executing): output and assignment lost -> threat, but already realized: no window.
//   B (unstarted): unreachable -> deadline, output, assignment; latest start 11 - 5 = 6 s.
//   C: waits for B -> deadline and output; latest start 20 - 7.5 = 12.5 s.
//   D: other robot, independent -> unaffected, not in the region.
//   B_i = 6 s - 2 s = 4 s.
TEST(ImpactAnalysis, RobotUnavailabilityThreatensItsQueueAndDependents)
{
  const Mission m = inline_mission(
    "{mission_id: queue, subtasks: ["
    " {id: A, robot: robot1, deadline_sec: 6},"
    " {id: D, robot: robot2, deadline_sec: 8},"
    " {id: B, robot: robot1, deadline_sec: 11},"
    " {id: C, robot: robot2, deadline_sec: 20, predecessors: [B]}]}");
  const Plan plan = Plan::initial(m);
  const DurationSnapshot p = durations(m);
  RuntimeState actual = RuntimeState::initial(m);
  actual.now_ms = 2000;
  set_executing(m, actual, "A", 0, 3000);
  set_executing(m, actual, "D", 0, 5500);
  actual.robots[m.robot_index("robot1")].available = false;
  actual.robots[m.robot_index("robot1")].unavailable_since_ms = 2000;
  const RuntimeState expected =
    dwafp::expected_state(m, plan, actual, reference(m, plan, p), {}, {m.robot_index("robot1")});

  const ImpactAnalysis a = dwafp::analyze_impact(m, plan, expected, actual, p);
  EXPECT_EQ(names(m, a.sources), (Names{"A", "B"}));
  EXPECT_EQ(names(m, a.threatened), (Names{"A", "B", "C"}));
  EXPECT_EQ(a.threats[0].violation_ms, dwafp::kUnreachable);  // A: already realized
  EXPECT_EQ(a.threats[1].violation_ms, 6000);                 // B: latest start on robot1
  EXPECT_EQ(a.threats[2].violation_ms, 12500);                // C: latest start on robot2
  EXPECT_EQ(names(m, a.region), (Names{"A", "B", "C"}));
  EXPECT_EQ(a.response_window_ms, 4000);
}

// With max_region_size = 1, the chain delay (region of 2 subtasks) is flagged as not local.
TEST(ImpactAnalysis, LargeRegionIsFlaggedForEscalation)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);
  const DurationSnapshot p = durations(m);
  RuntimeState actual = RuntimeState::initial(m);
  actual.now_ms = 4000;
  set_executing(m, actual, "T1", 0, 4000);
  const RuntimeState expected =
    dwafp::expected_state(m, plan, actual, reference(m, plan, p), {m.task_index("T1")}, {});
  dwafp::ImpactConfig config;
  config.max_region_size = 1;
  EXPECT_TRUE(dwafp::analyze_impact(m, plan, expected, actual, p, config).region_too_large);
}

// The two states must describe the same instant.
TEST(ImpactAnalysis, RejectsStatesAtDifferentTimes)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);
  const DurationSnapshot p = durations(m);
  RuntimeState a = RuntimeState::initial(m);
  RuntimeState b = a;
  b.now_ms = 1000;
  EXPECT_THROW(dwafp::analyze_impact(m, plan, a, b, p), std::invalid_argument);
}
