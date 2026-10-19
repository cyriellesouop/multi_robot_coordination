// Unit tests of the DWAFP workflow model (Phase C1).
//
// Times in the comments are nominal seconds, in the code nominal ms. Durations come from
// config/mock_fleet.yaml: robot1 1.0 s/unit, robot2 1.5 s/unit, robot3 0.8 s/unit, 5 units/task.
// So one subtask takes 5 s on robot1, 7.5 s on robot2, 4 s on robot3.
//
// Every scenario that changes the state also checks the key property of the paper:
// incremental propagation gives EXACTLY the same prediction as full propagation, and its
// validation horizon is exactly the set of subtasks whose prediction changed.
//
// ---------------------------------------------------------------------------------------------
// How to read a gtest test (GoogleTest, the C++ test framework used by ROS 2)
// ---------------------------------------------------------------------------------------------
//   TEST(SuiteName, TestName) { ... }   one test. Tests with the same SuiteName form a group.
//                                       No main() is needed: gtest provides it and runs them all.
//   EXPECT_EQ(a, b)     checks a == b. If not, the test is marked FAILED but continues,
//                       so one run reports every wrong value.
//   ASSERT_EQ(a, b)     same check, but stops the test at once if it fails. Used when the rest of
//                       the test would make no sense (e.g. wrong number of subtasks).
//   EXPECT_TRUE(x) / EXPECT_FALSE(x)      checks that x is true / false.
//   EXPECT_THROW(code, Type)              checks that `code` throws an exception of type Type
//                                         (here: that an invalid input is rejected).
//   EXPECT_NO_THROW(code)                 checks that `code` does NOT throw.
//   EXPECT_EQ(...) << "text"              adds "text" to the failure message.
//
// Every test follows the same three steps:
//   1. ARRANGE : load a mission, build the plan, set the runtime state (who runs what, at time t).
//   2. ACT     : run the predictor (full and / or incremental).
//   3. ASSERT  : compare the predicted times / outcomes with values computed by hand.
// ---------------------------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>   // std::sort, std::find
#include <random>      // std::mt19937 (random generator for the randomized test)
#include <stdexcept>   // std::invalid_argument
#include <string>
#include <vector>

#include "actions/mission_loader.hpp"
#include "actions/workflow.hpp"

// "using" lets the tests write Mission instead of dwafp::Mission everywhere.
using dwafp::DurationSnapshot;
using dwafp::Index;
using dwafp::Mission;
using dwafp::Plan;
using dwafp::Prediction;
using dwafp::Predictor;
using dwafp::RuntimeState;
using dwafp::TaskStatus;

// =============================================================================================
// Helpers used by many tests (unnamed namespace = visible only in this file)
// =============================================================================================
namespace
{

// ACTIONS_CONFIG_DIR is set by CMakeLists.txt to the package's config/ folder, so the tests
// check the very files the fleet launch and the simulator use.
const std::string kConfigDir = ACTIONS_CONFIG_DIR;
const std::string kFleet = kConfigDir + "/mock_fleet.yaml";

// Loads config/missions/<mission_file> with the real fleet. Example: load("chain.yaml").
Mission load(const std::string & mission_file)
{
  return dwafp::load_mission(kConfigDir + "/missions/" + mission_file, kFleet);
}

// Builds a mission from YAML text written directly in the test, with the same 3 robots and
// speeds as mock_fleet.yaml. Used for small situations that need no file of their own.
// YAML "flow style" {a: 1, b: [x, y]} is the one-line form of the usual indented YAML.
Mission inline_mission(const std::string & yaml)
{
  return dwafp::mission_from_yaml(yaml, "robots: [{name: robot1, step_duration_sec: 1.0},"
                                        " {name: robot2, step_duration_sec: 1.5},"
                                        " {name: robot3, step_duration_sec: 0.8}]");
}

// Duration table p_jr of the mission, from the configuration-based model (work * robot rate).
DurationSnapshot durations(const Mission & mission, const RuntimeState & state)
{
  return DurationSnapshot::capture(mission, dwafp::ConfigDurationModel{}, state);
}

// Puts subtask `id` in the state "executing since start_ms, the robot says remaining_ms left".
// This is how a test simulates robot feedback: a larger remaining_ms = a delay.
// `RuntimeState & s` (no const) because the function modifies the state passed to it.
void set_executing(const Mission & m, RuntimeState & s, const std::string & id, dwafp::TimeMs start_ms,
  dwafp::TimeMs remaining_ms)
{
  auto & task = s.tasks[m.task_index(id)];
  task.status = TaskStatus::kExecuting;
  task.start_ms = start_ms;
  task.remaining_ms = remaining_ms;
}

// Puts subtask `id` in a finished state: kCompleted (output produced) or kFailed (no output),
// between start_ms and end_ms.
void set_finished(const Mission & m, RuntimeState & s, const std::string & id, TaskStatus status,
  dwafp::TimeMs start_ms, dwafp::TimeMs end_ms)
{
  auto & task = s.tasks[m.task_index(id)];
  task.status = status;
  task.start_ms = start_ms;
  task.end_ms = end_ms;
}

// Converts a list of subtask indices into their names, e.g. {0, 2} -> {"T1", "T3"}, so a failed
// check prints readable ids instead of numbers.
std::vector<std::string> names(const Mission & m, const std::vector<Index> & tasks)
{
  std::vector<std::string> out;
  for (const Index j : tasks) {out.push_back(m.task(j).id);}
  return out;
}

// THE central check of this file, used by every scenario with a change.
//   baseline  : prediction of the OLD state (before the event), at the same time t
//   new_state : the state after the event (e.g. T1 now reports more time left)
//   changed   : the subtasks the event touched directly (e.g. {T1})
// It runs the incremental propagation from the baseline and the full propagation of the new state,
// checks that
//   1) both give the same prediction for every subtask, and
//   2) the horizon returned by incremental() is exactly the set of subtasks whose prediction
//      differs from the baseline (nothing missed, nothing extra),
// and returns the horizon so the test can also check its content.
// `Prediction baseline` is taken BY VALUE (a copy), because incremental() modifies it.
std::vector<Index> incremental_matches_full(
  const Mission & m, const Plan & plan, const RuntimeState & new_state, const DurationSnapshot & p,
  Prediction baseline, const std::vector<Index> & changed)
{
  const Predictor predictor(m, plan, new_state, p);
  const Prediction old_prediction = baseline;  // keep the "before" values for check 2
  const std::vector<Index> horizon = predictor.incremental(baseline, changed);  // baseline is now updated
  const Prediction full = predictor.full();

  // 1) Same prediction for every subtask.
  for (Index j = 0; j < m.task_count(); ++j) {
    EXPECT_EQ(baseline.tasks[j], full.tasks[j]) << "subtask " << m.task(j).id;
  }
  // 2) The horizon is exactly the set of subtasks whose prediction changed.
  std::vector<Index> really_changed;
  for (Index j = 0; j < m.task_count(); ++j) {
    if (old_prediction.tasks[j] != full.tasks[j]) {really_changed.push_back(j);}
  }
  std::vector<Index> sorted_horizon = horizon;  // horizon is in topological order; sort to compare
  std::sort(sorted_horizon.begin(), sorted_horizon.end());
  EXPECT_EQ(sorted_horizon, really_changed);
  return horizon;
}

}  // namespace

// gtest prints a TaskPrediction when EXPECT_EQ fails (otherwise it prints raw bytes).
// Example of a failure message: {start 9000, finish 13000, outcome ok, deadline ok}.
namespace dwafp
{
void PrintTo(const TaskPrediction & p, std::ostream * os)
{
  *os << "{start " << p.start_ms << ", finish " << p.finish_ms << ", outcome "
      << (p.outcome_ok ? "ok" : "MISSING") << ", deadline " << (p.deadline_ok ? "ok" : "MISSED") << "}";
}
}  // namespace dwafp

// =============================================================================================
// Loading and validation: wrong inputs must be rejected BEFORE any prediction is made
// =============================================================================================

// Loads chain.yaml and checks that the file content and the fleet durations arrive correctly.
TEST(MissionLoading, ChainFileIsLoadedWithFleetDurations)
{
  const Mission m = load("chain.yaml");
  ASSERT_EQ(m.task_count(), 2u);   // T1, T2   ("2u" = unsigned 2, same type as task_count())
  ASSERT_EQ(m.robot_count(), 3u);  // robot1, robot2, robot3 from mock_fleet.yaml
  const Index t1 = m.task_index("T1");
  const Index t2 = m.task_index("T2");
  EXPECT_EQ(m.task(t1).deadline_ms, 6000);                             // deadline_sec: 6.0 -> 6000 ms
  EXPECT_EQ(m.dependency_predecessors(t2), std::vector<Index>{t1});    // T2 waits for T1
  EXPECT_EQ(m.task(t2).target.x, 4.0);  // opaque pose is kept as given

  // p_jr for EVERY pair, not only the assignee: T1 on robot1 / robot2 / robot3.
  // (5 units * 1000 / 1500 / 800 ms per unit)
  const RuntimeState s = RuntimeState::initial(m);
  const DurationSnapshot p = durations(m, s);
  EXPECT_EQ(p.at(t1, m.robot_index("robot1")), 5000);
  EXPECT_EQ(p.at(t1, m.robot_index("robot2")), 7500);
  EXPECT_EQ(p.at(t1, m.robot_index("robot3")), 4000);
}

// Nine small missions, each with exactly ONE mistake. Each one must be refused with
// std::invalid_argument; if one is accepted, the failure message prints that mission.
TEST(MissionLoading, InvalidMissionsAreRejected)
{
  // Each mission below has exactly one mistake.
  const std::vector<std::string> bad = {
    // unknown robot
    "{mission_id: m, subtasks: [{id: T1, robot: robot9, deadline_sec: 5}]}",
    // unknown predecessor
    "{mission_id: m, subtasks: [{id: T1, robot: robot1, deadline_sec: 5, predecessors: [T0]}]}",
    // cycle in the dependency graph
    "{mission_id: m, subtasks: [{id: T1, robot: robot1, deadline_sec: 5, predecessors: [T2]},"
    " {id: T2, robot: robot2, deadline_sec: 5, predecessors: [T1]}]}",
    // self dependency
    "{mission_id: m, subtasks: [{id: T1, robot: robot1, deadline_sec: 5, predecessors: [T1]}]}",
    // duplicated id
    "{mission_id: m, subtasks: [{id: T1, robot: robot1, deadline_sec: 5},"
    " {id: T1, robot: robot2, deadline_sec: 5}]}",
    // missing deadline
    "{mission_id: m, subtasks: [{id: T1, robot: robot1}]}",
    // wrong type
    "{mission_id: m, subtasks: [{id: T1, robot: robot1, deadline_sec: 5, work: fast}]}",
    // work <= 0
    "{mission_id: m, subtasks: [{id: T1, robot: robot1, deadline_sec: 5, work: 0}]}",
    // queue order contradicts a dependency: robot1 would run T2 before its predecessor T1
    "{mission_id: m, subtasks: [{id: T2, robot: robot1, deadline_sec: 5, predecessors: [T1]},"
    " {id: T1, robot: robot1, deadline_sec: 5}]}",
  };
  for (const std::string & yaml : bad) {
    // The { ... } block is the "code" given to EXPECT_THROW. The last mission is a valid mission
    // but an impossible PLAN, so building the initial plan is part of the check too.
    EXPECT_THROW(
    {
      const Mission m = inline_mission(yaml);
      (void)Plan::initial(m);  // (void): we only care whether it throws, not about the result
    }, std::invalid_argument) << yaml;
  }
}

// Plans built from explicit queues (this is what repair candidates will do in C3).
// Each queue list is {robot1's queue, robot2's queue, robot3's queue}.
TEST(PlanValidation, QueuesMustAssignEverySubtaskOnceWithoutCycle)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  const Index t2 = m.task_index("T2");
  EXPECT_THROW(Plan::from_queues(m, {{t1}, {}, {}}), std::invalid_argument);          // T2 missing
  EXPECT_THROW(Plan::from_queues(m, {{t1, t2}, {t2}, {}}), std::invalid_argument);    // T2 twice
  EXPECT_THROW(Plan::from_queues(m, {{t2, t1}, {}, {}}), std::invalid_argument);      // cycle
  EXPECT_THROW(Plan::from_queues(m, {{t1}, {t2}}), std::invalid_argument);            // 2 queues
  EXPECT_NO_THROW(Plan::from_queues(m, {{t1, t2}, {}, {}}));  // both on robot1, in order: valid
}

// A robot runs its queue in order, so "B executing while A (before B in the queue) has not
// started" is physically impossible. The Predictor must refuse such a state.
TEST(StateValidation, QueueStatusesMustFollowTheFixedOrder)
{
  const Mission m = inline_mission(
    "{mission_id: q, subtasks: [{id: A, robot: robot1, deadline_sec: 20},"
    " {id: B, robot: robot1, deadline_sec: 20}]}");   // robot1 queue: [A, B]
  const Plan plan = Plan::initial(m);
  RuntimeState s = RuntimeState::initial(m);
  s.now_ms = 1000;
  set_executing(m, s, "B", 0, 1000);  // B runs while A, before it in the queue, never started
  const DurationSnapshot p = durations(m, s);
  EXPECT_THROW(Predictor(m, plan, s, p), std::invalid_argument);
}

// =============================================================================================
// Chain demo (config/missions/chain.yaml)
//   T1 (robot1, 5 s, deadline 6 s) --> T2 (robot2, 7.5 s, deadline 14 s)
// =============================================================================================

// Nothing has started (t = 0). Expected timeline:
//   T1 |=====|                 0 - 5 s     (deadline 6 s)
//   T2       |=======|         5 - 12.5 s  (deadline 14 s)
TEST(ChainDemo, NominalScheduleMeetsAllDeadlines)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);
  const RuntimeState s = RuntimeState::initial(m);
  const DurationSnapshot p = durations(m, s);
  const Prediction pr = Predictor(m, plan, s, p).full();

  // TaskPrediction{start, finish, outcome_ok, deadline_ok}
  EXPECT_EQ(pr.tasks[m.task_index("T1")], (dwafp::TaskPrediction{0, 5000, true, true}));
  EXPECT_EQ(pr.tasks[m.task_index("T2")], (dwafp::TaskPrediction{5000, 12500, true, true}));
  EXPECT_TRUE(dwafp::threatened_tasks(s, pr).empty());  // nobody is at risk
}

// At t = 4 s, robot1 gets blocked: T1 needs 4 s more instead of 1 s.
//   before: T1 ends 5 s, T2 5 - 12.5 s   -> all fine
//   after : T1 ends 8 s (> 6 s), T2 8 - 15.5 s (> 14 s) -> both threatened
// The delay of T1 propagates to T2 through the dependency T1 -> T2.
TEST(ChainDemo, DelayInT1ThreatensBothDeadlines)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);

  // At t = 4 s, T1 runs on schedule: 1 s left -> baseline prediction at the same time.
  RuntimeState before = RuntimeState::initial(m);
  before.now_ms = 4000;
  set_executing(m, before, "T1", 0, 1000);
  const DurationSnapshot p = durations(m, before);
  const Prediction baseline = Predictor(m, plan, before, p).full();
  EXPECT_TRUE(dwafp::threatened_tasks(before, baseline).empty());

  // Deviation: robot1 is blocked and now reports 4 s left -> T1 ends at 8 s instead of 5 s.
  RuntimeState after = before;  // copy, then change only T1
  after.tasks[m.task_index("T1")].remaining_ms = 4000;
  const std::vector<Index> horizon =
    incremental_matches_full(m, plan, after, p, baseline, {m.task_index("T1")});
  EXPECT_EQ(names(m, horizon), (std::vector<std::string>{"T1", "T2"}));  // both predictions moved

  const Prediction pr = Predictor(m, plan, after, p).full();
  EXPECT_EQ(pr.tasks[m.task_index("T1")].finish_ms, 8000);   // > 6 s deadline
  EXPECT_EQ(pr.tasks[m.task_index("T2")].start_ms, 8000);    // waits for T1
  EXPECT_EQ(pr.tasks[m.task_index("T2")].finish_ms, 15500);  // > 14 s deadline
  EXPECT_EQ(names(m, dwafp::threatened_tasks(after, pr)), (std::vector<std::string>{"T1", "T2"}));
}

// Same situation but only 0.5 s late: T1 ends 5.5 s (<= 6 s), T2 5.5 - 13 s (<= 14 s).
// The times change (so both are in the horizon) but the slack absorbs the delay: no threat.
// This is the case where DWAFP must NOT repair anything.
TEST(ChainDemo, SmallDelayIsAbsorbedBySlack)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);
  RuntimeState before = RuntimeState::initial(m);
  before.now_ms = 4000;
  set_executing(m, before, "T1", 0, 1000);
  const DurationSnapshot p = durations(m, before);
  const Prediction baseline = Predictor(m, plan, before, p).full();

  // 0.5 s late: T1 ends at 5.5 s (<= 6 s), T2 at 13 s (<= 14 s). Times change, nothing threatened.
  RuntimeState after = before;
  after.tasks[m.task_index("T1")].remaining_ms = 1500;
  const std::vector<Index> horizon =
    incremental_matches_full(m, plan, after, p, baseline, {m.task_index("T1")});
  EXPECT_EQ(names(m, horizon), (std::vector<std::string>{"T1", "T2"}));
  EXPECT_TRUE(dwafp::threatened_tasks(after, Predictor(m, plan, after, p).full()).empty());
}

// Outcome failure: at t = 3 s, T1 stops WITHOUT producing its output.
//   T2 can still be timed (it would run 3 - 10.5 s, inside its 14 s deadline)...
//   ...but its input is missing (outcome_ok = false), so T2 is threatened by its OUTCOME.
// This is the "keep times finite, mark the missing output" rule of the formulation.
TEST(ChainDemo, OutcomeFailureOfT1KeepsT2TimesFiniteButMissesItsInput)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);
  RuntimeState before = RuntimeState::initial(m);
  before.now_ms = 3000;
  set_executing(m, before, "T1", 0, 2000);
  const DurationSnapshot p = durations(m, before);
  const Prediction baseline = Predictor(m, plan, before, p).full();

  // T1 ends at 3 s WITHOUT its output.
  RuntimeState after = before;
  set_finished(m, after, "T1", TaskStatus::kFailed, 0, 3000);
  incremental_matches_full(m, plan, after, p, baseline, {m.task_index("T1")});

  const Prediction pr = Predictor(m, plan, after, p).full();
  const dwafp::TaskPrediction t2 = pr.tasks[m.task_index("T2")];
  EXPECT_EQ(t2.start_ms, 3000);    // finite: f_hat(T1) = t_fail
  EXPECT_EQ(t2.finish_ms, 10500);
  EXPECT_TRUE(t2.deadline_ok);     // the deadline alone would hold...
  EXPECT_FALSE(t2.outcome_ok);     // ...but its mandatory input is missing
  // T1 is a realized failure, not a prediction: only T2 is reported as threatened.
  EXPECT_EQ(names(m, dwafp::threatened_tasks(after, pr)), (std::vector<std::string>{"T2"}));
}

// =============================================================================================
// Fork-join demo (config/missions/fork_join.yaml)
//   T1 (robot1, 5 s,   deadline 6 s) ---+
//                                       +--> T3 (robot3, 4 s, deadline 14 s)
//   T2 (robot2, 7.5 s, deadline 9 s) ---+
// =============================================================================================

namespace
{

// Shared setup for the fork-join delay tests ("fixture"). At t = 4 s:
//   before: T1 running, 1 s left (ends 5 s); T2 running, 3.5 s left (ends 7.5 s)
//   after : same, except that T1 now reports `t1_remaining_ms` left
// Each test creates one with its own delay, e.g. ForkJoinCase c(5000) -> T1 ends at 9 s.
struct ForkJoinCase
{
  Mission m = load("fork_join.yaml");
  Plan plan = Plan::initial(m);
  RuntimeState before = RuntimeState::initial(m);
  RuntimeState after;
  DurationSnapshot p;
  Prediction baseline;

  // "explicit" = this constructor must be called on purpose (ForkJoinCase c(5000)), never as an
  // automatic conversion from a number.
  explicit ForkJoinCase(dwafp::TimeMs t1_remaining_ms)
  {
    before.now_ms = 4000;
    set_executing(m, before, "T1", 0, 1000);  // ends at 5 s
    set_executing(m, before, "T2", 0, 3500);  // ends at 7.5 s
    p = durations(m, before);
    baseline = Predictor(m, plan, before, p).full();
    after = before;
    after.tasks[m.task_index("T1")].remaining_ms = t1_remaining_ms;
  }
};

}  // namespace

// Nothing has started. T3 needs BOTH inputs, so it waits for the slower branch T2:
//   T1 |=====|                  0 - 5 s
//   T2 |=======|                0 - 7.5 s
//   T3         |====|           7.5 - 11.5 s  (deadline 14 s: 2.5 s of slack)
TEST(ForkJoinDemo, NominalJoinWaitsForTheSlowerBranch)
{
  const Mission m = load("fork_join.yaml");
  const Plan plan = Plan::initial(m);
  const RuntimeState s = RuntimeState::initial(m);
  const DurationSnapshot p = durations(m, s);
  const Prediction pr = Predictor(m, plan, s, p).full();
  EXPECT_EQ(pr.tasks[m.task_index("T3")], (dwafp::TaskPrediction{7500, 11500, true, true}));
}

// T1 is 1 s late (ends 6 s), but T3 waits for T2 (7.5 s) anyway, so T3 does not move.
// The incremental propagation must stop at T1: horizon = {T1}.
TEST(ForkJoinDemo, DelayHiddenByTheSlowerBranchStopsAtT1)
{
  // T1 ends at 6 s: still before T2 (7.5 s), so T3 does not move and propagation stops at T1.
  ForkJoinCase c(2000);
  const std::vector<Index> horizon =
    incremental_matches_full(c.m, c.plan, c.after, c.p, c.baseline, {c.m.task_index("T1")});
  EXPECT_EQ(names(c.m, horizon), (std::vector<std::string>{"T1"}));
}

// T1 is 4 s late (ends 9 s): T3 now waits for T1 and runs 9 - 13 s (still <= 14 s).
// Branch T2 is independent of T1: its prediction must stay identical and it must NOT be in
// the horizon. This is the "repair only the affected branch" property of the advisor's demo.
TEST(ForkJoinDemo, DelayInT1MovesT3ButNeverTouchesBranchT2)
{
  // T1 ends at 9 s: T3 starts at 9 s, ends at 13 s (<= 14 s). Only T1's own deadline is threatened.
  ForkJoinCase c(5000);
  const std::vector<Index> horizon =
    incremental_matches_full(c.m, c.plan, c.after, c.p, c.baseline, {c.m.task_index("T1")});
  EXPECT_EQ(names(c.m, horizon), (std::vector<std::string>{"T1", "T3"}));  // T2 not in it

  const Prediction pr = Predictor(c.m, c.plan, c.after, c.p).full();
  EXPECT_EQ(pr.tasks[c.m.task_index("T2")], c.baseline.tasks[c.m.task_index("T2")]);  // unchanged
  EXPECT_EQ(pr.tasks[c.m.task_index("T3")].finish_ms, 13000);
  EXPECT_EQ(names(c.m, dwafp::threatened_tasks(c.after, pr)), (std::vector<std::string>{"T1"}));
}

// T1 is 6 s late (ends 11 s): T3 runs 11 - 15 s, beyond its 14 s deadline -> T3 threatened too.
TEST(ForkJoinDemo, LargeDelayInT1ThreatensTheJoin)
{
  // T1 ends at 11 s: T3 ends at 15 s > 14 s.
  ForkJoinCase c(7000);
  incremental_matches_full(c.m, c.plan, c.after, c.p, c.baseline, {c.m.task_index("T1")});
  const Prediction pr = Predictor(c.m, c.plan, c.after, c.p).full();
  EXPECT_EQ(pr.tasks[c.m.task_index("T3")].finish_ms, 15000);
  EXPECT_EQ(names(c.m, dwafp::threatened_tasks(c.after, pr)), (std::vector<std::string>{"T1", "T3"}));
}

// =============================================================================================
// Robot-queue edges and robot unavailability
//   robot1 queue: A (5 s, d 6 s) then B (5 s, d 11 s, independent of A)
//   robot2 queue: D (7.5 s, d 8 s) then C (7.5 s, d 20 s); C depends on B
//
//   robot1:  A ----queue----> B
//                             |  dependency
//   robot2:  D ----queue----> C
// =============================================================================================

namespace
{
// Mission order = queue order: robot1 gets [A, B], robot2 gets [D, C].
const char * kQueueMission =
  "{mission_id: queue, subtasks: ["
  " {id: A, robot: robot1, deadline_sec: 6},"
  " {id: D, robot: robot2, deadline_sec: 8},"
  " {id: B, robot: robot1, deadline_sec: 11},"
  " {id: C, robot: robot2, deadline_sec: 20, predecessors: [B]}]}";
}  // namespace

// B does not need anything from A, but robot1 must finish A before starting B (queue edge).
// So a delay of A moves B, and C follows B through its dependency. D (other robot) is untouched.
//   before (t = 2 s): A 0-5, B 5-10, D 0-7.5, C 10-17.5
//   after  (A +2 s) : A 0-7, B 7-12 (> 11 s deadline), D unchanged, C 12-19.5
TEST(QueueEdges, DelayShiftsLaterTasksOfTheSameRobotQueue)
{
  const Mission m = inline_mission(kQueueMission);
  const Plan plan = Plan::initial(m);
  RuntimeState before = RuntimeState::initial(m);
  before.now_ms = 2000;
  set_executing(m, before, "A", 0, 3000);  // ends at 5 s
  set_executing(m, before, "D", 0, 5500);  // ends at 7.5 s
  const DurationSnapshot p = durations(m, before);
  const Prediction baseline = Predictor(m, plan, before, p).full();
  EXPECT_EQ(baseline.tasks[m.task_index("B")].finish_ms, 10000);
  EXPECT_EQ(baseline.tasks[m.task_index("C")].start_ms, 10000);  // after B (dependency) and D (queue)

  // A is 2 s late. B has NO dependency on A, yet it moves: robot1 must finish A first (queue edge).
  // C follows through its dependency on B. D (other robot, independent) is untouched.
  RuntimeState after = before;
  after.tasks[m.task_index("A")].remaining_ms = 5000;  // A ends at 7 s
  const std::vector<Index> horizon =
    incremental_matches_full(m, plan, after, p, baseline, {m.task_index("A")});
  EXPECT_EQ(names(m, horizon), (std::vector<std::string>{"A", "B", "C"}));

  const Prediction pr = Predictor(m, plan, after, p).full();
  EXPECT_EQ(pr.tasks[m.task_index("B")].finish_ms, 12000);  // misses its 11 s deadline
  EXPECT_EQ(names(m, dwafp::threatened_tasks(after, pr)), (std::vector<std::string>{"A", "B"}));
}

// A fails at t = 3 s. Queue edges carry TIMING only:
//   robot1 is free at 3 s, so B starts at 3 s (earlier than planned) and ends at 8 s,
//   and B keeps its output because it does not depend on A.
// Nothing is threatened (A itself is a realized failure, not a threat).
TEST(QueueEdges, FailedTaskFreesItsRobotWithoutRemovingTheNextTasksOutput)
{
  const Mission m = inline_mission(kQueueMission);
  const Plan plan = Plan::initial(m);
  RuntimeState s = RuntimeState::initial(m);
  s.now_ms = 3000;
  set_finished(m, s, "A", TaskStatus::kFailed, 0, 3000);
  set_executing(m, s, "D", 0, 4500);
  const DurationSnapshot p = durations(m, s);
  const Prediction pr = Predictor(m, plan, s, p).full();

  // Queue edges carry timing only: B starts when A failed, and keeps its own output.
  EXPECT_EQ(pr.tasks[m.task_index("B")], (dwafp::TaskPrediction{3000, 8000, true, true}));
  EXPECT_TRUE(dwafp::threatened_tasks(s, pr).empty());
}

// Robot-unavailability failure: robot1 breaks down at t = 2 s.
//   A (running on robot1)  -> treated as failed at 2 s, no output
//   B (queued on robot1)   -> cannot run there, no output
//   C (needs B's output)   -> no output, by propagation along the dependency B -> C
//   D (robot2, independent) -> unaffected
TEST(RobotUnavailability, RunningAndQueuedTasksLoseTheirOutputs)
{
  const Mission m = inline_mission(kQueueMission);
  const Plan plan = Plan::initial(m);
  RuntimeState before = RuntimeState::initial(m);
  before.now_ms = 2000;
  set_executing(m, before, "A", 0, 3000);
  set_executing(m, before, "D", 0, 5500);
  const DurationSnapshot p = durations(m, before);
  const Prediction baseline = Predictor(m, plan, before, p).full();

  // robot1 becomes unavailable at 2 s. Every task of its queue is "changed" (its robot changed).
  RuntimeState after = before;
  after.robots[m.robot_index("robot1")].available = false;
  after.robots[m.robot_index("robot1")].unavailable_since_ms = 2000;
  const std::vector<Index> horizon = incremental_matches_full(
    m, plan, after, p, baseline, plan.queue(m.robot_index("robot1")));
  EXPECT_EQ(names(m, horizon), (std::vector<std::string>{"A", "B", "C"}));

  const Prediction pr = Predictor(m, plan, after, p).full();
  EXPECT_EQ(pr.tasks[m.task_index("A")].finish_ms, 2000);  // treated as failed at t_u
  EXPECT_FALSE(pr.tasks[m.task_index("A")].outcome_ok);
  EXPECT_FALSE(pr.tasks[m.task_index("B")].outcome_ok);    // cannot run on robot1
  EXPECT_FALSE(pr.tasks[m.task_index("C")].outcome_ok);    // its input B is missing
  EXPECT_TRUE(pr.tasks[m.task_index("D")].outcome_ok);     // other robot, unaffected
  EXPECT_EQ(names(m, dwafp::threatened_tasks(after, pr)), (std::vector<std::string>{"A", "B", "C"}));
}

// =============================================================================================
// Incremental propagation: safety check and randomized proof that incremental == full
// =============================================================================================

// incremental() needs a prediction made at the SAME time t. A prediction made at t = 0 reused at
// t = 1 s would give wrong "unchanged" decisions, so it must be refused with an exception.
TEST(Incremental, RefusesAPredictionMadeAtAnotherTime)
{
  const Mission m = load("chain.yaml");
  const Plan plan = Plan::initial(m);
  RuntimeState s = RuntimeState::initial(m);
  const DurationSnapshot p = durations(m, s);
  Prediction old = Predictor(m, plan, s, p).full();  // made at t = 0
  s.now_ms = 1000;
  EXPECT_THROW(Predictor(m, plan, s, p).incremental(old, {0}), std::invalid_argument);
}

// Hand-written scenarios cover the cases we thought of; this test covers the ones we did not.
// 300 rounds, each with:
//   1. a random workflow : 12 subtasks, random robots, work, deadlines and dependencies;
//   2. a random valid state at t = 10 s (some finished, some running, some waiting);
//   3. a random change   : new remaining times, OR a robot availability change, OR a plan edit
//                          (one waiting subtask moved to another robot, like a substitution);
//   4. the central check : incremental_matches_full() (same prediction, exact horizon).
// A fixed seed (42) makes the "random" sequence identical on every run, so a failure can be
// reproduced; SCOPED_TRACE prints the round number and the mission if a check fails.
TEST(Incremental, MatchesFullPropagationOnRandomWorkflows)
{
  std::mt19937 rng(42);  // fixed seed: the test is deterministic
  // uniform(lo, hi) = a random integer between lo and hi, both included.
  const auto uniform = [&rng](int lo, int hi) {return std::uniform_int_distribution<int>(lo, hi)(rng);};
  const char * robots[] = {"robot1", "robot2", "robot3"};

  for (int round = 0; round < 300; ++round) {
    // --- 1. Random workflow, written as YAML text then loaded like any mission. ---
    // Random DAG of 12 subtasks: dependencies only go from a lower to a higher index, and the
    // queues follow index order, so the combined graph is acyclic.
    // Each earlier subtask h becomes a predecessor of j with probability 1/4 (uniform(0, 3) == 0).
    std::string yaml = "{mission_id: random, subtasks: [";
    const int n = 12;
    for (int j = 0; j < n; ++j) {
      yaml += std::string(j ? "," : "") + "{id: T" + std::to_string(j) + ", robot: " +
        robots[uniform(0, 2)] + ", work: " + std::to_string(uniform(1, 8)) +
        ", deadline_sec: " + std::to_string(uniform(5, 60)) + ", predecessors: [";
      bool first = true;
      for (int h = 0; h < j; ++h) {
        if (uniform(0, 3) == 0) {
          yaml += std::string(first ? "" : ",") + "T" + std::to_string(h);
          first = false;
        }
      }
      yaml += "]}";
    }
    yaml += "]}";
    const Mission m = inline_mission(yaml);
    const Plan plan = Plan::initial(m);

    // --- 2. Random valid state at t = 10 s. ---
    // On each robot, a prefix of finished tasks, then maybe one
    // executing task, then unstarted ones (the fixed queue order the Predictor checks).
    // About 1 finished task in 5 is a failure; each robot gets a random availability time a_r.
    RuntimeState before = RuntimeState::initial(m);
    before.now_ms = 10000;
    for (Index r = 0; r < m.robot_count(); ++r) {
      const auto & q = plan.queue(r);
      const std::size_t finished = static_cast<std::size_t>(uniform(0, static_cast<int>(q.size())));
      for (std::size_t k = 0; k < finished; ++k) {
        const int start = uniform(0, 5000);
        set_finished(m, before, m.task(q[k]).id,
          uniform(0, 4) == 0 ? TaskStatus::kFailed : TaskStatus::kCompleted, start, start + uniform(0, 5000));
      }
      if (finished < q.size() && uniform(0, 1)) {  // one time in two, the next task is running
        set_executing(m, before, m.task(q[finished]).id, uniform(0, 10000), uniform(0, 8000));
      }
      before.robots[r].available_from_ms = uniform(0, 15000);
    }
    const DurationSnapshot p = durations(m, before);
    const Prediction baseline = Predictor(m, plan, before, p).full();

    // --- 3. Random change of the same state at the same time, with the matching "changed" list.
    // `changed` must list every subtask whose OWN inputs the change touched; incremental()
    // finds the rest by itself. If this list were incomplete, the check below would fail.
    RuntimeState after = before;
    Plan after_plan = plan;
    std::vector<Index> changed;
    switch (uniform(0, 2)) {
      case 0:  // the executing tasks report new remaining times (delay or speed-up)
        for (Index j = 0; j < m.task_count(); ++j) {
          if (after.tasks[j].status == TaskStatus::kExecuting) {
            after.tasks[j].remaining_ms = uniform(0, 20000);
            changed.push_back(j);
          }
        }
        break;
      case 1: {  // one robot becomes unavailable or changes availability: all its tasks changed
          const Index r = static_cast<Index>(uniform(0, 2));
          if (uniform(0, 1)) {
            after.robots[r].available = false;
            after.robots[r].unavailable_since_ms = uniform(0, 10000);
          } else {
            after.robots[r].available_from_ms = uniform(0, 30000);
          }
          changed = plan.queue(r);
          break;
        }
      case 2: {  // plan edit: move one unstarted task to the end of another robot's queue
          std::vector<Index> unstarted;
          for (Index j = 0; j < m.task_count(); ++j) {
            if (before.tasks[j].status == TaskStatus::kUnstarted) {unstarted.push_back(j);}
          }
          if (unstarted.empty()) {break;}  // nothing to move in this round
          const Index j = unstarted[static_cast<std::size_t>(uniform(0, static_cast<int>(unstarted.size()) - 1))];
          const Index from = plan.robot_of(j);
          const Index to = (from + static_cast<Index>(uniform(1, 2))) % 3;  // a different robot
          auto queues = plan.queues();
          queues[from].erase(std::find(queues[from].begin(), queues[from].end(), j));
          queues[to].push_back(j);
          try {
            after_plan = Plan::from_queues(m, queues);
          } catch (const std::invalid_argument &) {
            break;  // the edit created a cycle: rejected, as a repair candidate would be
          }
          // Inputs that changed: j itself (robot + queue predecessor) and the task that now
          // follows j's old predecessor in the old queue (its queue predecessor changed).
          changed.push_back(j);
          if (plan.queue_successor(j) != dwafp::kNoIndex) {changed.push_back(plan.queue_successor(j));}
          break;
        }
    }

    // --- 4. The central check. SCOPED_TRACE adds this text to any failure inside the call. ---
    SCOPED_TRACE("round " + std::to_string(round) + ": " + yaml);
    incremental_matches_full(m, after_plan, after, p, baseline, changed);
  }
}
