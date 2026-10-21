// Unit tests of DWAFP Stage I, single-edit repair (Phase C3).
//
// Same conventions as the other tests: times in comments are nominal seconds, in code nominal ms.
// One subtask takes 5 s on robot1, 7.5 s on robot2, 4 s on robot3 (work 5, mock_fleet speeds).
// Every test runs the full event pipeline:
//   reference (mission start) -> actual state at t -> expected_state -> analyze_impact
//   -> generate_single_edits
// and checks the candidates against values computed by hand. Every admissible candidate is also
// checked against a FULL propagation of its edited plan (the incremental validation must agree).

#include <gtest/gtest.h>

#include <algorithm>  // std::find
#include <random>
#include <string>
#include <vector>

#include "actions/impact.hpp"
#include "actions/mission_loader.hpp"
#include "actions/repair.hpp"
#include "actions/workflow.hpp"

using dwafp::DurationSnapshot;
using dwafp::ImpactAnalysis;
using dwafp::Index;
using dwafp::Mission;
using dwafp::Plan;
using dwafp::Prediction;
using dwafp::Predictor;
using dwafp::RepairCandidate;
using dwafp::RepairPolicy;
using dwafp::RuntimeState;
using dwafp::StageOneResult;
using dwafp::TaskStatus;
using dwafp::TimeMs;

namespace
{

const std::string kConfigDir = ACTIONS_CONFIG_DIR;

Mission inline_mission(const std::string & yaml)
{
  return dwafp::mission_from_yaml(yaml, "robots: [{name: robot1, step_duration_sec: 1.0},"
                                        " {name: robot2, step_duration_sec: 1.5},"
                                        " {name: robot3, step_duration_sec: 0.8}]");
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

// One deviation event, from the observed state to Stage I. `deviating` names the tasks that
// deviated (used to rebuild the expected state from the mission-start reference).
struct Event
{
  Mission m;
  Plan plan;
  DurationSnapshot p;
  RuntimeState actual;
  ImpactAnalysis impact;
  StageOneResult stage1;

  Event(Mission mission, RuntimeState observed, const std::vector<std::string> & deviating,
    const dwafp::RepairConfig & config = {})
  : m(std::move(mission)), plan(Plan::initial(m)),
    p(DurationSnapshot::capture(m, dwafp::ConfigDurationModel{}, observed)),
    actual(std::move(observed))
  {
    const RuntimeState s0 = RuntimeState::initial(m);
    const Prediction reference = Predictor(m, plan, s0, p).full();
    std::vector<Index> tasks;
    for (const auto & id : deviating) {tasks.push_back(m.task_index(id));}
    const RuntimeState expected = dwafp::expected_state(m, plan, actual, reference, tasks, {});
    impact = dwafp::analyze_impact(m, plan, expected, actual, p);
    stage1 = dwafp::generate_single_edits(m, plan, actual, p, impact, config);
  }

  // The admissible candidate (policy, task, robot), or nullptr if it was not admissible.
  const RepairCandidate * find(RepairPolicy policy, const std::string & task, const std::string & robot) const
  {
    for (const auto & c : stage1.admissible) {
      if (c.policy == policy && c.task == m.task_index(task) && c.robot == m.robot_index(robot)) {return &c;}
    }
    return nullptr;
  }

  const RepairCandidate & best() const {return stage1.admissible.at(stage1.best.value());}

  TimeMs finish(const RepairCandidate & c, const std::string & task) const
  {
    return c.prediction.tasks[m.task_index(task)].finish_ms;
  }

  // The incremental prediction of every admissible candidate equals a full propagation.
  void expect_consistent() const
  {
    for (const auto & c : stage1.admissible) {
      const Prediction full = Predictor(m, c.plan, c.state, p).full();
      for (Index j = 0; j < m.task_count(); ++j) {
        EXPECT_EQ(c.prediction.tasks[j], full.tasks[j]) << "candidate on " << m.task(c.task).id;
      }
    }
  }
};

// The chain used by most tests: T1 (robot1) --> T2 (robot2), with the given deadlines.
Mission chain(double d1_sec, double d2_sec)
{
  return inline_mission(
    "{mission_id: chain, subtasks: ["
    " {id: T1, robot: robot1, deadline_sec: " + std::to_string(d1_sec) + "},"
    " {id: T2, robot: robot2, deadline_sec: " + std::to_string(d2_sec) + ", predecessors: [T1]}]}");
}

}  // namespace

// T1 (d 20 s) --> T2 (d 14 s); robot3 idle. Reference: T1 0-5 s, T2 5-12.5 s.
// At t = 2 s robot1 is blocked and predicts T1 at 8 s: T2 8-15.5 s > 14 s. A_i = {T2}, B_i = 12 s.
// The five primitives, computed by hand:
//   retry T1 on robot1            : T1 2-7,   T2 7-14.5    R = 1 (T2 late)       D = 1  C = 2 s
//   takeover T1 -> robot2 [T1,T2] : T1 2-9.5, T2 9.5-17    R = 1                 D = 2  C = 5 s
//   substitution T1 -> robot3     : T1 2-6,   T2 6-13.5    R = 0                 D = 1  C = 2 s
//   substitution T2 -> robot1     : next slot, after T1: T2 8-13   R = 0         D = 1  C = 0
//   substitution T2 -> robot3     : T2 8-12                R = 0                 D = 1  C = 0
// Two candidates tie at [0, 1, 0]: the substitutions of the DOWNSTREAM task T2. The first one
// generated wins (robot order): T2 in robot1's next slot, after robot1 finishes T1.
TEST(StageOne, SubstitutionOfTheDownstreamTaskIsBest)
{
  const Mission m = chain(20, 14);
  RuntimeState s = RuntimeState::initial(m);
  s.now_ms = 2000;
  set_executing(m, s, "T1", 0, 6000);
  const Event e(m, s, {"T1"});

  ASSERT_EQ(e.impact.response_window_ms, 12000);
  EXPECT_EQ(e.stage1.generated, 5u);
  EXPECT_EQ(e.stage1.admissible.size(), 5u);
  // T2 is threatened only by its deadline: it would be late, not invalid, so it is not held.
  EXPECT_TRUE(e.stage1.to_hold.empty());

  const RepairCandidate * retry = e.find(RepairPolicy::kRetry, "T1", "robot1");
  ASSERT_TRUE(retry);
  EXPECT_EQ(retry->residual_risk, 1);
  EXPECT_EQ(retry->disruption, 1);
  EXPECT_EQ(retry->cost_ms, 2000);  // 2 s of T1's work discarded

  const RepairCandidate * takeover = e.find(RepairPolicy::kTakeover, "T1", "robot2");
  ASSERT_TRUE(takeover);  // robot2 is idle because its next task T2 waits for T1
  EXPECT_EQ(takeover->residual_risk, 1);
  EXPECT_EQ(takeover->disruption, 2);  // T1 moved, T2's queue predecessor changed
  EXPECT_EQ(takeover->cost_ms, 5000);  // 2 s discarded + T1 1.5 s later + T2 1.5 s later

  const RepairCandidate * sub_t1 = e.find(RepairPolicy::kSubstitution, "T1", "robot3");
  ASSERT_TRUE(sub_t1);
  EXPECT_EQ(sub_t1->residual_risk, 0);
  EXPECT_EQ(e.finish(*sub_t1, "T2"), 13500);
  EXPECT_EQ(sub_t1->restoration_ms, 4000);  // T1 ends at 6 s

  const RepairCandidate * sub_t2_r3 = e.find(RepairPolicy::kSubstitution, "T2", "robot3");
  ASSERT_TRUE(sub_t2_r3);
  EXPECT_EQ(e.finish(*sub_t2_r3, "T2"), 12000);
  EXPECT_EQ(sub_t2_r3->restoration_ms, 10000);  // ends at 12 s, t = 2 s
  EXPECT_FALSE(sub_t2_r3->after_current_task);  // robot3 is idle

  const RepairCandidate & best = e.best();
  EXPECT_EQ(best.policy, RepairPolicy::kSubstitution);
  EXPECT_EQ(best.task, m.task_index("T2"));
  EXPECT_EQ(best.robot, m.robot_index("robot1"));
  EXPECT_TRUE(best.after_current_task);  // robot1 is busy: next-slot insertion after T1
  EXPECT_EQ(e.finish(best, "T2"), 13000);
  EXPECT_EQ(best.disruption, 1);
  EXPECT_EQ(best.cost_ms, 0);
  EXPECT_EQ(best.restoration_ms, 11000);
  EXPECT_TRUE(best.preventive());
  EXPECT_EQ(sub_t2_r3->residual_risk, best.residual_risk);  // the tie
  EXPECT_EQ(sub_t2_r3->disruption, best.disruption);
  EXPECT_EQ(sub_t2_r3->cost_ms, best.cost_ms);
  e.expect_consistent();
}

// Same event, with 3 s of decision + dispatch overhead: every edited task is released at
// 2 + 3 = 5 s at the earliest.
namespace
{
const Event & latency_event()
{
  static const Mission m = chain(20, 14);
  static const Event e = [] {
      RuntimeState s = RuntimeState::initial(m);
      s.now_ms = 2000;
      set_executing(m, s, "T1", 0, 6000);
      dwafp::RepairConfig config;
      config.decision_overhead_ms = 1000;
      config.dispatch_overhead_ms = 2000;
      return Event(m, s, {"T1"}, config);
    }();
  return e;
}
}  // namespace

// Latency ABSORBED by dependency waiting: T2 would wait for T1 until 8 s anyway, so being released
// at 5 s costs nothing. T2 on robot3 still runs 8-12 s (R = 0), and T2 in robot1's next slot
// 8-13 s: the substitutions of T2 stay preventive, exactly as without latency.
TEST(StageOne, DispatchLatencyAbsorbedByDependencyWaiting)
{
  const Event & e = latency_event();
  const Mission & m = e.m;
  const RepairCandidate * sub_t2 = e.find(RepairPolicy::kSubstitution, "T2", "robot3");
  ASSERT_TRUE(sub_t2);
  EXPECT_EQ(e.finish(*sub_t2, "T2"), 12000);  // same as with no latency
  EXPECT_TRUE(sub_t2->preventive());
  const RepairCandidate & best = e.best();
  EXPECT_EQ(best.task, m.task_index("T2"));
  EXPECT_EQ(e.finish(best, "T2"), 13000);
  e.expect_consistent();
}

// Latency makes an UPSTREAM repair too late: restarting T1 on robot3 now runs 5-9 s instead of
// 2-6 s, so T2 runs 9-16.5 s > 14 s (R = 1). Without latency the same candidate is preventive
// (test SubstitutionOfTheDownstreamTaskIsBest: T2 6-13.5 s). The retry gets worse the same way.
TEST(StageOne, DispatchLatencyMakesAnUpstreamRepairTooLate)
{
  const Event & e = latency_event();
  const RepairCandidate * sub_t1 = e.find(RepairPolicy::kSubstitution, "T1", "robot3");
  ASSERT_TRUE(sub_t1);
  EXPECT_EQ(e.finish(*sub_t1, "T1"), 9000);
  EXPECT_EQ(e.finish(*sub_t1, "T2"), 16500);
  EXPECT_EQ(sub_t1->residual_risk, 1);
  EXPECT_FALSE(sub_t1->preventive());
  const RepairCandidate * retry = e.find(RepairPolicy::kRetry, "T1", "robot1");
  ASSERT_TRUE(retry);
  EXPECT_EQ(e.finish(*retry, "T1"), 10000);  // released at 5 s, 5 s of work
  EXPECT_FALSE(retry->preventive());
}

// Takeover, and rejection of candidates that break another task.
//   T1 (robot1, d 20) --> T2 (robot2, d 13.6);  T1 --> W (robot3, d 13)
// Reference: T1 0-5, T2 5-12.5, W 5-9. At t = 2 s T1 predicts 9 s: T2 9-16.5 (> 13.6, threatened),
// W 9-13 (<= 13, fine). A_i = {T2}; W is not on a path to T2, so it is outside the region.
//   retry T1                      : T1 2-7, T2 7-14.5 (> 13.6)              admissible, R = 1
//   takeover T1 -> robot2 [T1,T2] : W 9.5-13.5 > 13: W newly violated     REJECTED
//   takeover T1 -> robot3 [T1,W]  : T1 2-6, W 6-10, T2 6-13.5              R = 0, best
//   subst.   T2 -> robot1, next slot after T1 : T2 9-14 (> 13.6)          admissible, R = 1
//   subst.   T2 -> robot3 [T2,W]  : T2 9-13, W pushed to 13-17 > 13      REJECTED
// The two rejections are exactly the "no new violation outside the region" check: W is not in
// the region, but it is in those candidates' validation horizon.
TEST(StageOne, TakeoverIsBestAndCandidatesBreakingAnotherTaskAreRejected)
{
  const Mission m = inline_mission(
    "{mission_id: takeover, subtasks: ["
    " {id: T1, robot: robot1, deadline_sec: 20},"
    " {id: T2, robot: robot2, deadline_sec: 13.6, predecessors: [T1]},"
    " {id: W, robot: robot3, deadline_sec: 13, predecessors: [T1]}]}");
  RuntimeState s = RuntimeState::initial(m);
  s.now_ms = 2000;
  set_executing(m, s, "T1", 0, 7000);  // predicts 9 s
  const Event e(m, s, {"T1"});

  EXPECT_EQ(e.impact.region, (std::vector<Index>{m.task_index("T1"), m.task_index("T2")}));
  EXPECT_EQ(e.stage1.generated, 5u);
  EXPECT_EQ(e.stage1.rejected_new_violation, 2u);
  const RepairCandidate * next_slot = e.find(RepairPolicy::kSubstitution, "T2", "robot1");
  ASSERT_TRUE(next_slot);
  EXPECT_TRUE(next_slot->after_current_task);
  EXPECT_EQ(e.finish(*next_slot, "T2"), 14000);  // waits for robot1 to finish T1 at 9 s
  EXPECT_EQ(next_slot->residual_risk, 1);
  EXPECT_FALSE(e.find(RepairPolicy::kTakeover, "T1", "robot2"));
  EXPECT_FALSE(e.find(RepairPolicy::kSubstitution, "T2", "robot3"));

  const RepairCandidate & best = e.best();
  EXPECT_EQ(best.policy, RepairPolicy::kTakeover);
  EXPECT_EQ(best.task, m.task_index("T1"));
  EXPECT_EQ(best.robot, m.robot_index("robot3"));
  EXPECT_EQ(e.finish(best, "T1"), 6000);
  EXPECT_EQ(e.finish(best, "W"), 10000);   // robot3 resumes its own task after T1
  EXPECT_EQ(e.finish(best, "T2"), 13500);
  EXPECT_EQ(best.disruption, 2);           // T1 moved, W's queue predecessor changed
  e.expect_consistent();
}

// Retry wins when it is the least disruptive preventive repair, and only when the blockage is
// transient.
//   T1 (robot1, d 20) --> T2 (robot2, d 14.5); robot3 unavailable (cannot receive anything).
// At t = 1 s T1 predicts 10 s: T2 10-17.5 > 14.5. A_i = {T2}, B_i = 13.5 s.
//   retry T1                         : T1 1-6, T2 6-13.5         R = 0, D = 1, C = 1 s  -> best
//   takeover T1 -> robot2 [T1,T2]    : T1 1-8.5, T2 8.5-16       R = 1
//   subst. T2 -> robot1, next slot   : T2 10-15                  R = 1
namespace
{
RuntimeState blocked_at_1s(const Mission & m, bool transient)
{
  RuntimeState s = RuntimeState::initial(m);
  s.now_ms = 1000;
  set_executing(m, s, "T1", 0, 9000);
  s.tasks[m.task_index("T1")].same_robot_retry_feasible = transient;
  s.robots[m.robot_index("robot3")].available = false;
  return s;
}
}  // namespace

TEST(StageOne, RetryIsBestWhenItIsTheLeastDisruptive)
{
  const Mission m = chain(20, 14.5);
  const Event e(m, blocked_at_1s(m, true), {"T1"});

  EXPECT_EQ(e.stage1.generated, 3u);  // nothing can go to robot3 (unavailable)
  const RepairCandidate & best = e.best();
  EXPECT_EQ(best.policy, RepairPolicy::kRetry);
  EXPECT_EQ(e.finish(best, "T1"), 6000);
  EXPECT_EQ(e.finish(best, "T2"), 13500);
  EXPECT_EQ(best.disruption, 1);  // only T1's retry flag
  EXPECT_EQ(best.cost_ms, 1000);  // 1 s of T1's work discarded
  e.expect_consistent();
}

// Same event, but the blockage is PERSISTENT (same_robot_retry_feasible = false): retrying on robot1 would
// meet the same obstacle, so retry is not generated. The remaining single edits all leave T2
// late: no Stage I repair (Stage II or escalation).
TEST(StageOne, PersistentBlockageExcludesRetry)
{
  const Mission m = chain(20, 14.5);
  const Event e(m, blocked_at_1s(m, false), {"T1"});

  EXPECT_EQ(e.stage1.generated, 2u);
  EXPECT_FALSE(e.find(RepairPolicy::kRetry, "T1", "robot1"));
  EXPECT_FALSE(e.stage1.best);
  e.expect_consistent();
}

// chain.yaml event (T1 3 s late at t = 4 s): T1's own deadline (6 s) cannot be saved by any
// single edit (the fastest restart, on robot3, ends at 8 s). No preventive Stage I repair, but the
// admissible candidates with R > 0 are kept for Stage II; the best of them, T2 -> robot3, leaves
// only T1's deadline violated (R = 1).
TEST(StageOne, NoPreventiveSingleEditWhenTheSourceDeadlineCannotBeSaved)
{
  const Mission m = dwafp::load_mission(kConfigDir + "/missions/chain.yaml", kConfigDir + "/mock_fleet.yaml");
  RuntimeState s = RuntimeState::initial(m);
  s.now_ms = 4000;
  set_executing(m, s, "T1", 0, 4000);
  const Event e(m, s, {"T1"});

  EXPECT_FALSE(e.stage1.best);
  EXPECT_EQ(e.stage1.admissible.size(), 5u);  // incl. T2 in robot1's next slot (8-13 s, R = 1)
  const RepairCandidate * sub_t2 = e.find(RepairPolicy::kSubstitution, "T2", "robot3");
  ASSERT_TRUE(sub_t2);
  EXPECT_EQ(sub_t2->residual_risk, 1);
  e.expect_consistent();
}

// Outcome failure: T1 (d 15) fails at 3 s; robot3 unavailable. T2 needs T1's output by its latest
// start d_T2 - 7.5 s. The retry restores the output at 8 s; whether that prevents the failure is
// decided by T2's own deadline, through propagation:
//   loose slack, T2 d 25 : retry T1 3-8, T2 8-15.5 <= 25   R = 0 -> accepted (best)
//   tight slack, T2 d 14 : retry T1 3-8, T2 8-15.5 >  14   R = 1 -> rejected
// (takeover T1 -> robot2 [T1,T2]: T2 10.5-18, accepted only with the loose deadline, and more
// disruptive; T2 -> robot1 cannot help, T1's output is still missing.)
namespace
{
Event outcome_failure_event(double t2_deadline_sec)
{
  const Mission m = chain(15, t2_deadline_sec);
  RuntimeState s = RuntimeState::initial(m);
  s.now_ms = 3000;
  set_failed(m, s, "T1", 0, 3000);
  s.robots[m.robot_index("robot3")].available = false;
  return Event(m, s, {"T1"});
}
}  // namespace

TEST(StageOne, LooseSlackOutcomeFailureRetryAccepted)
{
  const Event e = outcome_failure_event(25);
  const Mission & m = e.m;
  ASSERT_EQ(e.impact.response_window_ms, 14500);  // latest start 25 - 7.5 = 17.5 s, t = 3 s
  const RepairCandidate & best = e.best();
  EXPECT_EQ(best.policy, RepairPolicy::kRetry);
  EXPECT_EQ(best.task, m.task_index("T1"));
  EXPECT_EQ(e.finish(best, "T1"), 8000);
  EXPECT_EQ(e.finish(best, "T2"), 15500);
  EXPECT_TRUE(best.prediction.tasks[m.task_index("T2")].outcome_ok);  // output restored
  EXPECT_EQ(best.restoration_ms, 5000);  // output back at 8 s, before T2's latest start 17.5 s
  e.expect_consistent();
}

TEST(StageOne, TightSlackOutcomeFailureRetryRejected)
{
  const Event e = outcome_failure_event(14);
  ASSERT_EQ(e.impact.response_window_ms, 3500);  // latest start 14 - 7.5 = 6.5 s, t = 3 s
  const RepairCandidate * retry = e.find(RepairPolicy::kRetry, "T1", "robot1");
  ASSERT_TRUE(retry);
  EXPECT_EQ(e.finish(*retry, "T2"), 15500);
  EXPECT_EQ(retry->residual_risk, 1);  // output restored, but T2's deadline is missed
  EXPECT_FALSE(e.stage1.best);
  e.expect_consistent();
}

// No idle replacement robot: every robot is busy, and the blockage is persistent (no retry).
//   robot1: T1 (d 20, blocked, predicts 11 s)
//   robot2: X (0-7.5 s, independent)  then  T2 (d 15, needs T1)
//   robot3: Y (0-12 s, independent, work 15)
// At t = 2 s T2 would run 11-18.5 s > 15 s. Every substitution is a next-slot one, and each is
// too late:
//   T1 -> robot2 after X : T1 7.5-15,  T2 15-22.5      T1 -> robot3 after Y : T1 12-16, T2 16-23.5
//   T2 -> robot1 after T1: T2 11-16                     T2 -> robot3 after Y : T2 12-16
// No Stage I repair: the event goes to Stage II and, failing that, to escalation.
TEST(StageOne, NoIdleReplacementRobotRequiresEscalation)
{
  const Mission m = inline_mission(
    "{mission_id: busy, subtasks: ["
    " {id: T1, robot: robot1, deadline_sec: 20},"
    " {id: X, robot: robot2, deadline_sec: 30},"
    " {id: T2, robot: robot2, deadline_sec: 15, predecessors: [T1]},"
    " {id: Y, robot: robot3, work: 15, deadline_sec: 30}]}");
  RuntimeState s = RuntimeState::initial(m);
  s.now_ms = 2000;
  set_executing(m, s, "T1", 0, 9000);
  s.tasks[m.task_index("T1")].same_robot_retry_feasible = false;  // persistent blockage
  set_executing(m, s, "X", 0, 5500);
  set_executing(m, s, "Y", 0, 10000);
  const Event e(m, s, {"T1"});

  EXPECT_EQ(e.impact.threatened, (std::vector<Index>{m.task_index("T2")}));
  EXPECT_EQ(e.stage1.generated, 4u);
  EXPECT_EQ(e.stage1.admissible.size(), 4u);
  for (const auto & c : e.stage1.admissible) {
    EXPECT_TRUE(c.after_current_task);  // no robot is idle
    EXPECT_EQ(c.residual_risk, 1);
  }
  EXPECT_FALSE(e.stage1.best);
  e.expect_consistent();
}

// The incumbent window B_i^0 = 0 does not reject a feasible repair (colleague's example).
//   robot1 queue: A (completed 0-5 s) then B (d 11 s, 5 s on robot1, 4 s on robot3)
// robot1 broke down at 5 s, which is detected at t = 7 s; in the expected world B runs 5-10 s.
// B's latest start on its CURRENT robot is 11 - 5 = 6 s, already past: B_i^0 = 0. But robot3,
// idle, needs only 4 s: B -> robot3 runs 7-11 s and meets the deadline (R = 0), so it is the
// best preventive repair. (B -> robot2 needs 7.5 s: 7-14.5 s, R = 1.)
TEST(StageOne, IncumbentWindowZeroDoesNotRejectAFasterRobot)
{
  const Mission m = inline_mission(
    "{mission_id: late, subtasks: ["
    " {id: A, robot: robot1, deadline_sec: 6},"
    " {id: B, robot: robot1, deadline_sec: 11}]}");
  const Plan plan = Plan::initial(m);
  const Index a = m.task_index("A");
  const Index b = m.task_index("B");
  const Index robot1 = m.robot_index("robot1");

  RuntimeState actual = RuntimeState::initial(m);
  actual.now_ms = 7000;
  actual.tasks[a].status = TaskStatus::kCompleted;
  actual.tasks[a].start_ms = 0;
  actual.tasks[a].end_ms = 5000;
  actual.robots[robot1].available = false;      // broke down at 5 s
  actual.robots[robot1].unavailable_since_ms = 5000;

  RuntimeState expected = actual;                // the world without the breakdown:
  expected.robots[robot1].available = true;
  expected.robots[robot1].unavailable_since_ms = 0;
  expected.tasks[b].status = TaskStatus::kExecuting;  // B would be running since 5 s
  expected.tasks[b].start_ms = 5000;
  expected.tasks[b].remaining_ms = 3000;

  const DurationSnapshot p = DurationSnapshot::capture(m, dwafp::ConfigDurationModel{}, actual);
  const ImpactAnalysis impact = dwafp::analyze_impact(m, plan, expected, actual, p);
  ASSERT_EQ(impact.threatened, (std::vector<Index>{b}));
  EXPECT_EQ(impact.earliest_violation_ms, 6000);  // latest start on robot1
  EXPECT_EQ(impact.response_window_ms, 0);        // B_i^0 = 0
  EXPECT_FALSE(impact.incumbent_window_open());

  const StageOneResult stage1 = dwafp::generate_single_edits(m, plan, actual, p, impact);
  ASSERT_TRUE(stage1.best);
  const RepairCandidate & best = stage1.admissible[*stage1.best];
  EXPECT_EQ(best.policy, RepairPolicy::kSubstitution);
  EXPECT_EQ(best.task, b);
  EXPECT_EQ(best.robot, m.robot_index("robot3"));
  EXPECT_EQ(best.prediction.tasks[b].start_ms, 7000);
  EXPECT_EQ(best.prediction.tasks[b].finish_ms, 11000);  // exactly at the deadline
  EXPECT_EQ(best.residual_risk, 0);
}

// A deviation absorbed by slack needs no repair: Stage I returns an empty result.
TEST(StageOne, NoCandidatesWhenNothingIsThreatened)
{
  const Mission m = chain(20, 30);
  RuntimeState s = RuntimeState::initial(m);
  s.now_ms = 2000;
  set_executing(m, s, "T1", 0, 4000);  // ends 6 s instead of 5 s, T2 6-13.5 <= 30
  const Event e(m, s, {"T1"});
  EXPECT_FALSE(e.impact.needs_repair());
  EXPECT_EQ(e.stage1.generated, 0u);
  EXPECT_TRUE(e.stage1.admissible.empty());
  EXPECT_FALSE(e.stage1.best);
}

// =============================================================================================
// decide(): Algorithm 1 up to Stage I, with the LocalPreventionInfeasible certificate
// =============================================================================================

namespace
{
// Runs decide() for an observed state, rebuilding the expected state from the mission-start
// reference like Event does.
dwafp::Decision decide_event(
  const Mission & m, const RuntimeState & actual, const std::vector<std::string> & deviating,
  const dwafp::RepairConfig & config = {})
{
  const Plan plan = Plan::initial(m);
  const DurationSnapshot p = DurationSnapshot::capture(m, dwafp::ConfigDurationModel{}, actual);
  const RuntimeState s0 = RuntimeState::initial(m);
  const Prediction reference = Predictor(m, plan, s0, p).full();
  std::vector<Index> tasks;
  for (const auto & id : deviating) {tasks.push_back(m.task_index(id));}
  const RuntimeState expected = dwafp::expected_state(m, plan, actual, reference, tasks, {});
  return dwafp::decide(m, plan, expected, actual, p, config);
}
}  // namespace

// chain.yaml, T1 predicts 8 s at t = 4 s; its own deadline is 6 s. Lower bound for T1:
// continue 8 s, or restart on the fastest robot (robot3, 4 s) from 4 s -> 8 s. 8 > 6 s: no
// permitted local repair, single or double, can save T1's deadline. decide() says so WITHOUT
// running the search. T2 (deadline-only threat) is not held: it may still run, late.
TEST(Decide, NoFeasibleLocalRepairWhenTheSourceDeadlineIsProvablyLost)
{
  const Mission m = dwafp::load_mission(kConfigDir + "/missions/chain.yaml", kConfigDir + "/mock_fleet.yaml");
  RuntimeState s = RuntimeState::initial(m);
  s.now_ms = 4000;
  set_executing(m, s, "T1", 0, 4000);
  const dwafp::Decision d = decide_event(m, s, {"T1"});
  EXPECT_EQ(d.status, dwafp::DecisionStatus::kNoFeasibleLocalRepair);
  EXPECT_EQ(d.unprotectable, (std::vector<Index>{m.task_index("T1")}));
  EXPECT_TRUE(d.to_hold.empty());
  EXPECT_EQ(d.stage1.generated, 0u);  // the search was skipped
}

// Outcome failure of a task that may NOT be restarted (retryable: false): T2's input can never
// exist, whatever the repair. T2 is unprotectable, and it is held (its input is missing).
TEST(Decide, NoFeasibleLocalRepairWhenALostOutputCannotBeReproduced)
{
  const Mission m = inline_mission(
    "{mission_id: lost, subtasks: [{id: T1, robot: robot1, deadline_sec: 15, retryable: false},"
    " {id: T2, robot: robot2, deadline_sec: 25, predecessors: [T1]}]}");
  RuntimeState s = RuntimeState::initial(m);
  s.now_ms = 3000;
  set_failed(m, s, "T1", 0, 3000);
  const dwafp::Decision d = decide_event(m, s, {"T1"});
  EXPECT_EQ(d.status, dwafp::DecisionStatus::kNoFeasibleLocalRepair);
  EXPECT_EQ(d.unprotectable, (std::vector<Index>{m.task_index("T2")}));
  EXPECT_EQ(d.to_hold, (std::vector<Index>{m.task_index("T2")}));
}

// The repairable chain of SubstitutionOfTheDownstreamTaskIsBest: the certificate finds nothing
// unprotectable, the search runs and returns a repair.
TEST(Decide, RepairWhenAPreventiveSingleEditExists)
{
  const Mission m = chain(20, 14);
  RuntimeState s = RuntimeState::initial(m);
  s.now_ms = 2000;
  set_executing(m, s, "T1", 0, 6000);
  const dwafp::Decision d = decide_event(m, s, {"T1"});
  EXPECT_EQ(d.status, dwafp::DecisionStatus::kRepair);
  EXPECT_TRUE(d.unprotectable.empty());
  ASSERT_TRUE(d.stage1.best);
  EXPECT_EQ(d.stage1.admissible[*d.stage1.best].task, m.task_index("T2"));
}

// The no-idle-robot case: the lower bound is optimistic (it ignores that robot3 is busy until
// 12 s), so it cannot prove infeasibility; the search runs and finds no single edit: Stage II.
TEST(Decide, NoSingleEditWhenInfeasibilityCannotBeProved)
{
  const Mission m = inline_mission(
    "{mission_id: busy, subtasks: ["
    " {id: T1, robot: robot1, deadline_sec: 20},"
    " {id: X, robot: robot2, deadline_sec: 30},"
    " {id: T2, robot: robot2, deadline_sec: 15, predecessors: [T1]},"
    " {id: Y, robot: robot3, work: 15, deadline_sec: 30}]}");
  RuntimeState s = RuntimeState::initial(m);
  s.now_ms = 2000;
  set_executing(m, s, "T1", 0, 9000);
  s.tasks[m.task_index("T1")].same_robot_retry_feasible = false;
  set_executing(m, s, "X", 0, 5500);
  set_executing(m, s, "Y", 0, 10000);
  const dwafp::Decision d = decide_event(m, s, {"T1"});
  EXPECT_EQ(d.status, dwafp::DecisionStatus::kNoSingleEdit);
  EXPECT_TRUE(d.unprotectable.empty());
  EXPECT_EQ(d.stage1.generated, 4u);
}

// Ablation scopes on the event of SubstitutionOfTheDownstreamTaskIsBest:
//   sources only (T1)      : 3 primitives; best = T1 -> robot3 (the downstream fix is not seen)
//   region (T1, T2)        : 5 primitives; best = T2 in robot1's next slot
//   all unfinished (T1, T2): the same 5 here (the region already contains every unfinished task)
TEST(StageOne, RepairScopesForTheAblation)
{
  const Mission m = chain(20, 14);
  RuntimeState s = RuntimeState::initial(m);
  s.now_ms = 2000;
  set_executing(m, s, "T1", 0, 6000);

  dwafp::RepairConfig sources;
  sources.scope = dwafp::RepairScope::kSources;
  sources.deviating_tasks = {m.task_index("T1")};  // trigger-task-only: the event source
  const Event e_src(m, s, {"T1"}, sources);
  EXPECT_EQ(e_src.stage1.generated, 3u);
  EXPECT_EQ(e_src.best().task, m.task_index("T1"));
  EXPECT_EQ(e_src.best().robot, m.robot_index("robot3"));

  const Event e_region(m, s, {"T1"});
  EXPECT_EQ(e_region.stage1.generated, 5u);
  EXPECT_EQ(e_region.best().task, m.task_index("T2"));

  dwafp::RepairConfig all;
  all.scope = dwafp::RepairScope::kAllUnfinished;
  const Event e_all(m, s, {"T1"}, all);
  EXPECT_EQ(e_all.stage1.generated, 5u);
}

// Trigger-task-only repair with an OVERDUE descendant. T1 (robot1, 0-5 s nominal) is still running
// at 5.5 s and will end at 8 s; T2 (robot2, 7.5 s, d 14 s) should have started at 5 s, so
// expected_state() rebuilds it as running and T2 appears in impact.sources next to T1. The event
// source is T1 only: trigger-task-only repair must edit T1 and never T2 (moving T2 is a downstream
// edit, which only the region scope may make). Without deviating_tasks the scope is refused, so
// no caller can fall back to impact.sources.
TEST(StageOne, TriggerTaskOnlyIgnoresOverdueDescendants)
{
  const Mission m = chain(20, 14);
  RuntimeState s = RuntimeState::initial(m);
  s.now_ms = 5500;
  set_executing(m, s, "T1", 0, 2500);  // 2.5 s remaining: ends at 8 s
  const Index t1 = m.task_index("T1");
  const Index t2 = m.task_index("T2");

  const Event region(m, s, {"T1"});
  const auto & src = region.impact.sources;
  EXPECT_NE(std::find(src.begin(), src.end(), t2), src.end());  // T2 is an impact "source"

  dwafp::RepairConfig trigger_only;
  trigger_only.scope = dwafp::RepairScope::kSources;
  trigger_only.deviating_tasks = {t1};
  const Event e(m, s, {"T1"}, trigger_only);
  EXPECT_GT(e.stage1.generated, 0u);
  for (const auto & c : e.stage1.admissible) {EXPECT_EQ(c.task, t1);}  // never the overdue T2

  dwafp::RepairConfig missing;
  missing.scope = dwafp::RepairScope::kSources;  // no deviating_tasks
  EXPECT_THROW(Event(m, s, {"T1"}, missing), std::invalid_argument);
}

// Soundness of the certificate on random events: whenever unprotectable_threats() is not empty,
// even the GLOBAL single-edit oracle (every unfinished task, every robot) finds no preventive
// repair. The test also checks that both outcomes actually occur (the property is not vacuous).
TEST(Decide, CertificateIsSoundOnRandomEvents)
{
  std::mt19937 rng(7);
  const auto uniform = [&rng](int lo, int hi) {return std::uniform_int_distribution<int>(lo, hi)(rng);};
  const char * robots[] = {"robot1", "robot2", "robot3"};
  int certified = 0, searched = 0;
  for (int round = 0; round < 400; ++round) {
    std::string yaml = "{mission_id: r, subtasks: [";
    for (int j = 0; j < 8; ++j) {
      yaml += std::string(j ? "," : "") + "{id: T" + std::to_string(j) + ", robot: " +
        robots[uniform(0, 2)] + ", work: " + std::to_string(uniform(2, 6)) + ", deadline_sec: " +
        std::to_string(uniform(8, 40)) + ", retryable: " + (uniform(0, 4) ? "true" : "false") +
        ", predecessors: [";
      bool first = true;
      for (int h = 0; h < j; ++h) {
        if (uniform(0, 3) == 0) {yaml += std::string(first ? "" : ",") + "T" + std::to_string(h); first = false;}
      }
      yaml += "]}";
    }
    yaml += "]}";
    const Mission m = inline_mission(yaml);
    const Plan plan = Plan::initial(m);
    RuntimeState s0 = RuntimeState::initial(m);
    const DurationSnapshot p = DurationSnapshot::capture(m, dwafp::ConfigDurationModel{}, s0);
    const Prediction nominal = Predictor(m, plan, s0, p).full();

    // State at a random time as planned, then a random running task deviates.
    const TimeMs t = uniform(500, 15000);
    RuntimeState actual = RuntimeState::initial(m);
    actual.now_ms = t;
    std::vector<Index> running;
    for (Index j = 0; j < m.task_count(); ++j) {
      const auto & q = nominal.tasks[j];
      if (q.finish_ms <= t) {
        actual.tasks[j].status = TaskStatus::kCompleted;
        actual.tasks[j].start_ms = q.start_ms;
        actual.tasks[j].end_ms = q.finish_ms;
      } else if (q.start_ms <= t) {
        actual.tasks[j].status = TaskStatus::kExecuting;
        actual.tasks[j].start_ms = q.start_ms;
        actual.tasks[j].remaining_ms = q.finish_ms - t;
        running.push_back(j);
      }
    }
    if (running.empty()) {continue;}
    const Index k = running[static_cast<std::size_t>(uniform(0, static_cast<int>(running.size()) - 1))];
    if (uniform(0, 2) == 0) {
      actual.tasks[k].status = TaskStatus::kFailed;
      actual.tasks[k].end_ms = t;
    } else {
      actual.tasks[k].remaining_ms += uniform(1000, 12000);
    }
    actual.tasks[k].same_robot_retry_feasible = uniform(0, 1) == 1;
    dwafp::RepairConfig config;
    config.dispatch_overhead_ms = uniform(0, 2) * 1000;

    const RuntimeState expected = dwafp::expected_state(m, plan, actual, nominal, {k}, {});
    const ImpactAnalysis impact = dwafp::analyze_impact(m, plan, expected, actual, p);
    if (!impact.needs_repair()) {continue;}
    const auto unprotectable = dwafp::unprotectable_threats(m, plan, actual, p, impact, config);
    config.scope = dwafp::RepairScope::kAllUnfinished;
    const StageOneResult global = dwafp::generate_single_edits(m, plan, actual, p, impact, config);
    if (!unprotectable.empty()) {
      ++certified;
      EXPECT_FALSE(global.best) << "round " << round << ": certified infeasible, yet repaired\n" << yaml;
    } else {
      ++searched;
    }
  }
  EXPECT_GT(certified, 10);
  EXPECT_GT(searched, 10);
}

// Latency semantics shared with the simulator: a RUNNING task moved to another robot keeps its old
// robot busy until activation. robot1 runs T1 (persistently blocked, predicts 9 s) then X
// (independent). With 3 s of latency, moving T1 to robot3 at t = 2 s frees robot1 only at 5 s:
// X is predicted to start at 5 s, not at 2 s.
TEST(StageOne, MovedRunningTaskKeepsItsOldRobotBusyUntilActivation)
{
  const Mission m = inline_mission(
    "{mission_id: busy_until, subtasks: ["
    " {id: T1, robot: robot1, deadline_sec: 20},"
    " {id: X, robot: robot1, deadline_sec: 40},"
    " {id: T2, robot: robot2, deadline_sec: 14, predecessors: [T1]}]}");
  RuntimeState s = RuntimeState::initial(m);
  s.now_ms = 2000;
  set_executing(m, s, "T1", 0, 7000);
  s.tasks[m.task_index("T1")].same_robot_retry_feasible = false;
  dwafp::RepairConfig config;
  config.dispatch_overhead_ms = 3000;
  const Event e(m, s, {"T1"}, config);
  const RepairCandidate * move = e.find(RepairPolicy::kSubstitution, "T1", "robot3");
  ASSERT_TRUE(move);
  EXPECT_EQ(move->prediction.tasks[m.task_index("X")].start_ms, 5000);
  EXPECT_EQ(move->prediction.tasks[m.task_index("T1")].start_ms, 5000);  // released at 5 s on robot3
  e.expect_consistent();
}

