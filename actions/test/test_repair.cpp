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
  EXPECT_EQ(e.stage1.to_hold, (std::vector<Index>{m.task_index("T2")}));  // threatened, unstarted

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
  s.tasks[m.task_index("T1")].retry_available = transient;
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

// Same event, but the blockage is PERSISTENT (retry_available = false): retrying on robot1 would
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
  s.tasks[m.task_index("T1")].retry_available = false;  // persistent blockage
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
