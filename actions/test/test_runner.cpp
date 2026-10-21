// Unit tests of the experiment runner (Phase C6, Step 5).

#include <gtest/gtest.h>

#include <vector>

#include "actions/runner.hpp"
#include "actions/mission_loader.hpp"

using dwafp::ControllerConfig;
using dwafp::Method;
using dwafp::RunResult;
using dwafp::Scenario;
using dwafp::ScenarioConfig;

namespace
{

// The headline world of test_controllers: chain T1 (robot1, d 30 s) -> T2 (robot2, d 16 s), robot1
// persistently obstructed on T1 at 40 %, 2x execution timeout. Built as a Scenario by hand.
Scenario headline()
{
  const dwafp::Mission m = dwafp::mission_from_yaml(
    "{mission_id: chain, subtasks: [{id: T1, robot: robot1, deadline_sec: 30},"
    " {id: T2, robot: robot2, deadline_sec: 16, predecessors: [T1]}]}",
    "robots: [{name: robot1, step_duration_sec: 1.0}, {name: robot2, step_duration_sec: 1.5},"
    " {name: robot3, step_duration_sec: 0.8}]");
  dwafp::WorldConfig w;
  w.horizon_ms = 120000;
  w.timeout_multiplier = 2.0;
  dwafp::InjectedDeviation d{dwafp::DeviationType::kDelay, m.task_index("T1"), dwafp::kNoIndex, 0.4, 0, 0, true};
  w.deviations.push_back(d);
  dwafp::Plan plan = dwafp::Plan::initial(m);
  return Scenario{m, plan, w, d, dwafp::SourceClass::kCrossRobotHandoff, 12500, "headline"};
}

RunResult run(const Scenario & s, Method method)
{
  ControllerConfig c;
  c.method = method;
  return dwafp::run_method(s, c);
}

}  // namespace

// The metrics of the headline world, computed by hand (see test_controllers):
//   no intervention: T1 times out at 10 s, T2 never starts -> 2 misses, 2 never completed
//   reactive       : T1 -> robot3 10-14 s (on time), T2 restarted on robot3 16-20 s (4 s late)
//                    -> 1 miss, 0 never completed, tardiness 4 s, both tasks moved, 2 edits
//   DWAFP          : T1 -> robot3 3.5-7.5 s, T2 7.5-15 s -> success, 1 edit, 1 task moved
// Added delay w.r.t. the nominal plan (T1 0-5 s on robot1, T2 5-12.5 s on robot2), completed tasks:
//   no intervention 0 (nothing completed), reactive 9 + 7.5 = 16.5 s, DWAFP 2.5 + 2.5 = 5 s
TEST(Runner, MetricsOfTheHeadlineWorld)
{
  const Scenario s = headline();
  const RunResult none = run(s, Method::kNoIntervention);
  EXPECT_EQ(none.deadline_tasks, 2);
  EXPECT_EQ(none.deadline_misses, 2);
  EXPECT_EQ(none.never_completed, 2);
  EXPECT_FALSE(none.success);
  EXPECT_EQ(none.timeouts, 1);
  EXPECT_EQ(none.edits_scheduled, 0);
  EXPECT_EQ(none.added_delay_ms, 0);
  EXPECT_TRUE(none.deviation_realized);  // the obstruction was reached
  EXPECT_DOUBLE_EQ(none.deadline_miss_rate(), 1.0);
  EXPECT_DOUBLE_EQ(none.completion_rate(), 0.0);

  const RunResult reactive = run(s, Method::kReactive);
  EXPECT_EQ(reactive.deadline_misses, 1);
  EXPECT_EQ(reactive.never_completed, 0);
  EXPECT_EQ(reactive.completed_task_tardiness_ms, 4000);
  EXPECT_EQ(reactive.makespan_ms, 20000);
  EXPECT_EQ(reactive.final_assignment_changes, 2);
  EXPECT_EQ(reactive.edits_installed, 2);
  EXPECT_EQ(reactive.stats.fallback_repairs, 2);
  EXPECT_EQ(reactive.added_delay_ms, 16500);

  const RunResult dwafp = run(s, Method::kDwafp);
  EXPECT_TRUE(dwafp.success);
  EXPECT_EQ(dwafp.deadline_misses, 0);
  EXPECT_EQ(dwafp.completed_task_tardiness_ms, 0);
  EXPECT_EQ(dwafp.makespan_ms, 15000);
  EXPECT_EQ(dwafp.final_assignment_changes, 1);
  EXPECT_EQ(dwafp.edits_installed, 1);
  EXPECT_EQ(dwafp.stats.preventive_repairs, 1);
  EXPECT_EQ(dwafp.stats.preventive_downstream, 0);  // it moved the source T1 itself
  EXPECT_EQ(dwafp.added_delay_ms, 5000);
  EXPECT_TRUE(dwafp.deviation_realized);  // DWAFP reacted to the realized obstruction
  EXPECT_TRUE(dwafp.all_completed());
  EXPECT_EQ(dwafp.stats.decision_times_us.size(), static_cast<std::size_t>(dwafp.stats.analyses));
  EXPECT_GT(dwafp.stats.decision_us, 0.0);
}

// Pairing: running the same generated scenario twice under the same method gives the same
// outcome (the whole world is reproducible), so differences between methods come from them.
TEST(Runner, SameScenarioSameOutcome)
{
  ScenarioConfig c;
  c.noise = 0.1;
  for (std::uint64_t seed = 1; seed <= 20; ++seed) {
    const Scenario s = dwafp::make_scenario(c, seed);
    for (const Method method : {Method::kReactive, Method::kDwafp}) {
      const RunResult a = run(s, method);
      const RunResult b = run(s, method);
      EXPECT_EQ(a.deadline_misses, b.deadline_misses);
      EXPECT_EQ(a.completed_task_tardiness_ms, b.completed_task_tardiness_ms);
      EXPECT_EQ(a.makespan_ms, b.makespan_ms);
      EXPECT_EQ(a.edits_installed, b.edits_installed);
    }
  }
}

// Paired statistics: identical vectors are all ties with a zero interval; a constant improvement
// of 1 gives mean -1, interval [-1, -1] and only wins; a mixed case has the right counts, mean,
// and an interval that contains the mean; the result is deterministic for a given seed.
TEST(Runner, PairedComparison)
{
  const std::vector<double> base{3, 1, 4, 1, 5, 9, 2, 6};
  const auto same = dwafp::compare(base, base);
  EXPECT_EQ(same.ties, 8);
  EXPECT_EQ(same.mean_diff, 0.0);
  EXPECT_EQ(same.ci_lo, 0.0);
  EXPECT_EQ(same.ci_hi, 0.0);

  std::vector<double> better = base;
  for (double & v : better) {v -= 1;}
  const auto b = dwafp::compare(better, base);
  EXPECT_EQ(b.wins, 8);
  EXPECT_DOUBLE_EQ(b.mean_diff, -1.0);
  EXPECT_DOUBLE_EQ(b.ci_lo, -1.0);
  EXPECT_DOUBLE_EQ(b.ci_hi, -1.0);

  const std::vector<double> mixed{2, 1, 5, 1, 3, 9, 2, 4};  // diffs -1 0 +1 0 -2 0 0 -2
  const auto x = dwafp::compare(mixed, base, 7);
  EXPECT_EQ(x.wins, 3);
  EXPECT_EQ(x.ties, 4);
  EXPECT_EQ(x.losses, 1);
  EXPECT_DOUBLE_EQ(x.mean_diff, -0.5);
  EXPECT_LE(x.ci_lo, x.mean_diff);
  EXPECT_GE(x.ci_hi, x.mean_diff);
  const auto again = dwafp::compare(mixed, base, 7);
  EXPECT_EQ(again.ci_lo, x.ci_lo);
  EXPECT_EQ(again.ci_hi, x.ci_hi);
  EXPECT_THROW(dwafp::compare({1, 2}, {1}), std::invalid_argument);
}

// Cluster bootstrap. With one row per cluster it is compare(). When every row is repeated in 3
// cells (the same seeds reused, as in pooled factor cells), treating the 3x rows as independent
// gives a NARROWER interval than the data support; the cluster bootstrap gives the interval of
// the 8 seeds, i.e. the same as one row per seed. Wins / ties / losses still count rows.
TEST(Runner, ClusteredComparison)
{
  const std::vector<double> base{3, 1, 4, 1, 5, 9, 2, 6};
  const std::vector<double> mixed{2, 1, 5, 1, 3, 9, 2, 4};  // diffs -1 0 +1 0 -2 0 0 -2
  std::vector<std::uint64_t> ids{1, 2, 3, 4, 5, 6, 7, 8};
  const auto plain = dwafp::compare(mixed, base, 7, 10000);
  const auto one_per_cluster = dwafp::compare_clustered(mixed, base, ids, 7, 10000);
  EXPECT_DOUBLE_EQ(one_per_cluster.mean_diff, plain.mean_diff);
  EXPECT_DOUBLE_EQ(one_per_cluster.ci_lo, plain.ci_lo);
  EXPECT_DOUBLE_EQ(one_per_cluster.ci_hi, plain.ci_hi);

  std::vector<double> m3, b3;
  std::vector<std::uint64_t> ids3;
  for (int cell = 0; cell < 3; ++cell) {  // the same 8 seeds in 3 factor cells
    m3.insert(m3.end(), mixed.begin(), mixed.end());
    b3.insert(b3.end(), base.begin(), base.end());
    ids3.insert(ids3.end(), ids.begin(), ids.end());
  }
  const auto naive = dwafp::compare(m3, b3, 7, 10000);
  const auto clustered = dwafp::compare_clustered(m3, b3, ids3, 7, 10000);
  EXPECT_DOUBLE_EQ(clustered.mean_diff, -0.5);
  EXPECT_DOUBLE_EQ(clustered.ci_lo, plain.ci_lo);   // the information of 8 seeds, not 24 runs
  EXPECT_DOUBLE_EQ(clustered.ci_hi, plain.ci_hi);
  EXPECT_GT(clustered.ci_hi - clustered.ci_lo, naive.ci_hi - naive.ci_lo);  // naive is too narrow
  EXPECT_EQ(clustered.n, 24);
  EXPECT_EQ(clustered.wins, 9);
  EXPECT_EQ(clustered.losses, 3);
  EXPECT_THROW(dwafp::compare_clustered(mixed, base, {1, 2}), std::invalid_argument);
}
