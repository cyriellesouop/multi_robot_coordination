// Experiment runner for the closed-loop evaluation. See include/actions/runner.hpp.

#include "actions/runner.hpp"

#include <algorithm>  // std::max, std::sort
#include <map>
#include <random>
#include <stdexcept>

namespace dwafp
{

RunResult run_method(const Scenario & scenario, const ControllerConfig & config)
{
  const Mission & m = scenario.mission;
  MethodController controller(m, scenario.plan, config);
  Simulator sim(m, scenario.plan, scenario.world);
  const std::vector<TaskRecord> & rec = sim.run(&controller);

  // Nominal prediction of the initial plan (no deviation, nominal durations): the reference of
  // added_delay_ms.
  const RuntimeState s0 = RuntimeState::initial(m);
  const DurationSnapshot p0 = DurationSnapshot::capture(m, ConfigDurationModel{}, s0);
  const Prediction nominal = Predictor(m, scenario.plan, s0, p0).full();

  RunResult r;
  r.total_tasks = static_cast<int>(m.task_count());
  for (Index j = 0; j < m.task_count(); ++j) {
    const TaskRecord & t = rec[j];
    const bool completed = t.status == FinalStatus::kCompleted;
    if (completed) {
      ++r.completed_tasks;
      r.makespan_ms = std::max(r.makespan_ms, t.end_ms);
      r.added_delay_ms += std::max<TimeMs>(0, t.end_ms - nominal.tasks[j].finish_ms);
    } else {
      ++r.never_completed;
    }
    if (has_deadline(m, j)) {
      ++r.deadline_tasks;
      if (!t.deadline_ok) {++r.deadline_misses;}
      if (completed) {r.completed_task_tardiness_ms += std::max<TimeMs>(0, t.end_ms - m.task(j).deadline_ms);}
    }
    r.timeouts += t.timeouts;
    // A task that never ran keeps robot = kNoIndex: it did not move.
    if (t.robot != kNoIndex && t.robot != scenario.plan.robot_of(j)) {++r.final_assignment_changes;}
    r.discarded_work_ms += t.discarded_work_ms;
    r.wasted_elapsed_ms += t.wasted_elapsed_ms;
  }
  r.success = r.never_completed == 0 && r.deadline_misses == 0;
  const EditCounts & e = sim.edit_counts();
  r.edits_scheduled = e.scheduled;
  r.edits_installed = e.installed;
  r.edits_rejected = e.rejected_task_completed + e.rejected_invalid;
  r.deviation_realized = true;
  for (const bool realized : sim.deviations_realized()) {r.deviation_realized = r.deviation_realized && realized;}
  r.stats = controller.stats();
  return r;
}

PairedSummary compare(const std::vector<double> & method, const std::vector<double> & baseline,
  std::uint64_t seed, int resamples)
{
  if (method.size() != baseline.size()) {throw std::invalid_argument("compare: different sizes");}
  PairedSummary s;
  s.n = static_cast<int>(method.size());
  if (s.n == 0) {return s;}
  std::vector<double> diff(method.size());
  double sum = 0;
  for (std::size_t i = 0; i < method.size(); ++i) {
    diff[i] = method[i] - baseline[i];
    sum += diff[i];
    if (diff[i] < 0) {++s.wins;} else if (diff[i] > 0) {++s.losses;} else {++s.ties;}
  }
  s.mean_diff = sum / s.n;

  // Bootstrap: resample the paired differences with replacement, take the 2.5 % and 97.5 %
  // percentiles of the resampled means.
  std::mt19937_64 rng(seed);
  std::uniform_int_distribution<std::size_t> pick(0, diff.size() - 1);
  std::vector<double> means(static_cast<std::size_t>(std::max(1, resamples)));
  for (double & mean : means) {
    double acc = 0;
    for (std::size_t i = 0; i < diff.size(); ++i) {acc += diff[pick(rng)];}
    mean = acc / s.n;
  }
  std::sort(means.begin(), means.end());
  const auto at = [&](double q) {
      const auto k = static_cast<std::size_t>(q * static_cast<double>(means.size() - 1));
      return means[k];
    };
  s.ci_lo = at(0.025);
  s.ci_hi = at(0.975);
  return s;
}

PairedSummary compare_clustered(const std::vector<double> & method, const std::vector<double> & baseline,
  const std::vector<std::uint64_t> & cluster, std::uint64_t seed, int resamples)
{
  if (method.size() != baseline.size() || method.size() != cluster.size()) {
    throw std::invalid_argument("compare_clustered: different sizes");
  }
  // Mean paired difference of every cluster (ordered by cluster id, so the result is deterministic).
  std::map<std::uint64_t, std::pair<double, int>> per_cluster;  // id -> (sum of differences, rows)
  for (std::size_t i = 0; i < method.size(); ++i) {
    auto & c = per_cluster[cluster[i]];
    c.first += method[i] - baseline[i];
    ++c.second;
  }
  std::vector<double> cluster_means;
  for (const auto & [id, c] : per_cluster) {cluster_means.push_back(c.first / c.second);}
  // Bootstrap of the cluster means: compare() of the means against 0 does exactly that.
  PairedSummary s = compare(cluster_means, std::vector<double>(cluster_means.size(), 0.0), seed, resamples);
  // Wins, ties and losses (and n) describe the individual paired runs, not the clusters.
  const PairedSummary rows = compare(method, baseline, seed, 1);
  s.n = rows.n;
  s.wins = rows.wins;
  s.ties = rows.ties;
  s.losses = rows.losses;
  return s;
}

}  // namespace dwafp
