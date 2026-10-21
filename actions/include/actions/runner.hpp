// Experiment runner for the closed-loop evaluation (Phase C6, Step 5).
//
// run_method() runs ONE scenario under ONE method and measures what really happened. Because a
// scenario fixes the mission, the plan, the deviation and the noise (keyed by seed, task, robot
// and attempt), running it under every method gives PAIRED observations: the only difference
// between two runs of the same scenario is the method.
//
// compare() summarises paired observations of one metric (lower is better) for a method against a
// baseline: wins / ties / losses, the mean difference (method - baseline), and a bootstrap 95 %
// confidence interval of that mean (percentile method, fixed seed, so it is reproducible).

#ifndef ACTIONS__RUNNER_HPP_
#define ACTIONS__RUNNER_HPP_

#include <cstdint>
#include <vector>

#include "actions/controllers.hpp"
#include "actions/scenarios.hpp"

namespace dwafp
{

// Outcome of one run (one scenario, one method).
struct RunResult
{
  // Outcomes (what really happened).
  int total_tasks{0};
  int completed_tasks{0};
  int deadline_tasks{0};       // tasks that have a deadline
  int deadline_misses{0};      // ... not completed by their deadline (never completed included)
  int never_completed{0};      // tasks not completed at the end of the run (any task)
  bool success{false};         // every task completed and every deadline met
  // Sum over COMPLETED tasks with a deadline of max(0, end - deadline). Unfinished tasks are not in
  // it (a method leaving tasks unfinished must not look good): read it with never_completed.
  TimeMs completed_task_tardiness_ms{0};
  // Latest completion time. Comparable only between runs where EVERY task completed.
  TimeMs makespan_ms{0};
  int timeouts{0};             // attempts ended by the execution timeout
  // Tasks whose FINAL robot differs from the initial plan (not the number of reassignments: a
  // task moved away and back counts 0; installed edits count every operation).
  int final_assignment_changes{0};
  bool deviation_realized{false};  // every injected deviation actually happened in this run
  // Sum over COMPLETED tasks of max(0, end - nominal finish in the initial plan): the delay the
  // run added to the workflow (the deviation's own delay plus what the method did). Unfinished
  // tasks are not in it (read it with never_completed), as for the tardiness.
  TimeMs added_delay_ms{0};

  double deadline_miss_rate() const {return deadline_tasks ? double(deadline_misses) / deadline_tasks : 0.0;}
  double completion_rate() const {return total_tasks ? double(completed_tasks) / total_tasks : 0.0;}
  bool all_completed() const {return completed_tasks == total_tasks;}
  // Cost of intervening.
  int edits_scheduled{0};
  int edits_installed{0};
  int edits_rejected{0};       // task completed before activation, or invalid
  TimeMs discarded_work_ms{0};
  TimeMs wasted_elapsed_ms{0};
  // What the controller did.
  ControllerStats stats;
};

// Runs `scenario` under the method of `config` (a fresh controller and a fresh world).
RunResult run_method(const Scenario & scenario, const ControllerConfig & config);

// Paired comparison of one metric, lower is better, method[i] and baseline[i] from the same
// scenario. A win is method < baseline.
struct PairedSummary
{
  int n{0};
  int wins{0};
  int ties{0};
  int losses{0};
  double mean_diff{0};  // mean of method - baseline (negative = the method is better)
  double ci_lo{0};      // bootstrap 95 % confidence interval of mean_diff
  double ci_hi{0};
};

// Throws std::invalid_argument if the two vectors have different sizes.
PairedSummary compare(const std::vector<double> & method, const std::vector<double> & baseline,
  std::uint64_t seed = 1, int resamples = 2000);

// The same with CLUSTERED observations: pair i belongs to cluster[i]. Use it when rows that share a
// cluster are not independent, e.g. pooled factor cells that reuse the same scenario seeds (the
// seed is the cluster). The paired differences are first averaged per cluster; the bootstrap then
// resamples those cluster means (cluster bootstrap), and mean_diff is their mean (the pooled mean
// when every cluster has the same number of rows). n, wins, ties and losses still count rows.
// With one row per cluster it equals compare() on the same rows. Throws std::invalid_argument if
// the three vectors have different sizes.
PairedSummary compare_clustered(const std::vector<double> & method, const std::vector<double> & baseline,
  const std::vector<std::uint64_t> & cluster, std::uint64_t seed = 1, int resamples = 10000);

}  // namespace dwafp

#endif  // ACTIONS__RUNNER_HPP_
