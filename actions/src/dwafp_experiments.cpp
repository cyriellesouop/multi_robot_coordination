// DWAFP closed-loop experiments, ROS-free (Phase C6, Step 5): the main comparison.
//
// Every scenario is executed in the simulator under the five methods (paired comparison):
//   no intervention, reactive, source-only, DWAFP, mission-wide.
// Results are reported PER DEVIATION TYPE (transient delay, persistent obstruction, outcome
// failure, robot breakdown), then for the default mix as an overall workload. Expected reading:
//   - delays test the PREVENTIVE benefit (DWAFP acts before a timeout or a deadline);
//   - unannounced failures and breakdowns test the shared recovery, so reactive and the
//     preventive methods should be close there (an informative result, not a weakness);
//   - by source class, a gain over source-only repair would appear where consequences cross
//     robots or reach downstream tasks.
//
// Primary metric: deadline misses (tasks with a deadline not completed by it; lower is better).
// For each comparison: mean paired difference (DWAFP - baseline) with a bootstrap 95 % interval,
// and wins / ties / losses over the scenarios.
//
// CONTROLLED configuration (this program; robustness to noise, latency, estimator, timeout and
// scale is Step 6): 4 robots (1.0 / 1.5 / 0.8 / 1.0 s per unit), 20 tasks, the four workflow
// shapes in turn (seed mod 4), deadlines on every task, duration-normalized slack 0.5..3, NO
// duration noise (so every method experiences exactly the injected deviation), 2x execution
// timeout, no dispatch latency, NOMINAL remaining-time estimate (r~ = r_hat; a persistent
// obstruction reported by the robot makes the task unreachable on it), trigger k = 3,
// epsilon = 0, tau = 2 s. The same seeds for every deviation type (same graphs).
//
// Usage:   ros2 run actions dwafp_experiments [runs_per_type] [csv_file] [noise]
// Default: 200 runs per deviation type, no CSV, noise 0 (the controlled configuration). A noise
// level > 0 (e.g. 0.1) runs the same comparison under duration noise (robustness).

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "actions/runner.hpp"
#include "actions/scenarios.hpp"

using namespace dwafp;

namespace
{

const Method kMethods[] = {Method::kNoIntervention, Method::kReactive, Method::kSourceOnly,
  Method::kDwafp, Method::kMissionWide};
constexpr int kMethodCount = 5;
constexpr int kNone = 0;
constexpr int kDwafp = 3;

// The controlled configuration, set explicitly (and recorded in the CSV).
ControllerConfig controller_config(Method method)
{
  ControllerConfig c;
  c.method = method;
  c.estimate = BlockageEstimate::kNominal;
  c.decision_ms = 0;
  c.dispatch_ms = 0;
  c.trigger.persistence_updates = 3;
  c.trigger.deviation_tolerance_ms = 0;
  c.trigger.blocked_threshold_ms = 2000;
  return c;
}

const char * estimate_name(BlockageEstimate e) {return e == BlockageEstimate::kNominal ? "nominal" : "conservative";}

struct Row
{
  std::uint64_t seed{0};
  SourceClass source_class{SourceClass::kNotApplicable};
  RunResult result[kMethodCount];
};

double percentile(std::vector<double> v, double q)
{
  if (v.empty()) {return 0.0;}
  std::sort(v.begin(), v.end());
  return v[static_cast<std::size_t>(q * static_cast<double>(v.size() - 1))];
}

void write_csv_header(std::ofstream & csv)
{
  csv << "experiment,seed,shape,deadlines,robots,tasks,noise,timeout_multiplier,estimate,latency_ms,"
         "deviation,source_class,method,deviation_realized,total_tasks,completed_tasks,deadline_tasks,"
         "deadline_misses,never_completed,success,completed_task_tardiness_ms,makespan_ms,timeouts,"
         "final_assignment_changes,edits_scheduled,edits_installed,edits_rejected,discarded_work_ms,"
         "wasted_elapsed_ms,trigger_events,analyses,preventive_repairs,no_feasible_local,no_single_edit,"
         "fallback_events,fallback_repairs,errors,decision_us,max_decision_us\n";
}

void write_csv_row(std::ofstream & csv, const std::string & experiment, const Scenario & s,
  const ScenarioConfig & c, const ControllerConfig & cc, const std::string & deviation, const RunResult & r)
{
  const ControllerStats & st = r.stats;
  csv << experiment << ',' << s.world.seed << ',' << shape_name(c.shape) << ','
      << deadline_structure_name(c.deadlines) << ',' << c.robots << ',' << c.tasks << ',' << c.noise << ','
      << c.timeout_multiplier << ',' << estimate_name(cc.estimate) << ',' << cc.decision_ms + cc.dispatch_ms
      << ',' << deviation << ',' << source_class_name(s.source_class) << ',' << method_name(cc.method) << ','
      << r.deviation_realized << ',' << r.total_tasks << ',' << r.completed_tasks << ',' << r.deadline_tasks
      << ',' << r.deadline_misses << ',' << r.never_completed << ',' << r.success << ','
      << r.completed_task_tardiness_ms << ',' << r.makespan_ms << ',' << r.timeouts << ','
      << r.final_assignment_changes << ',' << r.edits_scheduled << ',' << r.edits_installed << ','
      << r.edits_rejected << ',' << r.discarded_work_ms << ',' << r.wasted_elapsed_ms << ','
      << st.trigger_events << ',' << st.analyses << ',' << st.preventive_repairs << ','
      << st.no_feasible_local << ',' << st.no_single_edit << ',' << st.fallback_events << ','
      << st.fallback_repairs << ',' << st.errors << ',' << st.decision_us << ',' << st.max_decision_us << '\n';
}

// Outcomes per method (means per run; rates normalized by the tasks concerned).
void print_outcomes(const std::vector<Row> & rows)
{
  std::printf("  %-16s %7s %8s %8s %7s %6s %9s %11s %8s\n", "method", "success", "missRate", "complete",
    "misses", "never", "tardy(s)*", "makespan**", "realized");
  for (int k = 0; k < kMethodCount; ++k) {
    double success = 0, miss_rate = 0, completion = 0, misses = 0, never = 0, tardy = 0, realized = 0;
    double makespan = 0;
    int complete_runs = 0;
    for (const Row & row : rows) {
      const RunResult & r = row.result[k];
      success += r.success ? 1 : 0;
      miss_rate += r.deadline_miss_rate();
      completion += r.completion_rate();
      misses += r.deadline_misses;
      never += r.never_completed;
      tardy += static_cast<double>(r.completed_task_tardiness_ms) / 1000.0;
      realized += r.deviation_realized ? 1 : 0;
      if (r.all_completed()) {
        makespan += static_cast<double>(r.makespan_ms) / 1000.0;
        ++complete_runs;
      }
    }
    const double n = static_cast<double>(std::max<std::size_t>(1, rows.size()));
    std::printf("  %-16s %6.0f%% %7.1f%% %7.1f%% %7.2f %6.2f %9.2f %6.1f (%3d) %7.0f%%\n",
      method_name(kMethods[k]), 100 * success / n, 100 * miss_rate / n, 100 * completion / n, misses / n,
      never / n, tardy / n, complete_runs ? makespan / complete_runs : 0.0, complete_runs, 100 * realized / n);
  }
  std::printf("  * tardiness of COMPLETED tasks only (read with 'never')   ** mean over the runs in which every task completed (count)\n");
}

// What each method did, what it cost, and whether its edits were useful.
void print_interventions(const std::vector<Row> & rows)
{
  std::printf("  %-16s %6s %7s %7s %7s %8s %8s %11s %8s %8s %8s\n", "method", "edits", "prevent", "recover",
    "finalMv", "harmful", "unneeded", "decisions", "p50(us)", "p95(us)", "p99(us)");
  for (int k = 1; k < kMethodCount; ++k) {
    double edits = 0, prevent = 0, recover = 0, moved = 0;
    int harmful = 0, unneeded = 0;
    std::vector<double> times;
    for (const Row & row : rows) {
      const RunResult & r = row.result[k];
      const RunResult & none = row.result[kNone];
      edits += r.edits_installed;
      prevent += r.stats.preventive_repairs;
      recover += r.stats.fallback_repairs;
      moved += r.final_assignment_changes;
      // Harmful: more misses than doing nothing at all. Unneeded: an edit in a run that would have
      // succeeded without any intervention.
      if (r.deadline_misses > none.deadline_misses) {++harmful;}
      if (r.edits_installed > 0 && none.success) {++unneeded;}
      times.insert(times.end(), r.stats.decision_times_us.begin(), r.stats.decision_times_us.end());
    }
    const double n = static_cast<double>(std::max<std::size_t>(1, rows.size()));
    std::printf("  %-16s %6.2f %7.2f %7.2f %7.2f %8d %8d %11zu %8.0f %8.0f %8.0f\n", method_name(kMethods[k]),
      edits / n, prevent / n, recover / n, moved / n, harmful, unneeded, times.size(),
      percentile(times, 0.5), percentile(times, 0.95), percentile(times, 0.99));
  }
}

// DWAFP against each other method, on deadline misses and on tasks never completed.
void print_comparisons(const std::vector<Row> & rows)
{
  std::vector<double> dwafp_misses, dwafp_never;
  int realized_everywhere = 0;
  for (const Row & row : rows) {
    dwafp_misses.push_back(row.result[kDwafp].deadline_misses);
    dwafp_never.push_back(row.result[kDwafp].never_completed);
    bool all = true;
    for (const RunResult & r : row.result) {all = all && r.deviation_realized;}
    realized_everywhere += all ? 1 : 0;
  }
  std::printf("  deviation realized under every method in %d / %zu scenarios\n", realized_everywhere, rows.size());
  std::printf("  DWAFP vs           misses: mean diff [95%% CI]   W / T / L      never: mean diff [95%% CI]\n");
  for (int k = 0; k < kMethodCount; ++k) {
    if (k == kDwafp) {continue;}
    std::vector<double> misses, never;
    for (const Row & row : rows) {
      misses.push_back(row.result[k].deadline_misses);
      never.push_back(row.result[k].never_completed);
    }
    const PairedSummary a = compare(dwafp_misses, misses);
    const PairedSummary b = compare(dwafp_never, never);
    std::printf("    %-16s %+6.2f [%+6.2f, %+6.2f]  %3d / %3d / %3d   %+6.2f [%+6.2f, %+6.2f]\n",
      method_name(kMethods[k]), a.mean_diff, a.ci_lo, a.ci_hi, a.wins, a.ties, a.losses, b.mean_diff,
      b.ci_lo, b.ci_hi);
  }
}

// DWAFP against reactive and source-only, per source class (where does DWAFP gain?).
void print_by_source_class(const std::vector<Row> & rows)
{
  std::map<SourceClass, std::vector<const Row *>> by;
  for (const Row & row : rows) {by[row.source_class].push_back(&row);}
  std::printf("  by source class          n   DWAFP-reactive misses [95%% CI]   DWAFP-source-only misses [95%% CI]\n");
  for (const auto & [cls, list] : by) {
    std::vector<double> d, r, s;
    for (const Row * row : list) {
      d.push_back(row->result[kDwafp].deadline_misses);
      r.push_back(row->result[1].deadline_misses);
      s.push_back(row->result[2].deadline_misses);
    }
    const PairedSummary vr = compare(d, r);
    const PairedSummary vs = compare(d, s);
    std::printf("  %-22s %4zu   %+6.2f [%+6.2f, %+6.2f]          %+6.2f [%+6.2f, %+6.2f]\n",
      source_class_name(cls), list.size(), vr.mean_diff, vr.ci_lo, vr.ci_hi, vs.mean_diff, vs.ci_lo, vs.ci_hi);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  const int runs = argc > 1 ? std::max(1, std::atoi(argv[1])) : 200;
  const double noise = argc > 3 ? std::atof(argv[3]) : 0.0;
  if (!(noise >= 0.0 && noise < 1.0)) {
    std::fprintf(stderr, "noise must be in [0, 1)\n");
    return 1;
  }
  std::ofstream csv_file;
  std::ofstream * csv = nullptr;
  if (argc > 2 && std::string(argv[2]) != "-") {  // "-" = no CSV (to give a noise level only)
    csv_file.open(argv[2]);
    if (!csv_file) {
      std::fprintf(stderr, "cannot write %s\n", argv[2]);
      return 1;
    }
    csv = &csv_file;
    write_csv_header(*csv);
  }

  struct Type
  {
    const char * name;
    DeviationMix mix;
  };
  const Type types[] = {
    {"transient delay", {1, 0, 0, 0}},
    {"persistent obstruction", {0, 1, 0, 0}},
    {"outcome failure", {0, 0, 1, 0}},
    {"robot breakdown", {0, 0, 0, 1}},
    {"mix 35/25/25/15", DeviationMix{}},
  };
  const WorkflowShape shapes[] = {WorkflowShape::kRandomDag, WorkflowShape::kChains,
    WorkflowShape::kForkJoin, WorkflowShape::kRendezvous};

  const ControllerConfig shown = controller_config(Method::kDwafp);
  std::printf("DWAFP closed-loop comparison, %s configuration: %d paired runs per deviation type\n",
    noise == 0.0 ? "controlled" : "robustness", runs);
  std::printf("4 robots, 20 tasks, all four shapes, deadlines on every task, slack 0.5-3 x duration, noise %.0f%%,\n"
    "2x execution timeout, latency %lld ms, %s estimate, trigger k=%d eps=%lld ms tau=%lld ms\n", 100 * noise,
    static_cast<long long>(shown.decision_ms + shown.dispatch_ms), estimate_name(shown.estimate),
    shown.trigger.persistence_updates, static_cast<long long>(shown.trigger.deviation_tolerance_ms),
    static_cast<long long>(shown.trigger.blocked_threshold_ms));
  std::printf("(persistent obstruction: the robot reports that same-robot retry is not feasible; the\n"
    " preventive methods use this diagnostic before the execution timeout)\n");
  int errors = 0;
  for (const Type & type : types) {
    std::vector<Row> rows;
    for (int i = 1; i <= runs; ++i) {
      ScenarioConfig c;
      c.mix = type.mix;
      c.noise = noise;
      c.shape = shapes[static_cast<std::size_t>(i % 4)];
      const auto seed = static_cast<std::uint64_t>(i);
      const Scenario s = make_scenario(c, seed);
      Row row;
      row.seed = seed;
      row.source_class = s.source_class;
      for (int k = 0; k < kMethodCount; ++k) {
        const ControllerConfig cc = controller_config(kMethods[k]);
        row.result[k] = run_method(s, cc);
        errors += row.result[k].stats.errors;
        if (csv) {write_csv_row(*csv, noise == 0.0 ? "main" : "noise", s, c, cc, type.name, row.result[k]);}
      }
      rows.push_back(row);
    }
    std::printf("\n=== %s ===\n", type.name);
    print_outcomes(rows);
    print_interventions(rows);
    print_comparisons(rows);
    if (type.mix.robot_breakdown == 0) {print_by_source_class(rows);}
  }
  std::printf("\ncontroller errors (must be 0): %d\n", errors);
  if (csv) {std::printf("CSV written to %s\n", argv[2]);}
  return errors == 0 ? 0 : 1;
}
