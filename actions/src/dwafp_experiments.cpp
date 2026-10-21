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
// Usage, main comparison:
//   ros2 run actions dwafp_experiments [runs_per_type] [csv_file] [noise]
//   default: 200 runs per deviation type, no CSV, noise 0 (the controlled configuration); a noise
//   level > 0 (e.g. 0.1) runs the same comparison under duration noise (robustness).
// Usage, Step 6 sweeps (one factor changed at a time, everything else as in the controlled
// configuration, the same paired scenarios for every setting):
//   ros2 run actions dwafp_experiments <experiment> [runs] [csv_file]
//   deadlines : deadlines on every task vs on handoff and final tasks only; overall and per shape
//   noise     : duration noise 0, 5, 10, 20 %
//   latency   : dispatch latency 0, 1, 3 s
//   estimator : nominal vs conservative remaining-time estimate while blocked
//   timeout   : execution timeout 1.5, 2, 3 x the nominal duration
//   sweeps    : all of the above
// Usage, repair-location study (its own configuration, see exp_location):
//   ros2 run actions dwafp_experiments location [runs] [csv_file|-] [latency_ms]   (default 0 ms)

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>  // std::begin, std::end
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
         "fallback_events,fallback_repairs,errors,decision_us,max_decision_us,"
         "progress_lo,progress_hi,execution_time_ratio,spare_robots,busy_fleet,added_delay_ms,preventive_downstream,"
         "preventive_installed,preventive_downstream_installed\n";
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
      << st.fallback_repairs << ',' << st.errors << ',' << st.decision_us << ',' << st.max_decision_us << ','
      << c.progress_lo << ',' << c.progress_hi << ',' << c.execution_time_ratio << ',' << c.spare_robots << ','
      << c.busy_fleet << ',' << r.added_delay_ms << ',' << st.preventive_downstream << ','
      << st.preventive_installed << ',' << st.preventive_downstream_installed << '\n';
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

// =============================================================================================
// Step 6: sweeps (one factor at a time) with a compact one-line summary per setting
// =============================================================================================

struct DevType
{
  const char * name;
  DeviationMix mix;
};
const DevType kTransient{"transient delay", {1, 0, 0, 0}};
const DevType kPersistent{"persistent obstruction", {0, 1, 0, 0}};
const DevType kFailure{"outcome failure", {0, 0, 1, 0}};
const DevType kBreakdown{"robot breakdown", {0, 0, 0, 1}};
const DevType kMix{"mix 35/25/25/15", DeviationMix{}};
const WorkflowShape kShapes[] = {WorkflowShape::kRandomDag, WorkflowShape::kChains,
  WorkflowShape::kForkJoin, WorkflowShape::kRendezvous};
const std::vector<WorkflowShape> kAllShapes(std::begin(kShapes), std::end(kShapes));

// Runs `runs` paired scenarios (seeds 1..runs) of one setting for one deviation type: every
// scenario under the five methods. The shapes of `shapes` are used in turn (seed mod their
// number; kAllShapes = the four shapes, as in the main comparison); empty = the shape of `base`.
std::vector<Row> run_setting(const std::string & experiment, ScenarioConfig base, const ControllerConfig & controller,
  const DevType & type, int runs, const std::vector<WorkflowShape> & shapes, std::ofstream * csv, int & errors)
{
  std::vector<Row> rows;
  for (int i = 1; i <= runs; ++i) {
    ScenarioConfig c = base;
    c.mix = type.mix;
    if (!shapes.empty()) {c.shape = shapes[static_cast<std::size_t>(i) % shapes.size()];}
    const auto seed = static_cast<std::uint64_t>(i);
    const Scenario s = make_scenario(c, seed);
    Row row;
    row.seed = seed;
    row.source_class = s.source_class;
    for (int k = 0; k < kMethodCount; ++k) {
      ControllerConfig cc = controller;
      cc.method = kMethods[k];
      row.result[k] = run_method(s, cc);
      errors += row.result[k].stats.errors;
      if (csv) {write_csv_row(*csv, experiment, s, c, cc, type.name, row.result[k]);}
    }
    rows.push_back(row);
  }
  return rows;
}

void summary_header(const char * factor)
{
  std::printf("  %-24s %-23s %-13s %-11s %-34s %-34s %-6s %s\n", factor, "deviation", "success R/S/D",
    "missRate R/D", "DWAFP-reactive [95% CI] W/T/L", "DWAFP-source-only [95% CI] W/T/L", "D-M", "harm/unneeded");
}

// One line: the setting, then the key outcomes and the paired comparisons of DWAFP.
void summary_row(const std::string & label, const DevType & type, const std::vector<Row> & rows)
{
  const auto mean = [&](int k, auto metric) {
      double acc = 0;
      for (const Row & row : rows) {acc += metric(row.result[k]);}
      return acc / static_cast<double>(std::max<std::size_t>(1, rows.size()));
    };
  const auto success = [](const RunResult & r) {return r.success ? 100.0 : 0.0;};
  const auto rate = [](const RunResult & r) {return 100.0 * r.deadline_miss_rate();};
  const auto misses_of = [&](int k) {
      std::vector<double> v;
      for (const Row & row : rows) {v.push_back(row.result[k].deadline_misses);}
      return v;
    };
  const PairedSummary vr = compare(misses_of(kDwafp), misses_of(1));
  const PairedSummary vs = compare(misses_of(kDwafp), misses_of(2));
  const PairedSummary vm = compare(misses_of(kDwafp), misses_of(4));
  int harmful = 0, unneeded = 0;
  for (const Row & row : rows) {
    const RunResult & d = row.result[kDwafp];
    const RunResult & none = row.result[kNone];
    if (d.deadline_misses > none.deadline_misses) {++harmful;}
    if (d.edits_installed > 0 && none.success) {++unneeded;}
  }
  char a[64], b[64];
  std::snprintf(a, sizeof a, "%+5.2f [%+5.2f,%+5.2f] %3d/%3d/%d", vr.mean_diff, vr.ci_lo, vr.ci_hi, vr.wins, vr.ties, vr.losses);
  std::snprintf(b, sizeof b, "%+5.2f [%+5.2f,%+5.2f] %3d/%3d/%d", vs.mean_diff, vs.ci_lo, vs.ci_hi, vs.wins, vs.ties, vs.losses);
  std::printf("  %-24s %-23s %3.0f/%3.0f/%3.0f   %4.1f/%4.1f   %-34s %-34s %+5.2f  %d/%d\n", label.c_str(), type.name,
    mean(1, success), mean(2, success), mean(kDwafp, success), mean(1, rate), mean(kDwafp, rate), a, b,
    vm.mean_diff, harmful, unneeded);
}

ScenarioConfig controlled_scenario()
{
  ScenarioConfig c;   // 4 robots, 20 tasks, every-task deadlines, slack 0.5-3, 2x timeout
  c.noise = 0.0;
  return c;
}

void exp_deadlines(int runs, std::ofstream * csv, int & errors)
{
  std::printf("\n### deadlines: on every task vs on handoff and final tasks only (all shapes in turn)\n");
  summary_header("deadline structure");
  for (const DeadlineStructure d : {DeadlineStructure::kEveryTask, DeadlineStructure::kHandoffAndFinal}) {
    for (const DevType & t : {kTransient, kPersistent, kFailure, kBreakdown}) {
      ScenarioConfig c = controlled_scenario();
      c.deadlines = d;
      summary_row(deadline_structure_name(d), t,
        run_setting(std::string("deadlines:") + deadline_structure_name(d), c, controller_config(Method::kDwafp), t, runs, kAllShapes, csv, errors));
    }
  }
  std::printf("\n### deadlines per workflow shape (delays only)\n");
  summary_header("shape / deadlines");
  for (const WorkflowShape shape : kShapes) {
    for (const DeadlineStructure d : {DeadlineStructure::kEveryTask, DeadlineStructure::kHandoffAndFinal}) {
      for (const DevType & t : {kTransient, kPersistent}) {
        ScenarioConfig c = controlled_scenario();
        c.shape = shape;
        c.deadlines = d;
        const std::string label = std::string(shape_name(shape)) + " / " + deadline_structure_name(d);
        summary_row(label, t, run_setting("deadlines-shape:" + label, c, controller_config(Method::kDwafp), t, runs, {}, csv, errors));
      }
    }
  }
}

void exp_noise(int runs, std::ofstream * csv, int & errors)
{
  std::printf("\n### noise: duration noise of every attempt\n");
  summary_header("noise");
  for (const double noise : {0.0, 0.05, 0.10, 0.20}) {
    for (const DevType & t : {kTransient, kPersistent, kFailure, kBreakdown}) {
      ScenarioConfig c = controlled_scenario();
      c.noise = noise;
      const std::string label = std::to_string(static_cast<int>(100 * noise + 0.5)) + " %";
      summary_row(label, t, run_setting("noise:" + label, c, controller_config(Method::kDwafp), t, runs, kAllShapes, csv, errors));
    }
  }
}

void exp_latency(int runs, std::ofstream * csv, int & errors)
{
  std::printf("\n### latency: dispatch latency between the decision and its activation (every method)\n");
  summary_header("latency");
  for (const TimeMs latency : {0, 1000, 3000}) {
    for (const DevType & t : {kTransient, kPersistent}) {
      ControllerConfig cc = controller_config(Method::kDwafp);
      cc.dispatch_ms = latency;
      const std::string label = std::to_string(latency / 1000) + " s";
      summary_row(label, t, run_setting("latency:" + label, controlled_scenario(), cc, t, runs, kAllShapes, csv, errors));
    }
  }
}

void exp_estimator(int runs, std::ofstream * csv, int & errors)
{
  std::printf("\n### estimator: remaining-time estimate while blocked (every method)\n");
  summary_header("estimator");
  for (const BlockageEstimate e : {BlockageEstimate::kNominal, BlockageEstimate::kConservative}) {
    for (const DevType & t : {kTransient, kPersistent}) {
      ControllerConfig cc = controller_config(Method::kDwafp);
      cc.estimate = e;
      summary_row(estimate_name(e), t, run_setting(std::string("estimator:") + estimate_name(e), controlled_scenario(), cc, t, runs, kAllShapes, csv, errors));
    }
  }
}

void exp_timeout(int runs, std::ofstream * csv, int & errors)
{
  std::printf("\n### timeout: execution budget of an attempt, x its nominal duration (the same for every method)\n");
  summary_header("timeout multiplier");
  for (const double lambda : {1.5, 2.0, 3.0}) {
    for (const DevType & t : {kTransient, kPersistent, kFailure, kBreakdown}) {
      ScenarioConfig c = controlled_scenario();
      c.timeout_multiplier = lambda;
      char label[16];
      std::snprintf(label, sizeof label, "%.1fx", lambda);
      summary_row(label, t, run_setting(std::string("timeout:") + label, c, controller_config(Method::kDwafp), t, runs, kAllShapes, csv, errors));
    }
  }
}

// =============================================================================================
// Repair-location study: when does repairing DOWNSTREAM of the trigger task matter?
// =============================================================================================
//
// Hypothesis: a downstream repair becomes useful when restarting or moving a nearly completed
// source is expensive, while modifying an unstarted descendant can absorb the deviation. Full
// factorial design, every cell on the same seeds:
//   source progress      : 20 / 50 / 80 % of the source's work done when the deviation happens
//   idle-spare availability: no spare vs one idle spare robot (the same workflow plus one robot
//                       without any task); in both, every WORKFLOW robot executes a task at the
//                       deviation and 2 s later (busy_fleet)
//   execution-time ratio rho: 1 (homogeneous), 1.5 (moderate), 3 (high): durations 1 / sqrt(rho)
//                       / rho s per unit of work
//   slack of the final deadlines, x the task's duration: tight U[0, 0.25], moderate U[0, 0.5],
//                       default U[0.5, 3] (with the default, few delays threaten anything)
// Fixed: deadlines on final tasks only (the source has none), random DAG and rendezvous in turn
// (chains and fork-join never keep every robot busy: they run fewer parallel lines than robots),
// no noise, the same dispatch latency for every method, everything else as in the controlled
// configuration. Deviations: transient delay and persistent obstruction (the preventive cases).
//
// What is paired: the METHODS, always (same scenario). Across factor levels, the spare and the
// slack keep the same workflow and deviation (the spare is added after the workflow is drawn; the
// slack only changes deadlines, which source selection never uses); progress and rho do not (they
// change the nominal timing, hence which task is an eligible source, and failed eligibility draws
// another graph). Differences between factor levels are therefore DESCRIPTIVE associations.
//
// Statistics: pooled lines reuse the same seeds in several cells, so every interval of this study
// is a cluster bootstrap by seed (compare_clustered, 10 000 resamples); within one cell it equals
// the ordinary paired bootstrap.

constexpr int kReactive = 1;
constexpr int kSource = 2;
constexpr int kWide = 4;
constexpr int kLocationResamples = 10000;

void location_header(const char * first)
{
  std::printf("  %-24s %-22s %-6s %-9s %-5s %-9s %-11s %-35s %-11s %-38s %-11s %-10s %s\n", first, "deviation",
    "threat", "prev S/D", "down", "succ S/D", "Donly/Sonly", "DWAFP-trigger-only [95% CI] W/T/L", "discard S/D",
    "added delay S/D  D-S [95% CI] (joint)", "asgChg S/D", "D-M W/T/L", "D-R");
}

// One line of the study, over `rows` (every row a scenario, its seed the bootstrap cluster).
//   threat      = scenarios in which no intervention misses a deadline (the only ones where a
//                 repair can matter)
//   prev S/D    = scenarios with at least one INSTALLED preventive repair (trigger-task-only / DWAFP)
//   down        = share of DWAFP's installed preventive repairs that edit a task other than the
//                 trigger task (a downstream edit)
//   succ S/D    = success % (every task completed, every deadline met)
//   Donly/Sonly = scenarios that only DWAFP / only trigger-task-only completes with success
//   DWAFP-trigger-only = paired difference in misses per run, cluster-bootstrap 95 % CI, W/T/L
//   discard     = work thrown away by cancelled attempts (s per run)
//   added delay = sum over tasks of max(0, end - nominal finish), s per run, ONLY over the
//                 scenarios in which both methods complete every task ("joint", count printed:
//                 otherwise a method could look better by leaving tasks unfinished)
//   asgChg      = final assignment changes: tasks whose final robot differs from the initial plan
//   D-M W/T/L   = DWAFP against mission-wide, wins / ties / losses in misses
//   D-R         = DWAFP - reactive, misses per run
void location_row(const std::string & label, const DevType & type, const std::vector<Row> & rows)
{
  const double n = static_cast<double>(std::max<std::size_t>(1, rows.size()));
  const auto mean = [&](int k, auto metric) {
      double acc = 0;
      for (const Row & row : rows) {acc += metric(row.result[k]);}
      return acc / n;
    };
  const auto misses = [](const RunResult & r) {return static_cast<double>(r.deadline_misses);};
  const auto added_s = [](const RunResult & r) {return static_cast<double>(r.added_delay_ms) / 1000.0;};
  const auto discarded_s = [](const RunResult & r) {return static_cast<double>(r.discarded_work_ms) / 1000.0;};
  const auto changes = [](const RunResult & r) {return static_cast<double>(r.final_assignment_changes);};
  const auto success = [](const RunResult & r) {return r.success ? 100.0 : 0.0;};
  // Paired comparison of method a against b on `metric`, clustered by seed, over the rows kept.
  const auto paired = [&](int a, int b, auto metric, auto keep) {
      std::vector<double> va, vb;
      std::vector<std::uint64_t> seeds;
      for (const Row & row : rows) {
        if (!keep(row)) {continue;}
        va.push_back(metric(row.result[a]));
        vb.push_back(metric(row.result[b]));
        seeds.push_back(row.seed);
      }
      return compare_clustered(va, vb, seeds, 1, kLocationResamples);
    };
  const auto all_rows = [](const Row &) {return true;};
  const auto joint = [](const Row & row) {
      return row.result[kSource].all_completed() && row.result[kDwafp].all_completed();
    };

  int threat = 0, prev_s = 0, prev_d = 0, installed = 0, downstream = 0, d_only = 0, s_only = 0, n_joint = 0;
  double added_s_joint = 0, added_d_joint = 0;
  for (const Row & row : rows) {
    const RunResult & src = row.result[kSource];
    const RunResult & d = row.result[kDwafp];
    threat += row.result[kNone].deadline_misses > 0 ? 1 : 0;
    prev_s += src.stats.preventive_installed > 0 ? 1 : 0;
    prev_d += d.stats.preventive_installed > 0 ? 1 : 0;
    installed += d.stats.preventive_installed;
    downstream += d.stats.preventive_downstream_installed;
    d_only += (d.success && !src.success) ? 1 : 0;
    s_only += (src.success && !d.success) ? 1 : 0;
    if (joint(row)) {
      ++n_joint;
      added_s_joint += added_s(src);
      added_d_joint += added_s(d);
    }
  }
  const PairedSummary vs = paired(kDwafp, kSource, misses, all_rows);
  const PairedSummary delay = paired(kDwafp, kSource, added_s, joint);
  const PairedSummary vm = paired(kDwafp, kWide, misses, all_rows);
  const PairedSummary vr = paired(kDwafp, kReactive, misses, all_rows);

  char down[16], prev[24], succ[16], only[16], a[64], disc[24], add[72], chg[24], dm[24];
  if (installed > 0) {std::snprintf(down, sizeof down, "%3.0f%%", 100.0 * downstream / installed);} else {std::snprintf(down, sizeof down, "  -");}
  std::snprintf(prev, sizeof prev, "%d/%d", prev_s, prev_d);
  std::snprintf(succ, sizeof succ, "%.0f/%.0f", mean(kSource, success), mean(kDwafp, success));
  std::snprintf(only, sizeof only, "%d/%d", d_only, s_only);
  std::snprintf(a, sizeof a, "%+5.2f [%+5.2f,%+5.2f] %3d/%3d/%d", vs.mean_diff, vs.ci_lo, vs.ci_hi, vs.wins, vs.ties, vs.losses);
  std::snprintf(disc, sizeof disc, "%.2f/%.2f", mean(kSource, discarded_s), mean(kDwafp, discarded_s));
  const double nj = std::max(1, n_joint);
  std::snprintf(add, sizeof add, "%.1f/%.1f %+5.1f [%+5.1f,%+5.1f] (%d)", added_s_joint / nj, added_d_joint / nj,
    delay.mean_diff, delay.ci_lo, delay.ci_hi, n_joint);
  std::snprintf(chg, sizeof chg, "%.2f/%.2f", mean(kSource, changes), mean(kDwafp, changes));
  std::snprintf(dm, sizeof dm, "%d/%d/%d", vm.wins, vm.ties, vm.losses);
  std::printf("  %-24s %-22s %-6d %-9s %-5s %-9s %-11s %-35s %-11s %-38s %-11s %-10s %+5.2f\n", label.c_str(), type.name,
    threat, prev, down, succ, only, a, disc, add, chg, dm, vr.mean_diff);
}

void exp_location(int runs, std::ofstream * csv, int & errors, TimeMs latency_ms)
{
  struct Slack {const char * name; double lo; double hi;};
  const Slack slacks[] = {{"tight", 0.0, 0.25}, {"moderate", 0.0, 0.5}, {"default", 0.5, 3.0}};
  const double progresses[] = {0.2, 0.5, 0.8};
  const bool spares[] = {false, true};
  const double ratios[] = {1.0, 1.5, 3.0};
  const std::vector<WorkflowShape> shapes{WorkflowShape::kRandomDag, WorkflowShape::kRendezvous};
  ControllerConfig cc = controller_config(Method::kDwafp);
  cc.dispatch_ms = latency_ms;  // the same latency for every method

  std::printf("\n### location: repair location vs source progress, idle-spare availability and execution-time ratio\n");
  std::printf("4 workflow robots, each busy at the deviation (+1 idle spare robot in 'idle spare', duration sqrt(rho)), 20 tasks,\n"
    "random DAG and rendezvous in turn, deadlines on final tasks only, no noise, 2x timeout, dispatch latency %lld ms\n"
    "for every method, nominal estimate. rho = execution-time ratio (durations 1 / sqrt(rho) / rho s per unit).\n"
    "S = trigger-task-only (source-only) repair, D = DWAFP, M = mission-wide, R = reactive. 95%% CIs: cluster\n"
    "bootstrap by seed, %d resamples. Columns: threat = runs where no intervention misses a deadline; prev = runs with\n"
    "an installed preventive repair; down = share of DWAFP's installed preventive repairs that edit a task other than\n"
    "the trigger task; succ = success %%; Donly/Sonly = runs only D / only S completes on time; DWAFP-trigger-only =\n"
    "paired difference in misses per run; discard = work thrown away (s/run); added delay = delay added to the tasks\n"
    "w.r.t. the nominal plan (s/run), only over runs where S and D both complete every task (their number in\n"
    "brackets); asgChg = final assignment changes per run; D-M W/T/L = DWAFP vs mission-wide; D-R = DWAFP - reactive.\n",
    static_cast<long long>(latency_ms), kLocationResamples);
  for (const DevType & t : {kTransient, kPersistent}) {
    for (const Slack & slack : slacks) {
      // Every cell's rows, kept for the pooled lines below the table.
      struct Cell {double progress; bool spare; double rho; std::vector<Row> rows;};
      std::vector<Cell> cells;
      std::printf("\n%s, %s slack U[%.2f, %.2f]\n", t.name, slack.name, slack.lo, slack.hi);
      location_header("progress spare rho");
      for (const double progress : progresses) {
        for (const bool spare : spares) {
          for (const double rho : ratios) {
            ScenarioConfig c = controlled_scenario();
            c.deadlines = DeadlineStructure::kFinalOnly;
            c.slack_lo = slack.lo;
            c.slack_hi = slack.hi;
            c.busy_fleet = true;  // the workflow robots are busy; the spare (if any) is idle
            c.spare_robots = spare ? 1 : 0;
            c.progress_lo = c.progress_hi = progress;
            c.execution_time_ratio = rho;
            char label[32];
            std::snprintf(label, sizeof label, "%2.0f%% %-10s %.1f", 100 * progress, spare ? "idle spare" : "no spare", rho);
            cells.push_back({progress, spare, rho,
              run_setting(std::string("location:") + slack.name + " " + label, c, cc, t, runs, shapes, csv, errors)});
            location_row(label, t, cells.back().rows);
          }
        }
      }
      // Pooled over the other two factors (descriptive; clustered by seed, see above).
      const auto pooled = [&](auto keep) {
          std::vector<Row> all;
          for (const Cell & cell : cells) {
            if (keep(cell)) {all.insert(all.end(), cell.rows.begin(), cell.rows.end());}
          }
          return all;
        };
      std::printf("  -- pooled over the other factors --\n");
      for (const double progress : progresses) {
        char label[32];
        std::snprintf(label, sizeof label, "progress %.0f%%", 100 * progress);
        location_row(label, t, pooled([&](const Cell & cell) {return cell.progress == progress;}));
      }
      for (const bool spare : spares) {
        location_row(spare ? "idle spare" : "no spare", t, pooled([&](const Cell & cell) {return cell.spare == spare;}));
      }
      for (const double rho : ratios) {
        char label[32];
        std::snprintf(label, sizeof label, "rho %.1f", rho);
        location_row(label, t, pooled([&](const Cell & cell) {return cell.rho == rho;}));
      }
      location_row("all cells", t, pooled([](const Cell &) {return true;}));
    }
  }
}

}  // namespace

// The main comparison (Step 5): full tables per deviation type.
int main_comparison(int argc, char ** argv)
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

int main(int argc, char ** argv)
{
  // A first argument that is not a number names a Step 6 experiment.
  if (argc > 1 && !std::isdigit(static_cast<unsigned char>(argv[1][0]))) {
    const std::string name = argv[1];
    const int runs = argc > 2 ? std::max(1, std::atoi(argv[2])) : 200;
    std::ofstream csv_file;
    std::ofstream * csv = nullptr;
    if (argc > 3 && std::string(argv[3]) != "-") {  // "-" = no CSV (to give a latency only)
      csv_file.open(argv[3]);
      if (!csv_file) {
        std::fprintf(stderr, "cannot write %s\n", argv[3]);
        return 1;
      }
      csv = &csv_file;
      write_csv_header(*csv);
    }
    std::printf("DWAFP Step 6 experiment '%s': %d paired runs per setting and deviation type\n", name.c_str(), runs);
    if (name == "location") {
      // Its own configuration (printed by exp_location); optional 4th argument: latency in ms.
      const TimeMs latency = argc > 4 ? std::max(0, std::atoi(argv[4])) : 0;
      int errors = 0;
      exp_location(runs, csv, errors, latency);
      std::printf("\ncontroller errors (must be 0): %d\n", errors);
      if (csv) {std::printf("CSV written to %s\n", argv[3]);}
      return errors == 0 ? 0 : 1;
    }
    std::printf("Everything else as in the controlled configuration (4 robots, 20 tasks, every-task deadlines,\n"
      "slack 0.5-3, no noise, 2x timeout, no latency, nominal estimate, k=3, eps=0, tau=2 s).\n"
      "Columns: success of reactive / source-only / DWAFP; deadline-miss rate of reactive / DWAFP; paired\n"
      "difference in misses per run (negative = DWAFP better) with its 95%% CI and W/T/L; D-M = DWAFP -\n"
      "mission-wide; DWAFP runs that were harmful (more misses than no intervention) / unneeded.\n");
    int errors = 0;
    const bool all = name == "sweeps";
    bool known = false;
    if (all || name == "deadlines") {exp_deadlines(runs, csv, errors); known = true;}
    if (all || name == "noise") {exp_noise(runs, csv, errors); known = true;}
    if (all || name == "latency") {exp_latency(runs, csv, errors); known = true;}
    if (all || name == "estimator") {exp_estimator(runs, csv, errors); known = true;}
    if (all || name == "timeout") {exp_timeout(runs, csv, errors); known = true;}
    if (!known) {
      std::fprintf(stderr, "unknown experiment '%s' (deadlines, noise, latency, estimator, timeout, sweeps)\n", name.c_str());
      return 1;
    }
    std::printf("\ncontroller errors (must be 0): %d\n", errors);
    if (csv) {std::printf("CSV written to %s\n", argv[3]);}
    return errors == 0 ? 0 : 1;
  }
  return main_comparison(argc, argv);
}

