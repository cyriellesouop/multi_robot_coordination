// DWAFP Stage I experiments, ROS-free (no ROS node, no robot: only the dwafp_core library).
//
// A. Walkthrough: named deviation scenarios through decide() (Algorithm 1 up to Stage I), with
//    every candidate and its scores, to read the decisions one by one.
// B. Ablation (the main comparison), on the SAME missions and events:
//      no repair          : what the deviation breaks (violated conditions)
//      source-only        : the policy library applied to the deviating task only (closest to
//                           existing approaches, e.g. reallocating only the delayed task)
//      DWAFP Stage I      : the policy library applied inside the repair region
//      global single-edit : the policy library applied to every unfinished task (no localization)
//    It answers: do downstream repair locations add coverage over source-only repair? does the
//    region keep most of the global opportunities? does localization cut candidates and time?
// C. Scaling, DWAFP through decide():
//      C1 fixed mission size N = 40, robots R in {2, 4, 8, 16}
//      C2 fixed tasks per robot (weak scaling), (R, N) in {(2,10), (4,20), (8,40), (16,80)}
//      C3 fixed robots R = 3, N in {10, 20, 40, 80} (workload / queue-pressure experiment)
//    Locality L_R = |V_R| / |V_remaining| (V_remaining = tasks not completed at detection, the
//    failed source included), over the events that need repair only.
// D. Same-robot retry feasibility (transient vs persistent cause) x dispatch latency.
//    Note: the deviating task still completes after a finite delay; "persistent" here only means
//    that same-robot retry is not feasible. A literal permanent blockage would make the source's
//    completion unreachable instead; it is not simulated here.
//
// Generator (identical distributions in every configuration): dependencies only from a lower to a
// higher index, each with probability min(0.5, 3/N) (about 1.5 predecessors per task on average);
// work 3..7 units; robot speeds cycle through 1.0 / 1.5 / 0.8 s per unit; random assignment;
// deadline = nominal finish + absolute slack of 0.5..3 times the task's own duration (B3-B6: the
// tight-slack ablation uses 0.2..1.0 and 0.1..0.5). Events: one
// task running at a random time in the first 60 % of the nominal makespan deviates: a delay of
// 1..10 s (70 %) or an outcome failure (30 %).
//
// Usage:   ros2 run actions stage1_experiments [events_per_config] [seed] [csv_file]
// Defaults: 300 events, seed 1, no CSV. All times inside are nominal ms, printed in seconds or us.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "actions/impact.hpp"
#include "actions/mission_loader.hpp"
#include "actions/repair.hpp"
#include "actions/workflow.hpp"

using namespace dwafp;

namespace
{

// =============================================================================================
// Shared helpers
// =============================================================================================

const char * kFleetYaml =
  "robots: [{name: robot1, step_duration_sec: 1.0},"
  " {name: robot2, step_duration_sec: 1.5},"
  " {name: robot3, step_duration_sec: 0.8}]";

double sec(TimeMs ms) {return static_cast<double>(ms) / 1000.0;}

double since_us(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
}

const char * policy_name(const RepairCandidate & c)
{
  switch (c.policy) {
    case RepairPolicy::kRetry: return "retry";
    case RepairPolicy::kTakeover: return "takeover";
    case RepairPolicy::kSubstitution: return c.after_current_task ? "subst(next-slot)" : "substitution";
  }
  return "?";
}

const char * status_name(DecisionStatus s)
{
  switch (s) {
    case DecisionStatus::kContinue: return "CONTINUE";
    case DecisionStatus::kRepair: return "REPAIR";
    case DecisionStatus::kNoFeasibleLocalRepair: return "NO_FEASIBLE_LOCAL_REPAIR";
    case DecisionStatus::kNoSingleEdit: return "NO_SINGLE_EDIT (Stage II)";
  }
  return "?";
}

std::string task_list(const Mission & m, const std::vector<Index> & tasks)
{
  std::string out;
  for (const Index j : tasks) {out += (out.empty() ? "" : " ") + m.task(j).id;}
  return out.empty() ? "-" : out;
}

// Number of violated predicates over the threatened tasks, in a prediction.
int violated_conditions(const ImpactAnalysis & a, const Prediction & p)
{
  int n = 0;
  for (const Index j : a.threatened) {
    const TaskPrediction & q = p.tasks[j];
    n += (q.deadline_ok ? 0 : 1) + (q.outcome_ok ? 0 : 1) + (q.assignment_ok ? 0 : 1);
  }
  return n;
}

// =============================================================================================
// A. Walkthrough
// =============================================================================================

void print_decision(const char * title, const Mission & m, const Decision & d)
{
  std::printf("\n=== %s ===\n", title);
  const ImpactAnalysis & a = d.impact;
  if (!a.needs_repair()) {
    std::printf("no condition newly threatened -> %s (horizon: %s)\n", status_name(d.status),
      task_list(m, a.horizon).c_str());
    return;
  }
  std::string threats;
  for (const Threat & t : a.threats) {
    std::string what;
    if (t.deadline) {what += "deadline";}
    if (t.outcome) {what += std::string(what.empty() ? "" : "+") + "input";}
    if (t.assignment) {what += std::string(what.empty() ? "" : "+") + "robot";}
    threats += (threats.empty() ? "" : ", ") + m.task(t.task).id + " (" + what + ")";
  }
  std::printf("threatened A_i: %s\nregion: %s   horizon: %s   B_i^0 = %.1f s   hold: %s\n",
    threats.c_str(), task_list(m, a.region).c_str(), task_list(m, a.horizon).c_str(),
    is_reachable(a.response_window_ms) ? sec(a.response_window_ms) : -1.0,
    task_list(m, d.to_hold).c_str());
  if (d.status == DecisionStatus::kNoFeasibleLocalRepair) {
    std::printf("certificate: no permitted local repair can protect %s -> %s (no search)\n",
      task_list(m, d.unprotectable).c_str(), status_name(d.status));
    return;
  }
  std::printf("  %-17s %-5s %-7s %2s %2s %7s %10s\n", "policy", "task", "robot", "R", "D", "C(s)",
    "restore(s)");
  for (std::size_t i = 0; i < d.stage1.admissible.size(); ++i) {
    const RepairCandidate & c = d.stage1.admissible[i];
    std::printf("%s %-17s %-5s %-7s %2d %2d %7.1f %10.1f\n",
      d.stage1.best && *d.stage1.best == i ? "*" : " ", policy_name(c), m.task(c.task).id.c_str(),
      m.robot(c.robot).name.c_str(), c.residual_risk, c.disruption, sec(c.cost_ms),
      is_reachable(c.restoration_ms) ? sec(c.restoration_ms) : -1.0);
  }
  std::printf("-> %s", status_name(d.status));
  if (d.stage1.best) {
    const RepairCandidate & c = d.stage1.admissible[*d.stage1.best];
    std::printf(": %s of %s on %s", policy_name(c), m.task(c.task).id.c_str(), m.robot(c.robot).name.c_str());
  }
  std::printf("   (%.0f us)\n", d.impact_us + d.check_us + d.stage1_us);
}

template<typename SetState>
void walkthrough(const char * title, const std::string & yaml, const char * deviating, TimeMs now,
  SetState set_state, RepairConfig config = {})
{
  const Mission m = mission_from_yaml(yaml, kFleetYaml);
  const Plan plan = Plan::initial(m);
  RuntimeState actual = RuntimeState::initial(m);
  actual.now_ms = now;
  set_state(m, actual);
  const DurationSnapshot p = DurationSnapshot::capture(m, ConfigDurationModel{}, actual);
  const RuntimeState s0 = RuntimeState::initial(m);
  const Prediction reference = Predictor(m, plan, s0, p).full();
  const RuntimeState expected = expected_state(m, plan, actual, reference, {m.task_index(deviating)}, {});
  print_decision(title, m, decide(m, plan, expected, actual, p, config));
}

void executing(const Mission & m, RuntimeState & s, const char * id, TimeMs start, TimeMs remaining)
{
  auto & t = s.tasks[m.task_index(id)];
  t.status = TaskStatus::kExecuting;
  t.start_ms = start;
  t.remaining_ms = remaining;
}

void experiment_a()
{
  std::printf("################ A. Walkthrough ################\n");
  std::printf("(5 s per task on robot1, 7.5 s on robot2, 4 s on robot3; * = chosen repair)\n");
  const std::string chain =
    "{mission_id: chain, subtasks: [{id: T1, robot: robot1, deadline_sec: 6},"
    " {id: T2, robot: robot2, deadline_sec: 14, predecessors: [T1]}]}";
  walkthrough("A1 chain.yaml: T1 blocked at 4 s, predicts 8 s (own deadline 6 s)", chain, "T1", 4000,
    [](const Mission & m, RuntimeState & s) {executing(m, s, "T1", 0, 4000);});
  const std::string loose =
    "{mission_id: chain2, subtasks: [{id: T1, robot: robot1, deadline_sec: 20},"
    " {id: T2, robot: robot2, deadline_sec: 14, predecessors: [T1]}]}";
  walkthrough("A2 T1 (d 20 s) blocked at 2 s, predicts 8 s; T2 (d 14 s) threatened", loose, "T1", 2000,
    [](const Mission & m, RuntimeState & s) {executing(m, s, "T1", 0, 6000);});
  RepairConfig latency;
  latency.dispatch_overhead_ms = 3000;
  walkthrough("A3 same as A2, same-robot retry infeasible and 3 s dispatch latency", loose, "T1", 2000,
    [](const Mission & m, RuntimeState & s) {
      executing(m, s, "T1", 0, 6000);
      s.tasks[m.task_index("T1")].same_robot_retry_feasible = false;
    }, latency);
  const std::string fork =
    "{mission_id: fork_join, subtasks: [{id: T1, robot: robot1, deadline_sec: 6},"
    " {id: T2, robot: robot2, deadline_sec: 9},"
    " {id: T3, robot: robot3, deadline_sec: 14, predecessors: [T1, T2]}]}";
  walkthrough("A4 fork_join.yaml: T1 predicts 11 s at t = 4 s", fork, "T1", 4000,
    [](const Mission & m, RuntimeState & s) {
      executing(m, s, "T1", 0, 7000);
      executing(m, s, "T2", 0, 3500);
    });
  const std::string outcome =
    "{mission_id: outcome, subtasks: [{id: T1, robot: robot1, deadline_sec: 15},"
    " {id: T2, robot: robot2, deadline_sec: 25, predecessors: [T1]}]}";
  walkthrough("A5 T1 FAILS at 3 s (outcome failure), T2 has a loose deadline (25 s)", outcome, "T1", 3000,
    [](const Mission & m, RuntimeState & s) {
      auto & t = s.tasks[m.task_index("T1")];
      t.status = TaskStatus::kFailed;
      t.start_ms = 0;
      t.end_ms = 3000;
    });
}

// =============================================================================================
// Generator: random missions and events
// =============================================================================================

struct RandomMission
{
  Mission mission;
  Plan plan;
  Prediction nominal;
};

// Slack range: each deadline is the nominal finish plus [slack_lo, slack_hi] times the task's own
// duration (default 0.5..3; the tight-slack ablation uses smaller ranges).
struct SlackRange
{
  double lo{0.5};
  double hi{3.0};
};

RandomMission make_random_mission(int n, int robots, std::mt19937 & rng, SlackRange range = {})
{
  std::uniform_int_distribution<int> robot(0, robots - 1), work(3, 7);
  std::uniform_real_distribution<double> coin(0.0, 1.0), slack(range.lo, range.hi);
  const double rates[] = {1000, 1500, 800};
  std::vector<Robot> fleet;
  for (int r = 0; r < robots; ++r) {fleet.push_back({"robot" + std::to_string(r + 1), rates[r % 3]});}
  const double p_edge = std::min(0.5, 3.0 / n);

  std::vector<Subtask> tasks(static_cast<std::size_t>(n));
  for (int j = 0; j < n; ++j) {
    Subtask & t = tasks[static_cast<std::size_t>(j)];
    t.id = "T" + std::to_string(j);
    t.robot = fleet[static_cast<std::size_t>(robot(rng))].name;
    t.work = work(rng);
    t.deadline_ms = 1000000000;  // placeholder until the nominal schedule is known
    for (int h = 0; h < j; ++h) {
      if (coin(rng) < p_edge) {t.predecessors.push_back("T" + std::to_string(h));}
    }
  }
  {
    // Nominal schedule without deadline pressure, then absolute slack per task.
    const Mission draft("draft", tasks, fleet);
    const Plan plan = Plan::initial(draft);
    const RuntimeState s0 = RuntimeState::initial(draft);
    const DurationSnapshot p = DurationSnapshot::capture(draft, ConfigDurationModel{}, s0);
    const Prediction nominal = Predictor(draft, plan, s0, p).full();
    for (std::size_t j = 0; j < tasks.size(); ++j) {
      const TimeMs own = p.at(j, plan.robot_of(j));
      tasks[j].deadline_ms = nominal.tasks[j].finish_ms + static_cast<TimeMs>(slack(rng) * static_cast<double>(own));
    }
  }
  Mission mission("random", tasks, fleet);
  Plan plan = Plan::initial(mission);
  const RuntimeState s0 = RuntimeState::initial(mission);
  const DurationSnapshot p = DurationSnapshot::capture(mission, ConfigDurationModel{}, s0);
  Prediction nominal = Predictor(mission, plan, s0, p).full();
  return {std::move(mission), std::move(plan), std::move(nominal)};
}

struct EventSpec
{
  std::size_t mission{0};
  TimeMs time_ms{0};
  Index source{kNoIndex};
  bool failure{false};
  TimeMs delay_ms{0};
};

bool make_event(const RandomMission & rm, std::size_t index, std::mt19937 & rng, EventSpec & e)
{
  TimeMs makespan = 0;
  for (const auto & q : rm.nominal.tasks) {makespan = std::max(makespan, q.finish_ms);}
  std::uniform_int_distribution<TimeMs> when(500, std::max<TimeMs>(501, makespan * 6 / 10));
  for (int attempt = 0; attempt < 50; ++attempt) {
    const TimeMs t = when(rng);
    std::vector<Index> running;
    for (Index j = 0; j < rm.mission.task_count(); ++j) {
      const TaskPrediction & q = rm.nominal.tasks[j];
      if (q.start_ms <= t && t < q.finish_ms) {running.push_back(j);}
    }
    if (running.empty()) {continue;}
    e.mission = index;
    e.time_ms = t;
    e.source = running[std::uniform_int_distribution<std::size_t>(0, running.size() - 1)(rng)];
    e.failure = std::uniform_real_distribution<double>(0.0, 1.0)(rng) < 0.3;
    e.delay_ms = std::uniform_int_distribution<TimeMs>(1000, 10000)(rng);
    return true;
  }
  return false;
}

// A generated set of missions and one event per mission.
struct EventSet
{
  std::vector<RandomMission> missions;
  std::vector<EventSpec> events;
};

EventSet make_events(int n, int robots, int count, unsigned seed, SlackRange range = {})
{
  std::mt19937 rng(seed);
  EventSet set;
  while (static_cast<int>(set.events.size()) < count) {
    set.missions.push_back(make_random_mission(n, robots, rng, range));
    EventSpec e;
    if (make_event(set.missions.back(), set.missions.size() - 1, rng, e)) {set.events.push_back(e);}
  }
  return set;
}

// The observed state at the event: everything as planned until t, then the deviation.
RuntimeState observed_state(const RandomMission & rm, const EventSpec & e, bool retry_feasible)
{
  RuntimeState s = RuntimeState::initial(rm.mission);
  s.now_ms = e.time_ms;
  for (Index j = 0; j < rm.mission.task_count(); ++j) {
    const TaskPrediction & q = rm.nominal.tasks[j];
    TaskRuntime & task = s.tasks[j];
    if (q.finish_ms <= e.time_ms) {
      task.status = TaskStatus::kCompleted;
      task.start_ms = q.start_ms;
      task.end_ms = q.finish_ms;
    } else if (q.start_ms <= e.time_ms) {
      task.status = TaskStatus::kExecuting;
      task.start_ms = q.start_ms;
      task.remaining_ms = q.finish_ms - e.time_ms;
    }
  }
  TaskRuntime & src = s.tasks[e.source];
  if (e.failure) {
    src.status = TaskStatus::kFailed;
    src.end_ms = e.time_ms;
    src.remaining_ms = 0;
  } else {
    src.remaining_ms += e.delay_ms;
  }
  src.same_robot_retry_feasible = retry_feasible;
  return s;
}

// =============================================================================================
// B. Ablation
// =============================================================================================

struct MethodStats
{
  int repaired{0};
  double sum_generated{0}, sum_us{0}, sum_left{0};
};

void ablation(const char * title, const EventSet & set, bool retry_feasible, TimeMs latency,
  std::ofstream * csv)
{
  const char * names[] = {"no repair", "source-only", "DWAFP Stage I", "global single-edit"};
  const RepairScope scopes[] = {RepairScope::kSources, RepairScope::kSources, RepairScope::kRegion,
    RepairScope::kAllUnfinished};
  MethodStats st[4];
  int events = 0, need = 0, certified = 0, region_beats_source = 0, global_beats_region = 0;
  int global_outside_region = 0;
  int source_threatened = 0;   // the deviating task is itself in A_i (only a source edit can save it)
  int single_downstream = 0;   // A_i = exactly one task, downstream of the source
  double sum_consequences = 0;

  for (const EventSpec & e : set.events) {
    const RandomMission & rm = set.missions[e.mission];
    const Mission & m = rm.mission;
    const RuntimeState actual = observed_state(rm, e, retry_feasible);
    const DurationSnapshot p = DurationSnapshot::capture(m, ConfigDurationModel{}, actual);
    const RuntimeState expected = expected_state(m, rm.plan, actual, rm.nominal, {e.source}, {});
    const ImpactAnalysis impact = analyze_impact(m, rm.plan, expected, actual, p);
    ++events;
    if (!impact.needs_repair()) {continue;}
    ++need;
    RepairConfig rc;
    rc.dispatch_overhead_ms = latency;
    if (!unprotectable_threats(m, rm.plan, actual, p, impact, rc).empty()) {++certified;}
    const auto & a = impact.threatened;
    if (std::find(a.begin(), a.end(), e.source) != a.end()) {++source_threatened;}
    if (a.size() == 1 && a.front() != e.source) {++single_downstream;}

    // No repair: the violated conditions the deviation leaves.
    const int consequences = violated_conditions(impact, impact.updated);
    sum_consequences += consequences;
    st[0].sum_left += consequences;

    bool repaired[4] = {false, false, false, false};
    Index global_target = kNoIndex;
    for (int k = 1; k < 4; ++k) {
      rc.scope = scopes[k];
      const auto t0 = std::chrono::steady_clock::now();
      const StageOneResult r = generate_single_edits(m, rm.plan, actual, p, impact, rc);
      st[k].sum_us += since_us(t0);
      st[k].sum_generated += static_cast<double>(r.generated);
      repaired[k] = r.best.has_value();
      if (repaired[k]) {
        ++st[k].repaired;
        if (k == 3) {global_target = r.admissible[*r.best].task;}
      } else {
        st[k].sum_left += consequences;  // escalated: the consequences remain
      }
      if (csv) {
        *csv << "B," << title << ',' << names[k] << ',' << m.robot_count() << ',' << m.task_count()
             << ',' << e.mission << ',' << (e.failure ? "failure" : "delay") << ",1," << repaired[k]
             << ',' << impact.threatened.size() << ',' << impact.region.size() << ','
             << r.generated << ',' << consequences << '\n';
      }
    }
    if (repaired[2] && !repaired[1]) {++region_beats_source;}
    if (repaired[3] && !repaired[2]) {
      ++global_beats_region;
      const auto & region = impact.region;
      if (std::find(region.begin(), region.end(), global_target) == region.end()) {++global_outside_region;}
    }
  }

  std::printf("\n%s: %d events, %d need repair (%d provably without local repair)\n", title, events,
    need, certified);
  std::printf("  %-20s %9s %9s %11s %12s %14s\n", "method", "repaired", "coverage", "candidates",
    "search_us", "conds_left");
  const double nd = std::max(1, need);
  for (int k = 0; k < 4; ++k) {
    std::printf("  %-20s %9d %8.0f%% %11.1f %12.1f %14.2f\n", names[k], st[k].repaired,
      100.0 * st[k].repaired / nd, k ? st[k].sum_generated / nd : 0.0, k ? st[k].sum_us / nd : 0.0,
      st[k].sum_left / nd);
  }
  std::printf("  DWAFP repaired where source-only did not: %d events\n", region_beats_source);
  std::printf("  global repaired where DWAFP did not: %d events (%d of them by editing outside the region)\n",
    global_beats_region, global_outside_region);
  std::printf("  A_i contains the deviating task itself: %d events; A_i is one downstream task only: %d events\n",
    source_threatened, single_downstream);
  std::printf("  DWAFP keeps %.0f%% of the global single-edit repairs\n",
    st[3].repaired ? 100.0 * st[2].repaired / st[3].repaired : 100.0);
  (void)sum_consequences;
}

// =============================================================================================
// C. Scaling (DWAFP through decide())
// =============================================================================================

struct ScaleStats
{
  int events{0}, need{0}, repaired{0}, certified{0}, single_none{0};
  double sum_region{0}, sum_locality{0}, sum_threat{0}, sum_horizon{0}, sum_generated{0};
  double sum_impact_us{0}, sum_stage1_us{0}, sum_total_us{0}, max_total_us{0};
  int searched{0};
};

ScaleStats run_decide(const EventSet & set, bool retry_feasible, TimeMs latency, const std::string & label,
  std::ofstream * csv)
{
  ScaleStats st;
  RepairConfig rc;
  rc.dispatch_overhead_ms = latency;
  for (const EventSpec & e : set.events) {
    const RandomMission & rm = set.missions[e.mission];
    const Mission & m = rm.mission;
    const RuntimeState actual = observed_state(rm, e, retry_feasible);
    const DurationSnapshot p = DurationSnapshot::capture(m, ConfigDurationModel{}, actual);
    const RuntimeState expected = expected_state(m, rm.plan, actual, rm.nominal, {e.source}, {});
    const Decision d = decide(m, rm.plan, expected, actual, p, rc);
    ++st.events;
    st.sum_impact_us += d.impact_us;  // impact-analysis time over ALL events
    st.sum_horizon += static_cast<double>(d.impact.horizon.size());
    std::size_t remaining = 0;
    for (const auto & t : actual.tasks) {remaining += t.status != TaskStatus::kCompleted ? 1 : 0;}
    if (d.status != DecisionStatus::kContinue) {
      ++st.need;
      const double region = static_cast<double>(d.impact.region.size());
      st.sum_region += region;
      st.sum_locality += region / static_cast<double>(std::max<std::size_t>(1, remaining));
      st.sum_threat += static_cast<double>(d.impact.threatened.size());
      const double total = d.impact_us + d.check_us + d.stage1_us;
      st.sum_total_us += total;
      st.max_total_us = std::max(st.max_total_us, total);
      if (d.status == DecisionStatus::kNoFeasibleLocalRepair) {
        ++st.certified;
      } else {
        ++st.searched;
        st.sum_stage1_us += d.stage1_us;
        st.sum_generated += static_cast<double>(d.stage1.generated);
        if (d.status == DecisionStatus::kRepair) {++st.repaired;} else {++st.single_none;}
      }
    }
    if (csv) {
      *csv << "C," << label << ",DWAFP," << m.robot_count() << ',' << m.task_count() << ','
           << e.mission << ',' << (e.failure ? "failure" : "delay") << ','
           << (d.status != DecisionStatus::kContinue) << ',' << (d.status == DecisionStatus::kRepair)
           << ',' << d.impact.threatened.size() << ',' << d.impact.region.size() << ','
           << d.stage1.generated << ',' << remaining << ',' << d.impact_us << ',' << d.stage1_us << '\n';
    }
  }
  return st;
}

void scale_header()
{
  std::printf("  %-22s %5s %5s %6s %6s %6s | %6s %6s %6s %6s %5s | %8s %8s %8s %8s\n", "configuration",
    "need", "rep%", "noLoc%", "noSE%", "|A|", "|V_R|", "L_R", "horizn", "cand", "", "impact", "stageI",
    "total", "max");
}

void scale_row(const std::string & label, const ScaleStats & s)
{
  const double nd = std::max(1, s.need);
  const double sd = std::max(1, s.searched);
  std::printf("  %-22s %5d %4.0f%% %5.0f%% %5.0f%% %6.1f | %6.1f %5.0f%% %6.1f %6.1f %5s | %8.1f %8.1f %8.1f %8.1f\n",
    label.c_str(), s.need, 100.0 * s.repaired / nd, 100.0 * s.certified / nd, 100.0 * s.single_none / nd,
    s.sum_threat / nd, s.sum_region / nd, 100.0 * s.sum_locality / nd, s.sum_horizon / std::max(1, s.events),
    s.sum_generated / sd, "", s.sum_impact_us / std::max(1, s.events), s.sum_stage1_us / sd,
    s.sum_total_us / nd, s.max_total_us);
}

}  // namespace

int main(int argc, char ** argv)
{
  const int events = argc > 1 ? std::max(1, std::atoi(argv[1])) : 300;
  const unsigned seed = argc > 2 ? static_cast<unsigned>(std::atoi(argv[2])) : 1u;
  std::ofstream csv_file;
  std::ofstream * csv = nullptr;
  if (argc > 3) {
    csv_file.open(argv[3]);
    if (!csv_file) {
      std::fprintf(stderr, "cannot write %s\n", argv[3]);
      return 1;
    }
    csv = &csv_file;
    *csv << "experiment,configuration,method,robots,tasks,mission,kind,needs_repair,repaired,"
            "threatened,region,generated,extra,impact_us,stage1_us\n";
  }

  experiment_a();

  std::printf("\n################ B. Ablation (same missions and events, 4 robots, 20 tasks) ################\n");
  std::printf("coverage = repaired / events needing repair; candidates and search_us per event needing\n"
    "repair; conds_left = violated conditions of A_i left per event (escalated events keep them)\n");
  const EventSet ablation_set = make_events(20, 4, events, seed);
  ablation("B1 transient cause, no latency", ablation_set, true, 0, csv);
  ablation("B2 persistent cause, 1 s latency", ablation_set, false, 1000, csv);

  // Tight slack: the same generator and seed, smaller deadline slack. Here a deviation can more
  // often NOT be absorbed at the source, which is where downstream repair locations should matter.
  std::printf("\n---- Tight slack (deadline = nominal finish + 0.2..1.0 x own duration) ----\n");
  const EventSet tight = make_events(20, 4, events, seed, {0.2, 1.0});
  ablation("B3 tight slack, transient cause, no latency", tight, true, 0, csv);
  ablation("B4 tight slack, persistent cause, 1 s latency", tight, false, 1000, csv);
  std::printf("\n---- Very tight slack (deadline = nominal finish + 0.1..0.5 x own duration) ----\n");
  const EventSet very_tight = make_events(20, 4, events, seed, {0.1, 0.5});
  ablation("B5 very tight slack, transient cause, no latency", very_tight, true, 0, csv);
  ablation("B6 very tight slack, persistent cause, 1 s latency", very_tight, false, 1000, csv);

  std::printf("\n################ C. Scaling (DWAFP, transient cause, no latency) ################\n");
  std::printf("need = events needing repair; rep%% repaired, noLoc%% provably no local repair, noSE%% no single\n"
    "edit (Stage II); |A|, |V_R|, L_R = |V_R| / remaining tasks, total = decision time: over events\n"
    "needing repair; horizn and impact time: over all events; cand and stageI time: over searched events.\n"
    "Times in microseconds.\n");
  std::printf("\nC1 fixed mission size N = 40, varying robots\n");
  scale_header();
  for (const int r : {2, 4, 8, 16}) {
    const std::string label = "R=" + std::to_string(r) + " N=40";
    scale_row(label, run_decide(make_events(40, r, events, seed + 100 + r), true, 0, label, csv));
  }
  std::printf("\nC2 fixed tasks per robot (weak scaling, 5 tasks per robot)\n");
  scale_header();
  for (const int r : {2, 4, 8, 16}) {
    const std::string label = "R=" + std::to_string(r) + " N=" + std::to_string(5 * r);
    scale_row(label, run_decide(make_events(5 * r, r, events, seed + 200 + r), true, 0, label, csv));
  }
  std::printf("\nC3 fixed robots R = 3, varying mission size (workload / queue pressure)\n");
  scale_header();
  for (const int n : {10, 20, 40, 80}) {
    const std::string label = "R=3 N=" + std::to_string(n);
    scale_row(label, run_decide(make_events(n, 3, events, seed + 300 + n), true, 0, label, csv));
  }

  std::printf("\n################ D. Same-robot retry feasibility x dispatch latency (4 robots, 20 tasks) ################\n");
  std::printf("transient = same-robot retry feasible; persistent = not feasible (the source still completes\n"
    "after its finite delay: only the candidate policies change)\n");
  scale_header();
  for (const bool feasible : {true, false}) {
    for (const TimeMs latency : {0, 1000, 3000}) {
      const std::string label = std::string(feasible ? "transient" : "persistent") + ", " +
        std::to_string(latency / 1000) + " s";
      scale_row(label, run_decide(ablation_set, feasible, latency, label, csv));
    }
  }
  if (csv) {std::printf("\nCSV written to %s\n", argv[3]);}
  return 0;
}
