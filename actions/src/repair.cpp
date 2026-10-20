// DWAFP Stage I: single-edit repair candidates (Phase C3). See include/actions/repair.hpp.
//
// Running example in the comments (test StageOne.SubstitutionOfTheDownstreamTaskIsBest):
//   T1 (robot1, 5 s, deadline 20 s) --> T2 (robot2, 7.5 s, deadline 14 s); robot3 idle (4 s/task)
//   At t = 2 s robot1 is blocked and predicts T1 at 8 s, so T2 would run 8-15.5 s > 14 s.
//   A_i = {T2}, region = {T1, T2}, B_i = 14 - 2 = 12 s.

#include "actions/repair.hpp"

#include <algorithm>  // std::find, std::max, std::min
#include <chrono>     // timings reported by decide()
#include <stdexcept>  // std::invalid_argument
#include <tuple>      // std::tie (lexicographic comparison)

namespace dwafp
{

namespace
{

// Position of the first unstarted task in a robot queue: everything before it is finished or
// executing, so an inserted task can never overtake the robot's current task (non-preemptive).
std::size_t first_unstarted(const std::vector<Index> & queue, const RuntimeState & state)
{
  std::size_t pos = 0;
  while (pos < queue.size() && state.tasks[queue[pos]].status != TaskStatus::kUnstarted) {++pos;}
  return pos;
}

// True when robot r is running a task right now. A busy robot can still receive a substituted
// task, but only in the next slot, after its current task (non-preemptive).
bool is_busy(const Plan & plan, const RuntimeState & state, Index r)
{
  for (const Index j : plan.queue(r)) {
    if (state.tasks[j].status == TaskStatus::kExecuting) {return true;}
  }
  return false;
}

// Number of the three mandatory predicates violated in one prediction (0 to 3).
int violated_predicates(const TaskPrediction & p)
{
  return (p.deadline_ok ? 0 : 1) + (p.outcome_ok ? 0 : 1) + (p.assignment_ok ? 0 : 1);
}

// True when a predicate holds in `before` and is violated in `after`.
bool breaks_a_predicate(const TaskPrediction & before, const TaskPrediction & after)
{
  return (before.deadline_ok && !after.deadline_ok) ||
         (before.outcome_ok && !after.outcome_ok) ||
         (before.assignment_ok && !after.assignment_ok);
}

// Everything needed to build and evaluate one primitive pi = (policy, k, r_new).
struct Context
{
  const Mission & mission;
  const Plan & incumbent;
  const RuntimeState & actual;
  const DurationSnapshot & durations;
  const ImpactAnalysis & impact;
  const RepairConfig & config;
  StageOneResult & result;
};

// Builds pi, propagates it, checks admissibility and scores it. An admissible candidate is
// appended to result.admissible; a rejected one only increments the matching counter.
void evaluate(const Context & c, RepairPolicy policy, Index k, Index r_new, bool after_current_task)
{
  ++c.result.generated;
  const TimeMs now = c.actual.now_ms;
  const Index r_old = c.incumbent.robot_of(k);
  const TaskStatus status_before = c.actual.tasks[k].status;

  // --- Apply: edited copies of the state and the queues. ------------------------------------
  // T_k becomes unstarted again (a restart if it was executing or failed). Its own runtime entry
  // is cleared; nothing else in the state changes.
  RuntimeState state = c.actual;
  state.tasks[k] = TaskRuntime{};
  // Response window as part of the prediction: the edited task cannot start before the repair is
  // decided and dispatched.
  state.tasks[k].release_ms = now + c.config.decision_overhead_ms + c.config.dispatch_overhead_ms;
  // Same latency semantics as the world: a RUNNING task that moves to another robot keeps
  // occupying its old robot until the edit is activated (it is cancelled only then), so the old
  // robot cannot start its next task before the release time.
  if (status_before == TaskStatus::kExecuting && r_new != r_old) {
    state.robots[r_old].available_from_ms = std::max(state.robots[r_old].available_from_ms, state.tasks[k].release_ms);
  }
  std::vector<std::vector<Index>> queues = c.incumbent.queues();
  auto & old_queue = queues[r_old];
  old_queue.erase(std::find(old_queue.begin(), old_queue.end(), k));

  // Fixed insertion rule: the first position at or after the receiving robot's first unstarted
  // task that keeps the combined graph acyclic. Plan::from_queues rejects cycles by throwing.
  std::optional<Plan> plan;
  const std::size_t front = first_unstarted(queues[r_new], state);
  for (std::size_t pos = front; pos <= queues[r_new].size() && !plan; ++pos) {
    auto trial = queues;
    trial[r_new].insert(trial[r_new].begin() + static_cast<std::ptrdiff_t>(pos), k);
    try {
      plan = Plan::from_queues(c.mission, std::move(trial));
    } catch (const std::invalid_argument &) {
      // this position would make T_k wait for one of its own successors: try the next one
    }
  }
  if (!plan) {
    ++c.result.rejected_cycle;
    return;
  }

  RepairCandidate cand(std::move(*plan));
  cand.policy = policy;
  cand.task = k;
  cand.robot = r_new;
  cand.after_current_task = after_current_task;
  cand.state = std::move(state);

  // --- Propagate through the candidate's validation horizon. --------------------------------
  // Starting point: the prediction AFTER the deviation, BEFORE any repair (same t, same
  // snapshot). Inputs changed by the edit: T_k itself (state, robot, queue predecessor), the task
  // that followed T_k in its old queue, and the task that now follows T_k in its new queue
  // (their queue predecessor changed). incremental() finds everything else.
  std::vector<Index> changed{k};
  const Index old_next = c.incumbent.queue_successor(k);
  const Index new_next = cand.plan.queue_successor(k);
  if (old_next != kNoIndex) {changed.push_back(old_next);}
  if (new_next != kNoIndex && new_next != old_next) {changed.push_back(new_next);}

  const Predictor predictor(c.mission, cand.plan, cand.state, c.durations);
  cand.prediction = c.impact.updated;
  cand.horizon = predictor.incremental(cand.prediction, changed);

  // --- Admissibility (vi): no condition that held before the repair may be violated after it.
  // Only the horizon can have changed, so only the horizon is checked; this is what makes
  // "the local repair creates no violation outside its region" a checked property.
  // The threatened tasks A_i are judged by R instead: every one of their predicates is counted
  // there. (Example: after T1 fails, the held T2 keeps finite times that meet its deadline; a
  // retry restores T2's input but makes it late. That is a residual threat on T2 (R = 1), not a
  // new violation, and the retry stays available for a Stage II pair with a faster T2.)
  std::vector<bool> threatened(c.mission.task_count(), false);
  for (const Index j : c.impact.threatened) {threatened[j] = true;}
  for (const Index j : cand.horizon) {
    if (threatened[j]) {continue;}
    if (breaks_a_predicate(c.impact.updated.tasks[j], cand.prediction.tasks[j])) {
      ++c.result.rejected_new_violation;
      return;
    }
  }

  // --- Scores. -------------------------------------------------------------------------------
  // R: violated predicates over the originally threatened tasks A_i (all three predicates).
  for (const Index j : c.impact.threatened) {
    cand.residual_risk += violated_predicates(cand.prediction.tasks[j]);
  }

  // D: subtasks whose robot, queue predecessor or retry flag changed. Only T_k and its old and
  // new queue successors can have a new robot or queue predecessor; T_k's retry flag changes
  // when it restarts (it was executing or failed).
  std::vector<Index> footprint{k};
  if (old_next != kNoIndex) {footprint.push_back(old_next);}
  if (new_next != kNoIndex && new_next != old_next) {footprint.push_back(new_next);}
  for (const Index j : footprint) {
    const bool moved = c.incumbent.robot_of(j) != cand.plan.robot_of(j);
    const bool requeued = c.incumbent.queue_predecessor(j) != cand.plan.queue_predecessor(j);
    const bool restarted = j == k && status_before != TaskStatus::kUnstarted;
    if (moved || requeued || restarted) {++cand.disruption;}
  }

  // C: discarded progress (work thrown away by cancelling a task that was running on a working
  // robot) + delay the edit adds to any subtask (later finish than before the repair).
  if (status_before == TaskStatus::kExecuting && c.actual.robots[r_old].available) {
    cand.cost_ms += now - c.actual.tasks[k].start_ms;
  }
  for (const Index j : cand.horizon) {
    const TimeMs before = c.impact.updated.tasks[j].finish_ms;
    const TimeMs after = cand.prediction.tasks[j].finish_ms;
    if (is_reachable(before) && is_reachable(after) && after > before) {cand.cost_ms += after - before;}
  }

  // T_restoration: when the repaired task is predicted to finish (its output restored), measured
  // from t. The latency is already in the prediction, so this is only reported.
  const TimeMs f_k = cand.prediction.tasks[k].finish_ms;
  cand.restoration_ms = is_reachable(f_k) ? f_k - now : kUnreachable;

  c.result.admissible.push_back(std::move(cand));
}

}  // namespace

StageOneResult generate_single_edits(
  const Mission & mission, const Plan & incumbent, const RuntimeState & actual,
  const DurationSnapshot & durations, const ImpactAnalysis & impact, const RepairConfig & config)
{
  StageOneResult result;
  if (!impact.needs_repair()) {return result;}  // continue and monitor: nothing to repair
  if (config.decision_overhead_ms < 0 || config.dispatch_overhead_ms < 0) {
    throw std::invalid_argument("repair: overheads must be >= 0");
  }
  if (impact.updated.now_ms != actual.now_ms) {
    throw std::invalid_argument("repair: the impact analysis was made at another time");
  }

  // Hold guard (Algorithm 1): tasks whose input or robot is threatened are not dispatched.
  result.to_hold = tasks_to_hold(impact, actual);

  const Context c{mission, incumbent, actual, durations, impact, config, result};

  // Edit locations. DWAFP: the repair region. The ablation alternatives: only the deviating tasks
  // (sources), or every unfinished task (global single-edit oracle). Completed tasks are never
  // edited (constraint (ii)); every list is in topological order.
  std::vector<Index> targets;
  switch (config.scope) {
    case RepairScope::kRegion:
      targets = impact.region;
      break;
    case RepairScope::kSources:
      for (const Index j : impact.sources) {
        if (actual.tasks[j].status != TaskStatus::kCompleted) {targets.push_back(j);}
      }
      break;
    case RepairScope::kAllUnfinished:
      for (const Index j : incumbent.topological_order()) {
        if (actual.tasks[j].status != TaskStatus::kCompleted) {targets.push_back(j);}
      }
      break;
  }

  // Enumeration order = tie-break order: policy, then target (topological), then robot index.
  // 1. Same-robot retry: a failed or running task that is restartable (retryable), on its
  // operational robot, when the triggering condition is transient or has cleared
  // (same_robot_retry_feasible). A persistent blockage therefore leaves only substitution or escalation.
  for (const Index k : targets) {
    const TaskStatus s = actual.tasks[k].status;
    const Index r = incumbent.robot_of(k);
    if (!mission.task(k).retryable) {continue;}
    if (!actual.tasks[k].same_robot_retry_feasible) {continue;}
    if (s != TaskStatus::kFailed && s != TaskStatus::kExecuting) {continue;}
    if (!actual.robots[r].available) {continue;}
    evaluate(c, RepairPolicy::kRetry, k, r, false);
  }

  // 2. Direct substitution / blocked-successor takeover: an unstarted task, or a retryable
  // running / failed one (it restarts), moved to another available robot. A busy robot receives
  // it in the next slot (after its current task); an idle one immediately.
  for (const Index k : targets) {
    const TaskStatus s = actual.tasks[k].status;
    const bool movable = s == TaskStatus::kUnstarted ||
      ((s == TaskStatus::kExecuting || s == TaskStatus::kFailed) && mission.task(k).retryable);
    if (!movable) {continue;}
    for (Index r = 0; r < mission.robot_count(); ++r) {
      if (r == incumbent.robot_of(k)) {continue;}
      if (!actual.robots[r].available) {continue;}
      const bool busy = is_busy(incumbent, actual, r);
      // Takeover when the robot is idle BECAUSE its next task waits for T_k (blocked successor);
      // otherwise a plain substitution (immediate if idle, next-slot if busy). The edit is the
      // same, only the label differs.
      const auto & q = incumbent.queue(r);
      const std::size_t front = first_unstarted(q, actual);
      bool waits_for_k = false;
      if (!busy && front < q.size()) {
        const auto & preds = mission.dependency_predecessors(q[front]);
        waits_for_k = std::find(preds.begin(), preds.end(), k) != preds.end();
      }
      evaluate(c, waits_for_k ? RepairPolicy::kTakeover : RepairPolicy::kSubstitution, k, r, busy);
    }
  }

  // No gate on the incumbent window B_i^0: it uses the current robots' durations, and a candidate
  // with a faster robot can be preventive even when B_i^0 = 0. Each candidate has already been
  // judged by its own activation-aware propagation (R = 0).

  // BestLexicographic over the preventive candidates: smallest [R, D, C], first generated on ties.
  for (std::size_t i = 0; i < result.admissible.size(); ++i) {
    const RepairCandidate & cand = result.admissible[i];
    if (!cand.preventive()) {continue;}
    if (!result.best) {
      result.best = i;
      continue;
    }
    const RepairCandidate & best = result.admissible[*result.best];
    if (std::tie(cand.residual_risk, cand.disruption, cand.cost_ms) <
      std::tie(best.residual_risk, best.disruption, best.cost_ms))
    {
      result.best = i;
    }
  }
  return result;
}

std::vector<Index> tasks_to_hold(const ImpactAnalysis & impact, const RuntimeState & actual)
{
  std::vector<Index> hold;
  for (const Threat & t : impact.threats) {
    // Only an unstarted task can still be prevented from starting; only a missing input or an
    // unusable robot makes starting INVALID (a deadline threat only makes it late).
    if (actual.tasks[t.task].status != TaskStatus::kUnstarted) {continue;}
    if (t.outcome || t.assignment) {hold.push_back(t.task);}
  }
  return hold;
}

std::vector<Index> unprotectable_threats(
  const Mission & mission, const Plan & incumbent, const RuntimeState & actual,
  const DurationSnapshot & durations, const ImpactAnalysis & impact, const RepairConfig & config)
{
  const std::size_t n = mission.task_count();
  const TimeMs now = actual.now_ms;
  const TimeMs release = now + config.decision_overhead_ms + config.dispatch_overhead_ms;

  // Fastest duration of task j over the robots in `allowed` (kUnreachable if none is allowed).
  const auto fastest = [&](Index j, auto allowed) {
      TimeMs best = kUnreachable;
      for (Index r = 0; r < mission.robot_count(); ++r) {
        if (allowed(r)) {best = std::min(best, durations.at(j, r));}
      }
      return best;
    };
  // start + duration, staying kUnreachable when either part is.
  const auto add = [](TimeMs start, TimeMs p) {
      return (!is_reachable(start) || !is_reachable(p)) ? kUnreachable : start + p;
    };

  std::vector<TimeMs> lb_finish(n, kUnreachable);  // earliest possible finish
  std::vector<bool> output_possible(n, false);     // can its output exist at all?

  // Dependency predecessors come first in the incumbent's topological order (queue edges are
  // ignored by the bound but do not break that property).
  for (const Index j : incumbent.topological_order()) {
    const TaskRuntime & task = actual.tasks[j];
    const Index own = incumbent.robot_of(j);
    const bool own_available = actual.robots[own].available;

    // Inputs: the latest earliest-finish of the dependency predecessors, and whether they can all
    // produce their outputs.
    TimeMs inputs_ready = now;
    bool inputs_possible = true;
    for (const Index h : mission.dependency_predecessors(j)) {
      inputs_ready = std::max(inputs_ready, lb_finish[h]);
      inputs_possible = inputs_possible && output_possible[h];
    }

    // Restart (executing or failed task): a permitted robot, not before the release time.
    const auto restart = [&]() {
        if (!mission.task(j).retryable) {return kUnreachable;}
        const TimeMs p = fastest(j, [&](Index r) {
              if (!actual.robots[r].available) {return false;}
              return r != own || task.same_robot_retry_feasible;  // own robot: retry rule
            });
        return add(std::max(release, inputs_ready), p);
      };

    switch (task.status) {
      case TaskStatus::kCompleted:
        lb_finish[j] = task.end_ms;
        output_possible[j] = true;
        break;
      case TaskStatus::kExecuting: {
          const TimeMs cont = own_available && is_reachable(task.remaining_ms) ? now + task.remaining_ms : kUnreachable;
          lb_finish[j] = std::min(cont, restart());
          output_possible[j] = is_reachable(lb_finish[j]);
          break;
        }
      case TaskStatus::kFailed:
        lb_finish[j] = restart();
        output_possible[j] = is_reachable(lb_finish[j]);
        break;
      case TaskStatus::kUnstarted: {
          const TimeMs p = fastest(j, [&](Index r) {return actual.robots[r].available;});
          lb_finish[j] = add(inputs_ready, p);
          output_possible[j] = is_reachable(lb_finish[j]);
          break;
        }
    }
    output_possible[j] = output_possible[j] && inputs_possible;
  }

  // A repair must restore EVERY predicate of every task of A_i (R = 0): its deadline and its
  // output (the assignment predicate is restored as soon as a usable robot exists, which the
  // reachable bound already requires).
  std::vector<Index> unprotectable;
  for (const Index j : impact.threatened) {
    if (!output_possible[j] || lb_finish[j] > mission.task(j).deadline_ms) {
      unprotectable.push_back(j);
    }
  }
  return unprotectable;
}

Decision decide(
  const Mission & mission, const Plan & incumbent, const RuntimeState & expected,
  const RuntimeState & actual, const DurationSnapshot & durations, const RepairConfig & config)
{
  using Clock = std::chrono::steady_clock;
  const auto us = [](Clock::time_point a, Clock::time_point b) {
      return std::chrono::duration<double, std::micro>(b - a).count();
    };
  Decision d;

  const auto t0 = Clock::now();
  d.impact = analyze_impact(mission, incumbent, expected, actual, durations);
  const auto t1 = Clock::now();
  d.impact_us = us(t0, t1);
  if (!d.impact.needs_repair()) {
    d.status = DecisionStatus::kContinue;
    return d;
  }
  d.to_hold = tasks_to_hold(d.impact, actual);

  // LocalPreventionInfeasible: no search, no Stage II, hold and report.
  d.unprotectable = unprotectable_threats(mission, incumbent, actual, durations, d.impact, config);
  const auto t2 = Clock::now();
  d.check_us = us(t1, t2);
  if (!d.unprotectable.empty()) {
    d.status = DecisionStatus::kNoFeasibleLocalRepair;
    return d;
  }

  d.stage1 = generate_single_edits(mission, incumbent, actual, durations, d.impact, config);
  d.stage1_us = us(t2, Clock::now());
  d.status = d.stage1.best ? DecisionStatus::kRepair : DecisionStatus::kNoSingleEdit;
  return d;
}

}  // namespace dwafp
