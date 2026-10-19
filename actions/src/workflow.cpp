// DWAFP workflow model (Phase C1): validation, plan / combined graph, duration snapshot and
// forward propagation. See include/actions/workflow.hpp for the model and the equations.
//
// Reading guide (the file follows the order in which the objects are used):
//   1. Mission          : checks the static mission (names, robots, dependencies, no cycle).
//   2. Plan             : the robot queues + one topological order of the combined graph.
//   3. RuntimeState,    : what is happening now; how long each subtask would take on each robot.
//      DurationSnapshot
//   4. Predictor        : computes, for every subtask, predicted start / finish / outcome and
//                         whether its deadline and outcome predicates hold.
//      - full()         : recomputes everything (the reference result, the "oracle").
//      - incremental()  : recomputes only what an event changed (the validation horizon).
//   5. threatened_tasks : lists the unfinished subtasks whose predicates fail.
//
// Running example used in the comments: config/missions/fork_join.yaml
//   T1 (robot1, 5 s,   deadline 6 s) ---+
//                                       +--> T3 (robot3, 4 s, deadline 14 s)
//   T2 (robot2, 7.5 s, deadline 9 s) ---+
// Inside the library the subtasks are numbered by their position in the file: T1 = 0, T2 = 1,
// T3 = 2, and the robots by their position in mock_fleet.yaml: robot1 = 0, robot2 = 1, robot3 = 2.

#include "actions/workflow.hpp"

#include <algorithm>   // std::find, std::max
#include <cmath>       // std::llround
#include <functional>  // std::greater (turns priority_queue into a min-heap)
#include <queue>       // std::priority_queue
#include <stdexcept>   // std::invalid_argument, std::out_of_range
#include <utility>     // std::move, std::pair

namespace dwafp
{

// An unnamed namespace makes its content private to this .cpp file (invisible to other files).
namespace
{

// std::priority_queue normally pops the LARGEST element first. With std::greater it pops the
// SMALLEST first (a "min-heap"). Kahn's algorithm below pops the smallest ready index first, so
// the topological order is always the same for the same input, which keeps runs reproducible.
using MinIndexHeap = std::priority_queue<Index, std::vector<Index>, std::greater<Index>>;

}  // namespace

// ----------------------------------------------------------------------------------------------
// 1. Mission
// ----------------------------------------------------------------------------------------------

// The constructor receives the raw lists (as read from YAML) and turns them into a checked,
// indexed mission. If anything is wrong it throws std::invalid_argument, so a Mission object that
// exists is always valid: later code never has to re-check names or cycles.
//
// The ": id_(std::move(id)), ..." part is the member initializer list: it fills the members
// before the body runs. std::move hands over the vectors instead of copying them.
Mission::Mission(std::string id, std::vector<Subtask> subtasks, std::vector<Robot> robots)
: id_(std::move(id)), subtasks_(std::move(subtasks)), robots_(std::move(robots))
{
  // Step 1: robots first, because subtasks refer to them by name.
  for (Index r = 0; r < robots_.size(); ++r) {
    const Robot & robot = robots_[r];  // reference: reads the element without copying it
    if (robot.name.empty()) {
      throw std::invalid_argument("robot #" + std::to_string(r) + " has an empty name");
    }
    // It checks that the robot's speed, ms_per_work_unit, is strictly positive: 0 would mean the robot finishes any subtask instantly
    if (!(robot.ms_per_work_unit > 0.0)) {
      throw std::invalid_argument("robot '" + robot.name + "' must have a rate > 0");
    }
    // robot_by_name_ is a dictionary name -> index ({"robot1": 0, "robot2": 1, "robot3": 2}).
    // emplace() inserts the pair and returns {position, inserted?}; .second is false when the
    // name was already there, i.e. a duplicate robot name.
    if (!robot_by_name_.emplace(robot.name, r).second) {
      throw std::invalid_argument("duplicate robot name '" + robot.name + "'");
    }
  }

  // Step 2: give every subtask id its index ({"T1": 0, "T2": 1, "T3": 2}) BEFORE resolving
  // predecessors, so a predecessor may be declared after the subtask that depends on it.
  for (Index j = 0; j < subtasks_.size(); ++j) {
    const Subtask & task = subtasks_[j];
    if (task.id.empty()) {
      throw std::invalid_argument("subtask #" + std::to_string(j) + " has an empty id");
    }
    if (!task_by_id_.emplace(task.id, j).second) {
      throw std::invalid_argument("duplicate subtask id '" + task.id + "'");
    }
  }

  // Step 3: replace names by indices and build the dependency graph E_D in both directions:
  //   dep_pred_[j] = subtasks j waits for       fork-join: dep_pred_[T3] = {T1, T2}
  //   dep_succ_[h] = subtasks that wait for h   fork-join: dep_succ_[T1] = {T3}, dep_succ_[T2] = {T3}
  // Predecessor lists are used to COMPUTE a subtask; successor lists to know who to update next.
  initial_robot_.resize(subtasks_.size());
  dep_pred_.resize(subtasks_.size());
  dep_succ_.resize(subtasks_.size());
  for (Index j = 0; j < subtasks_.size(); ++j) {
    const Subtask & task = subtasks_[j];
    if (!(task.work > 0.0)) {
      throw std::invalid_argument("subtask '" + task.id + "' must have work > 0");
    }
    if (task.deadline_ms <= 0) {
      throw std::invalid_argument("subtask '" + task.id + "' must have a deadline > 0");
    }
    // find() returns end() when the key is absent: the robot name is unknown.
    const auto robot_it = robot_by_name_.find(task.robot);
    if (robot_it == robot_by_name_.end()) {
      throw std::invalid_argument(
              "subtask '" + task.id + "' is assigned to unknown robot '" + task.robot + "'");
    }
    initial_robot_[j] = robot_it->second;  // ->second is the value of the pair (the index)

    for (const std::string & pred_id : task.predecessors) {
      const auto pred_it = task_by_id_.find(pred_id);
      if (pred_it == task_by_id_.end()) {
        throw std::invalid_argument(
                "subtask '" + task.id + "' depends on unknown subtask '" + pred_id + "'");
      }
      const Index h = pred_it->second;
      if (h == j) {
        throw std::invalid_argument("subtask '" + task.id + "' depends on itself");
      }
      // A duplicated edge would be harmless for timing but is almost certainly a typo.
      if (std::find(dep_pred_[j].begin(), dep_pred_[j].end(), h) != dep_pred_[j].end()) {
        throw std::invalid_argument(
                "subtask '" + task.id + "' lists predecessor '" + pred_id + "' twice");
      }
      dep_pred_[j].push_back(h);  // edge h -> j seen from j. dep_pred_ is used to compute a subtask: "T3 starts after max(f̂ of T1, f̂ of T2)" in predict_task.
      dep_succ_[h].push_back(j);  // the same edge seen from h. dep_succ_ is used to know what to update next: "T1 changed, so T3 must be recomputed" in incremental() and in the topological sort.

    }
  }

  // Step 4: E_D must have no cycle (T1 waits for T2 and T2 waits for T1 could never start).
  // Kahn's algorithm is a method for finding a topological ordering of the vertices in a Directed Acyclic Graph (DAG) using a queue-based approach.
  // It orders the nodes linearly so that for every directed edge u → v, node u comes before node v.
  // Kahn's algorithm:
  //   - in_degree[j] = number of predecessors j still waits for;
  //   - a subtask with in_degree 0 is "ready"; take it out and decrement its successors;
  //   - if some subtask is never ready, it waits (directly or not) for itself: a cycle.
  // Fork-join: T1 and T2 start ready (0); taking T1 leaves T3 at 1; taking T2 leaves T3 at 0,
  // so T3 becomes ready; all 3 visited -> no cycle.
  std::vector<std::size_t> in_degree(subtasks_.size());
  MinIndexHeap ready;
  for (Index j = 0; j < subtasks_.size(); ++j) {
    in_degree[j] = dep_pred_[j].size();
    if (in_degree[j] == 0) {ready.push(j);}
  }
  std::size_t visited = 0;
  //ready holds the subtasks that are ready: everything they wait for has already been placed in the order.
  while (!ready.empty()) {
    const Index h = ready.top();  // smallest ready index
    ready.pop();
    ++visited;
    for (const Index j : dep_succ_[h]) {
      if (--in_degree[j] == 0) {ready.push(j);}  // j's last missing predecessor was h
    }
  }
  if (visited != subtasks_.size()) {
    throw std::invalid_argument("mission '" + id_ + "': the dependency graph contains a cycle");
  }
}

// Name -> index lookups for the code that only knows names (tests, adapter, logs).
Index Mission::task_index(const std::string & id) const
{
  const auto it = task_by_id_.find(id);
  if (it == task_by_id_.end()) {throw std::out_of_range("unknown subtask '" + id + "'");}
  return it->second;
}

Index Mission::robot_index(const std::string & name) const
{
  const auto it = robot_by_name_.find(name);
  if (it == robot_by_name_.end()) {throw std::out_of_range("unknown robot '" + name + "'");}
  return it->second;
}

// ----------------------------------------------------------------------------------------------
// 2. Plan: who executes what, in which order, and the combined graph G = (V, E_D u E_Q)
// ----------------------------------------------------------------------------------------------

// A "static" member function is called on the class, not on an object: Plan::initial(mission).
// It plays the role of a named constructor. The Plan constructor itself is private, so the only
// ways to get a Plan are initial() and from_queues(), which both validate.
Plan Plan::initial(const Mission & mission)
{
  // One empty queue per robot, then each subtask is appended to its initial robot's queue in
  // file order. Fork-join: queues = { robot1: [T1], robot2: [T2], robot3: [T3] }.
  std::vector<std::vector<Index>> queues(mission.robot_count());
  for (Index j = 0; j < mission.task_count(); ++j) {
    queues[mission.initial_robot(j)].push_back(j);
  }
  return from_queues(mission, std::move(queues));
}

Plan Plan::from_queues(const Mission & mission, std::vector<std::vector<Index>> queues)
{
  const std::size_t n = mission.task_count();
  if (queues.size() != mission.robot_count()) {
    throw std::invalid_argument(
            "plan has " + std::to_string(queues.size()) + " queues but the mission has " +
            std::to_string(mission.robot_count()) + " robots");
  }

  // Derived tables, one entry per subtask, filled from the queues below. kNoIndex = "none".
  //   robot_of_[j]   : which robot executes j
  //   queue_prev_[j] : the subtask executed just BEFORE j by the same robot
  //   queue_next_[j] : the subtask executed just AFTER j by the same robot
  // Example: robot1 queue [A, B]  ->  queue_prev_[B] = A, queue_next_[A] = B.
  Plan plan;
  plan.robot_of_.assign(n, kNoIndex);
  plan.queue_prev_.assign(n, kNoIndex);
  plan.queue_next_.assign(n, kNoIndex);

  // Constraint (i): every subtask has exactly one assignee, i.e. appears in exactly one queue once.
  for (Index r = 0; r < queues.size(); ++r) {
    for (std::size_t pos = 0; pos < queues[r].size(); ++pos) {
      const Index j = queues[r][pos];
      if (j >= n) {
        throw std::invalid_argument("plan: subtask index " + std::to_string(j) + " out of range");
      }
      if (plan.robot_of_[j] != kNoIndex) {  // already placed by an earlier queue entry
        throw std::invalid_argument("plan: subtask '" + mission.task(j).id + "' is queued twice");
      }
      plan.robot_of_[j] = r;
      // Queue edge E_Q: previous element of the same queue -> this element.
      if (pos > 0) {
        plan.queue_prev_[j] = queues[r][pos - 1];
        plan.queue_next_[queues[r][pos - 1]] = j;
      }
    }
  }
  for (Index j = 0; j < n; ++j) {
    if (plan.robot_of_[j] == kNoIndex) {  // never placed: nobody would execute it
      throw std::invalid_argument("plan: subtask '" + mission.task(j).id + "' is in no queue");
    }
  }

  // Topological order of the COMBINED graph G = (V, E_D u E_Q), with the same Kahn's algorithm
  // as in Mission, but now a subtask also waits for its queue predecessor (the robot must finish
  // the previous subtask first). So in_degree = dependency predecessors + 1 if it has a queue
  // predecessor.
  // Why it matters: a queue order that contradicts a dependency (T2 queued before its own
  // predecessor T1 on the same robot) creates a cycle T1 -> T2 (dependency) and T2 -> T1 (queue).
  // Such a plan could never execute, so it is rejected here, once, for every caller. This is how
  // repair candidates that would create a cycle will be rejected later.
  std::vector<std::size_t> in_degree(n);
  MinIndexHeap ready;
  for (Index j = 0; j < n; ++j) {
    in_degree[j] = mission.dependency_predecessors(j).size() +
      (plan.queue_prev_[j] != kNoIndex ? 1 : 0);
    if (in_degree[j] == 0) {ready.push(j);}
  }
  // order_ = the subtasks listed so that every predecessor (dependency or queue) comes before
  // its successors. rank_[j] = position of j in order_. Fork-join: order_ = [T1, T2, T3],
  // rank_ = {T1: 0, T2: 1, T3: 2}. Every edge h -> j of G has rank_[h] < rank_[j].
  plan.rank_.assign(n, 0);
  plan.order_.reserve(n);  // reserve: allocate once, the final size is known
  while (!ready.empty()) {
    const Index h = ready.top();
    ready.pop();
    plan.rank_[h] = plan.order_.size();  // its position = how many were placed before it
    plan.order_.push_back(h);
    // Successors of h in G: its dependency successors and its queue successor.
    for (const Index j : mission.dependency_successors(h)) {
      if (--in_degree[j] == 0) {ready.push(j);}
    }
    const Index next = plan.queue_next_[h];
    if (next != kNoIndex && --in_degree[next] == 0) {ready.push(next);}
  }
  if (plan.order_.size() != n) {
    throw std::invalid_argument(
            "plan: the queue order contradicts the dependencies (cycle in the execution graph)");
  }

  plan.queues_ = std::move(queues);
  return plan;
}

// ----------------------------------------------------------------------------------------------
// 3. Runtime state, durations
// ----------------------------------------------------------------------------------------------

// The state at t = 0: nothing has started, every robot is free.
RuntimeState RuntimeState::initial(const Mission & mission)
{
  RuntimeState state;
  state.now_ms = 0;
  state.tasks.assign(mission.task_count(), TaskRuntime{});    // all unstarted (default status)
  state.robots.assign(mission.robot_count(), RobotRuntime{});  // all available from t = 0
  return state;
}

// p_jr = work_j * rate_r. Fork-join, T3 (work 5) on robot3 (800 ms per unit): 5 * 800 = 4000 ms.
// llround rounds to the nearest integer, so 5 * 1500.0 gives exactly 7500 ms.
// The state parameter has no name (/*state*/) because this model does not need it; a Nav2 model
// would use it (for example the robot's current pose) to estimate the time.
TimeMs ConfigDurationModel::estimate(
  const Mission & mission, Index j, Index r, const RuntimeState & /*state*/) const
{
  return static_cast<TimeMs>(std::llround(mission.task(j).work * mission.robot(r).ms_per_work_unit));
}

// Asks the duration model once for EVERY (subtask, robot) pair and stores the answers in a flat
// table. Fork-join (3 subtasks x 3 robots = 9 values):
//              robot1  robot2  robot3
//        T1     5000    7500    4000
//        T2     5000    7500    4000
//        T3     5000    7500    4000
// (all subtasks have work 5 here, so the rows are equal; the robots differ by speed).
// The pairs on OTHER robots than the assignee are what a substitution candidate will use.
DurationSnapshot DurationSnapshot::capture(
  const Mission & mission, const DurationModel & model, const RuntimeState & state)
{
  DurationSnapshot snapshot;
  snapshot.robot_count_ = mission.robot_count();
  snapshot.table_.resize(mission.task_count() * mission.robot_count());
  for (Index j = 0; j < mission.task_count(); ++j) {
    for (Index r = 0; r < mission.robot_count(); ++r) {
      // `model` is a reference to the base class DurationModel: estimate() is virtual, so the
      // call goes to whatever model was passed (ConfigDurationModel today, a Nav2 one later).
      const TimeMs p = model.estimate(mission, j, r, state);
      if (p < 0) {
        throw std::invalid_argument(
                "duration model returned a negative duration for '" + mission.task(j).id +
                "' on '" + mission.robot(r).name + "'");
      }
      // A 2-D table stored in a 1-D vector, row by row: row j, column r -> j * columns + r.
      snapshot.table_[j * snapshot.robot_count_ + r] = p;
    }
  }
  return snapshot;
}

// ----------------------------------------------------------------------------------------------
// 4. Predictor: forward propagation
// ----------------------------------------------------------------------------------------------

// The Predictor stores references to its four inputs (no copy). It checks once that they fit
// together; full() and incremental() can then run many times without re-checking.
Predictor::Predictor(
  const Mission & mission, const Plan & plan, const RuntimeState & state,
  const DurationSnapshot & durations)
: mission_(mission), plan_(plan), state_(state), durations_(durations)
{
  if (plan_.task_count() != mission_.task_count()) {
    throw std::invalid_argument("predictor: the plan does not belong to this mission");
  }
  validate_state();
}

// Rejects a runtime state that the robots could not physically be in. It protects the
// prediction from a bug in whatever produces the state (simulator, ROS adapter).
void Predictor::validate_state() const
{
  if (state_.tasks.size() != mission_.task_count() ||
    state_.robots.size() != mission_.robot_count())
  {
    throw std::invalid_argument("predictor: the runtime state does not match the mission sizes");
  }
  if (state_.now_ms < 0) {throw std::invalid_argument("predictor: now_ms must be >= 0");}

  for (Index r = 0; r < mission_.robot_count(); ++r) {
    const RobotRuntime & robot = state_.robots[r];
    if (!robot.available && robot.unavailable_since_ms > state_.now_ms) {
      throw std::invalid_argument(
              "predictor: robot '" + mission_.robot(r).name + "' became unavailable in the future");
    }

    // A robot executes its fixed queue in order, so reading its queue from left to right the
    // statuses must look like:   finished finished ... [executing] unstarted unstarted ...
    // (finished = completed or failed; at most one executing). Anything else is impossible, for
    // example "B executing while A, before it in the same queue, has not started".
    // `phase` remembers how far along that pattern we are:
    //   0 = only finished subtasks so far, 1 = the executing one was seen, 2 = unstarted ones.
    int phase = 0;
    for (const Index j : plan_.queue(r)) {
      const TaskRuntime & task = state_.tasks[j];
      const std::string & id = mission_.task(j).id;
      switch (task.status) {
        case TaskStatus::kCompleted:
        case TaskStatus::kFailed:  // both cases share the same checks (no break between them)
          if (phase != 0) {
            throw std::invalid_argument(
                    "predictor: '" + id + "' is finished but comes after a task that is not");
          }
          if (task.start_ms > task.end_ms || task.end_ms > state_.now_ms) {
            throw std::invalid_argument(
                    "predictor: '" + id + "' needs start <= end <= now");
          }
          break;
        case TaskStatus::kExecuting:
          if (phase != 0) {  // a second executing task, or one after an unstarted task
            throw std::invalid_argument(
                    "predictor: '" + id + "' is executing but is not the robot's current task");
          }
          if (task.start_ms > state_.now_ms || task.remaining_ms < 0) {
            throw std::invalid_argument(
                    "predictor: '" + id + "' needs start <= now and remaining >= 0");
          }
          phase = 1;
          break;
        case TaskStatus::kUnstarted:
          phase = 2;
          break;
      }
    }
  }
}

// The heart of the model: the prediction of ONE subtask j. It only reads the predictions of j's
// predecessors (dependency predecessors and queue predecessor), which the caller guarantees are
// already final. Both full() and incremental() call this same function, so they apply exactly
// the same equations.
TaskPrediction Predictor::predict_task(Index j, const Prediction & prediction) const
{
  const TaskRuntime & task = state_.tasks[j];
  const Index r = plan_.robot_of(j);  // the robot that executes j in THIS plan
  const RobotRuntime & robot = state_.robots[r];
  const TimeMs now = state_.now_ms;

  TaskPrediction out;
  bool local_ok = true;  // y_local: can this task itself produce its output?

  switch (task.status) {
    case TaskStatus::kCompleted:
      // Known past: actual times, output produced.
      out.start_ms = task.start_ms;
      out.finish_ms = task.end_ms;
      break;

    case TaskStatus::kFailed:
      // Outcome failure: the robot is free again at t_fail (finite time for its queue successor),
      // but the output is missing (y = 0 propagates to dependency successors).
      out.start_ms = task.start_ms;
      out.finish_ms = task.end_ms;
      local_ok = false;
      break;

    case TaskStatus::kExecuting:
      out.start_ms = task.start_ms;
      if (robot.available) {
        // The deviation enters here: f_hat = t + r_hat_j(t), from the robot's own estimate.
        // Fork-join test: at t = 4 s robot1 says "5 s left" for T1 -> f_hat(T1) = 9 s.
        out.finish_ms = now + task.remaining_ms;
      } else {
        // Robot-unavailability failure: the running task stopped when the robot became
        // unavailable (never before it started). That time is real (it is in the past), so it
        // stays finite, but the output is missing and its current assignment is infeasible.
        out.finish_ms = std::max(task.start_ms, robot.unavailable_since_ms);
        local_ok = false;
        out.assignment_ok = false;
      }
      break;

    case TaskStatus::kUnstarted: {
        if (!robot.available) {
          // Its robot is unavailable: under this assignment the task never runs. It gets NO
          // finite time (not "zero duration", which would wrongly meet its deadline and pull
          // the next tasks of the queue forward), no output, and a violated assignment
          // predicate. Only a plan edit moving it to an available robot makes it finite again.
          out.start_ms = kUnreachable;
          out.finish_ms = kUnreachable;
          local_ok = false;
          out.assignment_ok = false;
          break;
        }
        // s_hat = max{ t, a_r, f_hat of dependency predecessors, f_hat of the queue predecessor }.
        // A subtask can only start when ALL of these are true: it is not in the past (t), its
        // robot is free (a_r), all its inputs exist (dependency predecessors), and its robot has
        // finished the previous subtask of its queue. The latest of these times wins.
        // The t term keeps an unstarted task from being predicted to start in the past.
        // Fork-join test: s_hat(T3) = max{4 s, 0, f_hat(T1) = 9 s, f_hat(T2) = 7.5 s} = 9 s.
        TimeMs start = std::max(now, robot.available_from_ms);
        for (const Index h : mission_.dependency_predecessors(j)) {
          start = std::max(start, prediction.tasks[h].finish_ms);
        }
        const Index prev = plan_.queue_predecessor(j);
        if (prev != kNoIndex) {start = std::max(start, prediction.tasks[prev].finish_ms);}
        // If a predecessor is unreachable, max() above already made start = kUnreachable: the
        // input never arrives, or the robot never gets past the previous task of its queue.
        out.start_ms = start;
        // f_hat = s_hat + p_{j, robot}. Fork-join: f_hat(T3) = 9 s + 4 s = 13 s.
        // Saturating addition: an unreachable start stays unreachable, and a sum that would
        // overflow the integer type is treated as unreachable instead of wrapping to a negative.
        const TimeMs p = durations_.at(j, r);
        out.finish_ms = (!is_reachable(start) || p > kUnreachable - start) ? kUnreachable : start + p;
        break;
      }
  }

  // y_hat_j = y_local AND y_hat of every DEPENDENCY predecessor. Queue edges carry timing only:
  // a failed task does not remove the output of the next, unrelated task on the same robot.
  // Chain test: T1 failed -> y_hat(T1) = 0 -> y_hat(T2) = 0 (T2 misses its input).
  bool outcome_ok = local_ok;
  for (const Index h : mission_.dependency_predecessors(j)) {
    outcome_ok = outcome_ok && prediction.tasks[h].outcome_ok;
  }
  out.outcome_ok = outcome_ok;
  // Temporal predicate: f_hat <= d. Fork-join: T3 13 s <= 14 s -> holds; T1 9 s > 6 s -> fails.
  // kUnreachable is larger than any deadline, so an unreachable task always fails it.
  out.deadline_ok = out.finish_ms <= mission_.task(j).deadline_ms;
  return out;
}

// Full propagation: compute every subtask once, in topological order. Because every
// predecessor comes earlier in that order, its prediction is always ready when needed.
// Fork-join: T1, then T2, then T3 (which reads T1 and T2).
Prediction Predictor::full() const
{
  Prediction prediction;
  prediction.now_ms = state_.now_ms;
  prediction.tasks.resize(mission_.task_count());
  for (const Index j : plan_.topological_order()) {
    prediction.tasks[j] = predict_task(j, prediction);
  }
  return prediction;
}

// Incremental propagation: same equations as full(), but it only visits the subtasks that an
// event can affect, and stops on a branch as soon as a prediction does not change.
//
// Fork-join, T1 delayed to end at 6 s (test DelayHiddenByTheSlowerBranchStopsAtT1):
//   start: worklist = [T1]
//   pop T1: 5 s -> 6 s, changed -> horizon [T1], add its successor T3 to the worklist
//   pop T3: s_hat = max{4 s, 6 s, 7.5 s} = 7.5 s as before -> unchanged -> stop here
//   result: horizon = [T1]. T2 was never visited.
// With T1 ending at 9 s instead, T3 moves to 9 s - 13 s and the horizon is [T1, T3].
std::vector<Index> Predictor::incremental(
  Prediction & prediction, const std::vector<Index> & changed) const
{
  // The t floor in s_hat depends on now_ms and every p_jr on the snapshot: an old prediction made
  // at another time would make "unchanged" meaningless, so it is refused rather than silently wrong.
  if (prediction.now_ms != state_.now_ms) {
    throw std::invalid_argument("incremental: the prediction was made at another time");
  }
  if (prediction.tasks.size() != mission_.task_count()) {
    throw std::invalid_argument("incremental: the prediction does not match the mission size");
  }

  // The worklist holds the subtasks still to (re)compute, as (rank, index) pairs. Pairs are
  // compared by their first element, so the min-heap pops the smallest RANK first, i.e. it walks
  // the graph in topological order. When j is popped, every predecessor of j that was in the
  // worklist has a smaller rank and is already final, so j is computed exactly once.
  using RankedTask = std::pair<std::size_t, Index>;
  std::priority_queue<RankedTask, std::vector<RankedTask>, std::greater<RankedTask>> worklist;
  // queued[j] avoids adding j twice (T3 would otherwise be added by both T1 and T2).
  std::vector<bool> queued(mission_.task_count(), false);
  // A lambda = a small local function. [&] lets it use the local variables above by reference.
  const auto enqueue = [&](Index j) {
      if (!queued[j]) {
        queued[j] = true;
        worklist.emplace(plan_.rank(j), j);
      }
    };
  for (const Index j : changed) {
    if (j >= mission_.task_count()) {
      throw std::invalid_argument("incremental: changed index " + std::to_string(j) + " out of range");
    }
    enqueue(j);
  }

  std::vector<Index> horizon;
  while (!worklist.empty()) {
    const Index j = worklist.top().second;  // .second of the pair = the subtask index
    worklist.pop();
    const TaskPrediction updated = predict_task(j, prediction);
    // Traversal of a branch stops as soon as a prediction (time or outcome) does not change:
    // nothing downstream of j can then change because of j.
    if (updated == prediction.tasks[j]) {continue;}
    prediction.tasks[j] = updated;  // overwrite the old value in the caller's prediction
    horizon.push_back(j);  // popped in rank order, so the horizon is in topological order
    // Successors in the combined graph G: dependency successors AND the queue successor.
    // Forgetting the queue successor would miss "A is late so B, next on the same robot, is too".
    for (const Index s : mission_.dependency_successors(j)) {enqueue(s);}
    const Index next = plan_.queue_successor(j);
    if (next != kNoIndex) {enqueue(next);}
  }
  return horizon;
}

// ----------------------------------------------------------------------------------------------
// 5. Threatened subtasks
// ----------------------------------------------------------------------------------------------

// Lists the unfinished subtasks whose deadline or outcome predicate fails. This is the set of
// tasks a repair must protect. Fork-join, T1 ending at 9 s: [T1] (9 s > 6 s); T3 is fine.
std::vector<Index> threatened_tasks(const RuntimeState & state, const Prediction & prediction)
{
  std::vector<Index> threatened;
  for (Index j = 0; j < prediction.tasks.size(); ++j) {
    const TaskStatus status = state.tasks.at(j).status;
    // Only unfinished tasks can be threatened; completed / failed ones are already decided.
    if (status != TaskStatus::kUnstarted && status != TaskStatus::kExecuting) {continue;}
    const TaskPrediction & p = prediction.tasks[j];
    // Any of the three mandatory predicates failing makes the task threatened.
    if (!p.deadline_ok || !p.outcome_ok || !p.assignment_ok) {threatened.push_back(j);}
  }
  return threatened;
}

}  // namespace dwafp
