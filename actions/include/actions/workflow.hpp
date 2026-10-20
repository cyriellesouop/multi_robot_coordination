// DWAFP workflow model (Phase C1).
//
// ROS-free core library: it only uses the C++ standard library, so the same code runs inside the
// discrete-event simulator, the unit tests and (later) the ROS coordinator adapter.
//
// What C1 provides is the deterministic PREDICTION foundation of the paper:
//   - the mission (subtasks, mandatory dependencies, robots),
//   - the plan (assignment + one fixed queue per robot) and the combined execution graph
//     G = (V, E_D u E_Q), i.e. dependency edges plus robot-queue edges,
//   - the runtime state (unstarted / executing / completed / failed, robot availability),
//   - a frozen duration snapshot p_jr taken from a duration model,
//   - forward propagation of predicted finish times f_hat and predicted outcomes y_hat,
//     in a full version (the oracle) and an incremental version that returns the
//     validation horizon (every task whose prediction changed).
// Repair generation (C3 and later) is built on top of this and is NOT part of C1.
//
// Units: every time in this library is an integer number of NOMINAL milliseconds measured from
// mission start (t = 0). The ROS adapter converts real seconds to nominal ms with time_scale.
// Integers keep predictions exact, so "full == incremental" can be tested with ==.
//
// Typical use (this is what the tests and, later, the simulator do):
//
//   Mission m = load_mission("fork_join.yaml", "mock_fleet.yaml");  // what must be done (static)
//   Plan plan = Plan::initial(m);                // who does what, in which order
//   RuntimeState s = RuntimeState::initial(m);   // what is happening now (t = 0, nothing started)
//   DurationSnapshot p =                         // how long each subtask takes on each robot
//     DurationSnapshot::capture(m, ConfigDurationModel{}, s);
//   Prediction pr = Predictor(m, plan, s, p).full();   // predicted start / finish / outcome
//   std::vector<Index> at_risk = threatened_tasks(s, pr);  // unfinished tasks whose deadline or
//                                                           // outcome predicate fails
//
// When an event changes the state (e.g. a robot reports a delay), the new prediction can be
// obtained with Predictor(m, plan, new_state, p).incremental(pr, {changed tasks}), which only
// recomputes the affected subtasks and returns them (the validation horizon).

#ifndef ACTIONS__WORKFLOW_HPP_
#define ACTIONS__WORKFLOW_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace dwafp
{

// Nominal time in milliseconds since mission start.
using TimeMs = std::int64_t;

// Index of a subtask or a robot inside a Mission (0 .. count-1). Indices are used everywhere
// inside the library because they are cheap to copy and compare; names are only for I/O.
using Index = std::size_t;

// "No task" marker, e.g. the queue predecessor of the first task in a robot queue.
constexpr Index kNoIndex = std::numeric_limits<Index>::max();

// "Never, under the current assignment": the predicted time of a subtask that cannot be executed
// as planned (its robot is unavailable), and of everything that waits for it. It is larger than
// any deadline, so the deadline predicate fails, and it stays unreachable through max() and
// additions, so it never pulls later subtasks forward. Times become finite again only after a
// plan edit (e.g. a reassignment) gives the subtask an available robot.
constexpr TimeMs kUnreachable = std::numeric_limits<TimeMs>::max();

// True for a real (finite) predicted time, false for kUnreachable.
constexpr bool is_reachable(TimeMs t) {return t != kUnreachable;}

// Target pose of a subtask. DWAFP never reasons about it: it is opaque metadata that the adapter
// copies into the ExecuteSubtask goal. Navigation time, if any, is hidden inside p_jr.
struct TargetPose
{
  std::string frame_id{"map"};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// One predefined subtask of the mission (static description, never changed at runtime).
struct Subtask
{
  std::string id;                         // unique name, e.g. "T1"
  std::string robot;                      // initial assignee (robot name)
  double work{5.0};                       // amount of work; p_jr = work * rate_r (see ConfigDurationModel) work is how big the subtask is,
  TimeMs deadline_ms{0};                  // absolute latest completion time from mission start
  std::vector<std::string> predecessors;  // mandatory dependency predecessors (E_D), by subtask id
  bool retryable{true};                   // true if the subtask may be safely restarted
  TargetPose target;                      // opaque, passed to the robot unchanged
};

// One robot of the team. Only its execution rate matters to DWAFP: capabilities are not modelled,
// so every robot may execute every subtask, at its own speed.
struct Robot
{
  std::string name;              // unique name = ROS namespace of the robot, e.g. "robot1"
  double ms_per_work_unit{1000}; // nominal ms needed for one unit of work (mock: step_duration_sec*1000)
};

// The static mission: subtasks, robots and the dependency graph E_D, validated once at construction.
class Mission
{
public:
  // Validates and indexes the mission. Throws std::invalid_argument when:
  // an id or a robot name is empty or duplicated, a subtask names an unknown robot or an unknown
  // predecessor, depends on itself or twice on the same predecessor, has work <= 0 or a
  // deadline <= 0, a robot has a rate <= 0, or the dependency graph E_D contains a cycle.
  Mission(std::string id, std::vector<Subtask> subtasks, std::vector<Robot> robots);

  const std::string & id() const {return id_;}
  std::size_t task_count() const {return subtasks_.size();}
  std::size_t robot_count() const {return robots_.size();}
  const Subtask & task(Index j) const {return subtasks_.at(j);}
  const Robot & robot(Index r) const {return robots_.at(r);}

  // Name -> index lookups. Throw std::out_of_range for an unknown name.
  Index task_index(const std::string & id) const;
  Index robot_index(const std::string & name) const;

  // Robot index of the initial assignee of subtask j (alpha^0_j).
  Index initial_robot(Index j) const {return initial_robot_.at(j);}

  // Dependency edges E_D, as index lists (in the order the predecessors were declared).
  const std::vector<Index> & dependency_predecessors(Index j) const {return dep_pred_.at(j);}
  const std::vector<Index> & dependency_successors(Index j) const {return dep_succ_.at(j);}

private:
  std::string id_;
  std::vector<Subtask> subtasks_;
  std::vector<Robot> robots_;
  std::unordered_map<std::string, Index> task_by_id_;
  std::unordered_map<std::string, Index> robot_by_name_;
  std::vector<Index> initial_robot_;
  std::vector<std::vector<Index>> dep_pred_;
  std::vector<std::vector<Index>> dep_succ_;
};

// Assignment + fixed robot queues (alpha and Q). A repair candidate is a modified copy of a Plan.
// A Plan is always valid by construction: every subtask is in exactly one queue (constraint (i))
// and the combined graph G = (V, E_D u E_Q) is acyclic. Its topological order is computed once
// here and reused by every propagation.
class Plan
{
public:
  // Initial plan: each robot's queue holds its initially assigned subtasks in mission (file) order.
  // Throws std::invalid_argument if this order contradicts the dependencies (cycle in G).
  static Plan initial(const Mission & mission);

  // Plan from explicit queues: queues[r] lists the subtasks of robot r in execution order.
  // Throws std::invalid_argument if the number of queues differs from the number of robots,
  // a subtask index is out of range, a subtask is missing or appears twice, or G has a cycle.
  static Plan from_queues(const Mission & mission, std::vector<std::vector<Index>> queues);

  Index robot_of(Index j) const {return robot_of_.at(j);}
  const std::vector<Index> & queue(Index r) const {return queues_.at(r);}
  const std::vector<std::vector<Index>> & queues() const {return queues_;}

  // Robot-queue edges E_Q: previous / next subtask in the same robot queue, or kNoIndex.
  Index queue_predecessor(Index j) const {return queue_prev_.at(j);}
  Index queue_successor(Index j) const {return queue_next_.at(j);}

  // A topological order of G and the position (rank) of each subtask in it.
  // Every edge h -> j of G satisfies rank(h) < rank(j).
  const std::vector<Index> & topological_order() const {return order_;}
  std::size_t rank(Index j) const {return rank_.at(j);}

  std::size_t task_count() const {return robot_of_.size();}

private:
  Plan() = default;  // only built through initial() / from_queues(), which validate

  std::vector<std::vector<Index>> queues_;
  std::vector<Index> robot_of_;
  std::vector<Index> queue_prev_;
  std::vector<Index> queue_next_;
  std::vector<Index> order_;
  std::vector<std::size_t> rank_;
};

// Runtime status of a subtask, as reported by the execution layer.
enum class TaskStatus
{
  kUnstarted,  // not dispatched yet (waiting, held, or queued)
  kExecuting,  // running on its assigned robot
  kCompleted,  // finished and produced its required output
  kFailed      // finished WITHOUT its required output (outcome failure)
};

// Runtime information of one subtask. Which fields are meaningful depends on the status.
struct TaskRuntime
{
  TaskStatus status{TaskStatus::kUnstarted};
  TimeMs start_ms{0};      // executing / completed / failed: actual start time
  TimeMs remaining_ms{0};  // executing: the robot's estimate r_hat_j(t) of its remaining time
  TimeMs end_ms{0};        // completed: actual finish time; failed: failure time t_fail
  // unstarted: earliest time the task may start, whatever else allows it (0 = no constraint).
  // A repair sets it to t + T_decision + T_dispatch on the task it edits, so the dispatch latency
  // is part of the prediction: it costs nothing when the task would wait for its inputs anyway,
  // and it can make an upstream repair too late.
  TimeMs release_ms{0};
  // Whether the SAME robot could try this task again now: true when the cause of the deviation or
  // failure is expected to clear (transient), false when it is expected to persist FOR THAT ROBOT
  // (persistent). This is a RUNTIME property, different from Subtask::retryable (restarting is
  // logically safe): a navigation task may be restartable, but retrying it while the same
  // obstacle is still there gives the same failure. false excludes same-robot retry only:
  // "persistent" does not mean the task is infeasible for every robot, and substitution remains
  // possible where another robot can execute it. (A cause that blocks EVERY robot, e.g. a shared
  // corridor, is not modelled by this flag: it would make every assignment of the task infeasible.)
  // Default true: retryable deviations are assumed transient unless reported otherwise.
  // The predictor does not read it; only the repair generation (repair.hpp) does.
  bool same_robot_retry_feasible{true};
};

// Runtime information of one robot.
struct RobotRuntime
{
  bool available{true};          // false = robot-unavailability failure (cannot continue or accept work)
  TimeMs available_from_ms{0};   // a_r: earliest time the robot can start new workflow work
  TimeMs unavailable_since_ms{0};  // when available == false: time t_u the robot became unavailable
};

// Runtime state S at time t = now_ms: one entry per subtask and per robot (Mission indices).
struct RuntimeState
{
  TimeMs now_ms{0};
  std::vector<TaskRuntime> tasks;
  std::vector<RobotRuntime> robots;

  // State at mission start: t = 0, every subtask unstarted, every robot available from t = 0.
  static RuntimeState initial(const Mission & mission);
};

// Source of duration estimates p_jr for EVERY (subtask, robot) pair, not only the current assignee,
// because repair candidates need the duration on the replacement robot too.
// Simulator / mock fleet: configuration based. Nav2 (C7): path-time or precomputed pose-to-pose.
class DurationModel
{
public:
  virtual ~DurationModel() = default;

  // Estimated execution duration p_jr (nominal ms, >= 0) of subtask j on robot r in state S.
  virtual TimeMs estimate(
    const Mission & mission, Index j, Index r, const RuntimeState & state) const = 0;
};

// Configuration-based duration model: p_jr = work_j * ms_per_work_unit_r, rounded to the nearest ms.
// This matches the mock robot, where one unit of work is one step of step_duration_sec.
class ConfigDurationModel : public DurationModel
{
public:
  TimeMs estimate(
    const Mission & mission, Index j, Index r, const RuntimeState & state) const override;
};

// Frozen table of p_jr. One snapshot is taken per repair event and used for impact analysis and
// for every candidate, so all candidates are compared with the same estimates even if the live
// model changes meanwhile (and so the computation stays deterministic).
class DurationSnapshot
{
public:
  // Asks the model once for every (subtask, robot) pair.
  // Throws std::invalid_argument if the model returns a negative duration.
  static DurationSnapshot capture(
    const Mission & mission, const DurationModel & model, const RuntimeState & state);

  TimeMs at(Index j, Index r) const {return table_.at(j * robot_count_ + r);}

private:
  std::size_t robot_count_{0};
  std::vector<TimeMs> table_;  // row-major: table_[j * robot_count_ + r]
};

// Prediction for one subtask: predicted start s_hat, finish f_hat (either may be kUnreachable),
// outcome y_hat and the result of its three mandatory predicates:
//   temporal   : f_hat <= d                     (false when f_hat is kUnreachable)
//   outcome    : y_hat = 1                      (its required output is predicted to exist)
//   assignment : its assigned robot can execute it (false for an unfinished subtask whose robot
//                is unavailable; only a plan edit that moves it to an available robot restores it)
struct TaskPrediction
{
  TimeMs start_ms{0};
  TimeMs finish_ms{0};
  bool outcome_ok{true};     // y_hat: true = the required output is predicted to exist
  bool deadline_ok{true};    // temporal predicate f_hat <= d
  bool assignment_ok{true};  // assignment-feasibility predicate: the assigned robot is available
  // Held: unstarted and missing a mandatory input (y_hat = 0), so the execution engine will not
  // dispatch it. Its own times stay finite (deadline analysis), but its robot cannot get past it
  // in its fixed queue: the next task of that queue is predicted unreachable ("stalled").
  bool held{false};

  bool operator==(const TaskPrediction & other) const
  {
    return start_ms == other.start_ms && finish_ms == other.finish_ms &&
           outcome_ok == other.outcome_ok && deadline_ok == other.deadline_ok &&
           assignment_ok == other.assignment_ok && held == other.held;
  }
  bool operator!=(const TaskPrediction & other) const {return !(*this == other);}
};

// Prediction for the whole workflow at time now_ms (one entry per subtask, Mission indices).
struct Prediction
{
  TimeMs now_ms{0};
  std::vector<TaskPrediction> tasks;
};

// Forward propagation over the combined graph G of one Plan.
//
// For every subtask j (f = f_hat, y = y_hat, alpha_j = robot_of(j)):
//   completed : f = actual end,  y_local = 1
//   failed    : f = t_fail,      y_local = 0
//   executing : f = t + r_hat_j(t), y_local = 1
//               (if its robot is unavailable: it stopped at t_u, so f = max(start, t_u),
//                y_local = 0 and the assignment predicate is violated)
//   unstarted : s = max{ t, a_alpha, release_j, f of every dependency predecessor,
//                        f of the queue predecessor }   (release_j: repair dispatch latency, 0 if none)
//               f = s + p_{j,alpha}, y_local = 1
//               (if its robot is unavailable: s = f = kUnreachable, y_local = 0 and the
//                assignment predicate is violated: it can never run under this assignment)
//   y = y_local AND y of every DEPENDENCY predecessor (queue edges carry timing only).
// A failed predecessor therefore keeps finite times downstream (deadline analysis still works)
// while y = 0 marks the tasks that miss a mandatory output and must not be dispatched.
// An unreachable predecessor (dependency or queue) makes its successors unreachable too: the
// input never arrives, or the robot never gets past it in its fixed queue.
// Held tasks (unstarted with y = 0) are never dispatched, so their QUEUE successor is stalled:
// its start is kUnreachable. Their DEPENDENCY successors keep finite times (y = 0 already
// marks them), as required for deadline analysis.
//
// The Predictor keeps REFERENCES to its arguments: they must outlive it and stay unchanged.
class Predictor
{
public:
  // Throws std::invalid_argument if the plan or the state does not match the mission
  // (sizes), or if the state is inconsistent with the fixed queues (see validate_state()).
  Predictor(
    const Mission & mission, const Plan & plan, const RuntimeState & state,
    const DurationSnapshot & durations);

  // Full propagation: every subtask, in topological order. This is the oracle.
  Prediction full() const;

  // Incremental propagation. `prediction` is a previous prediction made at the SAME time now_ms
  // with the SAME duration snapshot; `changed` lists the subtasks whose own inputs changed since
  // (their runtime state, their robot, their queue predecessor, their robot's availability...).
  // Starting from `changed`, it recomputes subtasks in topological order and follows dependency
  // and queue successors only while a prediction actually changes. It updates `prediction` in
  // place and returns the validation horizon: the subtasks whose prediction changed, in
  // topological order. Afterwards `prediction` equals full() (this is what the tests check).
  // Throws std::invalid_argument if prediction.now_ms differs from the state time, the sizes
  // differ, or `changed` contains an invalid index.
  std::vector<Index> incremental(Prediction & prediction, const std::vector<Index> & changed) const;

private:
  // Checks that, in every robot queue, statuses follow the fixed order
  // (completed | failed)* , at most one executing, then unstarted*; and that times are coherent.
  void validate_state() const;

  // Prediction of subtask j, assuming its predecessors in G are already final in `prediction`.
  TaskPrediction predict_task(Index j, const Prediction & prediction) const;

  const Mission & mission_;
  const Plan & plan_;
  const RuntimeState & state_;
  const DurationSnapshot & durations_;
};

// Subtasks that are not finished (unstarted or executing) and have at least one threatened
// mandatory predicate (deadline, outcome or assignment), in index order. Failed subtasks are not listed:
// they are realized failures (the source of the deviation), not predicted threats.
std::vector<Index> threatened_tasks(const RuntimeState & state, const Prediction & prediction);

}  // namespace dwafp

#endif  // ACTIONS__WORKFLOW_HPP_
