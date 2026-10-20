// DWAFP closed-loop simulator, Step 1: the "world" (Phase C6).
//
// ROS-free discrete-event simulation that EXECUTES a mission over time, so that methods can be
// compared on what really happens, not on what they predict. It separates strictly:
//   - the GROUND TRUTH, known only to the simulator: the real duration of every attempt (nominal
//     duration x noise), the injected deviations (blockages, outcome failures, robot breakdowns),
//     and whether a blockage cause is transient or persistent;
//   - what a CONTROLLER sees, like a real coordinator: robot feedback (progress-based remaining
//     time estimate r_hat, BLOCKED), task results, robot breakdowns, and the RuntimeState built
//     from them. Controllers never read the ground truth.
//
// Dispatcher (the same for every method): a task starts as soon as its robot is available and
// idle, it is the next unfinished task of that robot's fixed queue, all its dependency
// predecessors COMPLETED, it is not held, and its release time has passed. A robot whose next
// task is not ready waits (fixed queue, as in the predictor).
//
// Robot estimate: r_hat = nominal duration x (share of work still to do). The robot knows its
// nominal speed and its progress, not the noise and not when a blockage will end, so during a
// blockage r_hat stays constant while time passes: the predicted completion t + r_hat slips.
//
// Time is integer nominal ms and advances from event to event. Without noise and deviations the
// executed times equal the predictions of workflow.hpp exactly.
//
// Simultaneous events at the same time t are processed in this FIXED order:
//   1. attempts that end at t (a completion at t counts as completed, even if the robot breaks
//      down at t, and meets a deadline equal to t);
//   2. robot breakdowns at t (the running attempt stops without its output);
//   3. deadline misses at t (every task not completed by its deadline, reported once);
//   4. feedback of the running attempts (if t is a multiple of the feedback period);
//   5. dispatch.

#ifndef ACTIONS__SIMULATOR_HPP_
#define ACTIONS__SIMULATOR_HPP_

#include <cstdint>
#include <vector>

#include "actions/impact.hpp"     // FeedbackSample
#include "actions/workflow.hpp"

namespace dwafp
{

// ----------------------------------------------------------------------------------------------
// Ground truth: what will happen (the scenario), known only to the simulator
// ----------------------------------------------------------------------------------------------

enum class DeviationType
{
  kDelay,           // the robot gets blocked during the task for some time
  kOutcomeFailure,  // the attempt ends WITHOUT its output
  kRobotBreakdown   // the robot becomes unavailable for good
};

struct InjectedDeviation
{
  DeviationType type{DeviationType::kDelay};
  Index task{kNoIndex};     // delay / failure: the affected task
  Index robot{kNoIndex};    // breakdown: the robot that breaks down
  double at_fraction{0.5};  // delay / failure: share of the task's work done when it happens
  TimeMs at_time_ms{0};     // breakdown: absolute time
  TimeMs delay_ms{0};       // transient delay: how long the obstruction lasts (ignored if persistent)
  // The FIRST attempt of the task is hit. The obstruction concerns the robot-task pair of that
  // attempt (another robot executing the task is not affected):
  //   delay, TRANSIENT : one absolute blockage interval [b, b + delay_ms], where b is the time the
  //                      first attempt reaches at_fraction of its work. A later attempt of the
  //                      same pair that reaches that point before the obstruction clears waits
  //                      until the SAME absolute clearing time (it is not a new delay).
  //   delay, PERSISTENT: the pair cannot progress past at_fraction of the work until the horizon;
  //                      every attempt of the pair stops there. Substitution may still work.
  //   failure          : the first attempt fails at at_fraction (persistent: every attempt of the
  //                      pair fails there).
  // When blocked, the robot reports same_robot_retry_feasible = !persistent. This is an ASSUMED
  // robot diagnostic (e.g. from its local recovery behaviour), not knowledge of when the
  // obstruction will clear: the controller never sees the clearing time.
  bool persistent{false};
};

struct WorldConfig
{
  std::uint64_t seed{1};           // noise seed (common random numbers across methods)
  double noise{0.0};               // real duration = nominal x U[1 - noise, 1 + noise], noise < 1
  TimeMs feedback_period_ms{500};  // feedback of every executing task at multiples of this period
  TimeMs horizon_ms{3600000};      // safety stop (one nominal hour)
  std::vector<InjectedDeviation> deviations;
};

// ----------------------------------------------------------------------------------------------
// What really happened
// ----------------------------------------------------------------------------------------------

enum class FinalStatus
{
  kCompleted,    // finished with its output
  kFailed,       // its last attempt failed (outcome failure or robot breakdown)
  kNeverStarted, // never dispatched (missing input, held, unusable robot, or a stuck queue)
  kIncomplete    // started, but still running or blocked when the run ended (horizon)
};

struct TaskRecord
{
  FinalStatus status{FinalStatus::kNeverStarted};
  Index robot{kNoIndex};   // robot of the last attempt
  int attempts{0};
  TimeMs start_ms{0};      // last attempt
  TimeMs end_ms{0};        // last attempt: completion or failure time
  // Cost of the attempts that did not complete (failed or, in Step 2, cancelled):
  TimeMs discarded_work_ms{0};  // physical progress lost (working time, blocked time excluded)
  TimeMs wasted_elapsed_ms{0};  // elapsed time spent on them (blocked time included)
  bool deadline_ok{false};      // completed, and end_ms <= deadline
  TimeMs deadline_missed_ms{kUnreachable};  // when the deadline miss was observed, if any
};

// ----------------------------------------------------------------------------------------------
// What a controller receives (Step 2 builds controllers on this interface)
// ----------------------------------------------------------------------------------------------

class SimulationObserver
{
public:
  virtual ~SimulationObserver() = default;
  // One feedback message of an executing task (like ExecuteSubtask feedback).
  virtual void on_feedback(const FeedbackSample & /*sample*/) {}
  // A task attempt ended; success = completed with its output.
  virtual void on_task_finished(Index /*task*/, bool /*success*/, TimeMs /*time_ms*/) {}
  // A robot became unavailable.
  virtual void on_robot_unavailable(Index /*robot*/, TimeMs /*time_ms*/) {}
  // A task was not completed by its deadline (reported once, at the deadline). It may still be
  // running, blocked, failed or not started; the miss is recorded for good.
  virtual void on_deadline_missed(Index /*task*/, TimeMs /*time_ms*/) {}
};

// ----------------------------------------------------------------------------------------------
// The simulator
// ----------------------------------------------------------------------------------------------

class Simulator
{
public:
  // Keeps a reference to `mission` (it must outlive the simulator) and copies the plan.
  // Throws std::invalid_argument for an invalid configuration or deviation.
  Simulator(const Mission & mission, Plan plan, WorldConfig config);

  // Processes every event of the next event time (dispatching included) and notifies the
  // observer, if any. Returns false when nothing can happen any more (the run is over).
  bool step(SimulationObserver * observer = nullptr);

  // Runs to the end and returns the record.
  const std::vector<TaskRecord> & run(SimulationObserver * observer = nullptr);

  // --- What a controller may read ----------------------------------------------------------
  TimeMs now() const {return state_.now_ms;}
  const Plan & plan() const {return plan_;}
  // The observable state: statuses, actual starts and ends, r_hat of executing tasks (from the
  // last feedback), robot availability, release times, same-robot retry feasibility.
  const RuntimeState & state() const {return state_;}
  const std::vector<TaskRecord> & records() const {return records_;}

  // --- What a controller may change (Step 2 adds plan edits) --------------------------------
  // Tasks the dispatcher must not start (hold guard); replaces the previous set.
  void set_held(const std::vector<Index> & tasks);

private:
  // Ground truth of one running attempt.
  struct Attempt
  {
    bool active{false};
    TimeMs start_ms{0};
    TimeMs true_duration_ms{0};      // working time needed (nominal x noise)
    TimeMs nominal_duration_ms{0};   // what the robot believes (p_jr)
    bool blocks{false};              // this attempt gets blocked
    TimeMs block_start_ms{0};        // absolute
    TimeMs block_end_ms{0};          // absolute; kUnreachable for a persistent obstruction
    bool fails{false};               // this attempt ends with an outcome failure
    TimeMs end_ms{0};                // completion or failure time (absolute; kUnreachable if never)
  };

  TimeMs true_duration(Index j, Index r, int attempt) const;
  TimeMs worked_ms(const Attempt & a, TimeMs t) const;   // working time done at time t
  void start_attempt(Index j, TimeMs t);
  void finish_attempt(Index j, bool success, TimeMs t, SimulationObserver * observer);
  void dispatch(TimeMs t);
  TimeMs next_event_time() const;

  const Mission & mission_;
  Plan plan_;
  WorldConfig config_;
  RuntimeState state_;
  std::vector<TaskRecord> records_;
  std::vector<Attempt> attempts_;       // per task: the running attempt
  std::vector<bool> held_;
  std::vector<Index> first_attempt_robot_;  // robot of attempt 1 (for persistent causes)
  std::vector<bool> breakdown_done_;    // per deviation index
  std::vector<TimeMs> clear_ms_;        // per deviation: absolute clearing time of a transient
                                        // obstruction, once the first attempt has reached it
  std::vector<bool> deadline_reported_; // per task: miss already reported
  bool started_{false};                 // the first step processes time 0
  bool finished_{false};                // the run is over (statuses finalized)
};

}  // namespace dwafp

#endif  // ACTIONS__SIMULATOR_HPP_
