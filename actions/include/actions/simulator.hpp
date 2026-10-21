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
//   5. installation of the scheduled edits whose activation time is t (an edit whose task has
//      completed by then is rejected: a completion at t comes first);
//   6. the observer's act(): the ONLY moment a controller may hold tasks or edit the plan;
//   7. dispatch (so a task restarted or moved at t can start at t).

#ifndef ACTIONS__SIMULATOR_HPP_
#define ACTIONS__SIMULATOR_HPP_

#include <cstdint>
#include <string>
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
  // Execution timeout of every attempt: q = timeout_multiplier x nominal duration p_jr of the task
  // on the robot running it. An attempt that would run longer (blocked, slowed down, or
  // persistently obstructed) fails at start + q: the robot reports a TIMEOUT result (no output).
  // This is the robot's own budget (like timeout_sec of the ExecuteSubtask goal), independent of
  // the mission deadline. An attempt that completes EXACTLY at start + q succeeds (the timeout
  // fires only for an attempt that would end later). 0 = no execution timeout.
  double timeout_multiplier{0.0};
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
  kNeverStarted,   // never dispatched (attempts = 0): missing input, held, unusable robot, or a
                   // stuck queue
  kUnstartedAtEnd, // had attempts, but was cancelled or reset by a repair and not run again
  kIncomplete      // started, but still running or blocked when the run ended (horizon)
};

struct TaskRecord
{
  FinalStatus status{FinalStatus::kNeverStarted};
  Index robot{kNoIndex};   // robot of the last attempt
  int attempts{0};
  int timeouts{0};         // attempts ended by the execution timeout
  TimeMs start_ms{0};      // last attempt
  TimeMs end_ms{0};        // last attempt: completion or failure time
  // Cost of the attempts that did not complete (failed, or cancelled by a repair):
  TimeMs discarded_work_ms{0};  // physical progress lost (working time, blocked time excluded)
  TimeMs wasted_elapsed_ms{0};  // elapsed time spent on them (blocked time included)
  bool deadline_ok{false};      // completed, and end_ms <= deadline
  TimeMs deadline_missed_ms{kUnreachable};  // when the deadline miss was observed, if any
};

// ----------------------------------------------------------------------------------------------
// What a controller receives (Step 2 builds controllers on this interface)
// ----------------------------------------------------------------------------------------------

// Counts of the repairs a controller asked for (for the experiments: how many repairs were
// actually put in place, and why the others were not). Each edit is counted exactly once.
struct EditCounts
{
  int scheduled{0};                // schedule_edit() calls (any activation time)
  int installed{0};                // edits installed (scheduled ones and direct apply_edit() calls)
  int rejected_task_completed{0};  // an edited task had completed (before activation)
  int rejected_invalid{0};         // the edit did not fit the state / plan (invalid)
};

class Simulator;

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
  // A scheduled edit reached its activation time: installed, or rejected because an edited task
  // completed before activation or the edit no longer fits the state.
  virtual void on_edit_installed(bool /*installed*/, TimeMs /*time_ms*/) {}
  // A task was not completed by its deadline (reported once, at the deadline). It may still be
  // running, blocked, failed or not started; the miss is recorded for good.
  virtual void on_deadline_missed(Index /*task*/, TimeMs /*time_ms*/) {}
  // Called once per time step, after all the notifications above and before dispatch. A
  // controller decides here and acts through sim.set_held() and sim.apply_edit().
  virtual void act(Simulator & /*sim*/) {}
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

  const Mission & mission() const {return mission_;}
  const EditCounts & edit_counts() const {return edit_counts_;}
  // Per injected deviation (same order as WorldConfig::deviations): did it actually happen?
  //   delay     : an attempt of the source REACHED the obstruction (it was blocked)
  //   failure   : an attempt of the source reached the failure point and failed there
  //   breakdown : the breakdown time was reached
  // A method that moves or cancels the source before that point avoids it (not realized).
  const std::vector<bool> & deviations_realized() const {return realized_;}

  // --- What a controller may change (only from SimulationObserver::act) -----------------------
  // Tasks the dispatcher must not start (hold guard); replaces the previous set.
  void set_held(const std::vector<Index> & tasks);

  // Asks for a step at time t (> now), even if nothing else happens then, so the controller's
  // act() runs at t (a timer: e.g. releasing a hold, or a timeout of a reactive method).
  void request_wakeup(TimeMs t);

  // Schedules a repair decided now, to be INSTALLED at activation_ms = now + T_decision +
  // T_dispatch. Until then the world goes on with the old plan: the running task keeps making
  // progress and the old robot queues stay active. At activation the edit is installed as by
  // apply_edit(), or rejected (on_edit_installed(false)) if an edited task has completed by then
  // or the edit no longer fits the state. activation_ms <= now installs it immediately.
  void schedule_edit(const Plan & new_plan, const std::vector<Index> & edited, TimeMs activation_ms);

  // Installs a repair NOW (zero latency): `new_plan` becomes the incumbent plan, and every task
  // of `edited`
  //   - if running: is CANCELLED (its progress is discarded and counted in discarded_work_ms and
  //     wasted_elapsed_ms) and becomes unstarted, to be dispatched again;
  //   - if failed: becomes unstarted again (a retry);
  //   - if unstarted: stays unstarted (e.g. moved to another queue);
  //   and may start no earlier than release_ms (at least now).
  // Retry diagnostic: same robot -> the current diagnostic is kept; another robot -> the
  // diagnostic of the NEW robot-task pair (feasible unless that pair reported a persistent
  // obstruction before). Nothing changes if the edit is invalid: throws std::invalid_argument
  // when the plan is of another mission, an edited task is completed, a running task changes
  // robot without being edited, or the state does not fit the new queues (Predictor check).
  void apply_edit(const Plan & new_plan, const std::vector<Index> & edited, TimeMs release_ms);

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
    bool fails{false};               // this attempt ends with an outcome failure (or a timeout)
    bool times_out{false};           // ... because the execution timeout was reached
    TimeMs end_ms{0};                // completion or failure time (absolute; kUnreachable if never)
    int block_deviation{-1};         // index of the injected delay that blocks it, or -1
    int failure_deviation{-1};       // index of the injected outcome failure that ends it, or -1
    TimeMs failure_ms{0};            // when that outcome failure happens (if not timed out first)
  };
  // Marks the injected deviations of attempt `a` that have happened by time t.
  void mark_realized(const Attempt & a, TimeMs t);

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
  std::vector<TimeMs> wakeups_;         // times requested by the controller, still to come
  // Edits waiting for their activation time, in scheduling order.
  struct PendingEdit
  {
    Plan plan;
    std::vector<Index> edited;
    TimeMs activation_ms;
  };
  std::vector<PendingEdit> pending_edits_;
  EditCounts edit_counts_;

  // Why an edit was or was not installed.
  enum class EditOutcome {kInstalled, kTaskCompleted, kInvalid};
  // Tries to install an edit, counts the outcome in edit_counts_, and returns it (with the reason
  // in `error` when it is not installed). Nothing changes when it is not installed.
  EditOutcome install(const Plan & new_plan, const std::vector<Index> & edited, TimeMs release_ms,
    std::string * error);
  // The installation itself (throws std::invalid_argument, without side effects, if invalid).
  void install_or_throw(const Plan & new_plan, const std::vector<Index> & edited, TimeMs release_ms);
  // Per (task, robot): the robot reported a persistent obstruction on that task (row-major).
  std::vector<bool> persistent_pair_;
  std::vector<bool> realized_;          // per injected deviation: it happened
  bool started_{false};                 // the first step processes time 0
  bool finished_{false};                // the run is over (statuses finalized)
};

}  // namespace dwafp

#endif  // ACTIONS__SIMULATOR_HPP_
