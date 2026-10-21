// The compared methods as controllers of the closed-loop simulator (Phase C6, Step 3).
//
// Every method sees only what a coordinator sees (feedback, task results, breakdowns, deadline
// misses, the observable state) and acts through Simulator::schedule_edit(), with the same policy
// library, validation, dispatch latency and remaining-time estimator. The architecture is:
//
//   deviation feedback (trigger event)
//     -> source-only / DWAFP / mission-wide attempt PREVENTION (decide(): impact analysis,
//        infeasibility certificate, Stage I; only a preventive repair, R = 0, is executed;
//        otherwise the event is escalated and the trigger of the task is re-armed)
//   actual failure, robot breakdown, or deadline miss (timeout) of an unfinished task
//     -> every repair-enabled method uses the SAME reactive RECOVERY: one edit of the affected
//        task if it lowers, in the prediction, first the number of tasks that would never
//        complete, then the total tardiness (a breakdown hands every unfinished task of the robot
//        to the recovery, one edit each).
//
//   no intervention : neither prevention nor recovery (lower reference)
//   reactive        : recovery only
//   source-only     : prevention restricted to the deviating task + common recovery
//   DWAFP           : prevention inside the repair region + common recovery
//   mission-wide    : prevention on any unfinished task + common recovery (upper reference)
// So DWAFP - reactive measures the value of acting BEFORE failures, not differences in recovery.
//
// Remaining-time estimator (the same for every method), while a robot reports BLOCKED:
//   nominal      : r~ = r_hat (t + r_hat already includes the delay accumulated so far)
//   conservative : r~ = r_hat + b(t), with b(t) the time blocked so far (a heuristic: the
//                  obstruction lasts at least as long again)
// In both, a PERSISTENT obstruction explicitly reported by the robot (same-robot retry not
// feasible) makes the task unreachable on that robot: it will not get past it.

#ifndef ACTIONS__CONTROLLERS_HPP_
#define ACTIONS__CONTROLLERS_HPP_

#include <string>
#include <vector>

#include "actions/impact.hpp"
#include "actions/repair.hpp"
#include "actions/simulator.hpp"
#include "actions/workflow.hpp"

namespace dwafp
{

enum class Method
{
  kNoIntervention,
  kReactive,
  kSourceOnly,
  kDwafp,
  kMissionWide
};

enum class BlockageEstimate
{
  kNominal,      // r~ = r_hat (primary configuration)
  kConservative  // r~ = r_hat + time blocked so far (sensitivity configuration)
};

const char * method_name(Method m);

struct ControllerConfig
{
  Method method{Method::kDwafp};
  TriggerConfig trigger;            // k, epsilon, tau
  TimeMs decision_ms{0};            // T_decision, part of the dispatch latency
  TimeMs dispatch_ms{0};            // T_dispatch
  BlockageEstimate estimate{BlockageEstimate::kNominal};  // see the header comment
};

// What a controller did during one run (for the experiments).
struct ControllerStats
{
  int trigger_events{0};       // DWAFP trigger events (preventive methods)
  int analyses{0};             // decide() calls (one per trigger event batch)
  int continued{0};            // ... nothing newly threatened
  int preventive_repairs{0};   // ... a preventive repair was scheduled
  int no_feasible_local{0};    // ... certificate: no permitted local repair
  int no_single_edit{0};       // ... no preventive single edit
  int fallback_events{0};      // tasks handed to the common recovery (failures, breakdowns, misses)
  int fallback_repairs{0};     // ... that led to an edit
  int errors{0};               // decisions abandoned because a state or edit was invalid (should be 0)
  double decision_us{0};       // total wall-clock time spent deciding
  double max_decision_us{0};
  std::vector<double> decision_times_us;  // every single decision (for latency percentiles)
};

class MethodController : public SimulationObserver
{
public:
  // `mission` must outlive the controller; `initial_plan` gives the first reference prediction.
  MethodController(const Mission & mission, const Plan & initial_plan, ControllerConfig config);

  void on_feedback(const FeedbackSample & sample) override;
  void on_task_finished(Index task, bool success, TimeMs time_ms) override;
  void on_robot_unavailable(Index robot, TimeMs time_ms) override;  // needs the plan: see act()
  void on_deadline_missed(Index task, TimeMs time_ms) override;
  void on_edit_installed(bool installed, TimeMs time_ms) override;
  void act(Simulator & sim) override;

  const ControllerStats & stats() const {return stats_;}
  const ControllerConfig & config() const {return config_;}

private:
  bool preventive() const;
  bool has_fallback() const {return config_.method != Method::kNoIntervention;}
  RepairConfig repair_config() const;
  // The observable state, with the controller's blockage estimate applied.
  RuntimeState observed(const Simulator & sim) const;
  Prediction predict(const Simulator & sim, const RuntimeState & state) const;
  // Preventive decision for deviating tasks; true if a repair was scheduled.
  bool try_preventive(Simulator & sim, const std::vector<Index> & tasks);
  // Fallback recovery of one task; true if an edit was scheduled.
  bool try_recover(Simulator & sim, Index task);
  void schedule(Simulator & sim, const RepairCandidate & candidate);
  void timed(double us);

  const Mission & mission_;
  ControllerConfig config_;
  ControllerStats stats_;
  Prediction reference_;          // the accepted prediction (trigger reference, expected state)
  DeviationTrigger trigger_;
  bool edit_in_flight_{false};    // a scheduled edit waits for its activation
  bool refresh_reference_{false}; // an edit was rejected: rebuild the reference
  std::vector<Index> broken_robots_;  // breakdowns not yet handed to the recovery
  std::vector<Index> pending_trigger_;   // tasks with a trigger event (prevention)
  std::vector<Index> pending_recovery_;  // tasks for the common recovery, in arrival order
  std::vector<TimeMs> blocked_since_;   // per task: start of the current blocked period, or none
  std::vector<TimeMs> last_feedback_;   // per task: time of the last feedback sample
};

}  // namespace dwafp

#endif  // ACTIONS__CONTROLLERS_HPP_
