// Scenario generator for the closed-loop experiments (Phase C6, Step 4).
//
// A scenario = a mission (tasks, dependencies, robots, deadlines), its initial plan (one fixed
// queue per robot, in task order), and the world it runs in (duration noise + ONE injected
// deviation). Everything is drawn from a ScenarioConfig and a seed, so a scenario is reproducible
// and every method can be run on exactly the same one (paired comparison).
//
// Identical distributions in every configuration (only the studied parameter changes):
//   robots   : speeds cycle through 1.0 / 1.5 / 0.8 s per unit of work (robot1, robot2, ...), or
//              all 1.0 with homogeneous_rates (for scaling experiments: cycling speeds would change
//              the fleet composition with the robot count, e.g. 2 -> 3 robots adds the fastest)
//   work     : uniform integer in [work_min, work_max] units
//   deadline : d_j = f0_j + u_j x p_j, u_j ~ U[slack_lo, slack_hi] (duration-normalized additive
//              slack: proportional to the task's own nominal duration p_j, independent of its
//              position in the mission), so the nominal plan always meets every deadline
//   deviation: type drawn from the mix; source = a task starting in the first `window` share of
//              the nominal makespan (with SourceSelection::kWithDependents, only tasks that have at
//              least one dependency successor: the deviation can propagate; if a graph has none,
//              a new graph is drawn deterministically from (seed, attempt), never a fallback to
//              kAny, and make_scenario throws after 100 graphs); it happens at
//              U[0.2, 0.8] of the source's work; a transient delay lasts U[delay_lo, delay_hi] x
//              the source's own nominal duration (relative, so that with the default 0.2..1.0 and
//              a 2x execution budget it stays within the budget except through noise: a transient
//              delay tests prevention, not a timeout); a breakdown hits, at a time in the window, a
//              robot that still has unfinished work
//   timeout  : every attempt has an execution budget of timeout_multiplier x its nominal duration
//              (the robot reports a timeout failure after it), independent of the deadlines
//
// Workflow shapes (task indices are always a topological order, so queue order = task order):
//   random DAG : each earlier task is a predecessor with probability min(0.5, 3/N)
//                (about 1.5 predecessors per task on average)
//   chains     : max(2, R/2) parallel chains, task j in chain j mod C after task j - C
//   fork-join  : fork -> w = min(3, max(2, R)) parallel branches -> join, repeated (the next
//                stage's branches start from the previous join)
//   rendezvous : C = min(4, max(2, R)) lines of work, line c on robot c mod R; every 3rd round the
//                task of line 0 also needs the previous task of every other line (a meeting point)
// Robot assignment has LOCALITY, as in real workflows (a robot mostly continues its own line of
// work, with occasional handoffs):
//   random DAG, chains : a task stays on the robot of its first predecessor with probability 0.7,
//                        otherwise (or without predecessor) it gets a uniformly random robot
//   fork-join          : branch b on robot (fork's robot + b) mod R, so branches run in parallel;
//                        the join on the robot of branch 0
//   rendezvous         : line c on robot c mod R
//
// Deadline structures:
//   every task        : every task has a deadline
//   handoff and final : only tasks whose output is needed on ANOTHER robot (handoff) and tasks
//                       without dependency successors (final results) have one; the others get
//                       kNoDeadline (they only matter through what depends on them)

#ifndef ACTIONS__SCENARIOS_HPP_
#define ACTIONS__SCENARIOS_HPP_

#include <cstdint>
#include <string>

#include "actions/simulator.hpp"
#include "actions/workflow.hpp"

namespace dwafp
{

// "No deadline": far beyond any experiment horizon (about 31 years in ms).
constexpr TimeMs kNoDeadline = 1000LL * 1000 * 1000 * 1000;

inline bool has_deadline(const Mission & m, Index j) {return m.task(j).deadline_ms < kNoDeadline;}

enum class WorkflowShape
{
  kRandomDag,
  kChains,
  kForkJoin,
  kRendezvous
};

enum class DeadlineStructure
{
  kEveryTask,
  kHandoffAndFinal
};

// Which tasks can be the source of a delay or an outcome failure.
enum class SourceSelection
{
  kWithDependents,  // primary: a task with at least one dependency successor (propagation)
  kAny              // secondary, general workload: any task starting in the window
};

// Where the consequences of the deviation can go, by the structure around the source (the
// strongest applicable class).
enum class SourceClass
{
  kSourceOnly,          // no dependency successor and no later task on its robot
  kSameRobotQueue,      // later tasks on its robot, but no dependency successor
  kDownstreamDependency,// dependency descendants, all on its own robot
  kCrossRobotHandoff,   // a dependency DESCENDANT (direct or not) on another robot
  kNotApplicable        // robot breakdown (no source task)
};

const char * source_class_name(SourceClass c);
const char * shape_name(WorkflowShape s);
const char * deadline_structure_name(DeadlineStructure d);

// Relative weights of the deviation types (they do not need to sum to 1).
struct DeviationMix
{
  double transient_delay{0.35};
  double persistent_delay{0.25};
  double outcome_failure{0.25};
  double robot_breakdown{0.15};
};

struct ScenarioConfig
{
  int robots{4};
  int tasks{20};
  WorkflowShape shape{WorkflowShape::kRandomDag};
  DeadlineStructure deadlines{DeadlineStructure::kEveryTask};
  int work_min{3};
  int work_max{7};
  double slack_lo{0.5};
  double slack_hi{3.0};
  DeviationMix mix;
  double delay_lo{0.2};             // transient delay, x the source's nominal duration
  double delay_hi{1.0};
  double window{0.6};               // deviations happen in the first 60 % of the nominal makespan
  SourceSelection source{SourceSelection::kWithDependents};
  bool homogeneous_rates{false};    // all robots 1.0 s per unit of work
  double noise{0.0};                // duration noise of the world, U[1 - noise, 1 + noise]
  double timeout_multiplier{2.0};   // execution timeout of an attempt, x its nominal duration
  TimeMs horizon_ms{600000};        // simulation horizon (10 nominal minutes)
};

struct Scenario
{
  Mission mission;
  Plan plan;
  WorldConfig world;
  InjectedDeviation deviation;      // the injected deviation (also in world.deviations)
  SourceClass source_class{SourceClass::kNotApplicable};
  TimeMs nominal_makespan_ms{0};
  std::string description;          // one line, for logs
};

// Builds the scenario of `seed` for `config`. Throws std::invalid_argument for an invalid
// configuration (robots < 1, tasks < 2, empty work or slack range, all mix weights zero), or when
// no graph with an eligible source can be drawn (kWithDependents).
Scenario make_scenario(const ScenarioConfig & config, std::uint64_t seed);

}  // namespace dwafp

#endif  // ACTIONS__SCENARIOS_HPP_
