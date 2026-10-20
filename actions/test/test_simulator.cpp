// Unit tests of the closed-loop simulator, Step 1: the world (Phase C6).
//
// Durations from config/mock_fleet.yaml: 5 s per task on robot1, 7.5 s on robot2, 4 s on robot3.
// Each test runs a mission in the simulator and compares WHAT HAPPENED with values computed by
// hand or with the C1 prediction.

#include <gtest/gtest.h>

#include <functional>
#include <random>
#include <string>
#include <vector>

#include "actions/mission_loader.hpp"
#include "actions/simulator.hpp"
#include "actions/workflow.hpp"

using dwafp::DeviationType;
using dwafp::FeedbackSample;
using dwafp::FinalStatus;
using dwafp::Index;
using dwafp::InjectedDeviation;
using dwafp::Mission;
using dwafp::Plan;
using dwafp::Simulator;
using dwafp::TaskRecord;
using dwafp::TimeMs;
using dwafp::WorldConfig;

namespace
{

const std::string kConfigDir = ACTIONS_CONFIG_DIR;

Mission load(const std::string & file)
{
  return dwafp::load_mission(kConfigDir + "/missions/" + file, kConfigDir + "/mock_fleet.yaml");
}

Mission inline_mission(const std::string & yaml)
{
  return dwafp::mission_from_yaml(yaml, "robots: [{name: robot1, step_duration_sec: 1.0},"
                                        " {name: robot2, step_duration_sec: 1.5},"
                                        " {name: robot3, step_duration_sec: 0.8}]");
}

// Records everything a controller would receive.
struct Recorder : dwafp::SimulationObserver
{
  std::vector<FeedbackSample> feedback;
  std::vector<std::pair<Index, bool>> finished;
  std::vector<Index> broken;
  std::vector<std::pair<Index, TimeMs>> missed;  // (task, time) of every deadline miss
  void on_deadline_missed(Index task, TimeMs t) override {missed.emplace_back(task, t);}
  void on_feedback(const FeedbackSample & s) override {feedback.push_back(s);}
  void on_task_finished(Index task, bool success, TimeMs) override {finished.emplace_back(task, success);}
  void on_robot_unavailable(Index robot, TimeMs) override {broken.push_back(robot);}

  // The feedback sample of `task` at time t (fails the test if there is none).
  FeedbackSample at(Index task, TimeMs t) const
  {
    for (const auto & s : feedback) {
      if (s.task == task && s.time_ms == t) {return s;}
    }
    ADD_FAILURE() << "no feedback for task " << task << " at " << t;
    return {};
  }
};

// Executed times must equal the C1 prediction at t = 0 when nothing deviates.
void expect_matches_prediction(const Mission & m)
{
  const Plan plan = Plan::initial(m);
  const dwafp::RuntimeState s0 = dwafp::RuntimeState::initial(m);
  const auto p = dwafp::DurationSnapshot::capture(m, dwafp::ConfigDurationModel{}, s0);
  const dwafp::Prediction pr = dwafp::Predictor(m, plan, s0, p).full();
  Simulator sim(m, plan, WorldConfig{});
  const std::vector<TaskRecord> & rec = sim.run();
  for (Index j = 0; j < m.task_count(); ++j) {
    EXPECT_EQ(rec[j].status, FinalStatus::kCompleted) << m.task(j).id;
    EXPECT_EQ(rec[j].start_ms, pr.tasks[j].start_ms) << m.task(j).id;
    EXPECT_EQ(rec[j].end_ms, pr.tasks[j].finish_ms) << m.task(j).id;
    EXPECT_EQ(rec[j].deadline_ok, pr.tasks[j].deadline_ok) << m.task(j).id;
  }
}

}  // namespace

// Without noise or deviation the world does exactly what C1 predicts, on the demo missions and on
// random missions (dependencies and queue edges, waiting robots).
TEST(Simulator, NominalRunMatchesThePrediction)
{
  expect_matches_prediction(load("chain.yaml"));
  expect_matches_prediction(load("fork_join.yaml"));
  std::mt19937 rng(3);
  const auto uniform = [&rng](int lo, int hi) {return std::uniform_int_distribution<int>(lo, hi)(rng);};
  const char * robots[] = {"robot1", "robot2", "robot3"};
  for (int round = 0; round < 50; ++round) {
    std::string yaml = "{mission_id: r, subtasks: [";
    for (int j = 0; j < 12; ++j) {
      yaml += std::string(j ? "," : "") + "{id: T" + std::to_string(j) + ", robot: " +
        robots[uniform(0, 2)] + ", work: " + std::to_string(uniform(2, 7)) +
        ", deadline_sec: " + std::to_string(uniform(10, 80)) + ", predecessors: [";
      bool first = true;
      for (int h = 0; h < j; ++h) {
        if (uniform(0, 3) == 0) {yaml += std::string(first ? "" : ",") + "T" + std::to_string(h); first = false;}
      }
      yaml += "]}";
    }
    yaml += "]}";
    SCOPED_TRACE("round " + std::to_string(round));
    expect_matches_prediction(inline_mission(yaml));
  }
}

// Noise is reproducible (same seed, same run), varies with the seed, and stays in its bounds.
TEST(Simulator, NoiseIsReproducibleAndBounded)
{
  const Mission m = load("fork_join.yaml");
  WorldConfig config;
  config.noise = 0.2;
  config.seed = 11;
  Simulator a(m, Plan::initial(m), config);
  Simulator b(m, Plan::initial(m), config);
  const auto & ra = a.run();
  const auto & rb = b.run();
  config.seed = 12;
  Simulator c(m, Plan::initial(m), config);
  const auto & rc = c.run();
  bool differs = false;
  for (Index j = 0; j < m.task_count(); ++j) {
    EXPECT_EQ(ra[j].end_ms, rb[j].end_ms);
    differs = differs || ra[j].end_ms != rc[j].end_ms;
    const TimeMs nominal = dwafp::ConfigDurationModel{}.estimate(m, j, ra[j].robot, a.state());
    const TimeMs took = ra[j].end_ms - ra[j].start_ms;
    EXPECT_GE(took, static_cast<TimeMs>(0.8 * nominal) - 1);
    EXPECT_LE(took, static_cast<TimeMs>(1.2 * nominal) + 1);
  }
  EXPECT_TRUE(differs);
}

// A transient blockage of T1 at 40 % of its work, for 3 s:
//   T1 works 0-2 s, blocked 2-5 s, works 5-8 s -> ends 8 s; T2 then runs 8-15.5 s.
// Feedback: while blocked, r_hat stays at 5 s x 60 % = 3 s (the robot does not know when the
// blockage ends), and the robot reports that its own retry is feasible (transient cause).
TEST(Simulator, BlockageDelaysTheTaskAndItsSuccessorExactly)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  const Index t2 = m.task_index("T2");
  WorldConfig config;
  config.deviations.push_back({DeviationType::kDelay, t1, dwafp::kNoIndex, 0.4, 0, 3000, false});
  Simulator sim(m, Plan::initial(m), config);
  Recorder obs;
  const auto & rec = sim.run(&obs);

  EXPECT_EQ(rec[t1].end_ms, 8000);
  EXPECT_EQ(rec[t2].start_ms, 8000);
  EXPECT_EQ(rec[t2].end_ms, 15500);
  EXPECT_FALSE(rec[t1].deadline_ok);  // 8 s > 6 s
  EXPECT_FALSE(rec[t2].deadline_ok);  // 15.5 s > 14 s

  EXPECT_EQ(obs.at(t1, 1000).remaining_ms, 4000);  // 20 % done
  EXPECT_FALSE(obs.at(t1, 1000).blocked);
  for (const TimeMs t : {2000, 3000, 4500}) {
    EXPECT_TRUE(obs.at(t1, t).blocked) << t;
    EXPECT_EQ(obs.at(t1, t).remaining_ms, 3000) << t;  // frozen while blocked
  }
  EXPECT_FALSE(obs.at(t1, 5500).blocked);
  EXPECT_EQ(obs.at(t1, 5500).remaining_ms, 2500);
  EXPECT_TRUE(sim.state().tasks[t1].same_robot_retry_feasible);
}

// A PERSISTENT obstruction: robot1 can never get past 40 % of T1. T1 stays blocked until the
// horizon (60 s here) and ends INCOMPLETE (not "never started"); T2 never starts; the robot
// reports that its own retry is not feasible.
TEST(Simulator, PersistentBlockageLeavesTheTaskIncompleteAtTheHorizon)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  const Index t2 = m.task_index("T2");
  WorldConfig config;
  config.horizon_ms = 60000;
  config.deviations.push_back({DeviationType::kDelay, t1, dwafp::kNoIndex, 0.4, 0, 0, true});
  Simulator sim(m, Plan::initial(m), config);
  Recorder obs;
  const auto & rec = sim.run(&obs);
  EXPECT_EQ(rec[t1].status, FinalStatus::kIncomplete);
  EXPECT_EQ(rec[t2].status, FinalStatus::kNeverStarted);
  EXPECT_FALSE(sim.state().tasks[t1].same_robot_retry_feasible);
  EXPECT_EQ(obs.at(t1, 59500).remaining_ms, 3000);  // still frozen at 60 % left
  EXPECT_TRUE(obs.at(t1, 59500).blocked);
}

// A deadline miss is reported exactly once per task, at the deadline, also while the task is
// still running: in the blockage scenario T1 (d 6 s, ends 8 s) and T2 (d 14 s, runs 8-15.5 s).
TEST(Simulator, DeadlineMissIsReportedOnceAtTheDeadline)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  const Index t2 = m.task_index("T2");
  WorldConfig config;
  config.deviations.push_back({DeviationType::kDelay, t1, dwafp::kNoIndex, 0.4, 0, 3000, false});
  Simulator sim(m, Plan::initial(m), config);
  Recorder obs;
  const auto & rec = sim.run(&obs);
  EXPECT_EQ(obs.missed, (std::vector<std::pair<Index, TimeMs>>{{t1, 6000}, {t2, 14000}}));
  EXPECT_EQ(rec[t1].deadline_missed_ms, 6000);
  EXPECT_EQ(rec[t2].deadline_missed_ms, 14000);
}

// A task that never starts also misses its deadline: T1 fails at 3 s (its deadline 6 s is
// missed too), T2 never starts and misses 14 s.
TEST(Simulator, UnstartedTaskMissesItsDeadline)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  const Index t2 = m.task_index("T2");
  WorldConfig config;
  config.deviations.push_back({DeviationType::kOutcomeFailure, t1, dwafp::kNoIndex, 0.6, 0, 0, false});
  Simulator sim(m, Plan::initial(m), config);
  Recorder obs;
  sim.run(&obs);
  EXPECT_EQ(obs.missed, (std::vector<std::pair<Index, TimeMs>>{{t1, 6000}, {t2, 14000}}));
}

// Fixed event order at one timestamp: robot1 breaks down at 5 s, exactly when T1 completes. The
// completion is processed first, so T1 counts as COMPLETED (deadline met), T2 starts at 5 s on
// robot2 and finishes at 12.5 s; robot1 is unavailable afterwards.
TEST(Simulator, CompletionAndBreakdownAtTheSameTimeFollowTheDeclaredOrder)
{
  const Mission m = load("chain.yaml");
  WorldConfig config;
  InjectedDeviation breakdown;
  breakdown.type = DeviationType::kRobotBreakdown;
  breakdown.robot = m.robot_index("robot1");
  breakdown.at_time_ms = 5000;
  config.deviations.push_back(breakdown);
  Simulator sim(m, Plan::initial(m), config);
  Recorder obs;
  const auto & rec = sim.run(&obs);
  EXPECT_EQ(rec[m.task_index("T1")].status, FinalStatus::kCompleted);
  EXPECT_TRUE(rec[m.task_index("T1")].deadline_ok);
  EXPECT_EQ(rec[m.task_index("T2")].end_ms, 12500);
  EXPECT_FALSE(sim.state().robots[m.robot_index("robot1")].available);
  EXPECT_TRUE(obs.missed.empty());
}

// Discarded work vs wasted elapsed time: T1 works 2 s (40 %), is blocked 5 s, and robot1 breaks
// down at 7 s. 2 s of progress are discarded, but 7 s of time were spent on the attempt.
TEST(Simulator, DiscardedWorkIsSeparatedFromWastedElapsedTime)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  WorldConfig config;
  config.deviations.push_back({DeviationType::kDelay, t1, dwafp::kNoIndex, 0.4, 0, 5000, false});
  InjectedDeviation breakdown;
  breakdown.type = DeviationType::kRobotBreakdown;
  breakdown.robot = m.robot_index("robot1");
  breakdown.at_time_ms = 7000;
  config.deviations.push_back(breakdown);
  Simulator sim(m, Plan::initial(m), config);
  const auto & rec = sim.run();
  EXPECT_EQ(rec[t1].status, FinalStatus::kFailed);
  EXPECT_EQ(rec[t1].discarded_work_ms, 2000);
  EXPECT_EQ(rec[t1].wasted_elapsed_ms, 7000);
}

// Outcome failure of T1 at 60 % of its work: it fails at 3 s without its output. T2 never starts
// (its input is missing), and the run ends on its own.
TEST(Simulator, OutcomeFailureStopsTheDependentTask)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  const Index t2 = m.task_index("T2");
  WorldConfig config;
  config.deviations.push_back({DeviationType::kOutcomeFailure, t1, dwafp::kNoIndex, 0.6, 0, 0, false});
  Simulator sim(m, Plan::initial(m), config);
  Recorder obs;
  const auto & rec = sim.run(&obs);
  EXPECT_EQ(rec[t1].status, FinalStatus::kFailed);
  EXPECT_EQ(rec[t1].end_ms, 3000);
  EXPECT_EQ(rec[t1].discarded_work_ms, 3000);
  EXPECT_EQ(rec[t1].wasted_elapsed_ms, 3000);
  EXPECT_EQ(rec[t2].status, FinalStatus::kNeverStarted);
  ASSERT_EQ(obs.finished.size(), 1u);
  EXPECT_EQ(obs.finished[0], std::make_pair(t1, false));
}

// robot1 (queue A, B) breaks down at 2 s. A stops without its output, B never starts, C (needs B)
// never starts; D on robot2 is unaffected and completes at 7.5 s.
TEST(Simulator, RobotBreakdownStopsItsQueue)
{
  const Mission m = inline_mission(
    "{mission_id: queue, subtasks: ["
    " {id: A, robot: robot1, deadline_sec: 6},"
    " {id: D, robot: robot2, deadline_sec: 8},"
    " {id: B, robot: robot1, deadline_sec: 11},"
    " {id: C, robot: robot2, deadline_sec: 20, predecessors: [B]}]}");
  WorldConfig config;
  InjectedDeviation breakdown;
  breakdown.type = DeviationType::kRobotBreakdown;
  breakdown.robot = m.robot_index("robot1");
  breakdown.at_time_ms = 2000;
  config.deviations.push_back(breakdown);
  Simulator sim(m, Plan::initial(m), config);
  Recorder obs;
  const auto & rec = sim.run(&obs);
  EXPECT_EQ(rec[m.task_index("A")].status, FinalStatus::kFailed);
  EXPECT_EQ(rec[m.task_index("A")].end_ms, 2000);
  EXPECT_EQ(rec[m.task_index("B")].status, FinalStatus::kNeverStarted);
  EXPECT_EQ(rec[m.task_index("C")].status, FinalStatus::kNeverStarted);
  EXPECT_EQ(rec[m.task_index("D")].status, FinalStatus::kCompleted);
  EXPECT_EQ(rec[m.task_index("D")].end_ms, 7500);
  EXPECT_EQ(obs.broken, (std::vector<Index>{m.robot_index("robot1")}));
  EXPECT_FALSE(sim.state().robots[m.robot_index("robot1")].available);
}

// A held task is not dispatched, even when it is ready.
TEST(Simulator, HeldTaskIsNotDispatched)
{
  const Mission m = load("chain.yaml");
  Simulator sim(m, Plan::initial(m), WorldConfig{});
  sim.set_held({m.task_index("T2")});
  const auto & rec = sim.run();
  EXPECT_EQ(rec[m.task_index("T1")].status, FinalStatus::kCompleted);
  EXPECT_EQ(rec[m.task_index("T2")].status, FinalStatus::kNeverStarted);
}

// Fixed queue: robot2 runs T2 (needs T1) then E (independent). E is ready at 0 s but waits for
// T2: robot2 idles until T1 ends at 5 s, runs T2 5-12.5 s, then E 12.5-20 s.
TEST(Simulator, RobotWaitsForTheNextTaskOfItsFixedQueue)
{
  const Mission m = inline_mission(
    "{mission_id: wait, subtasks: ["
    " {id: T1, robot: robot1, deadline_sec: 6},"
    " {id: T2, robot: robot2, deadline_sec: 14, predecessors: [T1]},"
    " {id: E, robot: robot2, deadline_sec: 30}]}");
  Simulator sim(m, Plan::initial(m), WorldConfig{});
  const auto & rec = sim.run();
  EXPECT_EQ(rec[m.task_index("T2")].start_ms, 5000);
  EXPECT_EQ(rec[m.task_index("E")].start_ms, 12500);
  EXPECT_EQ(rec[m.task_index("E")].end_ms, 20000);
}

// =============================================================================================
// Step 2: controller interface (act(), set_held, apply_edit)
// =============================================================================================

namespace
{
// A controller that performs one scripted action at the first step at or after `at`.
struct Scripted : dwafp::SimulationObserver
{
  TimeMs at{0};
  std::function<void(Simulator &)> action;
  bool done{false};
  std::vector<std::pair<bool, TimeMs>> installs;  // results of scheduled edits
  Scripted(TimeMs when, std::function<void(Simulator &)> what) : at(when), action(std::move(what)) {}
  void on_edit_installed(bool installed, TimeMs t) override {installs.emplace_back(installed, t);}
  void act(Simulator & sim) override
  {
    if (!done && sim.now() >= at) {
      done = true;
      action(sim);
    }
  }
};

// chain.yaml with a blockage of T1 at 40 % of its work (2 s in), for `delay` ms.
WorldConfig blocked_chain(const Mission & m, TimeMs delay, bool persistent)
{
  WorldConfig config;
  config.horizon_ms = 60000;
  config.deviations.push_back({DeviationType::kDelay, m.task_index("T1"), dwafp::kNoIndex, 0.4, 0, delay, persistent});
  return config;
}
}  // namespace

// Transient obstruction of T1: blocked at 2 s, it clears at the ABSOLUTE time 7 s.
// A retry at 3 s (same robot) reaches the obstruction at 3 + 2 = 5 s, before it clears, so it
// waits only until 7 s (not for a new 5 s delay), then works 3 s: T1 ends 10 s, T2 runs 10-17.5 s.
// The cancelled first attempt: 2 s of progress discarded, 3 s of time wasted.
TEST(SimulatorControl, RetryBeforeTheObstructionClearsWaitsForTheOriginalClearingTime)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  Simulator sim(m, Plan::initial(m), blocked_chain(m, 5000, false));
  Scripted retry(3000, [&](Simulator & s) {s.apply_edit(s.plan(), {t1}, s.now());});
  const auto & rec = sim.run(&retry);
  EXPECT_EQ(rec[t1].attempts, 2);
  EXPECT_EQ(rec[t1].start_ms, 3000);
  EXPECT_EQ(rec[t1].end_ms, 10000);
  EXPECT_EQ(rec[t1].discarded_work_ms, 2000);
  EXPECT_EQ(rec[t1].wasted_elapsed_ms, 3000);
  EXPECT_EQ(rec[m.task_index("T2")].end_ms, 17500);
}

// Same obstruction, retry at 6 s: the new attempt reaches the obstruction at 8 s, after it cleared
// at 7 s, so it is not blocked at all: T1 runs 6-11 s.
TEST(SimulatorControl, RetryAfterTheObstructionClearsIsNotBlocked)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  Simulator sim(m, Plan::initial(m), blocked_chain(m, 5000, false));
  Scripted retry(6000, [&](Simulator & s) {s.apply_edit(s.plan(), {t1}, s.now());});
  const auto & rec = sim.run(&retry);
  EXPECT_EQ(rec[t1].end_ms, 11000);
  EXPECT_EQ(rec[t1].wasted_elapsed_ms, 6000);
}

// Persistent obstruction of robot1 on T1. Retrying on robot1 hits it again: T1 ends incomplete.
TEST(SimulatorControl, RetryOnThePersistentlyBlockedPairIsBlockedAgain)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  Simulator sim(m, Plan::initial(m), blocked_chain(m, 0, true));
  Scripted retry(3000, [&](Simulator & s) {s.apply_edit(s.plan(), {t1}, s.now());});
  const auto & rec = sim.run(&retry);
  EXPECT_EQ(rec[t1].attempts, 2);
  EXPECT_EQ(rec[t1].status, FinalStatus::kIncomplete);
}

// Substitution under a persistent obstruction: at 3 s T1 is moved to robot3 with a 0.5 s dispatch
// latency. robot3 is not obstructed: T1 runs 3.5-7.5 s (4 s), then T2 7.5-15 s on robot2.
TEST(SimulatorControl, SubstitutionMovesTheTaskToAnotherRobot)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  const Index t2 = m.task_index("T2");
  Simulator sim(m, Plan::initial(m), blocked_chain(m, 0, true));
  Scripted move(3000, [&](Simulator & s) {
      s.apply_edit(Plan::from_queues(m, {{}, {t2}, {t1}}), {t1}, s.now() + 500);
    });
  const auto & rec = sim.run(&move);
  EXPECT_EQ(rec[t1].status, FinalStatus::kCompleted);
  EXPECT_EQ(rec[t1].robot, m.robot_index("robot3"));
  EXPECT_EQ(rec[t1].start_ms, 3500);
  EXPECT_EQ(rec[t1].end_ms, 7500);
  EXPECT_EQ(rec[t1].discarded_work_ms, 2000);
  EXPECT_EQ(rec[t1].wasted_elapsed_ms, 3000);
  EXPECT_EQ(rec[t2].start_ms, 7500);
  EXPECT_EQ(rec[t2].end_ms, 15000);
}

// Common random numbers: with noise, a controller that retries T1 does not change the noise of
// an unaffected task. In fork_join, T2 (robot2, independent of T1) runs exactly the same with or
// without the retry; T1's second attempt gets its own draw.
TEST(SimulatorControl, ExtraRetriesDoNotChangeTheNoiseOfOtherTasks)
{
  const Mission m = load("fork_join.yaml");
  const Index t1 = m.task_index("T1");
  const Index t2 = m.task_index("T2");
  WorldConfig config;
  config.noise = 0.2;
  config.seed = 5;
  Simulator plain(m, Plan::initial(m), config);
  const auto rec_plain = plain.run();
  Simulator retried(m, Plan::initial(m), config);
  Scripted retry(2000, [&](Simulator & s) {s.apply_edit(s.plan(), {t1}, s.now());});
  const auto rec_retried = retried.run(&retry);
  EXPECT_EQ(rec_retried[t1].attempts, 2);
  EXPECT_EQ(rec_plain[t2].start_ms, rec_retried[t2].start_ms);
  EXPECT_EQ(rec_plain[t2].end_ms, rec_retried[t2].end_ms);
}

// An invalid edit changes nothing: moving the RUNNING T1 to robot3 without restarting it throws,
// and the run then proceeds exactly as planned (T1 0-5 s, T2 5-12.5 s). Editing a completed task
// throws too.
TEST(SimulatorControl, InvalidEditsAreRejectedWithoutSideEffects)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  const Index t2 = m.task_index("T2");
  Simulator sim(m, Plan::initial(m), WorldConfig{});
  bool threw = false;
  Scripted bad(1000, [&](Simulator & s) {
      try {
        s.apply_edit(Plan::from_queues(m, {{}, {t2}, {t1}}), {}, s.now());
      } catch (const std::invalid_argument &) {
        threw = true;
      }
    });
  const auto & rec = sim.run(&bad);
  EXPECT_TRUE(threw);
  EXPECT_EQ(rec[t1].end_ms, 5000);
  EXPECT_EQ(rec[t2].end_ms, 12500);
  EXPECT_THROW(sim.apply_edit(sim.plan(), {t1}, sim.now()), std::invalid_argument);  // completed
  // Both rejections are counted, each with its reason; nothing was installed.
  EXPECT_EQ(sim.edit_counts().installed, 0);
  EXPECT_EQ(sim.edit_counts().rejected_invalid, 1);         // running task moved without restart
  EXPECT_EQ(sim.edit_counts().rejected_task_completed, 1);  // T1 already completed
}

// Hold, then release: a controller holds T2 until 9 s; T2 then starts at 9 s and ends 16.5 s.
TEST(SimulatorControl, HeldTaskStartsWhenReleased)
{
  const Mission m = load("chain.yaml");
  const Index t2 = m.task_index("T2");
  struct HoldUntil : dwafp::SimulationObserver
  {
    Index task;
    TimeMs until;
    void act(Simulator & s) override
    {
      s.set_held(s.now() < until ? std::vector<Index>{task} : std::vector<Index>{});
      // Nothing else happens at 9 s: ask to be called then (only while it is still ahead).
      if (s.now() < until) {s.request_wakeup(until);}
    }
  } hold;
  hold.task = t2;
  hold.until = 9000;
  Simulator sim(m, Plan::initial(m), WorldConfig{});
  const auto & rec = sim.run(&hold);
  EXPECT_EQ(rec[t2].start_ms, 9000);
  EXPECT_EQ(rec[t2].end_ms, 16500);
}

// A task cancelled by a repair and never run again is "unstarted at the end", not "never
// started": T1 is cancelled at 3 s and held for good.
TEST(SimulatorControl, CancelledTaskIsUnstartedAtTheEnd)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  Simulator sim(m, Plan::initial(m), WorldConfig{});
  Scripted cancel(3000, [&](Simulator & s) {
      s.apply_edit(s.plan(), {t1}, s.now());
      s.set_held({t1});
    });
  const auto & rec = sim.run(&cancel);
  EXPECT_EQ(rec[t1].attempts, 1);
  EXPECT_EQ(rec[t1].status, FinalStatus::kUnstartedAtEnd);
}

// Dispatch latency in the world: a move of T1 to robot3 is DECIDED at 1 s and ACTIVATED at 3 s.
// Until 3 s, T1 keeps running on robot1 and makes progress: 3 s of work are discarded at
// activation (not 1 s). T1 then runs 3-7 s on robot3, T2 7-14.5 s.
TEST(SimulatorControl, RunningTaskContinuesDuringTheLatency)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  const Index t2 = m.task_index("T2");
  Simulator sim(m, Plan::initial(m), WorldConfig{});
  Scripted move(1000, [&](Simulator & s) {
      s.schedule_edit(Plan::from_queues(m, {{}, {t2}, {t1}}), {t1}, s.now() + 2000);
    });
  const auto & rec = sim.run(&move);
  EXPECT_EQ(move.installs, (std::vector<std::pair<bool, TimeMs>>{{true, 3000}}));
  EXPECT_EQ(sim.edit_counts().scheduled, 1);
  EXPECT_EQ(sim.edit_counts().installed, 1);
  EXPECT_EQ(rec[t1].discarded_work_ms, 3000);
  EXPECT_EQ(rec[t1].start_ms, 3000);
  EXPECT_EQ(rec[t1].end_ms, 7000);
  EXPECT_EQ(rec[t1].robot, m.robot_index("robot3"));
  EXPECT_EQ(rec[t2].end_ms, 14500);
}

// The old queues stay active until the edit is installed. robot1 runs A then B; at 1 s a move of
// B to robot3 is scheduled for 6 s. A ends at 5 s, so robot1 starts B at 5 s under the OLD plan;
// at 6 s the edit cancels it (1 s discarded) and B runs 6-10 s on robot3.
TEST(SimulatorControl, OldQueuesStayActiveUntilTheEditIsInstalled)
{
  const Mission m = inline_mission(
    "{mission_id: q, subtasks: [{id: A, robot: robot1, deadline_sec: 20},"
    " {id: B, robot: robot1, deadline_sec: 20}]}");
  const Index a = m.task_index("A");
  const Index b = m.task_index("B");
  Simulator sim(m, Plan::initial(m), WorldConfig{});
  Scripted move(1000, [&](Simulator & s) {
      s.schedule_edit(Plan::from_queues(m, {{a}, {}, {b}}), {b}, 6000);
    });
  const auto & rec = sim.run(&move);
  EXPECT_EQ(move.installs, (std::vector<std::pair<bool, TimeMs>>{{true, 6000}}));
  EXPECT_EQ(rec[b].attempts, 2);               // first on robot1 (old plan), then on robot3
  EXPECT_EQ(rec[b].discarded_work_ms, 1000);   // robot1 worked on B from 5 s to 6 s
  EXPECT_EQ(rec[b].start_ms, 6000);
  EXPECT_EQ(rec[b].end_ms, 10000);
  EXPECT_EQ(rec[b].robot, m.robot_index("robot3"));
}

// An edit whose task completes before activation is rejected: a retry of T1 is scheduled at 1 s
// for 6 s, but T1 completes at 5 s. Nothing changes: T1 0-5 s (1 attempt), T2 5-12.5 s.
TEST(SimulatorControl, EditIsRejectedIfItsTaskCompletesBeforeActivation)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  Simulator sim(m, Plan::initial(m), WorldConfig{});
  Scripted retry(1000, [&](Simulator & s) {s.schedule_edit(s.plan(), {t1}, 6000);});
  const auto & rec = sim.run(&retry);
  EXPECT_EQ(retry.installs, (std::vector<std::pair<bool, TimeMs>>{{false, 6000}}));
  EXPECT_EQ(sim.edit_counts().scheduled, 1);
  EXPECT_EQ(sim.edit_counts().installed, 0);
  EXPECT_EQ(sim.edit_counts().rejected_task_completed, 1);
  EXPECT_EQ(sim.edit_counts().rejected_invalid, 0);
  EXPECT_EQ(rec[t1].attempts, 1);
  EXPECT_EQ(rec[t1].end_ms, 5000);
  EXPECT_EQ(rec[m.task_index("T2")].end_ms, 12500);
}

// The retry diagnostic belongs to the (task, robot) pair. robot1 reports a persistent obstruction
// on T1 (retry infeasible). Moved to robot3, T1's diagnostic is robot3's: feasible. Moved back to
// robot1, the remembered pair diagnostic applies again: infeasible.
TEST(SimulatorControl, RetryDiagnosticFollowsTheRobotTaskPair)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  const Index t2 = m.task_index("T2");
  struct MoveTwice : dwafp::SimulationObserver
  {
    const Mission * m{nullptr};
    Index t1{0}, t2{0};
    std::vector<bool> seen;  // diagnostic right after each move
    void act(Simulator & s) override
    {
      if (s.now() == 3000) {
        s.apply_edit(Plan::from_queues(*m, {{}, {t2}, {t1}}), {t1}, s.now());
        seen.push_back(s.state().tasks[t1].same_robot_retry_feasible);
      } else if (s.now() == 4000) {
        s.apply_edit(Plan::from_queues(*m, {{t1}, {t2}, {}}), {t1}, s.now());
        seen.push_back(s.state().tasks[t1].same_robot_retry_feasible);
      }
    }
  } mover;
  mover.m = &m;
  mover.t1 = t1;
  mover.t2 = t2;
  Simulator sim(m, Plan::initial(m), blocked_chain(m, 0, true));
  sim.run(&mover);
  EXPECT_EQ(mover.seen, (std::vector<bool>{true, false}));
}

// Execution timeout: with a budget of 2 x the nominal duration, the persistently obstructed T1
// (5 s nominal on robot1) is given up at 10 s: the robot reports a failure (timeout), 2 s of
// progress are discarded and 10 s were spent. T2 never starts (its input is missing).
TEST(Simulator, ExecutionTimeoutEndsAStuckAttempt)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  WorldConfig config;
  config.timeout_multiplier = 2.0;
  config.deviations.push_back({DeviationType::kDelay, t1, dwafp::kNoIndex, 0.4, 0, 0, true});
  Simulator sim(m, Plan::initial(m), config);
  Recorder obs;
  const auto & rec = sim.run(&obs);
  EXPECT_EQ(rec[t1].status, FinalStatus::kFailed);
  EXPECT_EQ(rec[t1].timeouts, 1);
  EXPECT_EQ(rec[t1].end_ms, 10000);
  EXPECT_EQ(rec[t1].discarded_work_ms, 2000);
  EXPECT_EQ(rec[t1].wasted_elapsed_ms, 10000);
  EXPECT_EQ(rec[m.task_index("T2")].status, FinalStatus::kNeverStarted);
  EXPECT_EQ(obs.finished, (std::vector<std::pair<Index, bool>>{{t1, false}}));
  // A task within its budget is not affected: a 3 s transient blockage ends at 8 s < 10 s.
  WorldConfig short_block;
  short_block.timeout_multiplier = 2.0;
  short_block.deviations.push_back({DeviationType::kDelay, t1, dwafp::kNoIndex, 0.4, 0, 3000, false});
  Simulator ok(m, Plan::initial(m), short_block);
  EXPECT_EQ(ok.run()[t1].end_ms, 8000);
  // The multiplier must be 0 (none) or at least 1.
  WorldConfig bad;
  bad.timeout_multiplier = 0.5;
  EXPECT_THROW(Simulator(m, Plan::initial(m), bad), std::invalid_argument);
}

// Timeout boundary: an attempt that completes EXACTLY at start + q succeeds. T1 (5 s nominal,
// budget 10 s) works 2 s, is blocked 5 s, works 3 s: it completes at exactly 10 s.
TEST(Simulator, CompletionExactlyAtTheTimeoutSucceeds)
{
  const Mission m = load("chain.yaml");
  const Index t1 = m.task_index("T1");
  WorldConfig config;
  config.timeout_multiplier = 2.0;
  config.deviations.push_back({DeviationType::kDelay, t1, dwafp::kNoIndex, 0.4, 0, 5000, false});
  Simulator sim(m, Plan::initial(m), config);
  const auto & rec = sim.run();
  EXPECT_EQ(rec[t1].status, FinalStatus::kCompleted);
  EXPECT_EQ(rec[t1].end_ms, 10000);
  EXPECT_EQ(rec[t1].timeouts, 0);
}

