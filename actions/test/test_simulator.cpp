// Unit tests of the closed-loop simulator, Step 1: the world (Phase C6).
//
// Durations from config/mock_fleet.yaml: 5 s per task on robot1, 7.5 s on robot2, 4 s on robot3.
// Each test runs a mission in the simulator and compares WHAT HAPPENED with values computed by
// hand or with the C1 prediction.

#include <gtest/gtest.h>

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
