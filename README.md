# multi_robot_coordination

Prevention of failure propagation in multi-robot systems, built on ROS 2 Humble (C++).

## About the project

Several robots work together on a **global mission**. The mission is split into **subtasks** linked by
**dependencies**: one subtask may need another to finish first, to produce an outcome, to meet a deadline,
or to bring a robot or object to a location. There can be dependencies among some and each subtask has an initial robot assignment.

During execution, a robot can **deviate** from the plan: it gets delayed, blocked, or fails. Because of the
dependencies, one deviation can make later subtasks fail too. This project aims to stop that chain reaction
**before** it happens:

1. **Detect** the deviation from the robots' execution feedback.
2. **Analyse its impact**: follow the dependencies to find which downstream subtasks are now at risk
   (for example, a deadline margin becomes negative).
3. **Repair locally**: change only the affected part of the workflow (reassign a subtask to another robot,
   retry it, reroute it, or hold it) instead of replanning the whole mission.

Among the valid repairs, the coordinator chooses in strict priority order:

1. the fewest downstream subtasks still at risk,
2. then the fewest changes to the original workflow,
3. then the lowest added cost (delay, travel distance).

## Architecture

```
Coordinator (action client)      holds the workflow, detects deviations, repairs the workflow
        │
        │  ExecuteSubtask action  (one action server per robot)
        ▼
Robot (action server)            accepts or rejects a subtask, executes it,
                                 sends progress feedback, handles cancellation,
                                 returns success or failure
```

The `ExecuteSubtask` action is the only contract between the coordinator and the robots. The robots are
currently **simulated (mock) action servers** running on a PC. They will later be replaced by servers that
drive **NVIDIA Nova Carter** robots through Nav2. The workflow model, the deviation detector and the repair
algorithm must not depend on a specific robot, so that a robot can be replaced without changing the core
algorithm.

## Time budgets and time estimates

### Who decides what

The coordinator and the robots make two different decisions about time:

| | Coordinator | Robot |
|---|---|---|
| Knows | the workflow, the dependencies between subtasks, the deadlines | its position, speed, battery and capabilities |
| Decides | **what** to do and **by when** (the `timeout_sec` budget of each goal) | **whether it can** finish in that budget, and **how long** it will take |

The coordinator turns each subtask's deadline into a time budget when it sends the goal
(`timeout_sec = deadline - now`). The robot never changes that budget. It can only refuse a goal it
cannot finish in time. Refusing at the start costs nothing, while a timeout at the end wastes the whole
budget and delays the subtasks that depend on it. This is the same idea as the Contract Net Protocol used
in multi-robot task allocation, where a robot answers whether it can take a task before it is assigned.

### Accept, reject, abort

- **Rejected** (in the robot's `goal_callback`): the subtask never started. Used for anything the robot can
  decide instantly: missing ids, unsupported task type, invalid or too short time budget, subtask already
  completed, robot busy. The coordinator should send the subtask to another robot or fix the request.
  A rejection carries no reason; the robot logs it (`[WARN]`, also visible on `/rosout`).
- **Aborted** (during execution): the subtask started and failed, for example on a timeout. Time has been
  spent, so this is a real deviation for the coordinator to analyse.

### Detecting a deviation before the timeout

The robot reports `estimated_time_remaining_sec` in its feedback: the time it still needs to **complete**
the subtask, not the time left before the timeout. The coordinator can compute the time left before the
deadline by itself; only the robot knows how long it still needs. The coordinator then computes:

```
margin = time left before the deadline - estimated time to completion
```

A negative margin means the subtask is at risk, possibly long before its timeout fires. For example, with
`timeout_sec = 30`, 10 s after the start:

| | On track | Blocked at 50 % |
|---|---|---|
| Time left before the deadline | 20 s | 20 s |
| Estimated time to completion | 10 s | 40 s |
| Margin | +10 s | -20 s: at risk now, 20 s before the timeout |

The margin is computed in the **coordinator**, not in the robot: the deadline belongs to the workflow, the
impact on downstream subtasks requires the dependency graph, and keeping this logic in one place keeps the
robot side robot-agnostic.

### How a robot estimates its time

- **Mock robot (now):** a fixed duration, 5 steps of 1 s (`kNavigationSteps` x `kStepDurationSec`).
- **Nova Carter with Nav2 (later):**
  1. current position from localization (TF transform `map` -> `base_link`),
  2. path to the target from the Nav2 planner (`ComputePathToPose`), summing its segment lengths,
  3. distance divided by an average speed derived from the controller parameters,
  4. plus the work at the target (for example an inspection duration from the goal `parameters`),
  5. corrected with past measurements (estimated against actual durations).

  During execution, Nav2's `NavigateToPose` feedback already provides estimated time remaining, distance
  remaining, navigation time and number of recoveries, which is why `ExecuteSubtask` feedback mirrors them.
  Because `goal_callback` must answer quickly, a real robot would only use a fast bound there (straight-line
  distance divided by maximum speed) and compute the full estimate elsewhere.

### Planned evolution

Today the robot's own check (budget longer than its expected duration) is the only feasibility check. To
choose between robots during allocation and repair, the coordinator will need each robot's estimate
**before** sending a goal. The planned design is:

- each robot exposes an estimation service ("how long would you need for this subtask?"),
- the coordinator compares the estimates with the deadline and chooses the robot,
- the robot's check in `goal_callback` stays as a safety net.

### Time source

The mock robots and the coordinator run on **system (wall) time only**. Each mock robot measures deadlines and
reported durations with the node clock (`this->now()`) but simulates execution by waiting on the steady (wall)
clock, and the coordinator uses wall timers; these agree only with system time. With `use_sim_time:=true` the
node clock would follow the `/clock` topic, which can pause or run at another rate, and the two would diverge,
so both nodes refuse to start with `use_sim_time:=true`. Experiments are sped up with `time_scale` instead.

The Nova Carter robot server will run on ROS time with Nav2 (for example in Isaac Sim with `use_sim_time:=true`);
its execution follows Nav2 rather than a simulated wait, so the question will be handled there.

## Recording and replaying executions with rosbag

`ros2 bag` records ROS 2 topics to a file and replays them later. It does not estimate or decide anything,
but it supports the project in several ways:

- **Calibrating time estimates:** record many executions (goals, feedback, results), then compare each
  robot's estimated and actual durations offline to correct its estimator.
- **Testing the deviation detector reproducibly:** record a run where a robot gets delayed or blocked, then
  replay it as often as needed while developing detection, with exactly the same input every time.
- **Evaluating repair strategies:** run the same recorded scenarios against different repair strategies and
  compare the results (subtasks still at risk, changes to the workflow, added delay).
- **Post-mortem analysis:** when a mission fails, the recording shows which subtask deviated first and how
  the failure spread through the dependencies.

Action feedback and goal status are published on hidden topics, so they must be included explicitly:

```bash
ros2 bag record --include-hidden-topics -o run1 \
  /execute_subtask/_action/feedback /execute_subtask/_action/status /rosout
ros2 bag play run1
```

Replaying a bag republishes the messages but does not recreate the action server or the goal handles. To
benefit from replay, the deviation detector should take its input (feedback, deadlines) in a form that can
come either from a live action client or from recorded messages.

## Repository layout

This repository is the `src/` folder of a colcon workspace.

```
src/
├── README.md
├── TESTING.md
├── .gitignore
├── results/
│   ├── results_paper.md
│   ├── main_noise0.csv, main_noise0.txt
│   └── main_noise10.csv, main_noise10.txt
├── .vscode/
│   └── c_cpp_properties.json
├── .claude/
│   ├── settings.json
│   └── skills/finish-feature/SKILL.md
├── multi_robot_interfaces/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── action/
│       └── ExecuteSubtask.action
└── actions/
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   ├── mock_fleet.yaml
    │   └── missions/
    │       ├── chain.yaml
    │       └── fork_join.yaml
    ├── include/actions/
    │   ├── workflow.hpp
    │   ├── impact.hpp
    │   ├── repair.hpp
    │   ├── simulator.hpp
    │   ├── controllers.hpp
    │   ├── scenarios.hpp
    │   ├── runner.hpp
    │   └── mission_loader.hpp
    ├── launch/
    │   └── mock_fleet.launch.py
    ├── src/
    │   ├── robot_action_server.cpp
    │   ├── coordinator_action_client.cpp
    │   ├── workflow.cpp
    │   ├── impact.cpp
    │   ├── repair.cpp
    │   ├── simulator.cpp
    │   ├── controllers.cpp
    │   ├── scenarios.cpp
    │   ├── runner.cpp
    │   ├── dwafp_experiments.cpp
    │   ├── stage1_experiments.cpp
    │   └── mission_loader.cpp
    └── test/
        ├── test_workflow.cpp
        ├── test_impact.cpp
        ├── test_repair.cpp
        ├── test_simulator.cpp
        ├── test_controllers.cpp
        ├── test_scenarios.cpp
        └── test_runner.cpp
```

### Root files

| File | Purpose |
|---|---|
| `README.md` | This file: project overview, layout, build and run instructions. |
| `TESTING.md` | Manual tests with the commands to type and the expected results: the mock robot alone (accept, reject, cancel, retry), the coordinator client (deviations, cancel, robot not available), the robot parameters (speed, time scale, name, namespace) and the fleet launch file. |
| `results/` | Results for the paper: `results_paper.md` (every result that may go into the paper, its configuration, the data file and the command that reproduces it, and which claims it supports), the data of the closed-loop comparison without noise (`main_noise0.csv`) and with 10 % duration noise (`main_noise10.csv`), of the sensitivity sweeps (`sweeps.csv`) and of the repair-location study (`location.csv.gz`, `location_latency1s.csv.gz`, gzipped because each is about 24 MB), one line per scenario and method, with the full terminal outputs (`.txt`). |
| `.gitignore` | Keeps colcon output (`build/`, `install/`, `log/`) and Python caches out of the repository. |
| `.vscode/c_cpp_properties.json` | VS Code IntelliSense configuration with the ROS 2 Humble include paths, so the editor can resolve `rclcpp` and the generated interface headers. |
| `.claude/settings.json` | Claude Code project settings: no AI attribution in commits, and a confirmation prompt before every `git commit` and `git push`. |
| `.claude/skills/finish-feature/SKILL.md` | Claude Code skill (`/finish-feature`): builds and tests the changed packages, adds a changelog entry to this README, and commits after approval. The developer pushes manually. |

### `multi_robot_interfaces`: interface package

Defines the messages exchanged between the coordinator and the robots. It contains no executable code.

| File | Purpose |
|---|---|
| `action/ExecuteSubtask.action` | The action sent by the coordinator to one robot to execute one subtask (see below). |
| `CMakeLists.txt` | Generates the C++ and Python code for the action with `rosidl_generate_interfaces`. Depends on `geometry_msgs` and `diagnostic_msgs`. |
| `package.xml` | Package manifest: declares the interface generators and runtime, and membership of `rosidl_interface_packages`. |

**`ExecuteSubtask` in brief**

- **Goal** (coordinator → robot): `mission_id`, `subtask_id`, `task_type` (`navigate`, `pick`, `place`,
  `inspect`, ...), `target_pose` (`PoseStamped`), task-specific `parameters` (key/value list), `timeout_sec` (must be greater than 0).
- **Result** (robot → coordinator): `success`, `error_code` (`ERROR_NONE`, `ERROR_CANCELED`, `ERROR_TIMEOUT`,
  `ERROR_HARDWARE_FAULT`, `ERROR_INVALID_GOAL`, `ERROR_NAVIGATION_FAILED`, `ERROR_UNSUPPORTED_TASK`), `message`,
  execution and navigation times, remaining distance, number of recoveries, `final_pose`.
- **Feedback** (robot → coordinator, repeatedly): `robot_name`, `state` (`NAVIGATING`, `WORKING`, `BLOCKED`),
  `progress` (0 to 1), `current_pose`, estimated time remaining, remaining distance, navigation time,
  number of recoveries, `status_message`.

Run `ros2 interface show multi_robot_interfaces/action/ExecuteSubtask` to see the full definition.

### `actions`: robot and coordinator nodes

| File | Purpose |
|---|---|
| `src/robot_action_server.cpp` | **Mock robot.** Action server `robot_action_server` on `execute_subtask`. Rejects a goal if `mission_id` or `subtask_id` is empty, if the task type is not `navigate` (the only task implemented so far), if `timeout_sec` is not greater than the 5 s it needs, if the same subtask already succeeded, or if it is busy with another subtask. Executes accepted goals in a separate thread: simulates navigation in 5 steps while publishing feedback (progress and estimated time to completion), and checks for cancellation and timeout at least every 0.1 s and once more after the last step. Parameters: `robot_name` (how the robot identifies itself in its feedback and logs; by default its namespace, e.g. `robot1` for `/robot1`; read-only), `step_duration_sec` (nominal duration of a step, default 1.0: the robot's speed, so robots can differ) and `time_scale` (speed-up of the whole experiment, default 1.0); the 5 s it needs, `simulated_delay_sec` and `timeout_sec` checks all follow them. A canceled or timed-out goal returns a partial result (times, recoveries, remaining distance, last pose). It uses a multi-threaded executor, and the thread running the goal is joined on shutdown (Ctrl+C aborts an active goal cleanly). Successful subtasks are remembered; failed or canceled ones can be retried. It can simulate a deviation: with the parameter `simulated_delay_sec` greater than 0, the robot gets blocked for that many nominal seconds at step `delay_at_step` (default 2), publishes `BLOCKED` feedback with a growing time estimate and one recovery, and still honours cancellation and timeout. See [TESTING.md](TESTING.md) for manual tests. |
| `src/coordinator_action_client.cpp` | **Coordinator.** Action client node `coordinator_action_client`. Runs without a namespace and creates one action client per robot of the fleet (parameter `robots`, default `[robot1, robot2, robot3]`, the names used by `mock_fleet.launch.py`), each on the absolute action `/<robot>/execute_subtask`, so it can drive the whole fleet and several subtasks run in parallel on different robots. It queues its subtasks and sends each as soon as its robot's server is available: it never blocks waiting for a robot, it checks every 100 ms, and warns after 2 s if a robot is still missing (the other robots are not affected, and it stops cleanly with Ctrl+C while waiting). It keeps one record per subtask, indexed by mission and subtask id (robot, state, attempt number, goal handle, deadline, deviation flags, cancel state), so state is never shared between subtasks, and every callback carries the attempt it belongs to so a late answer from an old attempt is ignored. Logs name a subtask as `mission/subtask#attempt@robot`. The test mission is hard-coded: `navigate` subtasks `subtask1` ... of `mission1`, timeout 30 s each, one per robot in turn (`subtask_count` sets how many), logs whether the goal was accepted or rejected, then the final status and result fields. A result or a rejection only updates its own subtask; the coordinator stops by itself only when every subtask is in a final state, after logging a mission summary (one line per subtask and the counts). The parameter `subtask_count` (default 0 = one subtask per robot) sets the number of subtasks, assigned to the robots in turn. It computes, from each feedback message, the margin between the time left before the deadline and the time the robot still needs, and reports a deviation when it starts and when it ends (robot `BLOCKED`, margin negative = subtask at risk), not on every message; the per-feedback progress line is at debug level. With the parameter `time_scale` (same value as the robots') the scenario is given in nominal seconds and all times in its logs are nominal, so a run prints the same numbers at any speed. The deadline is currently the goal's own `timeout_sec`; it will come from the workflow model. With the parameter `cancel_after_sec` greater than 0 it sends a cancel request that many seconds after the goal and logs the robot's answer (accepted, rejected, unknown or already finished) and the `CANCELED` result; the default is 0, no cancel. It uses a single-threaded executor and never blocks in a callback. It can cancel one subtask without touching the others and reassign it to another robot as a new attempt (the building blocks of the repairs); it does not repair the workflow by itself yet. |
| `include/actions/workflow.hpp`, `src/workflow.cpp` | **DWAFP workflow model** (`dwafp` namespace), a ROS-free C++ library (`dwafp_core`) that the nodes do not use yet. A `Mission` holds the subtasks (assignee, work, deadline, dependency predecessors, retryable, opaque target pose) and the robots (speed) and rejects unknown robots or predecessors and dependency cycles. A `Plan` holds one fixed queue per robot and the combined graph (dependency edges plus robot-queue edges) with its topological order; a plan whose queue order contradicts a dependency is rejected. A `RuntimeState` holds the status of every subtask (unstarted, executing, completed, failed) and of every robot (available or not). A `DurationModel` gives the duration p_jr of every subtask on every robot (`ConfigDurationModel`: work times the robot's speed), and a `DurationSnapshot` freezes these values. The `Predictor` computes, for every subtask, the predicted start and finish time and whether its output exists, and checks three conditions (deadline, outcome, and assignment: the assigned robot is available). A subtask whose robot is unavailable gets no finite time under its current assignment (`kUnreachable`, never a zero duration), and neither does anything that waits for it; times become finite again after a reassignment. A subtask that is missing an input is held (not dispatched): its own times stay finite, but the next subtask of its robot's queue is unreachable, because the robot cannot get past it. `full()` for the whole workflow, `incremental()` only along the edges where a prediction changes, returning the validation horizon (every subtask whose prediction changed). `threatened_tasks()` lists the unfinished subtasks that violate one of the three conditions. All times are integer nominal milliseconds from mission start. |
| `include/actions/impact.hpp`, `src/impact.cpp` | **Deviation trigger and impact analysis** (C2), on top of the workflow model. `DeviationTrigger` turns robot feedback into a deviation event when the predicted completion `t + r̂` is later than the accepted prediction by more than a tolerance ε for k consecutive updates, or when a robot stays blocked for at least τ (no deadline is used: a subtask that still meets its own deadline can delay a tighter one downstream); it fires once per episode. `expected_state()` rebuilds the state at the same time as if the deviation had not happened, from the reference prediction (unstarted, executing or completed according to the reference start and finish, including descendants that are overdue only because of the deviation). `analyze_impact()` compares the two states and returns the sources, the deviation horizon, the newly threatened tasks (conditions satisfied in the expected prediction and violated now, which can include the deviating subtask itself), the repair region (sources, threatened tasks and the dependency or queue paths between them), the violation time of each threat and the response window B = time left before the first violation (a deadline threat is violated at the deadline; a missing input or an unavailable robot at the task's latest start, deadline minus its duration on its current robot); `needs_repair()` is false when the deviation is absorbed, and `preventable()` is false when B is 0. |
| `include/actions/repair.hpp`, `src/repair.cpp` | **Stage I single-edit repair** (C3), on top of the impact analysis. `generate_single_edits()` enumerates, for every task of the repair region, a **same-robot retry** (only if the task is restartable, its robot is available and `same_robot_retry_feasible` says the cause is transient or has cleared; a persistent cause on that robot excludes retry but not substitution), a **substitution** to another available robot (a busy robot receives the task in the next slot, after its current task, which is never interrupted) and a **takeover** (the same edit when the receiving robot is idle because its next task waits for the moved one). Each primitive is applied to a copy of the plan and state (the edited task is released at t + decision + dispatch latency), propagated incrementally through its own validation horizon, and rejected if any task outside the threatened set loses a condition it had. Admissible candidates are scored with residual risk R, disruption D (tasks whose robot, queue predecessor or retry flag changed) and cost C (discarded progress plus added delay); the best preventive one is the lexicographic minimum of [R, D, C]. Candidates with R above 0 are kept for the two-edit stage; the incumbent response window (computed with the current robots' durations) never rejects a candidate, because a faster robot can still save a task. `tasks_to_hold()` holds only unstarted tasks whose input or robot is threatened (a task that is only late keeps running). `unprotectable_threats()` is a one-pass lower bound on every task's finish and output under all permitted edits; `decide()` uses it to return `NoFeasibleLocalRepair` (hold, no search) when a threatened condition cannot be protected by any permitted local edit, `Repair` when a preventive single edit exists, or `NoSingleEdit` (two-edit stage next). `RepairScope` restricts the edit locations to the region, the event sources only (trigger-task-only: the tasks given in `RepairConfig::deviating_tasks`, which is required; never `ImpactAnalysis::sources`, which also holds the overdue descendants rebuilt in the expected state), or all unfinished tasks, for the ablation. |
| `include/actions/simulator.hpp`, `src/simulator.cpp` | **Closed-loop simulator (the world and the controller interface).** A discrete-event simulation that executes a mission over time so that methods can be compared on what really happens. The ground truth is known only to the simulator: real durations (nominal times a noise factor drawn from a hash of seed, task, robot and attempt, so every method sees the same noise), and the injected deviations: a transient blockage (one absolute interval with a fixed clearing time), a persistent one (the robot-task pair never progresses past the blocking point), an outcome failure, or a robot breakdown. A controller sees only what a coordinator would: feedback (the robot's remaining-time estimate, which stays constant while it is blocked, and BLOCKED), task results, breakdowns, and each deadline miss (reported once), through a `SimulationObserver`. One dispatcher serves every method (a task starts when it is next in its robot's fixed queue, its robot is available and idle, its inputs completed, it is not held and its release time has passed). Simultaneous events follow a fixed order (completions, breakdowns, deadline misses, feedback, dispatch). The record gives each task's final status (completed, failed, never started, incomplete), attempts, times, discarded work and wasted elapsed time, and whether its deadline was met. A controller (a `SimulationObserver`) gets one turn per time step in `act()`, where it can hold tasks, ask for a wake-up, install a repair at once (`apply_edit`) or schedule it with a dispatch latency (`schedule_edit`): until the activation time the running task keeps progressing and the old queues stay active, and at activation a running edited task is cancelled (its progress is counted as discarded) and restarted or moved, or the edit is rejected if an edited task completed in the meantime (or no longer fits the state); an invalid edit changes nothing. `edit_counts()` reports how many edits were scheduled, installed, and rejected because a task had completed or because the edit was invalid. The same-robot retry diagnostic belongs to the task-robot pair, so a task moved to another robot takes that pair's diagnostic. A task cancelled by a repair and not run again ends as unstarted at the end. Every attempt can have an execution timeout (a multiplier of its nominal duration, independent of the mission deadline): an attempt that would end later fails at the timeout like any failure, one that completes exactly at it succeeds. The controllers themselves (the compared methods) come next. |
| `include/actions/controllers.hpp`, `src/controllers.cpp` | **The compared methods as simulator controllers.** One `MethodController` with the method as a setting: no intervention, reactive, source-only, DWAFP and mission-wide. All see only what a coordinator sees and share the policy library, the validation, the dispatch latency and the remaining-time estimator. Actual failures, robot breakdowns and deadline misses of unfinished tasks go to the same recovery for every repair-enabled method (one edit of the affected task that lowers first the number of tasks that would never complete, then the total tardiness; a breakdown hands every unfinished task of the robot to it). Trigger events from robot feedback go to prevention for trigger-task-only, also called source-only (edits restricted to the task whose feedback fired the trigger), DWAFP (the repair region) and mission-wide (any unfinished task): `decide()` runs the impact analysis, the infeasibility certificate and Stage I, and only a preventive repair is executed; otherwise the event is escalated and the trigger is re-armed. So the difference between DWAFP and reactive measures the value of acting before a failure. While a robot is blocked, the remaining time is its own estimate (nominal, primary) or that estimate plus the time blocked so far (conservative, sensitivity); a persistent obstruction reported by the robot makes the task unreachable on it in both. The controller counts trigger events, analyses, repairs (and how many preventive ones edit a task other than the trigger task: downstream edits; scheduled and actually installed), escalations by reason, recovery edits and decision time. |
| `include/actions/scenarios.hpp`, `src/scenarios.cpp` | **Scenario generator** for the closed-loop experiments. `make_scenario(config, seed)` returns a mission, its initial plan and its world (duration noise, a 2x execution timeout and one injected deviation), reproducibly. Workflow shapes: random DAG, chains, fork-join and rendezvous, with robot locality (a robot mostly continues its own line of work). Deadlines are duration-normalized additive slack on the nominal finish (so the nominal plan always meets them), on every task, only on handoff and final tasks, or only on final tasks. The deviation is a transient delay (relative to the source's duration), a persistent obstruction, an outcome failure or a robot breakdown, drawn from a configurable mix; the source is a task starting in the first 60 % of the makespan and, by default, one with a dependency successor (a graph without one is regenerated deterministically, never replaced by another rule, and the generator throws if none can be found). The source class (source only, same-robot queue, downstream dependency, cross-robot handoff over all descendants) is recorded. Robots can be homogeneous for scaling experiments, or have an execution-time ratio ρ (durations 1 / √ρ / ρ s per unit of work). For the repair-location study the source progress at the deviation can be fixed, idle spare robots without any task can be added (the workflow and the deviation stay the same), and the source can be required to deviate while every workflow robot is busy. |
| `include/actions/runner.hpp`, `src/runner.cpp` | **Experiment runner.** `run_method(scenario, config)` executes one scenario under one method and measures what really happened: deadline misses (and the miss rate over tasks with a deadline), tasks completed and never completed, success, tardiness of completed tasks, makespan, timeouts, final assignment changes, edits scheduled, installed and rejected, discarded work, wasted time, the delay added to completed tasks with respect to the nominal plan, whether the injected deviation was realized, and the controller's statistics with every decision time. `compare()` gives paired wins, ties and losses, the mean difference and a bootstrap 95 % confidence interval; `compare_clustered()` gives the same with a cluster bootstrap (the paired differences averaged per cluster, e.g. per seed reused in several factor cells, then the cluster means resampled). |
| `src/dwafp_experiments.cpp` | **Closed-loop comparison** (`dwafp_experiments`, ROS-free): every scenario is executed under no intervention, reactive, source-only, DWAFP and mission-wide, per deviation type (transient delay, persistent obstruction, outcome failure, robot breakdown) and for a mix. It prints the outcomes, the interventions (harmful and unneeded ones, decision latency percentiles), DWAFP against each method with confidence intervals, and a breakdown by source class. The configuration is set explicitly and recorded in the CSV; a noise level can be given (0 = controlled configuration). With a name (`deadlines`, `noise`, `latency`, `estimator`, `timeout`, `sweeps`) it runs the sensitivity sweeps, changing one factor at a time. `location` runs the repair-location study: a full factorial of source progress, idle-spare availability, execution-time ratio and slack of final-only deadlines, reporting downstream edits, runs only DWAFP or only trigger-task-only repair saves, discarded work, added delay over jointly completed runs, final assignment changes and DWAFP against mission-wide, with cluster-bootstrap intervals by seed. |
| `src/stage1_experiments.cpp` | **ROS-free Stage I experiments** (`stage1_experiments`, uses only `dwafp_core`, no ROS node): a walkthrough of named scenarios; an ablation on the same random missions and events (no repair, source-only, DWAFP region, global single-edit) with normal, tight and very tight deadline slack; scaling with fixed mission size and varying robots, with fixed tasks per robot, and with fixed robots and varying mission size (repair-region size and its share of the remaining tasks, timings); and same-robot retry feasibility against dispatch latency. Every chosen repair is re-checked with a full propagation. |
| `include/actions/mission_loader.hpp`, `src/mission_loader.cpp` | Loads a `Mission` from a mission YAML file and the fleet file `config/mock_fleet.yaml` (robot speed from `step_duration_sec`), with an error message naming the file and the subtask for every mistake. |
| `config/missions/chain.yaml`, `config/missions/fork_join.yaml` | The two reference missions: a chain (T1 then T2) and a fork-join (T1 and T2 in parallel, then T3). Deadlines are nominal seconds from mission start. |
| `test/test_workflow.cpp` | 19 unit tests of the workflow library: loading and validation, nominal timing and injected delays on the chain and fork-join missions, queue edges, outcome failure, robot unavailability (including the restored times after a reassignment), the stall behind a held subtask, and a randomized test (300 workflows) checking that `incremental()` always gives the same prediction as `full()`. |
| `test/test_impact.cpp` | 17 unit tests of the trigger and the impact analysis: persistence, tolerance, blockage and reset of the trigger (including a source that still meets its own deadline), the expected state, and the impact of a delay, a failure and a robot breakdown on the chain, fork-join and path missions (threatened tasks, region, response window), including deviations observed after the nominal completion. |
| `launch/mock_fleet.launch.py` | Starts one `robot_action_server` per robot of the fleet file, each in its own namespace (`/<name>`), with the same `time_scale`. Launch arguments: `time_scale` (default 1.0) and `fleet_config` (YAML file, default `config/mock_fleet.yaml`). It stops with an error if the file has no robots, a robot without a name, or a name used twice. |
| `config/mock_fleet.yaml` | The mock fleet: one entry per robot with its `name` (namespace) and optionally `step_duration_sec` (speed), `simulated_delay_sec` and `delay_at_step`. Three robots with different speeds by default. All durations are nominal seconds. The workflow library reads the same file for the robots' speeds. |
| `test/test_repair.cpp` | 20 unit tests of Stage I and `decide()`: the candidates and their scores computed by hand for a blocked source, response-window effects (dispatch latency absorbed by dependency waiting or making an upstream repair too late), takeover, rejection of a candidate that breaks another task, retry on transient and persistent causes, outcome failures with loose and tight slack, a case where no robot is idle, an incumbent window of 0 that does not reject a faster robot, the repair scopes (trigger-task-only repair never edits an overdue descendant and is refused without its event source), the infeasibility certificate (including a randomized check against the global single-edit oracle) the decision statuses, and a moved running task keeping its old robot busy until activation. |
| `test/test_simulator.cpp` | 27 unit tests of the simulator and its controller interface: without noise or deviations the executed times equal the prediction exactly (demo missions and 50 random missions), reproducible and bounded noise, a transient blockage, a persistent one that stays incomplete at the horizon, an outcome failure, a robot breakdown, a held task, the fixed queue, one deadline-miss report per task (also for an unstarted task), the order of a completion and a breakdown at the same time, discarded work against wasted time, and, through scripted controllers, retries before and after a transient obstruction clears (the original clearing time is kept), retries on a persistent one, substitution, unchanged noise for unaffected tasks, invalid edits without side effects, hold and release, edits with dispatch latency (the running task keeps progressing, old queues stay active, an edit is rejected if its task completes before activation) the retry diagnostic of the task-robot pair, the execution timeout (including a completion exactly at the timeout), and whether each injected deviation was realized. |
| `test/test_controllers.cpp` | 9 unit tests of the methods on the same simulated world: with a persistent obstruction, DWAFP moves the blocked task early and meets both deadlines while the reactive method recovers only at the execution timeout and misses the second deadline, and no intervention never completes; a timeout handled identically by every repair-enabled method; the dispatch latency; source-only and mission-wide agreeing when the source must move; an outcome failure and a robot breakdown handled identically by reactive and DWAFP through the common recovery; an absorbed deviation causing no repair; the persistent diagnosis used by both estimators; and a downstream repair (moving the unstarted successor of a delayed task) that DWAFP and mission-wide make while trigger-task-only repair cannot. |
| `test/test_scenarios.cpp` | 15 unit tests of the generator: every shape, deadline structure and size is feasible when nothing deviates, full reproducibility, handoff-and-final deadlines, the structure of each shape, the deviation mix and window (source start, breakdown robot with work left), tasks without a deadline never producing deadline events, a guaranteed eligible source or a clear refusal, source classes over all descendants, homogeneous rates, final-only deadlines, a fixed source progress and the execution-time ratio, spare robots that leave the workflow unchanged, and a busy-fleet source (with its invalid settings refused). |
| `test/test_runner.cpp` | 4 unit tests of the runner: the metrics of a hand-computed scenario under no intervention, reactive and DWAFP (added delay included), reproducible paired runs, the paired statistics (wins, ties, losses, mean difference, confidence interval), and the cluster bootstrap (equal to the plain one with one row per cluster, wider than a naive pooled one when seeds repeat). |
| `CMakeLists.txt` | Builds the `dwafp_core` library and the `robot_action_server`, `coordinator_action_client` and `stage1_experiments` and `dwafp_experiments` executables, installs the executables and the `launch/` and `config/` folders, and registers the `test_workflow`, `test_impact`, `test_repair`, `test_simulator`, `test_controllers`, `test_scenarios` and `test_runner` unit tests. |
| `package.xml` | Package manifest: depends on `rclcpp`, `rclcpp_action`, `multi_robot_interfaces`, `geometry_msgs`, `diagnostic_msgs` and `yaml-cpp`; the launch file also needs `launch`, `launch_ros` and `python3-yaml`; the unit tests need `ament_cmake_gtest`. |

## Build

Requirements: Ubuntu 22.04 and ROS 2 Humble.

```bash
mkdir -p ~/ros2_ws && cd ~/ros2_ws
git clone https://github.com/cyriellesouop/multi_robot_coordination.git src
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Always run `colcon build` from the workspace root (`ros2_ws/`), not from `src/`.

Run the unit tests of the workflow library (add `--event-handlers console_direct+` to `colcon test` to see each test):

```bash
colcon test --packages-select actions
colcon test-result --test-result-base build/actions --verbose
```

## Run

Start the mock robot:

```bash
ros2 run actions robot_action_server
```

In a second terminal (after `source install/setup.bash`), send it a subtask:

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask \
  "{mission_id: m1, subtask_id: s1, task_type: navigate, timeout_sec: 10.0}" --feedback
```

You should see five feedback messages, then a result with status `SUCCEEDED`. Press `Ctrl+C` during
execution to request a cancellation.

To run the coordinator on the whole mock fleet, start the fleet with one command (three robots with different speeds, defined in `config/mock_fleet.yaml`), optionally faster than real time, then the coordinator in a second terminal:

```bash
ros2 launch actions mock_fleet.launch.py time_scale:=10.0
ros2 run actions coordinator_action_client --ros-args -p time_scale:=10.0
```

The coordinator sends one subtask to each robot at the same time. Each robot is at `/robot1/execute_subtask`, `/robot2/execute_subtask` and `/robot3/execute_subtask`; edit the YAML file (or pass `fleet_config:=<file>`) to change the robots, and give the coordinator the same names with `-p "robots:=[robot1,robot2,robot3]"`. Each subtask runs at its robot's speed (about 5.0, 7.5 and 4.0 s), and the coordinator ends with a mission summary. Give the same `time_scale` to the fleet and the coordinator; durations stay in nominal seconds, so the results are the same but a run takes a tenth of the time at `time_scale:=10.0`.

With `-p subtask_count:=4` the fourth subtask goes to robot1 while it is busy: the robot, which runs one subtask at a time, rejects it, and the other three still finish. The coordinator never waits for a missing robot: with a robot named in `robots` that is not running, its subtasks stay queued (with a warning after 2 s) and the others run.
A goal with `timeout_sec` of 5 s or less (or 0 or negative) is rejected by a robot with default speed.

To test cancellation, ask the coordinator to cancel its running subtasks 2 s after sending them. The robots stop within 0.1 s and the coordinator logs the cancel answers and `CANCELED` results with the partial data (about 2 s of execution):

```bash
ros2 run actions coordinator_action_client --ros-args -p time_scale:=10.0 -p cancel_after_sec:=2.0
```

Add `-p cancel_subtask:=subtask1` to cancel only that subtask: the others keep running and succeed. Add `-p reassign_to:=robot3` as well (with `-p subtask_count:=2` so that robot3 is idle) to resend the cancelled subtask to robot3 as attempt `#2`: the late `CANCELED` result of attempt `#1` from robot1 is logged as ignored and does not overwrite the new attempt.
To make a robot slower or faster than the others, set its `step_duration_sec` in the YAML file (for example `2.0` makes the subtask take 10 s instead of 5 s).

To start a single robot by hand instead, start it in its own namespace: the action becomes `/robot2/execute_subtask` and the robot reports itself as `robot2` (or as the name given with `-p robot_name:=...`):

```bash
ros2 run actions robot_action_server --ros-args -r __ns:=/robot2 -p time_scale:=10.0
ros2 run actions coordinator_action_client --ros-args -p time_scale:=10.0 -p "robots:=[robot2]"
```

To see a deviation detected before the timeout, give a robot a simulated blockage of 30 s, either with `simulated_delay_sec: 30.0` for it in the YAML file (rebuild afterwards) or on a single robot started by hand:

```bash
ros2 run actions robot_action_server --ros-args -r __ns:=/robot1 -p time_scale:=10.0 -p simulated_delay_sec:=30.0
ros2 run actions coordinator_action_client --ros-args -p time_scale:=10.0 -p "robots:=[robot1]"
```

The coordinator logs one `BLOCKED` warning at about 2 s, one `AT RISK` warning at about 15 s (when the time left before the deadline becomes shorter than the time the robot needs), and that subtask ends `ABORTED` with `ERROR_TIMEOUT` at 30 s while the other robots' subtasks finish normally. With a shorter blockage, for example `simulated_delay_sec:=5.0`, it logs when the robot is no longer blocked and the subtask succeeds. Add `--log-level coordinator_action_client:=debug` to see every feedback message. Restart the robots between runs, because a robot rejects a subtask that already succeeded.
See [TESTING.md](TESTING.md) for the complete list of manual tests.

To run the ROS-free Stage I experiments (walkthrough, ablation, scaling; about 7 s, no robots needed), optionally writing one CSV line per event:

```bash
ros2 run actions stage1_experiments 300 1 results.csv
```

To run the closed-loop comparison of the five methods (a few seconds each; arguments: scenarios per deviation type, CSV file, duration noise), as recorded in `results/`:

```bash
ros2 run actions dwafp_experiments 200 results/main_noise0.csv 0      # controlled, no noise
ros2 run actions dwafp_experiments 200 results/main_noise10.csv 0.1   # robustness, 10 % noise
```

The sensitivity sweeps (deadline structure, noise, dispatch latency, estimator, execution timeout; about 45 s) are run with a name instead of a number; `sweeps` runs all five, or give one of `deadlines`, `noise`, `latency`, `estimator`, `timeout`:

```bash
ros2 run actions dwafp_experiments sweeps 200 results/sweeps.csv > results/sweeps.txt
```

The repair-location study (about 100 s per run; last argument: the dispatch latency in ms, the same for every method):

```bash
ros2 run actions dwafp_experiments location 200 location.csv 0 > results/location.txt
ros2 run actions dwafp_experiments location 200 location_latency1s.csv 1000 > results/location_latency1s.txt
```

## Status

- [x] `ExecuteSubtask` action interface
- [x] Mock robot action server (first version)
- [x] Coordinator action client (first version: sends one goal and logs the result)
- [x] Workflow model: subtask DAG, robot queues, runtime state, forward propagation of predicted times and outcomes, with unit tests (library only: the coordinator does not use it yet)
- [ ] Deviation detection (first step done: the client reports the start and end of `BLOCKED` and of a negative margin; no workflow or impact analysis yet)
- [x] Impact analysis: deviation trigger, expected state, newly threatened tasks, repair region and response window (library only)
- [x] Stage I single-edit repair: retry, substitution, takeover, validation, scoring (library only)
- [x] Closed-loop simulator: the world (ground truth, dispatcher, feedback, deviations, outcome record) and the controller interface (hold, wake-up, edits with dispatch latency)
- [x] The compared methods as simulator controllers (no intervention, reactive, source-only, DWAFP, mission-wide)
- [x] Scenario generator (workflow shapes, deadline structures, deviation mix, source classes, execution timeout)
- [x] Experiment runner and closed-loop comparison of the five methods per deviation type (results in `results/`)
- [x] Sensitivity sweeps: deadline structure, noise, dispatch latency, estimator, execution timeout (results in `results/`)
- [x] Repair-location study (results in `results/`)
- [ ] Remaining experiments (closed-loop scaling, failure taxonomy)
- [ ] Two-edit repair, the full escalation procedure and the remaining baselines (the hold rule, the infeasibility certificate and the `decide()` statuses up to Stage I are done)
- [ ] Unit and integration tests (first step done: unit tests of the workflow library; none yet for the nodes)
- [ ] Nova Carter robot server (Nav2)

## Changelog

### 2026-10-21 - Add the repair-location study
- `dwafp_experiments location` crosses source progress, idle-spare availability, execution-time ratio and final-deadline slack, with the same latency for every method; pooled intervals use a cluster bootstrap by seed (`compare_clustered()`), and the runner and controller now report added delay and installed downstream edits.
- Run with `ros2 run actions dwafp_experiments location 200 location.csv 0` (and `1000` for 1 s latency); results in `results/location*.txt`, the data gzipped, and Section 1.9 of `results/results_paper.md`.

### 2026-10-21 - Restrict source-only repair to the trigger task
- The source-only baseline (now trigger-task-only) edits only the task whose feedback fired the trigger; before, it could also edit overdue descendants that the impact analysis counts among its sources. `RepairConfig::deviating_tasks` is now required for that scope, and the controller counts downstream preventive edits.
- The closed-loop results in `results/` were re-run (only the source-only rows change); `results/results_paper.md` has a correction note. The open-loop Stage I ablation is unchanged.

### 2026-10-21 - Add the sensitivity sweeps
- `dwafp_experiments` runs the Step 6 sweeps (deadline structure with a per-shape breakdown, noise, dispatch latency, estimator, execution timeout); results in `results/sweeps.csv`, `results/sweeps.txt` and Section 1.8 of `results/results_paper.md`.
- Run with `ros2 run actions dwafp_experiments sweeps 200 results/sweeps.csv > results/sweeps.txt`.

### 2026-10-21 - Explain how to read the results tables
- `results/results_paper.md` now defines every row (deviation types, with or without warning in the feedback) and every column or term (paired difference with its confidence interval, wins/ties/losses, success, miss rate, tardiness, makespan, harmful, unneeded, realized, latency percentiles), with a note above each table. No code change.

### 2026-10-21 - Add the experiment runner and the closed-loop comparison
- `run_method()` measures what really happened in one run, and `compare()` gives paired statistics with bootstrap confidence intervals; `dwafp_experiments` compares the five methods per deviation type, with harmful and unneeded interventions, decision latency percentiles and a breakdown by source class. The simulator now reports whether each injected deviation was realized.
- Results and the paper notes are in `results/` (`results_paper.md`); reproduce them with `ros2 run actions dwafp_experiments 200 results/main_noise0.csv 0` (and `0.1` for 10 % noise).

### 2026-10-20 - Add the scenario generator and the execution timeout
- `make_scenario(config, seed)` builds reproducible missions (random DAG, chains, fork-join, rendezvous) with duration-normalized deadline slack, on every task or on handoff and final tasks only, and one injected deviation with a recorded source class. Every attempt now has an execution timeout independent of the mission deadline, so the reactive method acts at the timeout, like a robot reporting a failure.
- Build and test with `colcon build --packages-select actions` and `colcon test --packages-select actions` (11 tests in `test/test_scenarios.cpp`).

### 2026-10-20 - Add the compared methods as simulator controllers
- No intervention, reactive, source-only, DWAFP and mission-wide run on the same simulated world with the same policy library, latency and remaining-time estimator (nominal or conservative while blocked). Failures, breakdowns and deadline misses go to one common recovery; only the preventive methods add early repairs on trigger events, so DWAFP against reactive measures the value of acting before failure. A moved running task now keeps its old robot busy until activation in the predictions, as in the simulator.
- Build and test with `colcon build --packages-select actions` and `colcon test --packages-select actions` (7 tests in `test/test_controllers.cpp`).

### 2026-10-20 - Add the simulator's controller interface
- A controller gets a turn per time step (`act()`) to hold tasks, request a wake-up and install repairs, either at once or with a dispatch latency (`schedule_edit`): the running task keeps progressing and the old queues stay active until activation, and an edit is rejected if its task completed before. The retry diagnostic now follows the task-robot pair, deadline misses and cancelled tasks are reported (`kUnstartedAtEnd`), an invalid edit changes nothing, and the simulator counts scheduled, installed and rejected edits with the reason of each rejection.
- Build and test with `colcon build --packages-select actions` and `colcon test --packages-select actions` (24 tests in `test/test_simulator.cpp`).

### 2026-10-20 - Add the closed-loop simulator (the world)
- New `simulator.hpp`/`simulator.cpp` in `dwafp_core`: executes a mission over time with real durations (reproducible noise), transient or persistent blockages, outcome failures and robot breakdowns, one shared dispatcher, feedback and deadline-miss notifications for a controller, and a record of what really happened. Without noise or deviations it reproduces the prediction exactly. It is the base for comparing methods on outcomes; controllers come next.
- Build and test with `colcon build --packages-select actions` and `colcon test --packages-select actions` (12 tests in `test/test_simulator.cpp`).

### 2026-10-20 - Add the infeasibility certificate, repair scopes and Stage I experiments
- `decide()` runs the impact analysis, a lower-bound certificate that detects when no permitted local repair can protect a threatened condition (`NoFeasibleLocalRepair`: hold only the tasks whose input or robot is threatened, skip the search), then the single-edit search. The incumbent response window no longer rejects candidates, `retry_available` is now `same_robot_retry_feasible`, and `RepairScope` allows source-only and global single-edit searches for comparison.
- New ROS-free `stage1_experiments` program (walkthrough, ablation with normal and tight slack, scaling, retry feasibility against dispatch latency): `ros2 run actions stage1_experiments 300 1 results.csv`; build and test with `colcon build --packages-select actions` and `colcon test --packages-select actions`.

### 2026-10-19 - Add Stage I single-edit repair
- New `repair.hpp`/`repair.cpp` in `dwafp_core`: same-robot retry (not for persistent blockages), substitution (next slot for a busy robot) and takeover, each validated through its own horizon and ranked by [R, D, C]; dispatch latency is part of the prediction (`release_ms`), the response window uses the latest start for missing inputs and unavailable robots, and a task whose input is missing is held and stalls the rest of its robot's queue. The coordinator does not use it yet.
- Build with `colcon build --packages-select actions`, then `colcon test --packages-select actions` (11 tests in `test/test_repair.cpp`).

### 2026-10-19 - Add the deviation trigger and the impact analysis
- New `impact.hpp`/`impact.cpp` in `dwafp_core`: a trigger that raises a deviation event when a robot's predicted completion is persistently later than the accepted prediction (or it stays blocked), the expected state at the event time, and the impact analysis (newly threatened tasks, repair region, response window). A subtask missing an input is now held and stalls the rest of its robot's queue. The coordinator does not use them yet.
- Build with `colcon build --packages-select actions`, then `colcon test --packages-select actions` (19 tests in `test/test_workflow.cpp`, 17 in `test/test_impact.cpp`).

### 2026-10-19 - Give tasks on an unavailable robot no finite completion time
- An unstarted subtask whose robot is unavailable used to be predicted as finished instantly, which could falsely meet its deadline and move the next tasks of the queue earlier. It now has no finite time (`kUnreachable`), no output and a violated third condition, assignment feasibility; the subtasks waiting for it are unreachable too, and times become finite again after a reassignment.
- Run `colcon test --packages-select actions` (18 tests, including the new `RobotUnavailability.ReassignmentRestoresFiniteTimes`).

### 2026-10-19 - Add the DWAFP workflow model and its predictions
- New ROS-free library `dwafp_core` in the `actions` package: a mission (subtasks, dependencies, robots), fixed robot queues with the combined dependency-and-queue graph, runtime state, a frozen duration table, and a predictor that gives the predicted start, finish and outcome of every subtask. `incremental()` recomputes only what a change affects and returns the validation horizon; missions load from YAML (`config/missions/chain.yaml`, `fork_join.yaml`). First step of the workflow model; the coordinator does not use it yet.
- Build with `colcon build --packages-select actions`, then `colcon test --packages-select actions` (17 tests in `test/test_workflow.cpp`).

### 2026-10-15 - Run the mock robots and the coordinator on system time only
- Both nodes refuse to start with `use_sim_time:=true`: they mix the node clock with wall-clock waits and timers, which agree only with system time; experiments are sped up with `time_scale` instead. The README's "Time source" section now states this choice.
- Check with `ros2 run actions robot_action_server --ros-args -p use_sim_time:=true` (refused); build with `colcon build --packages-select actions`.

### 2026-10-15 - Cancel a single subtask and reassign it to another robot
- The coordinator cancels one subtask without affecting the others (`cancel_subtask`) and can reassign it to another robot as a new attempt (`reassign_to`); callbacks carry the subtask, the attempt and the robot, and answers from an old attempt are ignored (fifth and last of the Phase B coordinator changes).
- Run the fleet with `ros2 launch actions mock_fleet.launch.py time_scale:=10.0`, then `ros2 run actions coordinator_action_client --ros-args -p time_scale:=10.0 -p subtask_count:=2 -p cancel_subtask:=subtask1 -p cancel_after_sec:=2.0 -p reassign_to:=robot3`.

### 2026-10-15 - Drive the whole fleet with one action client per robot
- The coordinator runs without a namespace and creates one action client per robot (parameter `robots`), so it sends subtasks to several robots in parallel; a missing robot only delays its own subtasks (fourth of the five Phase B coordinator changes).
- Run `ros2 launch actions mock_fleet.launch.py time_scale:=10.0`, then `ros2 run actions coordinator_action_client --ros-args -p time_scale:=10.0`; build with `colcon build --packages-select actions`.

### 2026-10-15 - Stop the coordinator only when the whole mission is finished
- A result or a rejection no longer shuts the coordinator down; each only updates its own subtask, and the coordinator logs a mission summary and stops when every subtask is final (third of the five Phase B coordinator changes). New test parameter `subtask_count`.
- Run `ros2 run actions coordinator_action_client --ros-args -r __ns:=/robot1 -p subtask_count:=2`: the second subtask is rejected (robot busy) while the first runs to `SUCCEEDED`.

### 2026-10-15 - Keep a separate state record for each subtask
- The coordinator client replaces its single-goal variables with one record per subtask (robot, state, attempt number, goal handle, deadline, deviation and cancel state); callbacks carry the attempt number and ignore answers from old attempts (second of the five Phase B coordinator changes).
- No change to how it is run: `ros2 run actions coordinator_action_client --ros-args -r __ns:=/robot1`; logs now read `mission1/subtask1#1@robot1`.

### 2026-10-15 - Stop the coordinator from blocking on a missing robot
- The coordinator client no longer blocks in `wait_for_action_server()`: it queues the goal and a 100 ms timer sends it when the robot's server is available, so a missing robot cannot freeze it (first of the five Phase B coordinator changes).
- Start the client before the robot with `ros2 run actions coordinator_action_client --ros-args -r __ns:=/robot1`, then start the robot; build with `colcon build --packages-select actions`.

### 2026-10-15 - Add a launch file to start a fleet of mock robots
- `ros2 launch actions mock_fleet.launch.py` starts one mock robot per entry of `config/mock_fleet.yaml`, each in its own namespace with its own speed and a shared `time_scale`, so a heterogeneous fleet starts with one command.
- Build with `colcon build --packages-select actions`, then run `ros2 launch actions mock_fleet.launch.py time_scale:=10.0`; the `robot_name` parameter now shows the real name.

### 2026-10-15 - Identify each robot by a name
- The mock robot has a read-only `robot_name` parameter (default: its namespace) used in its feedback and logs, so the coordinator can tell robots apart; started in a namespace, its action is `/<namespace>/execute_subtask`.
- Run `ros2 run actions robot_action_server --ros-args -r __ns:=/robot2` and the client with the same `-r __ns:=/robot2`; build with `colcon build --packages-select actions`.

### 2026-10-15 - Add a time scale and per-robot speed to run experiments faster
- Robots and the client take a `time_scale` parameter (same value everywhere) and robots a `step_duration_sec` parameter (their speed); durations stay in nominal seconds, so experiments run N times faster with the same results. The client now reports a blockage or a subtask at risk when it starts and ends instead of on every feedback, and logs nominal seconds.
- Run `ros2 run actions robot_action_server --ros-args -p time_scale:=10.0` and `ros2 run actions coordinator_action_client --ros-args -p time_scale:=10.0`; build with `colcon build --packages-select actions`.

### 2026-10-15 - Add goal cancellation to the client and server
- The client can cancel its goal (`cancel_after_sec`) and handles the robot's answer. The robot stops within 0.1 s, returns a partial result, claims itself atomically (no two goals accepted at once), uses a multi-threaded executor, and shuts down cleanly with an active goal.
- Run `ros2 run actions coordinator_action_client --ros-args -p cancel_after_sec:=2.0` with the robot server running; build with `colcon build --packages-select actions`.

### 2026-10-14 - Detect deviations from feedback and simulate a blocked robot
- The coordinator client logs accepted or rejected goals and every feedback message, and warns on `BLOCKED` and on a negative margin (deadline minus estimated time to completion). The mock robot can simulate a blockage with `simulated_delay_sec`; cancel and timeout checks and feedback publishing were moved into helper methods.
- Run `ros2 run actions robot_action_server --ros-args -p simulated_delay_sec:=30.0`, then `ros2 run actions coordinator_action_client`; build with `colcon build --packages-select actions`.

### 2026-10-14 - Accept or reject goals in the robot server
- The mock robot now rejects invalid, unsupported, too short, duplicate or concurrent subtasks in `goal_callback`, runs accepted goals in a separate thread, reports `CANCELED` on cancellation and reports time to completion in its feedback.
- Added [TESTING.md](TESTING.md) with the manual test commands, and documented time budgets and rosbag usage. Build with `colcon build --packages-select actions`.

### 2026-10-06 - Add coordinator client and validate timeout_sec
- The coordinator client now sends a goal and logs the result; the robot server rejects `timeout_sec` <= 0 and detects a timeout after the last step.
- Build with `colcon build --packages-select multi_robot_interfaces actions`, then run `ros2 run actions coordinator_action_client` with the robot server running.

### 2026-10-06 - Document project overview and repository layout
- Rewrote the README: project goal, architecture, repository layout, status checklist.
- Added build and run instructions for the mock robot (`colcon build`, then `ros2 run actions robot_action_server`).
