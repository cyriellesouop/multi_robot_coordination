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

All durations are measured with the node clock (`this->now()`). On a real robot this is the system time.
In simulation (Isaac Sim or Gazebo with `use_sim_time:=true`), the clock follows the simulator's `/clock`
topic, so the same code works in both cases.

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
    │   └── mock_fleet.yaml
    ├── launch/
    │   └── mock_fleet.launch.py
    └── src/
        ├── robot_action_server.cpp
        └── coordinator_action_client.cpp
```

### Root files

| File | Purpose |
|---|---|
| `README.md` | This file: project overview, layout, build and run instructions. |
| `TESTING.md` | Manual tests with the commands to type and the expected results: the mock robot alone (accept, reject, cancel, retry), the coordinator client (deviations, cancel, robot not available), the robot parameters (speed, time scale, name, namespace) and the fleet launch file. |
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
| `launch/mock_fleet.launch.py` | Starts one `robot_action_server` per robot of the fleet file, each in its own namespace (`/<name>`), with the same `time_scale`. Launch arguments: `time_scale` (default 1.0) and `fleet_config` (YAML file, default `config/mock_fleet.yaml`). It stops with an error if the file has no robots, a robot without a name, or a name used twice. |
| `config/mock_fleet.yaml` | The mock fleet: one entry per robot with its `name` (namespace) and optionally `step_duration_sec` (speed), `simulated_delay_sec` and `delay_at_step`. Three robots with different speeds by default. All durations are nominal seconds. |
| `CMakeLists.txt` | Builds and installs the `robot_action_server` and `coordinator_action_client` executables, and installs the `launch/` and `config/` folders. |
| `package.xml` | Package manifest: depends on `rclcpp`, `rclcpp_action`, `multi_robot_interfaces`, `geometry_msgs` and `diagnostic_msgs`; the launch file also needs `launch`, `launch_ros` and `python3-yaml`. |

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

## Status

- [x] `ExecuteSubtask` action interface
- [x] Mock robot action server (first version)
- [x] Coordinator action client (first version: sends one goal and logs the result)
- [ ] Workflow model (subtask DAG and dependency conditions)
- [ ] Deviation detection (first step done: the client reports the start and end of `BLOCKED` and of a negative margin; no workflow or impact analysis yet)
- [ ] Impact analysis and local workflow repair
- [ ] Unit and integration tests
- [ ] Nova Carter robot server (Nav2)

## Changelog

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
