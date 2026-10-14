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
    └── src/
        ├── robot_action_server.cpp
        └── coordinator_action_client.cpp
```

### Root files

| File | Purpose |
|---|---|
| `README.md` | This file: project overview, layout, build and run instructions. |
| `TESTING.md` | Manual tests for the mock robot server: build, commands to type and expected results for each accept, reject, cancel and retry case. |
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
| `src/robot_action_server.cpp` | **Mock robot.** Action server `robot_action_server` on `execute_subtask`. Rejects a goal if `mission_id` or `subtask_id` is empty, if the task type is not `navigate` (the only task implemented so far), if `timeout_sec` is not greater than the 5 s it needs, if the same subtask already succeeded, or if it is busy with another subtask. Executes accepted goals in a separate thread: simulates navigation in 5 steps of 1 s while publishing feedback (progress and estimated time to completion), and checks for cancellation and timeout at every step and once more after the last step. Successful subtasks are remembered; failed or canceled ones can be retried. It can simulate a deviation: with the parameter `simulated_delay_sec` greater than 0, the robot gets blocked for that many seconds at step `delay_at_step` (default 2), publishes `BLOCKED` feedback with a growing time estimate and one recovery, and still honours cancellation and timeout. See [TESTING.md](TESTING.md) for manual tests. |
| `src/coordinator_action_client.cpp` | **Coordinator.** Action client node `coordinator_action_client`. Waits for the `execute_subtask` server, sends one hard-coded `navigate` goal (mission `mission1`, subtask `subtask1`, timeout 30 s), logs whether the goal was accepted or rejected, then each feedback message (progress, state, time needed, time left before the deadline and the margin between them), then the final status and result fields, and shuts down (also when the goal is rejected). It warns when the robot reports `BLOCKED` and when the margin becomes negative (subtask at risk). The deadline is currently the goal's own `timeout_sec`; it will come from the workflow model. It does not cancel goals or repair the workflow yet. |
| `CMakeLists.txt` | Builds and installs the `robot_action_server` and `coordinator_action_client` executables. |
| `package.xml` | Package manifest: depends on `rclcpp`, `rclcpp_action`, `multi_robot_interfaces`, `geometry_msgs` and `diagnostic_msgs`. |

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

Or, with the mock robot running, start the coordinator client in a second terminal instead:

```bash
ros2 run actions coordinator_action_client
```

After about 5 s it logs the result (`SUCCEEDED`, `error_code=0`, execution and navigation times) and exits.
A goal with `timeout_sec` of 5 s or less (or 0 or negative) is rejected by the robot; the client logs the rejection and exits.

To see a deviation detected before the timeout, restart the robot with a simulated blockage of 30 s and run the client again:

```bash
ros2 run actions robot_action_server --ros-args -p simulated_delay_sec:=30.0
ros2 run actions coordinator_action_client
```

The client logs `BLOCKED` warnings from about 2 s, an `AT RISK` warning from about 15 s (when the time left before the deadline becomes shorter than the time the robot needs), and the goal ends `ABORTED` with `ERROR_TIMEOUT` at 30 s. Restart the robot between runs of the same subtask, because it rejects a subtask that already succeeded.
See [TESTING.md](TESTING.md) for the complete list of manual tests.

## Status

- [x] `ExecuteSubtask` action interface
- [x] Mock robot action server (first version)
- [x] Coordinator action client (first version: sends one goal and logs the result)
- [ ] Workflow model (subtask DAG and dependency conditions)
- [ ] Deviation detection (first step done: the client warns on `BLOCKED` and on a negative margin; no workflow or impact analysis yet)
- [ ] Impact analysis and local workflow repair
- [ ] Unit and integration tests
- [ ] Nova Carter robot server (Nav2)

## Changelog

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
