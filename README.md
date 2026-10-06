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

## Repository layout

This repository is the `src/` folder of a colcon workspace.

```
src/
├── README.md
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
| `src/robot_action_server.cpp` | **Mock robot.** Action server `robot_action_server` on `execute_subtask`. Accepts every goal, then aborts it with `ERROR_INVALID_GOAL` if the task type is empty or `timeout_sec` is not greater than 0, or with `ERROR_UNSUPPORTED_TASK` if the task type is not one of `navigate`, `deliver`, `inspect`. Simulates navigation in 5 steps of 1 s while publishing feedback, and checks for cancellation and timeout at every step and once more after the last step. Only `navigate` runs to completion so far. |
| `src/coordinator_action_client.cpp` | **Coordinator.** Action client node `coordinator_action_client`. Waits for the `execute_subtask` server, sends one hard-coded `navigate` goal (mission `mission1`, subtask `subtask1`, timeout 30 s), logs the final status and result fields, then shuts down. It does not use feedback or cancellation yet. |
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
A goal with `timeout_sec` of 0 or less is aborted with `error_code=4` (`ERROR_INVALID_GOAL`).

## Status

- [x] `ExecuteSubtask` action interface
- [x] Mock robot action server (first version)
- [x] Coordinator action client (first version: sends one goal and logs the result)
- [ ] Workflow model (subtask DAG and dependency conditions)
- [ ] Deviation detection
- [ ] Impact analysis and local workflow repair
- [ ] Unit and integration tests
- [ ] Nova Carter robot server (Nav2)

## Changelog

### 2026-10-06 - Add coordinator client and validate timeout_sec
- The coordinator client now sends a goal and logs the result; the robot server rejects `timeout_sec` <= 0 and detects a timeout after the last step.
- Build with `colcon build --packages-select multi_robot_interfaces actions`, then run `ros2 run actions coordinator_action_client` with the robot server running.

### 2026-10-06 - Document project overview and repository layout
- Rewrote the README: project goal, architecture, repository layout, status checklist.
- Added build and run instructions for the mock robot (`colcon build`, then `ros2 run actions robot_action_server`).
