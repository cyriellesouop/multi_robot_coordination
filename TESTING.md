# Manual tests

This guide explains how to build the `actions` package and check, by hand:

- how the mock robot (`robot_action_server`) accepts, rejects, executes and cancels `ExecuteSubtask` goals (part A);
- how the coordinator client (`coordinator_action_client`) sends a subtask, detects deviations and cancels (part B);
- the robot parameters: speed, time scale, name and namespace (part C);
- the fleet launch file that starts several robots at once (part D).

There are no automated tests yet. Each test below gives the commands to type and the result to expect.

## Prerequisites

- Ubuntu 22.04 with ROS 2 Humble installed.
- The repository cloned as the `src/` folder of a colcon workspace (see [README.md](README.md#build)).
  The commands below assume the workspace is `~/ros2_ws`. Replace it with your own path if needed.

## 1. Build

Always build from the workspace root, never from `src/`:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select multi_robot_interfaces actions
source install/setup.bash
```

## 2. Open the terminals

You need up to three terminals. Run these lines first in **every** terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

- **Terminal 1** runs the robot server (or the fleet).
- **Terminal 2** sends the test goals or runs the coordinator client.
- **Terminal 3** is needed for the "busy" test and to inspect parameters.

## 3. Things to know before testing

**The robot remembers succeeded subtasks.** A subtask (`mission_id` + `subtask_id`) that succeeded is rejected
if it is sent again to the same running robot. Failed and canceled subtasks are not remembered, so they can be
retried. To start again from a clean state, restart the robot (`Ctrl+C`, then start it again).

**The coordinator client always sends the same subtask**, `mission1/subtask1` with a 30 s budget (it is
hard-coded until the workflow model exists). So restart the robot before each coordinator test that ends in
`SUCCEEDED`, otherwise the next run is rejected as a duplicate.

**Nominal and real time.** With `time_scale:=10.0` the experiment runs ten times faster than real time.
Durations in the parameters and in the coordinator's logs are *nominal* seconds (the scenario's time);
the robot's own logs are in real seconds. Give the **same** `time_scale` to the robot and the client.

**Where the reasons appear.** When a goal is rejected, the reason appears in the **robot's** terminal as a
`[WARN]` line. `ros2 action send_goal` only prints `Goal was rejected.`

## 4. What the robot checks

Before accepting a goal, the robot rejects it if:

1. `mission_id` is empty,
2. `subtask_id` is empty,
3. `task_type` is empty or is not `navigate` (the only task implemented so far),
4. `timeout_sec` is not greater than 0,
5. `timeout_sec` is too short to finish: the robot needs 5 steps of `step_duration_sec / time_scale` real seconds
   (5 s with the defaults), and `timeout_sec` must be greater than that,
6. the same subtask (`mission_id` + `subtask_id`) has already **succeeded**,
7. the robot is already executing another subtask.

---

## Part A: robot server alone (with `ros2 action send_goal`)

Start the robot in terminal 1 and leave it running for all of part A:

```bash
ros2 run actions robot_action_server
```

Each test uses a different `subtask_id` on purpose (see section 3). Run the tests in this order.

### Test A1: success

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s1, task_type: navigate, timeout_sec: 10.0}" --feedback
```

Expected: 5 feedback messages with `progress` from 0.0 to 0.8 and `estimated_time_remaining_sec` from 5.0
to 1.0, then `Goal finished with status: SUCCEEDED` and `error_code: 0`.

### Test A2: duplicate subtask

The same subtask as test A1:

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s1, task_type: navigate, timeout_sec: 10.0}"
```

Expected: `Goal was rejected.` Terminal 1: `Rejecting subtask 'm1/s1': already completed`.

### Test A3: same subtask id in another mission

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m2, subtask_id: s1, task_type: navigate, timeout_sec: 10.0}"
```

Expected: accepted, then `SUCCEEDED`. A subtask is identified by its mission and its id together,
so `m2/s1` is a different subtask from `m1/s1`.

### Test A4: time budget too short

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s2, task_type: navigate, timeout_sec: 5.0}"
```

Expected: `Goal was rejected.` Terminal 1: `timeout_sec 5.0 s is too short, needs more than 5.0 s`.

### Test A5: invalid time budget

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s2, task_type: navigate, timeout_sec: 0.0}"
```

Expected: `Goal was rejected.` Terminal 1: `timeout_sec must be > 0, got 0.0`.

### Test A6: empty mission id

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: '', subtask_id: s2, task_type: navigate, timeout_sec: 10.0}"
```

Expected: `Goal was rejected.` Terminal 1: `mission_id is empty`.

### Test A7: empty subtask id

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: '', task_type: navigate, timeout_sec: 10.0}"
```

Expected: `Goal was rejected.` Terminal 1: `subtask_id is empty`.

### Test A8: empty task type

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s2, task_type: '', timeout_sec: 10.0}"
```

Expected: `Goal was rejected.` Terminal 1: `task_type is empty`.

### Test A9: unsupported task type

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s2, task_type: inspect, timeout_sec: 10.0}"
```

Expected: `Goal was rejected.` Terminal 1: `unsupported task_type 'inspect'`.

### Test A10: robot busy

This test needs terminal 3. Type the terminal 3 command in advance, so you can start it quickly.

Terminal 2:

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s3, task_type: navigate, timeout_sec: 10.0}"
```

Terminal 3, within 5 seconds:

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s4, task_type: navigate, timeout_sec: 10.0}"
```

Expected: `s4` is rejected (terminal 1: `robot is busy`), while `s3` still finishes with `SUCCEEDED` in terminal 2.

### Test A11: cancel

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s5, task_type: navigate, timeout_sec: 10.0}" --feedback
```

Press `Ctrl+C` in terminal 2 after one or two feedback messages.

Expected: the client sends a cancel request; terminal 1 shows `Received request to cancel subtask 's5'` and
`Subtask 's5' canceled while navigating after ... s`. The robot stops within 0.1 s.

### Test A12: retry after cancel

The same subtask as test A11:

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s5, task_type: navigate, timeout_sec: 10.0}"
```

Expected: accepted, then `SUCCEEDED`. Canceled subtasks are not recorded as completed, so they can be retried.

---

## Part B: coordinator client

The client sends `mission1/subtask1` (30 s nominal budget) to the robot of its namespace, logs whether it is
accepted, every deviation (when it starts and when it ends) and the result, then exits. Its logs name the
subtask as `mission/subtask#attempt@robot`, for example `mission1/subtask1#1@robot1`.

All tests of part B run at `time_scale:=10.0` in namespace `/robot1`. **Restart the robot (terminal 1) before
each test**, so it starts with the parameters of that test and has no completed subtask.

### Test B1: normal run

Terminal 1:

```bash
ros2 run actions robot_action_server --ros-args -r __ns:=/robot1 -p time_scale:=10.0
```

Terminal 2:

```bash
ros2 run actions coordinator_action_client --ros-args -r __ns:=/robot1 -p time_scale:=10.0
```

Expected (terminal 2): `queued`, `Sending mission1/subtask1#1@robot1`, `accepted by the robot, running`, then
`finished with status SUCCEEDED` with `execution_time` about 5.0 s, and the client exits. No warnings.

### Test B2: rejected subtask

Without restarting the robot after B1, run the client of B1 again.

Expected: `mission1/subtask1#1@robot1 was rejected by the robot` and the client exits.
Terminal 1: `Rejecting subtask 'mission1/subtask1': already completed`.

### Test B3: long blockage, deviation detected before the timeout

Terminal 1 (the robot gets blocked for 30 s nominal at step 2):

```bash
ros2 run actions robot_action_server --ros-args -r __ns:=/robot1 -p time_scale:=10.0 -p simulated_delay_sec:=30.0
```

Terminal 2: the client command of B1.

Expected:
- terminal 1: `Simulated deviation: subtask 'subtask1' will be blocked for 3.00 s (real) at step 2`;
- terminal 2: **one** `robot robot1 is BLOCKED (margin 25.0 s, recoveries 1)`, **one**
  `AT RISK: needs 16.0 s but only about 15 s left before the deadline` (about 15 s before the timeout), then
  `finished with status ABORTED`, `error_code=2`, `'Subtask timeout exceeded while blocked'`,
  `execution_time` about 30 s, `distance_remaining=0.60`, `recoveries=1`.

### Test B4: short blockage, the robot recovers

Terminal 1:

```bash
ros2 run actions robot_action_server --ros-args -r __ns:=/robot1 -p time_scale:=10.0 -p simulated_delay_sec:=5.0
```

Terminal 2: the client command of B1.

Expected: `is BLOCKED (margin 25.0 s ...)`, then `is no longer blocked (margin 20.0 s)`, then `SUCCEEDED`
with `execution_time` about 10 s and `recoveries=1`. No `AT RISK`.

### Test B5: cancel from the coordinator

Terminal 1: the robot command of B1.

Terminal 2 (cancel 2 s nominal after sending):

```bash
ros2 run actions coordinator_action_client --ros-args -r __ns:=/robot1 -p time_scale:=10.0 -p cancel_after_sec:=2.0
```

Expected: `will be canceled after 2.0 s`, `Sending cancel request`, `Cancel request ... accepted by the robot`,
then `finished with status CANCELED`, `error_code=1`, `execution_time` about 2.0 s.
A canceled subtask is not remembered, so this test can be repeated without restarting the robot.

### Test B6: client started before the robot

Start **terminal 2 first** with the client command of B1, wait more than 2 s, then start the robot of B1 in
terminal 1.

Expected (terminal 2): `queued`, then after 2 s
`robot action server '/robot1/execute_subtask' not available after 2.0 s (real), still waiting`; when the robot
starts: `robot action server is now available`, then `SUCCEEDED`. The client never blocks while waiting.

### Test B7: robot never starts

Start only the client of B1, wait for the "still waiting" warning, then press `Ctrl+C`.

Expected: the client stops immediately and cleanly (no `terminate called`, no `core dumped`).

### Test B8: see every feedback message

Same as B3, with the client at debug level:

```bash
ros2 run actions coordinator_action_client --ros-args -r __ns:=/robot1 -p time_scale:=10.0 --log-level coordinator_action_client:=debug
```

Expected: in addition, one line per feedback message (progress, state, time needed, time left, margin),
in nominal seconds.

---

## Part C: robot parameters

### Test C1: a slower robot (real time)

Terminal 1:

```bash
ros2 run actions robot_action_server --ros-args -r __ns:=/robot1 -p step_duration_sec:=2.0
```

Terminal 2:

```bash
ros2 run actions coordinator_action_client --ros-args -r __ns:=/robot1
```

Expected: `SUCCEEDED` after about 10 s (5 steps of 2 s), twice as long as the default robot.

### Test C2: a robot too slow for the budget

Terminal 1:

```bash
ros2 run actions robot_action_server --ros-args -r __ns:=/robot1 -p step_duration_sec:=7.0
```

Terminal 2: the client command of C1.

Expected: rejected. Terminal 1: `timeout_sec 30.0 s is too short, needs more than 35.0 s`.

### Test C3: invalid time scale

```bash
ros2 run actions robot_action_server --ros-args -p time_scale:=0.0
```

Expected: the robot refuses to start: `parameter 'time_scale' could not be set ... doesn't comply with floating point range`.

### Test C4: robot name and namespace

Terminal 1:

```bash
ros2 run actions robot_action_server --ros-args -r __ns:=/robot2
```

Expected: `Robot 'robot2': action server started on /robot2/execute_subtask`.

Terminal 3:

```bash
ros2 action list
ros2 param get /robot2/robot_action_server robot_name
ros2 param set /robot2/robot_action_server robot_name other
```

Expected: `/robot2/execute_subtask`; `String value is: robot2`; the `set` is refused (`read-only`).

Restart terminal 1 with an explicit name:

```bash
ros2 run actions robot_action_server --ros-args -r __ns:=/robot2 -p robot_name:=carter_A
```

Expected: `Robot 'carter_A': action server started on /robot2/execute_subtask`; a coordinator started with
`-r __ns:=/robot2` then reports `robot carter_A` in its deviation messages.

---

## Part D: fleet launch

### Test D1: start the fleet

Terminal 1 (three robots from `actions/config/mock_fleet.yaml`, ten times faster than real time):

```bash
ros2 launch actions mock_fleet.launch.py time_scale:=10.0
```

Expected: one `action server started` line per robot, for `/robot1`, `/robot2` and `/robot3`.

Terminal 3:

```bash
ros2 node list
ros2 action list
ros2 param get /robot2/robot_action_server step_duration_sec
```

Expected: three `robot_action_server` nodes and three `execute_subtask` actions (one per namespace);
robot2's `step_duration_sec` is `1.5`.

### Test D2: each robot runs at its own speed

With the fleet of D1 running, send the coordinator's subtask to each robot in turn (terminal 2):

```bash
ros2 run actions coordinator_action_client --ros-args -r __ns:=/robot1 -p time_scale:=10.0
ros2 run actions coordinator_action_client --ros-args -r __ns:=/robot2 -p time_scale:=10.0
ros2 run actions coordinator_action_client --ros-args -r __ns:=/robot3 -p time_scale:=10.0
```

Expected `execution_time` (nominal): about 5.0 s for robot1, 7.5 s for robot2, 4.0 s for robot3.
Only the robot addressed by each command receives a goal.

### Test D3: stop the fleet

Press `Ctrl+C` in terminal 1. Expected: all three robots stop.

To change the fleet, edit `actions/config/mock_fleet.yaml` and rebuild, or pass your own file with
`fleet_config:=<path to file>`.

---

## Summary

| # | Test | Expected |
|---|---|---|
| A1 | Success | `SUCCEEDED` |
| A2 | Duplicate subtask | rejected: already completed |
| A3 | Same subtask id, other mission | `SUCCEEDED` |
| A4 | Budget too short | rejected: too short |
| A5 | Invalid budget | rejected: must be > 0 |
| A6 | Empty mission id | rejected |
| A7 | Empty subtask id | rejected |
| A8 | Empty task type | rejected |
| A9 | Unsupported task type | rejected |
| A10 | Robot busy | second goal rejected, first `SUCCEEDED` |
| A11 | Cancel | execution stops within 0.1 s |
| A12 | Retry after cancel | `SUCCEEDED` |
| B1 | Coordinator, normal | `SUCCEEDED`, about 5 s, client exits |
| B2 | Coordinator, rejected | rejection logged, client exits |
| B3 | Long blockage | one `BLOCKED`, one `AT RISK` 15 s before the timeout, then `ABORTED` |
| B4 | Short blockage | `BLOCKED`, `no longer blocked`, `SUCCEEDED` |
| B5 | Cancel from the coordinator | `CANCELED` after 2 s |
| B6 | Client before the robot | waits without blocking, then `SUCCEEDED` |
| B7 | Robot never starts | clean stop on `Ctrl+C` |
| B8 | Debug level | one line per feedback |
| C1 | Slower robot | `SUCCEEDED` after about 10 s |
| C2 | Robot too slow | rejected: needs more than 35 s |
| C3 | Invalid time scale | robot refuses to start |
| C4 | Name and namespace | `/robot2/execute_subtask`, name `robot2` (or `carter_A`), read-only |
| D1 | Fleet starts | three robots, three namespaces |
| D2 | Robot speeds | 5.0 s, 7.5 s, 4.0 s |
| D3 | Fleet stops | all robots stop on `Ctrl+C` |

## Viewing the robots' messages elsewhere

Every log message is also published on the `/rosout` topic and saved under `~/.ros/log/`.
To follow the rejection reasons from another terminal:

```bash
ros2 topic echo /rosout
```

Or open `rqt_console` to filter them by severity.

## Troubleshooting

| Problem | Cause and fix |
|---|---|
| `No executable found` | The workspace was built from `src/`, or `install/setup.bash` was not sourced. Build from the workspace root and source it again. |
| `Waiting for an action server to become available...` (`send_goal`) or `not available ... still waiting` (coordinator) | The robot is not running, runs in another namespace, or in a terminal with a different `ROS_DOMAIN_ID`. Start it with the same namespace and use the same `ROS_DOMAIN_ID` everywhere. |
| A goal you expect to succeed is rejected as `already completed` | That subtask already succeeded on this robot. Use a new `subtask_id`, or restart the robot. |
| No `BLOCKED` in a blockage test | The robot was not restarted with `simulated_delay_sec`. Check with `ros2 param get /robot1/robot_action_server simulated_delay_sec`, and look for the `Simulated deviation` line in the robot's terminal. |
| Times look ten times too short or too long | The robot and the client use different `time_scale` values. Give both the same value. |
| Old behaviour after changing the code or the fleet YAML | Rebuild, then run `source install/setup.bash` again in every terminal. |
