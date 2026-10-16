# Manual tests

This guide explains how to build the `actions` package and check, by hand:

- how the mock robot (`robot_action_server`) accepts, rejects, executes and cancels `ExecuteSubtask` goals (part A);
- the fleet launch file that starts several robots at once (part B);
- how the coordinator (`coordinator_action_client`) drives the fleet: parallel subtasks, deviation detection,
  cancellation of one subtask, reassignment, missing robots (part C);
- the robot parameters: speed, time scale, name, namespace and time source (part D).

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

**The coordinator always sends the same test mission** (hard-coded until the workflow model exists):
`mission1/subtask1`, `mission1/subtask2`, ... with a 30 s budget each, one per robot in turn (`subtask1` to robot1,
`subtask2` to robot2, ...). So restart the robots (the fleet) before each coordinator test that ends in
`SUCCEEDED`, otherwise the next run is rejected as a duplicate. The coordinator runs without a namespace and
names each subtask in its logs as `mission/subtask#attempt@robot`, for example `mission1/subtask1#1@robot1`.

**Time source.** The mock robots and the coordinator run on system time only; both refuse `use_sim_time:=true`.

**Nominal and real time.** With `time_scale:=10.0` the experiment runs ten times faster than real time.
Durations in the parameters and in the coordinator's logs are *nominal* seconds (the scenario's time);
the robot's own logs are in real seconds. Give the **same** `time_scale` to the robots (the fleet) and the coordinator.

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

## Part B: fleet launch

The coordinator tests of part C need the fleet. The default fleet (`actions/config/mock_fleet.yaml`) has three
robots with different speeds: robot1 (5 s per subtask), robot2 (7.5 s) and robot3 (4 s).

### Test B1: start the fleet

Terminal 1 (ten times faster than real time):

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

### Test B2: stop the fleet

Press `Ctrl+C` in terminal 1. Expected: all three robots stop.

To change the fleet, edit `actions/config/mock_fleet.yaml` and rebuild, or pass your own file with
`fleet_config:=<path to file>` (used in tests C4 and C5).

---

## Part C: coordinator on the fleet

Terminal 1 runs the fleet of B1 (`ros2 launch actions mock_fleet.launch.py time_scale:=10.0`), terminal 2 the
coordinator. **Restart the fleet (terminal 1) before each test**, so the robots have no completed subtask.

### Test C1: one subtask per robot, in parallel

Terminal 2:

```bash
ros2 run actions coordinator_action_client --ros-args -p time_scale:=10.0
```

Expected: `Fleet of 3 robot(s): /robot1/execute_subtask /robot2/execute_subtask /robot3/execute_subtask`; the three
subtasks are sent and accepted at the same time (`subtask1#1@robot1`, `subtask2#1@robot2`, `subtask3#1@robot3`);
they finish at their robot's speed, `execution_time` about 4.0 s (robot3), 5.0 s (robot1) and 7.5 s (robot2); then
`Mission finished: 3 subtask(s)` with `succeeded=3`, and the coordinator exits.

### Test C2: run again without restarting the fleet

Run the command of C1 again.

Expected: the three subtasks are rejected (each robot already completed its subtask); the summary shows
`rejected=3` and the coordinator exits. A rejection only ends its own subtask.

### Test C3: more subtasks than robots

Restart the fleet, then:

```bash
ros2 run actions coordinator_action_client --ros-args -p time_scale:=10.0 -p subtask_count:=4
```

Expected: `subtask4` goes to robot1 while robot1 is busy with `subtask1`: `mission1/subtask4#1@robot1 was rejected
by the robot`, while the three others still succeed. Summary: `succeeded=3, rejected=1`.

### Test C4: one robot blocked, deviation detected before its timeout

Create a fleet file where robot2 gets blocked for 30 s nominal in each subtask:

```bash
cat > ~/fleet_robot2_blocked.yaml <<'EOF'
robots:
  - name: robot1
    step_duration_sec: 1.0
  - name: robot2
    step_duration_sec: 1.5
    simulated_delay_sec: 30.0
  - name: robot3
    step_duration_sec: 0.8
EOF
```

Terminal 1:

```bash
ros2 launch actions mock_fleet.launch.py time_scale:=10.0 fleet_config:=$HOME/fleet_robot2_blocked.yaml
```

Terminal 2: the command of C1.

Expected:
- terminal 1: `Simulated deviation: subtask 'subtask2' will be blocked for 3.00 s (real) at step 2` (robot2 only);
- terminal 2: **one** `mission1/subtask2#1@robot2: robot robot2 is BLOCKED`, **one**
  `mission1/subtask2#1@robot2 AT RISK: needs ... s but only about 15 s left before the deadline` (about 15 s before
  its timeout), then `subtask2` ends `ABORTED`, `error_code=2`, `'Subtask timeout exceeded while blocked'`,
  `execution_time` about 30 s, `recoveries=1`;
- robot1 and robot3 are not affected: their subtasks succeed (5.0 s and 4.0 s);
- summary: `succeeded=2, aborted=1`.

### Test C5: short blockage, the robot recovers

Same as C4 with `simulated_delay_sec: 5.0` for robot2 in the fleet file.

Expected: `robot robot2 is BLOCKED (margin ...)`, then `robot robot2 is no longer blocked (margin ...)`, then
`subtask2` succeeds with `execution_time` about 12.5 s and `recoveries=1`. No `AT RISK`. Summary: `succeeded=3`.

### Test C6: cancel one subtask only

Default fleet (B1). Terminal 2 (cancel `subtask1` 2 s nominal after sending it):

```bash
ros2 run actions coordinator_action_client --ros-args -p time_scale:=10.0 -p cancel_subtask:=subtask1 -p cancel_after_sec:=2.0
```

Expected: `Sending cancel request for mission1/subtask1#1@robot1`, `Cancel request ... accepted by the robot`, then
`subtask1` ends `CANCELED` (`error_code=1`, `execution_time` about 2.0 s); `subtask2` and `subtask3` keep running and
succeed. Summary: `succeeded=2, canceled=1`.

Without `cancel_subtask` (`-p cancel_after_sec:=2.0` only), every subtask is canceled: `canceled=3`.

### Test C7: reassign a canceled subtask to another robot

Default fleet (B1). Two subtasks only, so that robot3 is idle:

```bash
ros2 run actions coordinator_action_client --ros-args -p time_scale:=10.0 -p subtask_count:=2 -p cancel_subtask:=subtask1 -p cancel_after_sec:=2.0 -p reassign_to:=robot3
```

Expected:

```
Sending cancel request for mission1/subtask1#1@robot1
Reassigning mission1/subtask1#1@robot1 to robot robot3 as attempt #2
Ignoring cancel answer of mission1/subtask1#1@robot1: the current attempt is #2@robot3
Ignoring result of mission1/subtask1#1@robot1: the current attempt is #2@robot3
Sending mission1/subtask1#2@robot3 ...
mission1/subtask1#2@robot3 finished with status SUCCEEDED
```

The late `CANCELED` result of attempt `#1` (robot1) is ignored and does not overwrite attempt `#2` (robot3).
Summary: `mission1/subtask1#2@robot3: SUCCEEDED`, `mission1/subtask2#1@robot2: SUCCEEDED`.

### Test C8: a robot of the list is not running

Default fleet (B1). Terminal 2, with a fourth robot that does not exist:

```bash
ros2 run actions coordinator_action_client --ros-args -p time_scale:=10.0 -p "robots:=[robot1,robot2,robot3,robot4]"
```

Expected: the three subtasks of the running robots succeed; `subtask4` stays queued with
`robot action server '/robot4/execute_subtask' not available after 2.0 s (real), still waiting`. The mission is not
finished, so the coordinator keeps running: press `Ctrl+C`; it stops cleanly (no `terminate called`, no `core dumped`).

### Test C9: coordinator started before the fleet

Start **terminal 2 first** with the command of C1, wait more than 2 s, then start the fleet of B1 in terminal 1.

Expected: three `still waiting` warnings after 2 s; when the fleet starts, `robot action server is now available`
for each subtask, then the run of C1. The coordinator never blocks while waiting.

### Test C10: see every feedback message

Same as C4, with the coordinator at debug level:

```bash
ros2 run actions coordinator_action_client --ros-args -p time_scale:=10.0 --log-level coordinator_action_client:=debug
```

Expected: in addition, one line per feedback message (progress, state, time needed, time left, margin), in nominal seconds.

---

## Part D: robot parameters (single robot)

These tests start one robot by hand in namespace `/robot1` (or `/robot2`) and tell the coordinator to use only
that robot with `-p "robots:=[robot1]"`.

### Test D1: a slower robot (real time)

Terminal 1:

```bash
ros2 run actions robot_action_server --ros-args -r __ns:=/robot1 -p step_duration_sec:=2.0
```

Terminal 2:

```bash
ros2 run actions coordinator_action_client --ros-args -p "robots:=[robot1]"
```

Expected: `SUCCEEDED` after about 10 s (5 steps of 2 s), twice as long as the default robot.

### Test D2: a robot too slow for the budget

Terminal 1:

```bash
ros2 run actions robot_action_server --ros-args -r __ns:=/robot1 -p step_duration_sec:=7.0
```

Terminal 2: the coordinator command of D1.

Expected: rejected. Terminal 1: `timeout_sec 30.0 s is too short, needs more than 35.0 s`.

### Test D3: invalid time scale

```bash
ros2 run actions robot_action_server --ros-args -p time_scale:=0.0
```

Expected: the robot refuses to start: `parameter 'time_scale' could not be set ... doesn't comply with floating point range`.

### Test D4: simulation time refused

```bash
ros2 run actions robot_action_server --ros-args -p use_sim_time:=true
ros2 run actions coordinator_action_client --ros-args -p use_sim_time:=true
```

Expected: both refuse to start: `... does not support use_sim_time:=true; it runs on system time, use time_scale to
speed up experiments`.

### Test D5: robot name and namespace

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

Restart terminal 1 with an explicit name and simulate a short blockage:

```bash
ros2 run actions robot_action_server --ros-args -r __ns:=/robot2 -p robot_name:=carter_A -p simulated_delay_sec:=5.0
```

Terminal 2:

```bash
ros2 run actions coordinator_action_client --ros-args -p "robots:=[robot2]"
```

Expected: `Robot 'carter_A': action server started on /robot2/execute_subtask`; the coordinator reports
`robot carter_A is BLOCKED`, then `is no longer blocked`, then `SUCCEEDED`.

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
| B1 | Fleet starts | three robots, three namespaces |
| B2 | Fleet stops | all robots stop on `Ctrl+C` |
| C1 | One subtask per robot | three in parallel, 4.0 / 5.0 / 7.5 s, `succeeded=3` |
| C2 | Run again | `rejected=3`, coordinator exits |
| C3 | More subtasks than robots | `subtask4` rejected (busy), `succeeded=3, rejected=1` |
| C4 | Robot2 blocked | one `BLOCKED`, one `AT RISK`, `ABORTED`; others succeed |
| C5 | Short blockage | `BLOCKED`, `no longer blocked`, `succeeded=3` |
| C6 | Cancel one subtask | that one `CANCELED`, the others succeed |
| C7 | Reassignment | old attempt's answers ignored, `subtask1#2@robot3` `SUCCEEDED` |
| C8 | Robot not running | its subtask waits, others succeed, clean `Ctrl+C` |
| C9 | Coordinator before fleet | waits without blocking, then C1 |
| C10 | Debug level | one line per feedback |
| D1 | Slower robot | `SUCCEEDED` after about 10 s |
| D2 | Robot too slow | rejected: needs more than 35 s |
| D3 | Invalid time scale | robot refuses to start |
| D4 | Simulation time | robot and coordinator refuse to start |
| D5 | Name and namespace | `/robot2/execute_subtask`, name `robot2` (or `carter_A`), read-only |

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
| `Waiting for an action server to become available...` (`send_goal`) or `not available ... still waiting` (coordinator) | The robot is not running, runs in another namespace, its name is not in the coordinator's `robots` list, or it runs with a different `ROS_DOMAIN_ID`. Check `ros2 action list` and use the same `ROS_DOMAIN_ID` everywhere. |
| A goal you expect to succeed is rejected as `already completed` | That subtask already succeeded on this robot. Restart the robot (or the fleet). |
| No `BLOCKED` in a blockage test | The robot was not started with `simulated_delay_sec`. Check with `ros2 param get /robot2/robot_action_server simulated_delay_sec`, and look for the `Simulated deviation` line in the robot's terminal. |
| Times look ten times too short or too long | The robots and the coordinator use different `time_scale` values. Give them the same value. |
| `does not support use_sim_time:=true` | The mock robots and the coordinator run on system time only; remove `use_sim_time`. |
| Old behaviour after changing the code or the fleet YAML | Rebuild, then run `source install/setup.bash` again in every terminal. |
