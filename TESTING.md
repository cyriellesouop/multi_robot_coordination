# Manual tests for the mock robot action server

This guide explains how to build the `actions` package and check, by hand, how the mock robot
(`robot_action_server`) accepts, rejects, executes and cancels `ExecuteSubtask` goals.

There are no automated tests yet. Each test below gives the command to type and the result to expect.

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

You need three terminals. Run these lines first in **every** terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

- **Terminal 1** runs the robot server.
- **Terminal 2** sends the test goals.
- **Terminal 3** is only needed for the "busy" test.

## 3. Start the robot server (terminal 1)

```bash
ros2 run actions robot_action_server
```

Leave it running. When a goal is rejected, the client only prints `Goal was rejected.`;
the reason appears in this terminal as a `[WARN]` line.

## 4. What the server checks

Before accepting a goal, the server rejects it if:

1. `mission_id` is empty,
2. `subtask_id` is empty,
3. `task_type` is empty or is not `navigate` (the only task implemented so far),
4. `timeout_sec` is not greater than 0,
5. `timeout_sec` is too short to finish (the mock needs 5 s, so `timeout_sec` must be greater than 5.0),
6. the same subtask (`mission_id` + `subtask_id`) has already **succeeded**,
7. the robot is already executing another subtask.

Failed or canceled subtasks are not recorded, so they can be sent again (retried).

## 5. Tests (terminal 2)

Each test uses a different `subtask_id` on purpose: the server remembers every subtask that succeeded and
rejects it if it is sent again. To start again from a clean state, restart the server in terminal 1
(`Ctrl+C`, then `ros2 run actions robot_action_server`).

Run the tests in this order.

### Test 1: success

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s1, task_type: navigate, timeout_sec: 10.0}" --feedback
```

Expected: 5 feedback messages with `progress` from 0.0 to 0.8 and `estimated_time_remaining_sec` from 5.0
to 1.0, then `Goal finished with status: SUCCEEDED` and `error_code: 0`.

### Test 2: duplicate subtask

The same subtask as test 1:

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s1, task_type: navigate, timeout_sec: 10.0}"
```

Expected: `Goal was rejected.` Terminal 1: `Rejecting subtask 'm1/s1': already completed`.

### Test 3: same subtask id in another mission

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m2, subtask_id: s1, task_type: navigate, timeout_sec: 10.0}"
```

Expected: accepted, then `SUCCEEDED`. A subtask is identified by its mission and its id together,
so `m2/s1` is a different subtask from `m1/s1`.

### Test 4: time budget too short

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s2, task_type: navigate, timeout_sec: 5.0}"
```

Expected: `Goal was rejected.` Terminal 1: `timeout_sec 5.0 s is too short, needs more than 5.0 s`.

### Test 5: invalid time budget

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s2, task_type: navigate, timeout_sec: 0.0}"
```

Expected: `Goal was rejected.` Terminal 1: `timeout_sec must be > 0, got 0.0`.

### Test 6: empty mission id

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: '', subtask_id: s2, task_type: navigate, timeout_sec: 10.0}"
```

Expected: `Goal was rejected.` Terminal 1: `mission_id is empty`.

### Test 7: empty subtask id

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: '', task_type: navigate, timeout_sec: 10.0}"
```

Expected: `Goal was rejected.` Terminal 1: `subtask_id is empty`.

### Test 8: empty task type

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s2, task_type: '', timeout_sec: 10.0}"
```

Expected: `Goal was rejected.` Terminal 1: `task_type is empty`.

### Test 9: unsupported task type

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s2, task_type: inspect, timeout_sec: 10.0}"
```

Expected: `Goal was rejected.` Terminal 1: `unsupported task_type 'inspect'`.

### Test 10: robot busy

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

### Test 11: cancel

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s5, task_type: navigate, timeout_sec: 10.0}" --feedback
```

Press `Ctrl+C` in terminal 2 after one or two feedback messages.

Expected: the client sends a cancel request, and the server stops executing the subtask.

### Test 12: retry after cancel

The same subtask as test 11:

```bash
ros2 action send_goal /execute_subtask multi_robot_interfaces/action/ExecuteSubtask "{mission_id: m1, subtask_id: s5, task_type: navigate, timeout_sec: 10.0}"
```

Expected: accepted, then `SUCCEEDED`. Canceled subtasks are not recorded as completed, so they can be retried.

### Test 13: coordinator client

```bash
ros2 run actions coordinator_action_client
```

Expected: the client sends `mission1/subtask1` with a 30 s budget, logs the result (`SUCCEEDED`,
`error_code=0`, execution and navigation times) and exits.

If you run it a second time without restarting the server, the goal is rejected as a duplicate. The client
does not handle rejections yet, so it keeps waiting. Stop it with `Ctrl+C`.

## Summary

| # | Test | Expected |
|---|---|---|
| 1 | Success | `SUCCEEDED` |
| 2 | Duplicate subtask | rejected: already completed |
| 3 | Same subtask id, other mission | `SUCCEEDED` |
| 4 | Budget too short | rejected: too short |
| 5 | Invalid budget | rejected: must be > 0 |
| 6 | Empty mission id | rejected |
| 7 | Empty subtask id | rejected |
| 8 | Empty task type | rejected |
| 9 | Unsupported task type | rejected |
| 10 | Robot busy | second goal rejected, first `SUCCEEDED` |
| 11 | Cancel | execution stops |
| 12 | Retry after cancel | `SUCCEEDED` |
| 13 | Coordinator client | `SUCCEEDED` logged, client exits |

## Viewing the server's messages elsewhere

Every log message of the server is also published on the `/rosout` topic and saved under `~/.ros/log/`.
To follow the rejection reasons from another terminal:

```bash
ros2 topic echo /rosout
```

Or open `rqt_console` to filter them by severity.

## Troubleshooting

| Problem | Cause and fix |
|---|---|
| `No executable found` | The workspace was built from `src/`, or `install/setup.bash` was not sourced. Build from the workspace root and source it again. |
| `Waiting for an action server to become available...` | The server is not running, or it runs in a terminal with a different `ROS_DOMAIN_ID`. Start it in terminal 1 and use the same `ROS_DOMAIN_ID` everywhere. |
| A goal you expect to succeed is rejected as `already completed` | That subtask already succeeded earlier. Use a new `subtask_id`, or restart the server. |
| Old behaviour after changing the code | Rebuild, then run `source install/setup.bash` again in every terminal. |
