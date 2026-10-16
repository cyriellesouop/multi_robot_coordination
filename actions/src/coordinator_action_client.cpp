#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include "multi_robot_interfaces/action/execute_subtask.hpp"
#include <chrono>
#include <algorithm>
#include <map>
#include <string>
#include <utility>

using ExecuteSubtask = multi_robot_interfaces::action::ExecuteSubtask;
using ExecuteSubtaskGoalHandle = rclcpp_action::ClientGoalHandle<ExecuteSubtask>;
using CancelResponse = action_msgs::srv::CancelGoal::Response;
using namespace std::placeholders;

// Execution state of a subtask, as seen by the coordinator (Phase B, point 2).
enum class SubtaskState
{
    WAITING_FOR_ROBOT, // queued: will be sent when its robot's action server is available
    GOAL_SENT,         // goal sent, waiting for the robot to accept or reject it
    RUNNING,           // accepted by the robot, executing
    SUCCEEDED,         // final: the robot completed it
    ABORTED,           // final: the robot gave up (e.g. timeout)
    CANCELED,          // final: stopped after a cancel request
    REJECTED,          // final: the robot refused the goal
};

// Text of a state, for the logs.
static const char *to_string(SubtaskState state)
{
    switch (state)
    {
    case SubtaskState::WAITING_FOR_ROBOT: return "WAITING_FOR_ROBOT";
    case SubtaskState::GOAL_SENT: return "GOAL_SENT";
    case SubtaskState::RUNNING: return "RUNNING";
    case SubtaskState::SUCCEEDED: return "SUCCEEDED";
    case SubtaskState::ABORTED: return "ABORTED";
    case SubtaskState::CANCELED: return "CANCELED";
    case SubtaskState::REJECTED: return "REJECTED";
    }
    return "UNKNOWN";
}

// True for the states a subtask cannot leave by itself (Phase B, point 3). The mission is finished when every
// subtask is in one of them. (Later, a repair may restart a final subtask as a new attempt: retry or reassign.)
static bool is_final(SubtaskState state)
{
    return state == SubtaskState::SUCCEEDED || state == SubtaskState::ABORTED ||
           state == SubtaskState::CANCELED || state == SubtaskState::REJECTED;
}

// A subtask is identified by its mission and its id: subtask_id is only unique inside its mission.
using SubtaskKey = std::pair<std::string, std::string>; // (mission_id, subtask_id)

// Everything the coordinator knows about one subtask (Phase B, point 2).
// Each subtask has its own record, so several subtasks can be in flight at the same time without
// sharing any state (goal handle, deadline, deviation flags, cancel timer).
struct SubtaskRecord
{
    // --- what to execute (set when the subtask is queued) ---
    std::string mission_id;
    std::string subtask_id;
    std::string robot; // robot the subtask is assigned to (its name, e.g. "robot1")
    std::string task_type;
    geometry_msgs::msg::PoseStamped target_pose;
    double timeout_nominal_sec = 0.0; // time budget, nominal seconds

    // --- runtime state ---
    SubtaskState state = SubtaskState::WAITING_FOR_ROBOT;
    // Attempt number: incremented each time the subtask is sent to a robot (first send = 1; a retry or a
    // reassignment = 2, 3, ...). Every callback carries the attempt it belongs to, so a late answer from an old
    // attempt (e.g. the CANCELED result of a canceled attempt arriving after the subtask was resent) is ignored
    // instead of overwriting the state of the current attempt.
    int attempt = 0;
    ExecuteSubtaskGoalHandle::SharedPtr goal_handle; // handle of the current attempt once accepted; needed to cancel it
    rclcpp::Time deadline;                           // when the current attempt must be finished (real time)

    // deviation state of the current attempt, so a deviation is logged when it starts and ends, not on every feedback
    bool blocked = false; // robot reported BLOCKED
    bool at_risk = false; // margin < 0

    // cancellation state of the current attempt
    bool cancel_requested = false;              // a cancel request was sent and the final result is not in yet
    rclcpp::TimerBase::SharedPtr cancel_timer; // one-shot timer for the optional test cancel

    // waiting for the robot's action server (see dispatch_pending_goals)
    std::chrono::steady_clock::time_point waiting_since; // when the subtask was queued
    bool waiting_warned = false;                          // warn only once that the robot is not available
};

class CoordinatorActionClientNode : public rclcpp::Node // MODIFY NAME
{
public:
    CoordinatorActionClientNode() : Node("coordinator_action_client") // MODIFY NAME
    {
        coordinator_action_client_ = rclcpp_action::create_client<ExecuteSubtask>(this, "execute_subtask");
        // full action name, for the logs (rclcpp_action::Client has no getter for it in Humble)
        action_name_ = std::string(this->get_namespace()) == "/" ? "/execute_subtask" : std::string(this->get_namespace()) + "/execute_subtask";
        // name of the robot behind that action (the namespace, e.g. "robot1"); Phase B point 4 replaces this
        // single client with one client per robot of the fleet
        robot_name_ = std::string(this->get_namespace()) == "/" ? "robot" : std::string(this->get_namespace()).substr(1);

        // time_scale: how much faster than real time the experiment runs; must be the same value as the robots'.
        // Durations of the scenario (timeout, cancel time) are given in NOMINAL seconds and divided by
        // time_scale before use, so the scenario does not change when the experiment is sped up.
        // The robots' feedback and results are in real seconds; this node converts them back to nominal
        // seconds for its logs, so a run prints the same numbers at any time scale.
        // Decisions (e.g. margin < 0) are made on real values; only the printing is nominal.
        rcl_interfaces::msg::ParameterDescriptor time_scale_descriptor;
        time_scale_descriptor.description = "speed-up factor of the experiment (1 = real time)";
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.1;
        range.to_value = 1000.0;
        time_scale_descriptor.floating_point_range.push_back(range);
        this->declare_parameter("time_scale", 1.0, time_scale_descriptor);
        time_scale_ = this->get_parameter("time_scale").as_double(); // fixed for the whole run

        // cancel_after_sec: send a cancel request this many nominal seconds after a subtask is sent (0 = never).
        // Used to test cancellation; later the coordinator's own logic (e.g. a repair) will decide when to cancel.
        this->declare_parameter("cancel_after_sec", 0.0);

        // subtask_count: number of subtasks main() queues for the robot (subtask1, subtask2, ...), default 1.
        // A test aid until the workflow model provides the mission: with 2 or more, the robot (which runs one
        // subtask at a time) rejects the extra ones as busy while the first keeps running, which shows that one
        // rejected or finished subtask no longer stops the coordinator (Phase B point 3).
        this->declare_parameter("subtask_count", 1);

        RCLCPP_INFO(this->get_logger(), "Time scale %.1f: all times in this node's logs are nominal seconds", time_scale_);

        // Non-blocking server discovery (Phase B, point 1).
        // Goals are not sent directly: send_goal() queues them, and this timer sends a queued goal as soon as
        // the robot's action server is available. The coordinator never blocks waiting for a robot, so one
        // missing or slow-to-start robot cannot freeze it: its other callbacks keep running.
        // Discovery is not part of the scenario, so it uses a fixed real period (not scaled by time_scale).
        dispatch_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { dispatch_pending_goals(); });
    }

    // Queues a subtask; it is sent by dispatch_pending_goals() when the robot's action server is available.
    // timeout_sec is in nominal seconds: it is converted to real seconds with time_scale when the goal is sent.
    void send_goal(std::string mission_id, std::string subtask_id, std::string task_type, geometry_msgs::msg::PoseStamped target_pose, double timeout_sec)
    {
        const SubtaskKey key{mission_id, subtask_id};
        if (subtasks_.count(key) > 0)
        {
            // the record already exists: queuing it again would silently reset its state
            RCLCPP_ERROR(this->get_logger(), "Subtask %s/%s is already known, not queued again", mission_id.c_str(), subtask_id.c_str());
            return;
        }

        SubtaskRecord record;
        record.mission_id = mission_id;
        record.subtask_id = subtask_id;
        record.robot = robot_name_;
        record.task_type = task_type;
        record.target_pose = target_pose;
        record.timeout_nominal_sec = timeout_sec;
        record.state = SubtaskState::WAITING_FOR_ROBOT;
        record.waiting_since = std::chrono::steady_clock::now();
        subtasks_[key] = record;

        RCLCPP_INFO(this->get_logger(), "Subtask %s/%s queued for robot %s, will be sent when its action server is available",
                    mission_id.c_str(), subtask_id.c_str(), robot_name_.c_str());
    }

private:
    // ---------------------------------------------------------------------------------------------
    // Per-subtask helpers
    // ---------------------------------------------------------------------------------------------

    // Short label for the logs: "mission/subtask#attempt@robot", e.g. "mission1/subtask1#2@robot3".
    static std::string label(const SubtaskRecord &record)
    {
        return record.mission_id + "/" + record.subtask_id + "#" + std::to_string(record.attempt) + "@" + record.robot;
    }

    // Returns the record of `key` if `attempt` is still its current attempt, or nullptr otherwise.
    // Every callback calls this first: a callback from an old attempt (the subtask was resent since) must not
    // change the record, which now describes the new attempt. `what` names the event, for the log.
    SubtaskRecord *current_record(const SubtaskKey &key, int attempt, const char *what)
    {
        auto it = subtasks_.find(key);
        if (it == subtasks_.end())
            return nullptr; // unknown subtask (should not happen)
        if (it->second.attempt != attempt)
        {
            RCLCPP_INFO(this->get_logger(), "Ignoring %s of %s/%s attempt %d: the current attempt is %d",
                        what, key.first.c_str(), key.second.c_str(), attempt, it->second.attempt);
            return nullptr;
        }
        return &it->second;
    }

    // ---------------------------------------------------------------------------------------------
    // Sending goals (Phase B, point 1: never blocks)
    // ---------------------------------------------------------------------------------------------

    // Called every 100 ms by dispatch_timer_. Sends every queued subtask whose robot's action server is available,
    // and leaves the others queued (never waits). action_server_is_ready() only checks what ROS has already
    // discovered, so it does not block.
    void dispatch_pending_goals()
    {
        for (auto &entry : subtasks_)
        {
            SubtaskRecord &record = entry.second;
            if (record.state != SubtaskState::WAITING_FOR_ROBOT)
                continue; // already sent, or finished

            if (!coordinator_action_client_->action_server_is_ready())
            {
                // robot not available (not started yet, or gone): keep the subtask queued and try again at the next tick
                const double waiting_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - record.waiting_since).count();
                if (!record.waiting_warned && waiting_sec >= kWaitingWarningSec)
                {
                    RCLCPP_WARN(this->get_logger(), "Subtask %s/%s: robot action server '%s' not available after %.1f s (real), still waiting",
                                record.mission_id.c_str(), record.subtask_id.c_str(), action_name_.c_str(), waiting_sec);
                    record.waiting_warned = true;
                }
                continue;
            }

            if (record.waiting_warned)
                RCLCPP_INFO(this->get_logger(), "Subtask %s/%s: robot action server is now available",
                            record.mission_id.c_str(), record.subtask_id.c_str());
            send_attempt(entry.first, record);
        }
    }

    // Sends a new attempt of the subtask to its robot. Only called when the robot's action server is available.
    // Starts a new attempt: everything about the previous attempt (goal handle, deviation flags, cancel state)
    // is reset, and the callbacks are bound to the new attempt number.
    void send_attempt(const SubtaskKey &key, SubtaskRecord &record)
    {
        record.attempt += 1;
        record.state = SubtaskState::GOAL_SENT;
        record.goal_handle.reset();
        record.blocked = false;
        record.at_risk = false;
        record.cancel_requested = false;
        if (record.cancel_timer)
            record.cancel_timer->cancel();
        record.waiting_warned = false;
        const int attempt = record.attempt;

        const double real_timeout_sec = record.timeout_nominal_sec / time_scale_;

        // create a goal
        auto goal = ExecuteSubtask::Goal();
        goal.mission_id = record.mission_id;
        goal.subtask_id = record.subtask_id;
        goal.task_type = record.task_type;
        goal.target_pose = record.target_pose;
        goal.timeout_sec = real_timeout_sec; // the robot works in real seconds
        // Deadline of this attempt. It is the goal's own budget (send time + timeout_sec). Computed when the goal is
        // actually sent, so time spent waiting for the robot's server is not counted. Later it comes from the workflow.
        record.deadline = this->now() + rclcpp::Duration::from_seconds(real_timeout_sec);

        // Callbacks: each one is bound to (key, attempt), so it can find its record and check that it still
        // belongs to the current attempt (see current_record). Lambdas capture these values by copy.
        auto options = rclcpp_action::Client<ExecuteSubtask>::SendGoalOptions();
        options.goal_response_callback = [this, key, attempt](const ExecuteSubtaskGoalHandle::SharedPtr &goal_handle)
        { goal_response_callback(goal_handle, key, attempt); };
        options.feedback_callback = [this, key, attempt](ExecuteSubtaskGoalHandle::SharedPtr goal_handle,
                                                         const std::shared_ptr<const ExecuteSubtask::Feedback> feedback)
        { goal_feedback_callback(goal_handle, feedback, key, attempt); };
        options.result_callback = [this, key, attempt](const ExecuteSubtaskGoalHandle::WrappedResult &result)
        { goal_result_callback(result, key, attempt); };

        // send the goal
        RCLCPP_INFO(this->get_logger(), "Sending %s: task %s, timeout %.1f s (%.2f s real)",
                    label(record).c_str(), record.task_type.c_str(), record.timeout_nominal_sec, real_timeout_sec);
        coordinator_action_client_->async_send_goal(goal, options); // send the goal asynchronously

        // optional test cancel: a one-shot timer that sends a cancel request cancel_after_sec (nominal) after sending the goal
        const double cancel_after_nominal_sec = this->get_parameter("cancel_after_sec").as_double();
        const double cancel_after_sec = cancel_after_nominal_sec / time_scale_; // real seconds, for the timer
        if (cancel_after_sec > 0.0)
        {
            RCLCPP_INFO(this->get_logger(), "%s will be canceled after %.1f s (cancel_after_sec)", label(record).c_str(), cancel_after_nominal_sec);
            record.cancel_timer = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(cancel_after_sec)),
                // a lambda, not std::bind: create_wall_timer needs to see that the callback takes no argument
                [this, key, attempt]() { cancel_timer_callback(key, attempt); });
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Cancellation
    // ---------------------------------------------------------------------------------------------

    // Fires once, cancel_after_sec after the attempt was sent, and asks the robot to cancel it.
    void cancel_timer_callback(const SubtaskKey &key, int attempt)
    {
        SubtaskRecord *record = current_record(key, attempt, "test cancel");
        if (!record)
            return;
        record->cancel_timer->cancel(); // one-shot: a timer repeats unless it is canceled

        // nothing to cancel if the goal was not accepted yet, was rejected, or has already finished
        if (!record->goal_handle || record->state != SubtaskState::RUNNING)
        {
            RCLCPP_WARN(this->get_logger(), "Cannot cancel %s: not running (state %s)", label(*record).c_str(), to_string(record->state));
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Sending cancel request for %s", label(*record).c_str());
        record->cancel_requested = true;
        try
        {
            // asynchronous: the robot's answer arrives in cancel_response_callback,
            // and the final CANCELED result arrives later in goal_result_callback
            coordinator_action_client_->async_cancel_goal(
                record->goal_handle,
                [this, key, attempt](const CancelResponse::SharedPtr response) { cancel_response_callback(response, key, attempt); });
        }
        catch (const rclcpp_action::exceptions::UnknownGoalHandleError &)
        {
            // the goal ended between the check above and the request: there is nothing left to cancel
            record->cancel_requested = false;
            RCLCPP_WARN(this->get_logger(), "Cannot cancel %s: the goal has already finished", label(*record).c_str());
        }
    }

    // The robot's answer to the cancel request. Accepting only means the robot will stop;
    // the goal really ends when the CANCELED result arrives in goal_result_callback.
    void cancel_response_callback(const CancelResponse::SharedPtr response, const SubtaskKey &key, int attempt)
    {
        SubtaskRecord *record = current_record(key, attempt, "cancel answer");
        if (!record)
            return;
        const std::string name = label(*record);

        switch (response->return_code)
        {
        case CancelResponse::ERROR_NONE:
            RCLCPP_INFO(this->get_logger(), "Cancel request for %s accepted by the robot, waiting for the result", name.c_str());
            break;
        case CancelResponse::ERROR_REJECTED:
            // the robot refused to stop (e.g. stopping would be unsafe): the subtask keeps running
            record->cancel_requested = false;
            RCLCPP_WARN(this->get_logger(), "Cancel request for %s rejected by the robot", name.c_str());
            break;
        case CancelResponse::ERROR_UNKNOWN_GOAL_ID:
            RCLCPP_WARN(this->get_logger(), "Cancel request for %s: the robot does not know this goal", name.c_str());
            break;
        case CancelResponse::ERROR_GOAL_TERMINATED:
            RCLCPP_WARN(this->get_logger(), "Cancel request for %s: the goal had already finished", name.c_str());
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Cancel request for %s: unknown answer %d", name.c_str(), response->return_code);
            break;
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Goal response, feedback, result
    // ---------------------------------------------------------------------------------------------

    void goal_response_callback(const ExecuteSubtaskGoalHandle::SharedPtr &goal_handle, const SubtaskKey &key, int attempt)
    {
        SubtaskRecord *record = current_record(key, attempt, "goal response");
        if (!record)
            return;

        if (!goal_handle)
        {
            // a rejected goal never existed on the robot: there is nothing to cancel and no result will come
            record->state = SubtaskState::REJECTED;
            RCLCPP_WARN(this->get_logger(), "%s was rejected by the robot", label(*record).c_str());
            // Phase B point 3: a rejection only updates this subtask; the other subtasks keep running.
            // The coordinator stops only when the whole mission is finished.
            check_mission_finished();
            return;
        }

        record->goal_handle = goal_handle; // keep the goal handle: it is needed to cancel this attempt
        record->state = SubtaskState::RUNNING;
        RCLCPP_INFO(this->get_logger(), "%s accepted by the robot, running", label(*record).c_str());
    }

    // Called each time the robot server publishes feedback during execution.
    // Logs the robot's progress and checks whether the subtask can still finish before its deadline.
    // This is the first step of deviation detection: a deviation is seen while it happens,
    // not only when the robot's timeout fires at the end.
    void goal_feedback_callback(ExecuteSubtaskGoalHandle::SharedPtr goal_handle,
                                const std::shared_ptr<const ExecuteSubtask::Feedback> feedback,
                                const SubtaskKey &key, int attempt)
    {
        (void)goal_handle; // the record (found with key and attempt) already identifies the goal
        SubtaskRecord *record = current_record(key, attempt, "feedback");
        if (!record)
            return;
        const std::string name = label(*record);

        // 1. Execution state reported by the robot, converted to text for the logs.
        //    The numeric values are the STATE_* constants defined in ExecuteSubtask.action.
        std::string state;
        switch (feedback->state)
        {
        case ExecuteSubtask::Feedback::STATE_NAVIGATING:
            state = "NAVIGATING";
            break;
        case ExecuteSubtask::Feedback::STATE_WORKING:
            state = "WORKING";
            break;
        case ExecuteSubtask::Feedback::STATE_BLOCKED:
            state = "BLOCKED";
            break;
        default:
            state = "UNKNOWN";
            break;
        }

        // 2. Margin = time left before the deadline - time the robot still needs to complete.
        //    The coordinator knows the deadline; only the robot knows how long it still needs
        //    (estimated_time_remaining_sec is the time to completion, not the time before the timeout).
        //    A negative margin means the subtask will finish late, even if the robot's timeout has not fired yet.
        //    Computed on real seconds (the robot's estimate and the deadline are real); converted to nominal only for printing.
        const double time_left_sec = (record->deadline - this->now()).seconds();
        const double margin_sec = time_left_sec - feedback->estimated_time_remaining_sec;
        const double needs_nominal = nominal(feedback->estimated_time_remaining_sec);
        const double left_nominal = nominal(time_left_sec);
        const double margin_nominal = nominal(margin_sec);

        // Routine progress at DEBUG level: one line per feedback is too much at a high time scale.
        // Show it with: --ros-args --log-level coordinator_action_client:=debug (or record it with rosbag).
        RCLCPP_DEBUG(this->get_logger(),
                     "%s [%s]: progress=%.0f%%, needs %.1f s, %.1f s left before deadline, margin=%.1f s, recoveries=%d, '%s'",
                     name.c_str(), state.c_str(),
                     feedback->progress * 100.0, needs_nominal, left_nominal, margin_nominal,
                     feedback->number_of_recoveries, feedback->status_message.c_str());

        // 3. Deviation signals, reported when they START and when they END (state transitions),
        //    not on every feedback. Later, the transition to "at risk" is where the coordinator will run
        //    the impact analysis on the dependent subtasks and choose a repair.
        //    The flags live in the subtask's record, so each subtask has its own deviation state.

        //    BLOCKED is tracked separately: a blocked robot is a deviation even while the margin is still positive.
        const bool blocked_now = (feedback->state == ExecuteSubtask::Feedback::STATE_BLOCKED);
        if (blocked_now != record->blocked)
        {
            if (blocked_now)
                RCLCPP_WARN(this->get_logger(), "%s: robot %s is BLOCKED (margin %.1f s, recoveries %d)",
                            name.c_str(), feedback->robot_name.c_str(), margin_nominal, feedback->number_of_recoveries);
            else
                RCLCPP_INFO(this->get_logger(), "%s: robot %s is no longer blocked (margin %.1f s)",
                            name.c_str(), feedback->robot_name.c_str(), margin_nominal);
            record->blocked = blocked_now;
        }

        //    Negative margin: the subtask will finish after its deadline unless something changes.
        const bool at_risk_now = (margin_sec < 0.0);
        if (at_risk_now != record->at_risk)
        {
            if (at_risk_now)
                RCLCPP_WARN(this->get_logger(),
                            "%s AT RISK: needs %.1f s but only %.1f s left before the deadline (%.1f s late)",
                            name.c_str(), needs_nominal, left_nominal, -margin_nominal);
            else
                RCLCPP_INFO(this->get_logger(), "%s is back on time (margin %.1f s)", name.c_str(), margin_nominal);
            record->at_risk = at_risk_now;
        }
    }

    // calback to receive the result once the goal is done
    void goal_result_callback(const ExecuteSubtaskGoalHandle::WrappedResult &result, const SubtaskKey &key, int attempt)
    {
        SubtaskRecord *record = current_record(key, attempt, "result");
        if (!record)
            return;

        // the attempt is over: forget its handle and drop a pending test cancel, there is nothing left to cancel
        record->goal_handle.reset();
        record->cancel_requested = false;
        if (record->cancel_timer)
            record->cancel_timer->cancel();

        // 1. How the goal ended (action-level status), stored as the subtask's final state
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            record->state = SubtaskState::SUCCEEDED;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            record->state = SubtaskState::CANCELED;
            break;
        default: // ABORTED, or UNKNOWN (treated as a failure)
            record->state = SubtaskState::ABORTED;
            break;
        }

        // 2. What the robot reported (our ExecuteSubtask result)
        bool success = result.result->success;
        int error_code = result.result->error_code;
        std::string message = result.result->message;
        float execution_time = result.result->execution_time_sec;
        float navigation_time = result.result->navigation_time_sec;
        float distance_remaining = result.result->distance_remaining;
        int recoveries = result.result->number_of_recoveries;
        auto final_pose = result.result->final_pose;

        RCLCPP_INFO(this->get_logger(),
                    "%s finished with status %s: success=%s, error_code=%d, message='%s'",
                    label(*record).c_str(), to_string(record->state), success ? "true" : "false", error_code, message.c_str());

        RCLCPP_INFO(this->get_logger(),
                    "  execution_time=%.2f s, navigation_time=%.2f s, distance_remaining=%.2f m, recoveries=%d",
                    nominal(execution_time), nominal(navigation_time), distance_remaining, recoveries);

        RCLCPP_INFO(this->get_logger(),
                    "  final_pose: frame='%s', x=%.2f, y=%.2f, z=%.2f",
                    final_pose.header.frame_id.c_str(),
                    final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z);

        // Phase B point 3: a result only updates this subtask; the other subtasks keep running.
        // The coordinator stops only when the whole mission is finished.
        check_mission_finished();
    }

    // ---------------------------------------------------------------------------------------------
    // End of the mission (Phase B, point 3)
    // ---------------------------------------------------------------------------------------------

    // Called each time a subtask reaches a final state. If every subtask is now final, the mission is over:
    // log a summary (one line per subtask, then the counts) and stop the coordinator.
    // This is the only place where the coordinator shuts itself down; individual callbacks never do.
    void check_mission_finished()
    {
        if (mission_finished_ || subtasks_.empty())
            return; // already reported, or nothing was queued
        for (const auto &entry : subtasks_)
            if (!is_final(entry.second.state))
                return; // at least one subtask is still waiting or running: the mission goes on

        mission_finished_ = true;
        std::map<SubtaskState, int> counts; // number of subtasks per final state
        RCLCPP_INFO(this->get_logger(), "Mission finished: %zu subtask(s)", subtasks_.size());
        for (const auto &entry : subtasks_)
        {
            const SubtaskRecord &record = entry.second;
            counts[record.state] += 1;
            RCLCPP_INFO(this->get_logger(), "  %s: %s", label(record).c_str(), to_string(record.state));
        }
        RCLCPP_INFO(this->get_logger(), "  succeeded=%d, aborted=%d, canceled=%d, rejected=%d",
                    counts[SubtaskState::SUCCEEDED], counts[SubtaskState::ABORTED],
                    counts[SubtaskState::CANCELED], counts[SubtaskState::REJECTED]);

        // Stop spinning: rclcpp::spin() in main() returns. Later, a coordinator serving several missions
        // would keep running instead.
        rclcpp::shutdown();
    }

    // converts a real duration (as measured or reported by the robots) to nominal seconds, for the logs
    double nominal(double real_sec) const { return real_sec * time_scale_; }

    rclcpp_action::Client<ExecuteSubtask>::SharedPtr coordinator_action_client_;
    std::string action_name_; // full name of the robot's action (e.g. /robot1/execute_subtask), for the logs
    std::string robot_name_;  // name of the robot behind coordinator_action_client_ (e.g. robot1)
    double time_scale_ = 1.0; // value of the time_scale parameter, read once at start-up

    // All subtasks known to the coordinator, one record each, indexed by (mission_id, subtask_id).
    // Only accessed from this node's callbacks, which run one at a time (single-threaded executor), so no lock is needed.
    std::map<SubtaskKey, SubtaskRecord> subtasks_;
    bool mission_finished_ = false; // set once every subtask is final, so the summary is logged only once

    rclcpp::TimerBase::SharedPtr dispatch_timer_;     // periodic check that sends queued subtasks (see constructor)
    static constexpr double kWaitingWarningSec = 2.0; // real seconds before warning that a robot is not available
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoordinatorActionClientNode>(); // MODIFY NAME

    // queue subtask1 ... subtaskN of mission1 (N = subtask_count, at least 1); later the workflow provides the mission
    const int subtask_count = std::max<int>(1, static_cast<int>(node->get_parameter("subtask_count").as_int()));
    for (int i = 1; i <= subtask_count; ++i)
        node->send_goal("mission1", "subtask" + std::to_string(i), "navigate", geometry_msgs::msg::PoseStamped(), 30.0);

    // Single-threaded on purpose: all the coordinator's callbacks (goal response, feedback, result,
    // cancel timer, cancel response) run one at a time, so its state (the subtask records, and later the
    // workflow) needs no locks and runs are reproducible. This works because no callback blocks:
    // every request is asynchronous and its answer comes back in another callback.
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
