#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include "multi_robot_interfaces/action/execute_subtask.hpp"
#include <chrono>

using ExecuteSubtask = multi_robot_interfaces::action::ExecuteSubtask;
using ExecuteSubtaskGoalHandle = rclcpp_action::ClientGoalHandle<ExecuteSubtask>;
using CancelResponse = action_msgs::srv::CancelGoal::Response;
using namespace std::placeholders;

class CoordinatorActionClientNode : public rclcpp::Node // MODIFY NAME
{
public:
    CoordinatorActionClientNode() : Node("coordinator_action_client") // MODIFY NAME
    {
        coordinator_action_client_ = rclcpp_action::create_client<ExecuteSubtask>(this, "execute_subtask");
        // full action name, for the logs (rclcpp_action::Client has no getter for it in Humble)
        action_name_ = std::string(this->get_namespace()) == "/" ? "/execute_subtask" : std::string(this->get_namespace()) + "/execute_subtask";

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

        // cancel_after_sec: send a cancel request this many nominal seconds after the goal is sent (0 = never).
        // Used to test cancellation; later the coordinator's own logic (e.g. a repair) will decide when to cancel.
        this->declare_parameter("cancel_after_sec", 0.0);

        RCLCPP_INFO(this->get_logger(), "Time scale %.1f: all times in this node's logs are nominal seconds", time_scale_);

        // Non-blocking server discovery (Phase B, point 1).
        // Goals are not sent directly: send_goal() queues them, and this timer sends a queued goal as soon as
        // the robot's action server is available. The coordinator never blocks waiting for a robot, so one
        // missing or slow-to-start robot cannot freeze it: its other callbacks keep running.
        // Discovery is not part of the scenario, so it uses a fixed real period (not scaled by time_scale).
        dispatch_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { dispatch_pending_goal(); });
    }

    // Queues a subtask; it is sent by dispatch_pending_goal() when the robot's action server is available.
    // timeout_sec is in nominal seconds: it is converted to real seconds with time_scale when the goal is sent.
    void send_goal(std::string mission_id, std::string subtask_id, std::string task_type, geometry_msgs::msg::PoseStamped target_pose, double timeout_sec)
    {
        pending_goal_ = PendingGoal{mission_id, subtask_id, task_type, target_pose, timeout_sec};
        has_pending_goal_ = true;
        pending_since_ = std::chrono::steady_clock::now();
        waiting_warned_ = false;
        RCLCPP_INFO(this->get_logger(), "Subtask %s queued, will be sent when the robot's action server is available", subtask_id.c_str());
    }

private:
    // A goal waiting to be sent (only one for now; Phase B point 2 replaces this with per-subtask records).
    struct PendingGoal
    {
        std::string mission_id;
        std::string subtask_id;
        std::string task_type;
        geometry_msgs::msg::PoseStamped target_pose;
        double timeout_sec; // nominal seconds
    };
    PendingGoal pending_goal_;
    bool has_pending_goal_ = false;
    std::chrono::steady_clock::time_point pending_since_; // when the goal was queued, to warn if the robot stays unavailable
    bool waiting_warned_ = false;                          // warn only once per queued goal
    rclcpp::TimerBase::SharedPtr dispatch_timer_;          // periodic check that sends queued goals (see constructor)
    static constexpr double kWaitingWarningSec = 2.0;      // real seconds before warning that a robot is not available

    // Called every 100 ms by dispatch_timer_. Sends the queued goal if its robot's action server is available,
    // otherwise returns immediately (never waits). action_server_is_ready() only checks what ROS has already
    // discovered, so it does not block.
    void dispatch_pending_goal()
    {
        if (!has_pending_goal_)
            return; // nothing to send

        if (!coordinator_action_client_->action_server_is_ready())
        {
            // robot not available (not started yet, or gone): keep the goal queued and try again at the next tick
            const double waiting_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - pending_since_).count();
            if (!waiting_warned_ && waiting_sec >= kWaitingWarningSec)
            {
                RCLCPP_WARN(this->get_logger(), "Subtask %s: robot action server '%s' not available after %.1f s (real), still waiting",
                            pending_goal_.subtask_id.c_str(), action_name_.c_str(), waiting_sec);
                waiting_warned_ = true;
            }
            return;
        }

        has_pending_goal_ = false; // the goal is sent now: do not send it again at the next tick
        if (waiting_warned_)
            RCLCPP_INFO(this->get_logger(), "Subtask %s: robot action server is now available", pending_goal_.subtask_id.c_str());
        send_goal_now(pending_goal_.mission_id, pending_goal_.subtask_id, pending_goal_.task_type,
                      pending_goal_.target_pose, pending_goal_.timeout_sec);
    }

    // Sends the goal to the robot. Only called when its action server is available (see dispatch_pending_goal).
    void send_goal_now(const std::string &mission_id, const std::string &subtask_id, const std::string &task_type,
                       const geometry_msgs::msg::PoseStamped &target_pose, double timeout_sec)
    {
        const double real_timeout_sec = timeout_sec / time_scale_;

        // new goal: no deviation reported yet
        blocked_ = false;
        at_risk_ = false;

        // create a goal
        auto goal = ExecuteSubtask::Goal();
        goal.mission_id = mission_id;
        goal.subtask_id = subtask_id;
        goal.task_type = task_type;
        goal.target_pose = target_pose;
        goal.timeout_sec = real_timeout_sec; // the robot works in real seconds
        const rclcpp::Time deadline = this->now() + rclcpp::Duration::from_seconds(real_timeout_sec); // Deadline of a subtask. It is the goal's own budget (send time + timeout_sec).  Computed when the goal is actually sent, so time spent waiting for the robot's server is not counted.

        // add callbacks
        auto options = rclcpp_action::Client<ExecuteSubtask>::SendGoalOptions(); // the options for sending the goal
        // options.result_callback = std::bind(&CoordinatorActionClientNode::goal_result_callback, this, _1);
        options.result_callback = std::bind(&CoordinatorActionClientNode::goal_result_callback, this, _1, subtask_id);
        options.goal_response_callback = std::bind(&CoordinatorActionClientNode::goal_response_callback, this, _1, subtask_id);          // we registere the goal response callback to be called when the goal is accepted or rejected by the action server
        options.feedback_callback = std::bind(&CoordinatorActionClientNode::goal_feedback_callback, this, _1, _2, subtask_id, deadline); // // Called each time the robot publishes feedback (_1 = goal handle, _2 = feedback message).

        // send the goal
        RCLCPP_INFO(this->get_logger(), "Sending goal with mission_id: %s, subtask_id: %s, task_type: %s, timeout %.1f s (%.2f s real)",
                    goal.mission_id.c_str(), goal.subtask_id.c_str(), goal.task_type.c_str(), timeout_sec, real_timeout_sec);
        coordinator_action_client_->async_send_goal(goal, options); // send the goal asynchronously

        // optional test cancel: a one-shot timer that sends a cancel request cancel_after_sec (nominal) after sending the goal
        const double cancel_after_nominal_sec = this->get_parameter("cancel_after_sec").as_double();
        const double cancel_after_sec = cancel_after_nominal_sec / time_scale_; // real seconds, for the timer
        if (cancel_after_sec > 0.0)
        {
            RCLCPP_INFO(this->get_logger(), "Subtask %s will be canceled after %.1f s (cancel_after_sec)", subtask_id.c_str(), cancel_after_nominal_sec);
            cancel_timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(cancel_after_sec)),
                // a lambda, not std::bind: create_wall_timer needs to see that the callback takes no argument
                [this, subtask_id]() { cancel_timer_callback(subtask_id); });
        }
    }

    // Fires once, cancel_after_sec after the goal was sent, and asks the robot to cancel the subtask.
    void cancel_timer_callback(const std::string &subtask_id)
    {
        cancel_timer_->cancel(); // one-shot: a timer repeats unless it is canceled

        // nothing to cancel if the goal was not accepted yet, was rejected, or has already finished
        if (!goal_handle_)
        {
            RCLCPP_WARN(this->get_logger(), "Cannot cancel subtask %s: no active goal (not accepted yet or already finished)", subtask_id.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Sending cancel request for subtask %s", subtask_id.c_str());
        try
        {
            // asynchronous: the robot's answer arrives in cancel_response_callback,
            // and the final CANCELED result arrives later in goal_result_callback
            coordinator_action_client_->async_cancel_goal(
                goal_handle_, std::bind(&CoordinatorActionClientNode::cancel_response_callback, this, _1, subtask_id));
        }
        catch (const rclcpp_action::exceptions::UnknownGoalHandleError &)
        {
            // the goal ended between the check above and the request: there is nothing left to cancel
            RCLCPP_WARN(this->get_logger(), "Cannot cancel subtask %s: the goal has already finished", subtask_id.c_str());
        }
    }

    // The robot's answer to the cancel request. Accepting only means the robot will stop;
    // the goal really ends when the CANCELED result arrives in goal_result_callback.
    void cancel_response_callback(const CancelResponse::SharedPtr response, const std::string &subtask_id)
    {
        switch (response->return_code)
        {
        case CancelResponse::ERROR_NONE:
            RCLCPP_INFO(this->get_logger(), "Cancel request for subtask %s accepted by the robot, waiting for the result", subtask_id.c_str());
            break;
        case CancelResponse::ERROR_REJECTED:
            // the robot refused to stop (e.g. stopping would be unsafe): the subtask keeps running
            RCLCPP_WARN(this->get_logger(), "Cancel request for subtask %s rejected by the robot", subtask_id.c_str());
            break;
        case CancelResponse::ERROR_UNKNOWN_GOAL_ID:
            RCLCPP_WARN(this->get_logger(), "Cancel request for subtask %s: the robot does not know this goal", subtask_id.c_str());
            break;
        case CancelResponse::ERROR_GOAL_TERMINATED:
            RCLCPP_WARN(this->get_logger(), "Cancel request for subtask %s: the goal had already finished", subtask_id.c_str());
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Cancel request for subtask %s: unknown answer %d", subtask_id.c_str(), response->return_code);
            break;
        }
    }

    void goal_response_callback(const ExecuteSubtaskGoalHandle::SharedPtr &goal_handle, const std::string &subtask_id)
    {
        if (!goal_handle)
        {
            // a rejected goal never existed on the robot: there is nothing to cancel and no result will come
            RCLCPP_WARN(this->get_logger(), "Subtask %s was rejected by the action server", subtask_id.c_str());
            rclcpp::shutdown();
            return;
        }

        goal_handle_ = goal_handle; // keep the goal handle: it is needed to cancel the goal
        RCLCPP_INFO(this->get_logger(), "Subtask %s accepted by the action server, waiting for result", subtask_id.c_str());
    }

    // Called each time the robot server publishes feedback during execution.
    // Logs the robot's progress and checks whether the subtask can still finish before its deadline.
    // This is the first step of deviation detection: a deviation is seen while it happens,
    // not only when the robot's timeout fires at the end.
    void goal_feedback_callback(ExecuteSubtaskGoalHandle::SharedPtr goal_handle,
                                const std::shared_ptr<const ExecuteSubtask::Feedback> feedback,
                                const std::string &subtask_id,
                                const rclcpp::Time &deadline)
    {
        (void)goal_handle; // not needed here; will identify the goal when several goals are in flight

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
        const double time_left_sec = (deadline - this->now()).seconds();
        const double margin_sec = time_left_sec - feedback->estimated_time_remaining_sec;
        const double needs_nominal = nominal(feedback->estimated_time_remaining_sec);
        const double left_nominal = nominal(time_left_sec);
        const double margin_nominal = nominal(margin_sec);

        // Routine progress at DEBUG level: one line per feedback is too much at a high time scale.
        // Show it with: --ros-args --log-level coordinator_action_client:=debug (or record it with rosbag).
        RCLCPP_DEBUG(this->get_logger(),
                     "Subtask %s on %s [%s]: progress=%.0f%%, needs %.1f s, %.1f s left before deadline, margin=%.1f s, recoveries=%d, '%s'",
                     subtask_id.c_str(), feedback->robot_name.c_str(), state.c_str(),
                     feedback->progress * 100.0, needs_nominal, left_nominal, margin_nominal,
                     feedback->number_of_recoveries, feedback->status_message.c_str());

        // 3. Deviation signals, reported when they START and when they END (state transitions),
        //    not on every feedback. Later, the transition to "at risk" is where the coordinator will run
        //    the impact analysis on the dependent subtasks and choose a repair.

        //    BLOCKED is tracked separately: a blocked robot is a deviation even while the margin is still positive.
        const bool blocked_now = (feedback->state == ExecuteSubtask::Feedback::STATE_BLOCKED);
        if (blocked_now != blocked_)
        {
            if (blocked_now)
                RCLCPP_WARN(this->get_logger(), "Subtask %s: robot %s is BLOCKED (margin %.1f s, recoveries %d)",
                            subtask_id.c_str(), feedback->robot_name.c_str(), margin_nominal, feedback->number_of_recoveries);
            else
                RCLCPP_INFO(this->get_logger(), "Subtask %s: robot %s is no longer blocked (margin %.1f s)",
                            subtask_id.c_str(), feedback->robot_name.c_str(), margin_nominal);
            blocked_ = blocked_now;
        }

        //    Negative margin: the subtask will finish after its deadline unless something changes.
        const bool at_risk_now = (margin_sec < 0.0);
        if (at_risk_now != at_risk_)
        {
            if (at_risk_now)
                RCLCPP_WARN(this->get_logger(),
                            "Subtask %s AT RISK: needs %.1f s but only %.1f s left before the deadline (%.1f s late)",
                            subtask_id.c_str(), needs_nominal, left_nominal, -margin_nominal);
            else
                RCLCPP_INFO(this->get_logger(), "Subtask %s is back on time (margin %.1f s)", subtask_id.c_str(), margin_nominal);
            at_risk_ = at_risk_now;
        }
    }

    // calback to receive the result once the goal is done
    void goal_result_callback(const ExecuteSubtaskGoalHandle::WrappedResult &result, const std::string &subtask_id)
    {
        // the goal is over: forget its handle and drop a pending test cancel, there is nothing left to cancel
        goal_handle_.reset();
        if (cancel_timer_)
            cancel_timer_->cancel();

        // 1. How the goal ended (action-level status)
        std::string status;
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            status = "SUCCEEDED";
            break;
        case rclcpp_action::ResultCode::ABORTED:
            status = "ABORTED";
            break;
        case rclcpp_action::ResultCode::CANCELED:
            status = "CANCELED";
            break;
        default:
            status = "UNKNOWN";
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
                    "Subtask %s finished with status %s: success=%s, error_code=%d, message='%s'",
                    subtask_id.c_str(), status.c_str(), success ? "true" : "false", error_code, message.c_str());

        RCLCPP_INFO(this->get_logger(),
                    "  execution_time=%.2f s, navigation_time=%.2f s, distance_remaining=%.2f m, recoveries=%d",
                    nominal(execution_time), nominal(navigation_time), distance_remaining, recoveries);

        RCLCPP_INFO(this->get_logger(),
                    "  final_pose: frame='%s', x=%.2f, y=%.2f, z=%.2f",
                    final_pose.header.frame_id.c_str(),
                    final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z);
        // RCLCPP_INFO(this->get_logger(), "Goal %s result of  received", subtask_id.c_str());
        rclcpp::shutdown();
    }

    // converts a real duration (as measured or reported by the robots) to nominal seconds, for the logs
    double nominal(double real_sec) const { return real_sec * time_scale_; }

    rclcpp_action::Client<ExecuteSubtask>::SharedPtr coordinator_action_client_;
    std::string action_name_; // full name of the robot's action (e.g. /robot1/execute_subtask), for the logs
    rclcpp::TimerBase::SharedPtr cancel_timer_;      // one-shot timer for the optional test cancel
    ExecuteSubtaskGoalHandle::SharedPtr goal_handle_; // handle of the accepted goal, needed to cancel it; empty when no goal is active
    double time_scale_ = 1.0;                         // value of the time_scale parameter, read once at start-up

    // Last deviation state reported for the current goal, so a deviation is logged when it starts and ends,
    // not on every feedback. One goal at a time for now; with several goals this becomes a map per subtask.
    bool blocked_ = false; // robot reported BLOCKED
    bool at_risk_ = false; // margin < 0
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoordinatorActionClientNode>(); // MODIFY NAME
    node->send_goal("mission1", "subtask1", "navigate", geometry_msgs::msg::PoseStamped(), 30.0);

    // Single-threaded on purpose: all the coordinator's callbacks (goal response, feedback, result,
    // cancel timer, cancel response) run one at a time, so its state (goal_handle_, and later the
    // workflow) needs no locks and runs are reproducible. This works because no callback blocks:
    // every request is asynchronous and its answer comes back in another callback.
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}