#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "multi_robot_interfaces/action/execute_subtask.hpp"
#include <atomic>
#include <mutex>
#include <set>
#include <string>
#include <thread>

using ExecuteSubtask = multi_robot_interfaces::action::ExecuteSubtask;
using ExecuteSubtaskGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteSubtask>;
using namespace std::placeholders;

class RobotActionServerNode : public rclcpp::Node // MODIFY NAME
{
public:
    RobotActionServerNode() : Node("robot_action_server") // MODIFY NAME
    {
        // parameters to simulate a deviation (default: no deviation)
        this->declare_parameter("simulated_delay_sec", 0.0); // simulated_delay_sec: how long the robot stays blocked during navigation, 0 = never blocked
        this->declare_parameter("delay_at_step", 2);         // delay_at_step: navigation step at which the robot gets blocked (0 to kNavigationSteps - 1)

        // create the action server here
        robot_action_server_ = rclcpp_action::create_server<ExecuteSubtask>(
            this,
            "execute_subtask", // the name of the server such that the client can reach it
            // to bind the goal callback function to the server
            std::bind(&RobotActionServerNode::goal_callback, this, _1, _2),
            std::bind(&RobotActionServerNode::cancel_callback, this, _1),         // 1 because we only pass one argument (the goal handle) on the cancel callback
            std::bind(&RobotActionServerNode::handle_accepted_callback, this, _1) // bind the handle accepted callback function to the server
        );
        RCLCPP_INFO(this->get_logger(), "Action server has been started");
    }

private:
    // create action server and store it inside this node class : RobotActionServerNode
    rclcpp_action::Server<ExecuteSubtask>::SharedPtr robot_action_server_; // it is a smart pointer that will point to a server object created later.

    // goal_callback runs in the node's thread and execute_goal in its own thread, so shared state must be thread-safe
    std::atomic<bool> busy_{false}; // true while a subtask executes
    std::mutex completed_mutex_;
    std::set<std::string> completed_subtasks_; // keys "mission_id/subtask_id": subtask_id is only unique inside its mission, so subtask1 of mission1 and subtask1 of mission2 are different subtasks.

    // constants for the simulated navigation duration
    static constexpr int kNavigationSteps = 5;
    static constexpr double kStepDurationSec = 1.0;

    // called when the coordinator sends a goal: decide whether this robot takes the subtask
    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ExecuteSubtask::Goal> goal)
    {
        (void)uuid;

        RCLCPP_INFO(this->get_logger(), "Goal received: mission '%s', subtask '%s', task '%s', timeout %.1f s",
                    goal->mission_id.c_str(), goal->subtask_id.c_str(), goal->task_type.c_str(), goal->timeout_sec);

        // 1. mission id
        if (goal->mission_id.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Rejecting subtask '%s': mission_id is empty", goal->subtask_id.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        // 2. subtask id
        if (goal->subtask_id.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal of mission '%s': subtask_id is empty", goal->mission_id.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        // 3. task type
        if (goal->task_type.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Rejecting subtask '%s': task_type is empty", goal->subtask_id.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        // only navigate is implemented so far; add deliver and inspect when execute_goal supports them
        if (goal->task_type != "navigate")
        {
            RCLCPP_WARN(this->get_logger(), "Rejecting subtask '%s': unsupported task_type '%s'",
                        goal->subtask_id.c_str(), goal->task_type.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        // 4. time budget must be a positive number
        if (!(goal->timeout_sec > 0.0))
        {
            RCLCPP_WARN(this->get_logger(), "Rejecting subtask '%s': timeout_sec must be > 0, got %.1f",
                        goal->subtask_id.c_str(), goal->timeout_sec);
            return rclcpp_action::GoalResponse::REJECT;
        }

        // 5. time budget must be long enough for this robot to finish
        const double expected_duration_sec = kNavigationSteps * kStepDurationSec;
        if (goal->timeout_sec <= expected_duration_sec)
        {
            RCLCPP_WARN(this->get_logger(), "Rejecting subtask '%s': timeout_sec %.1f s is too short, needs more than %.1f s",
                        goal->subtask_id.c_str(), goal->timeout_sec, expected_duration_sec);
            return rclcpp_action::GoalResponse::REJECT;
        }

        // 6. subtask already completed successfully
        {
            std::lock_guard<std::mutex> lock(completed_mutex_);
            const std::string key = goal->mission_id + "/" + goal->subtask_id;
            if (completed_subtasks_.count(key) > 0)
            {
                RCLCPP_WARN(this->get_logger(), "Rejecting subtask '%s': already completed", key.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        // 7. robot already executing another subtask
        if (busy_)
        {
            RCLCPP_WARN(this->get_logger(), "Rejecting subtask '%s': robot is busy", goal->subtask_id.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        // all checks passed: claim the robot before accepting
        busy_ = true;
        RCLCPP_INFO(this->get_logger(), "Accepting subtask '%s'", goal->subtask_id.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // create a cancel callback function here
    //  it is executed when a client requests to cancel an ongoing goal
    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<ExecuteSubtaskGoalHandle> goal_handle)
    {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // this function is executed when a goal is accepted by the action server
    // goal_callback returns ACCEPT_AND_EXECUTE, then handle_accepted_callback starts execute_goal
    void handle_accepted_callback(const std::shared_ptr<ExecuteSubtaskGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing the goal");

        // run execute_goal in a separate thread to avoid blocking the node's thread, and release the robot when it ends.
        // .detach() lets the thread run on its own while handle_accepted_callback returns immediately,
        // so the node goes back to receiving goals and cancel requests.
        std::thread{[this, goal_handle]()
                    {
                        execute_goal(goal_handle);
                        busy_ = false; // robot is free again, whatever way execute_goal ended
                    }}
            .detach();
    }

    // runs in its own thread: executes the subtask and sets its final state (succeed, abort or canceled)
    void execute_goal(const std::shared_ptr<ExecuteSubtaskGoalHandle> goal_handle)
    {
        std::shared_ptr<const ExecuteSubtask::Goal> goal = goal_handle->get_goal(); // Get the user provided message describing the goal.
        auto result = std::make_shared<ExecuteSubtask::Result>();

        if (!goal)
        {
            result->success = false;
            result->error_code = ExecuteSubtask::Result::ERROR_INVALID_GOAL;
            result->message = "Invalid goal, Received null goal";
            goal_handle->abort(result);
            return;
        }

        // the goal fields were already validated in goal_callback
        const std::string mission_id = goal->mission_id;
        const std::string subtask_id = goal->subtask_id;
        const std::string task_type = goal->task_type;
        const auto target_pose = goal->target_pose;
        const auto start_time = this->get_clock()->now();
        // read the deviation parameters for this goal; they are read at every goal, so they can be changed
        const double delay_sec = this->get_parameter("simulated_delay_sec").as_double();
        const int delay_at_step = static_cast<int>(this->get_parameter("delay_at_step").as_int());

        RCLCPP_INFO(
            this->get_logger(),
            "Executing subtask '%s' of mission '%s' on robot '%s'",
            subtask_id.c_str(), mission_id.c_str(), this->get_name());

        // between goals with: ros2 param set /robot_action_server simulated_delay_sec <seconds>
        if (delay_sec > 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Simulated deviation: subtask '%s' will be blocked for %.1f s at step %d",
                        subtask_id.c_str(), delay_sec, delay_at_step);
        }

        // --- setup a dummy Nav2 path to target ---
        // For now, we simulate navigation; later, replace this with a real Nav2 client.
        auto current_pose = target_pose;
        current_pose.pose.position.x = 0.0;
        current_pose.pose.position.y = 0.0;

        const double timeout_sec = goal->timeout_sec;
        const auto timeout_deadline = this->now() + rclcpp::Duration::from_seconds(timeout_sec);

        float progress = 0.0f;
        float navigation_time_sec = 0.0f;
        int16_t recoveries = 0;

        // --- simulate navigation
        // in this simulation loop, we are publishing feedback repeatedly
        for (int step = 0; step < kNavigationSteps; ++step)
        {

            // --- simulated deviation: the robot is blocked before doing this step ---
            // Placed at the start of the step, so the normal cancel and timeout checks below run again
            // as soon as the robot is free.
            if (step == delay_at_step && delay_sec > 0.0)
            {
                const auto blocked_start = this->now();
                ++recoveries; // a blocked robot triggers a recovery attempt, as Nav2 would
                rclcpp::Rate blocked_rate(1.0 / kStepDurationSec);

                while ((this->now() - blocked_start).seconds() < delay_sec)
                {
                    // cancel and timeout must still work while the robot is blocked
                    if (stop_if_canceled_or_timed_out(goal_handle, result, timeout_deadline, " while blocked"))
                        return;

                    // the robot does not know how long it will stay blocked: its estimate is the remaining
                    // navigation plus the time already lost, so it grows the longer the robot is blocked
                    const double blocked_sec = (this->now() - blocked_start).seconds();
                    progress = step / static_cast<float>(kNavigationSteps); // no progress while blocked

                    publish_feedback(goal_handle, ExecuteSubtask::Feedback::STATE_BLOCKED, progress,
                                     (kNavigationSteps - step) * kStepDurationSec + blocked_sec,
                                     recoveries, start_time, current_pose, "Robot is blocked");

                    blocked_rate.sleep();
                }
                RCLCPP_INFO(this->get_logger(), "Subtask '%s' is no longer blocked, resuming navigation", subtask_id.c_str());
            }

            // stop here if the coordinator canceled the goal or the time budget is exhausted
            if (stop_if_canceled_or_timed_out(goal_handle, result, timeout_deadline, " while navigating"))
                return;
            // feedback is published at the start of each step, so it describes the work already done
            progress = step / static_cast<float>(kNavigationSteps); // fraction already done
            publish_feedback(goal_handle, ExecuteSubtask::Feedback::STATE_NAVIGATING, progress,
                             (kNavigationSteps - step) * kStepDurationSec,
                             recoveries, start_time, current_pose, "Navigating to target pose");

            // Rate means : Wait kStepDurationSec before continuing to the next iteration of this loop. This is used to slow down a loop so it does not run too fast.
            //  useful for visualizing progress, simulating realistic robot operation timing
            rclcpp::Rate loop_rate(1.0 / kStepDurationSec);
            loop_rate.sleep();
        }

        // check for cancel or timeout after the navigation loop
        if (stop_if_canceled_or_timed_out(goal_handle, result, timeout_deadline, ""))
            return;

        // update the navigation time after the navigation loop
        navigation_time_sec = (this->now() - start_time).seconds();

        // after navigation success, do task-specific operation
        if (task_type == "navigate")
        {
            // finish as soon as navigation is done
            result->success = true;
            result->error_code = ExecuteSubtask::Result::ERROR_NONE;
            result->message = "Navigation completed successfully";
            result->execution_time_sec = (this->now() - start_time).seconds();
            result->navigation_time_sec = navigation_time_sec;
            result->distance_remaining = 0.0f;
            result->number_of_recoveries = recoveries;
            result->final_pose = target_pose;

            // record the subtask as completed so goal_callback rejects it if it is sent again.
            // Only successful subtasks are recorded: failed or canceled ones must stay retryable.
            // Recorded before succeed() so a resend arriving right after the result is already rejected.
            {
                std::lock_guard<std::mutex> lock(completed_mutex_);
                completed_subtasks_.insert(mission_id + "/" + subtask_id);
            }

            goal_handle->succeed(result); // we pass the result when we set the final state
            return;
        }

        // safety net: every goal must end with a final state
        result->success = false;
        result->error_code = ExecuteSubtask::Result::ERROR_UNSUPPORTED_TASK;
        result->message = "Task type '" + task_type + "' is not implemented yet";
        goal_handle->abort(result);
    }

    // Ends the goal if the coordinator canceled it or its time budget is exhausted.
    // Returns true when the goal is finished, so callers just do: if (stop_if_canceled_or_timed_out(...)) return;
    // `context` is added to the result message to tell where it happened (e.g. "while blocked").
    bool stop_if_canceled_or_timed_out(const std::shared_ptr<ExecuteSubtaskGoalHandle> &goal_handle,
                                       const std::shared_ptr<ExecuteSubtask::Result> &result,
                                       const rclcpp::Time &timeout_deadline,
                                       const std::string &context)
    {
        if (goal_handle->is_canceling())
        {
            result->success = false;
            result->error_code = ExecuteSubtask::Result::ERROR_CANCELED;
            result->message = "goal cancelled by the coordinator" + context;
            goal_handle->canceled(result);
            return true;
        }
        if (this->now() > timeout_deadline)
        {
            result->success = false;
            result->error_code = ExecuteSubtask::Result::ERROR_TIMEOUT;
            result->message = "Subtask timeout exceeded" + context;
            goal_handle->abort(result);
            return true;
        }
        return false;
    }

    // Builds one feedback message and sends it to the coordinator.
    // Only the values that differ between callers are parameters; the rest is derived here,
    // so every feedback message is filled the same way.
    void publish_feedback(const std::shared_ptr<ExecuteSubtaskGoalHandle> &goal_handle,
                          uint8_t state,
                          float progress,
                          double estimated_time_remaining_sec, // time left before completion, not before the timeout
                          int16_t recoveries,
                          const rclcpp::Time &start_time,
                          const geometry_msgs::msg::PoseStamped &current_pose,
                          const std::string &status_message)
    {
        auto feedback = std::make_shared<ExecuteSubtask::Feedback>();
        feedback->robot_name = this->get_name();
        feedback->state = state;
        feedback->progress = progress;
        feedback->current_pose = current_pose;
        feedback->estimated_time_remaining_sec = static_cast<float>(estimated_time_remaining_sec);
        feedback->distance_remaining = 1.0f - progress; // mock only: distance is proportional to progress
        feedback->number_of_recoveries = recoveries;
        feedback->navigation_time_sec = (this->now() - start_time).seconds();
        feedback->status_message = status_message;
        goal_handle->publish_feedback(feedback);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                              // Initialize the ROS2 communication system
    auto node = std::make_shared<RobotActionServerNode>(); // Create an instance of the custom node class
    rclcpp::spin(node);                                    // Run the ROS2 event loop for the node
    rclcpp::shutdown();                                    // Shut down the ROS2 communication system (stop spinning the nodes)
    return 0;
}
