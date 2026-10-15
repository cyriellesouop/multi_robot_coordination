#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "multi_robot_interfaces/action/execute_subtask.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
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
        // Callback group for the action server's callbacks (goal, cancel, accepted).
        // MutuallyExclusive: these callbacks never run at the same time, so two goals arriving together
        // cannot both see busy_ == false and both be accepted. They are short, so this costs nothing;
        // the long work (execute_goal) runs in its own thread, and the multi-threaded executor in main()
        // keeps other callbacks (e.g. parameter services) responsive.
        // Later, the Nova Carter server will put its Nav2 action client in a second group.
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Timing parameters. Durations are given in NOMINAL seconds (the time scale of the scenario);
        // the robot divides them by time_scale to get the real time it waits.
        // step_duration_sec: nominal time of one navigation step = this robot's speed (robots can differ).
        // time_scale: how much faster than real time the whole experiment runs (same value for every node);
        //             1.0 = real time, 10.0 = ten times faster. All times the robot reports are real seconds.
        // The ranges are enforced by ROS: a value outside them is refused at start-up and by `ros2 param set`.
        this->declare_parameter("step_duration_sec", 1.0, positive_range_descriptor("nominal duration of one navigation step (s)", 0.01, 60.0));
        this->declare_parameter("time_scale", 1.0, positive_range_descriptor("speed-up factor of the experiment (1 = real time)", 0.1, 1000.0));

        // parameters to simulate a deviation (default: no deviation)
        this->declare_parameter("simulated_delay_sec", 0.0); // simulated_delay_sec: how long (nominal s) the robot stays blocked during navigation, 0 = never blocked
        this->declare_parameter("delay_at_step", 2);         // delay_at_step: navigation step at which the robot gets blocked (0 to kNavigationSteps - 1)

        // robot_name: how this robot identifies itself in its feedback and logs, so the coordinator can tell
        // robots apart. Default: the namespace the robot was started in (e.g. "robot1" for /robot1),
        // or the node name if there is no namespace. A value given with -p robot_name:=... replaces the default.
        // The default is computed BEFORE declaring the parameter, so the parameter holds the real name and
        // `ros2 param get <node> robot_name` shows it. Read-only: a robot keeps the same name for its whole run.
        rcl_interfaces::msg::ParameterDescriptor name_descriptor;
        name_descriptor.description = "name of this robot in feedback and logs (default: its namespace)";
        name_descriptor.read_only = true;
        robot_name_ = this->declare_parameter("robot_name", default_robot_name(), name_descriptor);
        if (robot_name_.empty())
            robot_name_ = default_robot_name(); // -p robot_name:="" given explicitly: fall back to the default

        // create the action server here.
        // "execute_subtask" is a relative name: started in namespace /robot1, the server is /robot1/execute_subtask,
        // so several robots can run side by side without changing this code.
        robot_action_server_ = rclcpp_action::create_server<ExecuteSubtask>(
            this,
            "execute_subtask", // the name of the server such that the client can reach it
            // to bind the goal callback function to the server
            std::bind(&RobotActionServerNode::goal_callback, this, _1, _2),
            std::bind(&RobotActionServerNode::cancel_callback, this, _1),         // 1 because we only pass one argument (the goal handle) on the cancel callback
            std::bind(&RobotActionServerNode::handle_accepted_callback, this, _1), // bind the handle accepted callback function to the server
            rcl_action_server_get_default_options(), 
            callback_group_
        );
        RCLCPP_INFO(this->get_logger(), "Robot '%s': action server started on %s/execute_subtask",
                    robot_name_.c_str(), std::string(this->get_namespace()) == "/" ? "" : this->get_namespace());
    }

    // Wait for the execution thread before the node is destroyed: the thread uses the node (this),
    // so it must not outlive it. After Ctrl+C, rclcpp::ok() becomes false and the thread stops
    // within one check period (see wait_step).
    ~RobotActionServerNode()
    {
        if (execution_thread_.joinable())
            execution_thread_.join();
    }

private:

    // goal_callback runs in the executor's threads and execute_goal in its own thread, so shared state must be thread-safe
    std::atomic<bool> busy_{false}; // true while a subtask executes
    std::mutex completed_mutex_;
    std::set<std::string> completed_subtasks_; // keys "mission_id/subtask_id": subtask_id is only unique inside its mission, so subtask1 of mission1 and subtask1 of mission2 are different subtasks.
    std::thread execution_thread_;             // thread running execute_goal for the current subtask

    // number of steps of the simulated navigation; the duration of a step comes from the parameters
    static constexpr int kNavigationSteps = 5;
    // longest time between two cancel checks while the robot waits: a cancel is honoured within this time
    // (shorter when steps are short, see wait_step)
    static constexpr double kMaxCheckPeriodSec = 0.1;

    // Parameter descriptor with an allowed range [min, max], so invalid values (e.g. 0 or negative) are refused.
    static rcl_interfaces::msg::ParameterDescriptor positive_range_descriptor(const std::string &description, double min, double max)
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = min;
        range.to_value = max;
        descriptor.floating_point_range.push_back(range);
        return descriptor;
    }

    std::string robot_name_; // value of the robot_name parameter (see constructor)

    // Default robot name: the namespace without its leading '/' (e.g. "/robot1" -> "robot1"),
    // or the node name when the node runs without a namespace ("/").
    std::string default_robot_name() const
    {
        const std::string ns = this->get_namespace();
        return (ns == "/" || ns.empty()) ? std::string(this->get_name()) : ns.substr(1);
    }

    double time_scale() const { return this->get_parameter("time_scale").as_double(); }

    // real duration (s) of one navigation step for this robot: its nominal step duration divided by the time scale
    double real_step_sec() const { return this->get_parameter("step_duration_sec").as_double() / time_scale(); }

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
        //    (real seconds, like timeout_sec: depends on this robot's speed and on the time scale)
        const double expected_duration_sec = kNavigationSteps * real_step_sec();
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

        // 7. robot already executing another subtask.
        //    Check and claim in one atomic step: if busy_ is false, set it to true and accept;
        //    otherwise reject. No other goal can slip in between the check and the claim.
        bool expected_free = false;
        if (!busy_.compare_exchange_strong(expected_free, true))
        {
            RCLCPP_WARN(this->get_logger(), "Rejecting subtask '%s': robot is busy", goal->subtask_id.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        // all checks passed and the robot is now claimed (busy_ == true)
        RCLCPP_INFO(this->get_logger(), "Accepting subtask '%s'", goal->subtask_id.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Executed when a client requests to cancel an ongoing goal.
    // Accepting only marks the goal as "canceling": execute_goal sees it (is_canceling()), stops the robot
    // and sends the CANCELED result with what was done so far.
    // A real robot could return REJECT when stopping is unsafe (e.g. in the middle of placing an object).
    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<ExecuteSubtaskGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel subtask '%s'", goal_handle->get_goal()->subtask_id.c_str());
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // this function is executed when a goal is accepted by the action server
    // goal_callback returns ACCEPT_AND_EXECUTE, then handle_accepted_callback starts execute_goal
    void handle_accepted_callback(const std::shared_ptr<ExecuteSubtaskGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing the goal");

        // The previous subtask's thread has already finished execute_goal (otherwise busy_ would still be
        // true and this goal would have been rejected); join it to release it before starting a new one.
        if (execution_thread_.joinable())
            execution_thread_.join();

        // run execute_goal in a separate thread so handle_accepted_callback returns immediately
        // and the node keeps receiving goals and cancel requests while the robot works.
        execution_thread_ = std::thread{[this, goal_handle]()
                                        {
                                            execute_goal(goal_handle);
                                            busy_ = false; // robot is free again, whatever way execute_goal ended
                                        }};
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
        // Read the timing and deviation parameters once for this goal, so they stay consistent while it runs.
        // They are read at every goal, so they can be changed between goals with, for example:
        //   ros2 param set /robot_action_server simulated_delay_sec <nominal seconds>
        // Nominal durations are converted to real seconds here (divided by time_scale).
        const double step_sec = real_step_sec();
        const double delay_sec = this->get_parameter("simulated_delay_sec").as_double() / time_scale();
        const int delay_at_step = static_cast<int>(this->get_parameter("delay_at_step").as_int());

        RCLCPP_INFO(
            this->get_logger(),
            "Executing subtask '%s' of mission '%s' on robot '%s' (step %.3f s real, time scale %.1f)",
            subtask_id.c_str(), mission_id.c_str(), robot_name_.c_str(), step_sec, time_scale());

        if (delay_sec > 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Simulated deviation: subtask '%s' will be blocked for %.2f s (real) at step %d",
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

        // a cancel can arrive between acceptance and the start of this thread: honour it before doing anything
        if (stop_if_canceled_or_timed_out(goal_handle, result, timeout_deadline, " before execution",
                                          start_time, progress, recoveries, current_pose))
            return;

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
                ++recoveries;                                           // a blocked robot triggers a recovery attempt, as Nav2 would
                progress = step / static_cast<float>(kNavigationSteps); // no progress while blocked

                while ((this->now() - blocked_start).seconds() < delay_sec)
                {
                    // cancel and timeout must still work while the robot is blocked
                    if (stop_if_canceled_or_timed_out(goal_handle, result, timeout_deadline, " while blocked",
                                                      start_time, progress, recoveries, current_pose))
                        return;

                    // the robot does not know how long it will stay blocked: its estimate is the remaining
                    // navigation plus the time already lost, so it grows the longer the robot is blocked
                    const double blocked_sec = (this->now() - blocked_start).seconds();

                    publish_feedback(goal_handle, ExecuteSubtask::Feedback::STATE_BLOCKED, progress,
                                     (kNavigationSteps - step) * step_sec + blocked_sec,
                                     recoveries, start_time, current_pose, "Robot is blocked");

                    wait_step(goal_handle, step_sec);
                }
                RCLCPP_INFO(this->get_logger(), "Subtask '%s' is no longer blocked, resuming navigation", subtask_id.c_str());
            }

            // stop here if the coordinator canceled the goal or the time budget is exhausted
            if (stop_if_canceled_or_timed_out(goal_handle, result, timeout_deadline, " while navigating",
                                              start_time, progress, recoveries, current_pose))
                return;

            // feedback is published at the start of each step, so it describes the work already done
            progress = step / static_cast<float>(kNavigationSteps); // fraction already done
            publish_feedback(goal_handle, ExecuteSubtask::Feedback::STATE_NAVIGATING, progress,
                             (kNavigationSteps - step) * step_sec,
                             recoveries, start_time, current_pose, "Navigating to target pose");

            // wait one step to simulate the time the step takes (returns early on a cancel request)
            wait_step(goal_handle, step_sec);
            if (!goal_handle->is_canceling())
                progress = (step + 1) / static_cast<float>(kNavigationSteps); // this step is completed
        }

        // check for cancel or timeout after the navigation loop
        if (stop_if_canceled_or_timed_out(goal_handle, result, timeout_deadline, "",
                                          start_time, progress, recoveries, current_pose))
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
    // The result also reports what was done so far (times, recoveries, where the robot stopped):
    // the coordinator needs it to decide a repair (e.g. reassign the subtask from the robot's current position).
    bool stop_if_canceled_or_timed_out(const std::shared_ptr<ExecuteSubtaskGoalHandle> &goal_handle,
                                       const std::shared_ptr<ExecuteSubtask::Result> &result,
                                       const rclcpp::Time &timeout_deadline,
                                       const std::string &context,
                                       const rclcpp::Time &start_time,
                                       float progress,
                                       int16_t recoveries,
                                       const geometry_msgs::msg::PoseStamped &current_pose)
    {
        const bool shutting_down = !rclcpp::ok(); // Ctrl+C on the robot server
        if (!shutting_down && !goal_handle->is_canceling() && this->now() <= timeout_deadline)
            return false; // keep going

        // partial result: what the robot had done when it stopped
        const double elapsed_sec = (this->now() - start_time).seconds();
        result->success = false;
        result->execution_time_sec = elapsed_sec;
        result->navigation_time_sec = elapsed_sec; // the mock only navigates
        result->distance_remaining = 1.0f - progress; // mock only: distance is proportional to progress
        result->number_of_recoveries = recoveries;
        result->final_pose = current_pose;            // where the robot stopped

        if (shutting_down)
        {
            // The goal must still reach a final state: a goal handle destroyed while active tries to publish
            // a result itself, which fails once ROS is shut down and crashes the server.
            // abort() sets the final state first, then publishes; the publish may fail because communication
            // is closing, which is expected here, so that error is ignored.
            result->error_code = ExecuteSubtask::Result::ERROR_HARDWARE_FAULT; // system-level stop, not the subtask's fault
            result->message = "Robot server shutting down" + context;
            try
            {
                goal_handle->abort(result);
            }
            catch (const std::exception &)
            {
            }
            return true;
        }

        if (goal_handle->is_canceling())
        {
            // cancel is checked first: if both happen at once, the coordinator's explicit request wins
            result->error_code = ExecuteSubtask::Result::ERROR_CANCELED;
            result->message = "goal cancelled by the coordinator" + context;
            RCLCPP_INFO(this->get_logger(), "Subtask '%s' canceled%s after %.1f s",
                        goal_handle->get_goal()->subtask_id.c_str(), context.c_str(), elapsed_sec);
            goal_handle->canceled(result);
            return true;
        }

        result->error_code = ExecuteSubtask::Result::ERROR_TIMEOUT;
        result->message = "Subtask timeout exceeded" + context;
        RCLCPP_WARN(this->get_logger(), "Subtask '%s' timed out%s after %.1f s",
                    goal_handle->get_goal()->subtask_id.c_str(), context.c_str(), elapsed_sec);
        goal_handle->abort(result);
        return true;
    }

    // Simulates the time one step takes (step_sec, real seconds), but wakes up regularly
    // and returns early if the coordinator asked to cancel or the node is shutting down.
    // The check period is kMaxCheckPeriodSec, or a tenth of the step if steps are shorter
    // (fast time scale), so a cancel is always honoured well within one step.
    // Uses the steady clock: it measures real elapsed time and never jumps.
    void wait_step(const std::shared_ptr<ExecuteSubtaskGoalHandle> &goal_handle, double step_sec)
    {
        using Clock = std::chrono::steady_clock;
        const double check_sec = std::min(kMaxCheckPeriodSec, step_sec / 10.0);
        const auto step_duration = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(step_sec));
        const auto check_period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(check_sec));
        const Clock::time_point step_end = Clock::now() + step_duration;

        while (rclcpp::ok() && !goal_handle->is_canceling())
        {
            const Clock::time_point now = Clock::now();
            if (now >= step_end)
                return;
            std::this_thread::sleep_for(std::min(check_period, step_end - now)); // never sleep past the end of the step
        }
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
        feedback->robot_name = robot_name_; // identifies this robot to the coordinator
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
    // create action server and store it inside this node class : RobotActionServerNode
    rclcpp_action::Server<ExecuteSubtask>::SharedPtr robot_action_server_; // it is a smart pointer that will point to a server object created later.
    rclcpp::CallbackGroup::SharedPtr callback_group_; // group of the action server's callbacks (see constructor)
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                              // Initialize the ROS2 communication system
    auto node = std::make_shared<RobotActionServerNode>(); // Create an instance of the custom node class

    // Multi-threaded executor: callbacks of different callback groups can run in parallel
    // (here the action server's group and the node's default group, e.g. parameter services).
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();    // returns on Ctrl+C
    rclcpp::shutdown(); // Shut down the ROS2 communication system; the node's destructor then joins the execution thread
    return 0;
}
