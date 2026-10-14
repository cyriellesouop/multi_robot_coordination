#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "multi_robot_interfaces/action/execute_subtask.hpp"

using ExecuteSubtask = multi_robot_interfaces::action::ExecuteSubtask;
using ExecuteSubtaskGoalHandle = rclcpp_action::ClientGoalHandle<ExecuteSubtask>;
using namespace std::placeholders;

class CoordinatorActionClientNode : public rclcpp::Node // MODIFY NAME
{
public:
    CoordinatorActionClientNode() : Node("coordinator_action_client") // MODIFY NAME
    {
        coordinator_action_client_ = rclcpp_action::create_client<ExecuteSubtask>( this, "execute_subtask");

    }

    void send_goal(std::string mission_id, std::string subtask_id, std::string task_type, geometry_msgs::msg::PoseStamped target_pose, double timeout_sec){
        //wait for the server
        coordinator_action_client_->wait_for_action_server();

        //create a goal
        auto goal = ExecuteSubtask::Goal();
        goal.mission_id = mission_id;
        goal.subtask_id = subtask_id;
        goal.task_type = task_type;
        goal.target_pose = target_pose;
        goal.timeout_sec = timeout_sec;
        const rclcpp::Time deadline = this->now() + rclcpp::Duration::from_seconds(timeout_sec); // Deadline of a subtask. It is the goal's own budget (send time + timeout_sec).  Computed after wait_for_action_server() so that time spent waiting for the server is not counted.

        //add callbacks
        auto options = rclcpp_action::Client<ExecuteSubtask>::SendGoalOptions(); // the options for sending the goal
        //options.result_callback = std::bind(&CoordinatorActionClientNode::goal_result_callback, this, _1);
        options.result_callback = std::bind(&CoordinatorActionClientNode::goal_result_callback, this, _1, subtask_id);
        options.goal_response_callback = std::bind(&CoordinatorActionClientNode::goal_response_callback, this, _1, subtask_id); // we registere the goal response callback to be called when the goal is accepted or rejected by the action server
        options.feedback_callback = std::bind(&CoordinatorActionClientNode::goal_feedback_callback, this, _1, _2, subtask_id, deadline); // // Called each time the robot publishes feedback (_1 = goal handle, _2 = feedback message).


        //send the goal
        RCLCPP_INFO(this->get_logger(), "Sending goal with mission_id: %s, subtask_id: %s, task_type: %s", goal.mission_id.c_str(), goal.subtask_id.c_str(), goal.task_type.c_str());
        coordinator_action_client_->async_send_goal(goal, options); // send the goal asynchronously


        //send a cancel reuest 2 seconds later
    }

private:

void goal_response_callback(const ExecuteSubtaskGoalHandle::SharedPtr &goal_handle, const std::string &subtask_id)

    {
        if (!goal_handle) {
            RCLCPP_WARN(this->get_logger(), "Goal was rejected by the action server");
            rclcpp::shutdown();   // no result will come for a rejected goal
            return;
        } 
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
    (void)goal_handle; // not needed yet; will be used to cancel the goal or identify it among several goals

    // 1. Execution state reported by the robot, converted to text for the logs.
    //    The numeric values are the STATE_* constants defined in ExecuteSubtask.action.
    std::string state;
    switch (feedback->state) {
        case ExecuteSubtask::Feedback::STATE_NAVIGATING: state = "NAVIGATING"; break;
        case ExecuteSubtask::Feedback::STATE_WORKING:    state = "WORKING";    break;
        case ExecuteSubtask::Feedback::STATE_BLOCKED:    state = "BLOCKED";    break;
        default:                                         state = "UNKNOWN";    break;
    }

    // 2. Margin = time left before the deadline - time the robot still needs to complete.
    //    The coordinator knows the deadline; only the robot knows how long it still needs
    //    (estimated_time_remaining_sec is the time to completion, not the time before the timeout).
    //    A negative margin means the subtask will finish late, even if the robot's timeout has not fired yet.
    const double time_left_sec = (deadline - this->now()).seconds();
    const double margin_sec = time_left_sec - feedback->estimated_time_remaining_sec;

    RCLCPP_INFO(this->get_logger(),
        "Subtask %s on %s [%s]: progress=%.0f%%, needs %.1f s, %.1f s left before deadline, margin=%.1f s, recoveries=%d, '%s'",
        subtask_id.c_str(), feedback->robot_name.c_str(), state.c_str(),
        feedback->progress * 100.0, feedback->estimated_time_remaining_sec, time_left_sec, margin_sec,
        feedback->number_of_recoveries, feedback->status_message.c_str());

    // 3. Deviation signals.
    //    BLOCKED is reported separately: a blocked robot is a deviation even while the margin is still positive.
    if (feedback->state == ExecuteSubtask::Feedback::STATE_BLOCKED) {
        RCLCPP_WARN(this->get_logger(), "Subtask %s: robot %s reports it is BLOCKED",
            subtask_id.c_str(), feedback->robot_name.c_str());
    }

    //    Negative margin: the subtask is at risk. Later, this is where the coordinator will run
    //    the impact analysis on the dependent subtasks and choose a repair.
    if (margin_sec < 0.0) {
        RCLCPP_WARN(this->get_logger(),
            "Subtask %s AT RISK: needs %.1f s but only %.1f s left before the deadline (%.1f s late)",
            subtask_id.c_str(), feedback->estimated_time_remaining_sec, time_left_sec, -margin_sec);
    }
}


//calback to receive the result once the goal is done
void goal_result_callback(const ExecuteSubtaskGoalHandle::WrappedResult &result, const std::string &subtask_id)
{
     // 1. How the goal ended (action-level status)
    std::string status;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: status = "SUCCEEDED"; break;
        case rclcpp_action::ResultCode::ABORTED:   status = "ABORTED";   break;
        case rclcpp_action::ResultCode::CANCELED:  status = "CANCELED";  break;
        default:                                   status = "UNKNOWN";   break;
    }
    

    // 2. What the robot reported (our ExecuteSubtask result)
    bool success = result.result->success;
    int error_code = result.result->error_code;
    std::string message = result.result->message;
    float execution_time =result.result->execution_time_sec;
    float navigation_time =result.result->navigation_time_sec;
    float distance_remaining =result.result->distance_remaining;
    int recoveries = result.result->number_of_recoveries;
    auto final_pose = result.result->final_pose;


    RCLCPP_INFO(this->get_logger(),
        "Subtask %s finished with status %s: success=%s, error_code=%d, message='%s'",
        subtask_id.c_str(), status.c_str(), success ? "true" : "false", error_code, message.c_str());

    RCLCPP_INFO(this->get_logger(),
        "  execution_time=%.2f s, navigation_time=%.2f s, distance_remaining=%.2f m, recoveries=%d",
        execution_time, navigation_time, distance_remaining, recoveries);

    RCLCPP_INFO(this->get_logger(),
        "  final_pose: frame='%s', x=%.2f, y=%.2f, z=%.2f",
        final_pose.header.frame_id.c_str(),
        final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z);
    //RCLCPP_INFO(this->get_logger(), "Goal %s result of  received", subtask_id.c_str());
    rclcpp::shutdown();
}

    rclcpp_action::Client<ExecuteSubtask>::SharedPtr coordinator_action_client_;
 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoordinatorActionClientNode>(); // MODIFY NAME
    node->send_goal("mission1", "subtask1", "navigate", geometry_msgs::msg::PoseStamped(), 30.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}