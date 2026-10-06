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

        //add callbacks
        auto options = rclcpp_action::Client<ExecuteSubtask>::SendGoalOptions(); // the options for sending the goal
        //options.result_callback = std::bind(&CoordinatorActionClientNode::goal_result_callback, this, _1);
        options.result_callback = std::bind(&CoordinatorActionClientNode::goal_result_callback, this, _1, subtask_id);

        //send the goal
        RCLCPP_INFO(this->get_logger(), "Sending goal with mission_id: %s, subtask_id: %s, task_type: %s", goal.mission_id.c_str(), goal.subtask_id.c_str(), goal.task_type.c_str());
        coordinator_action_client_->async_send_goal(goal, options); // send the goal asynchronously


        //send a cancel reuest 2 seconds later
    }

private:
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