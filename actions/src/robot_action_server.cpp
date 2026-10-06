#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "multi_robot_interfaces/action/execute_subtask.hpp"

using ExecuteSubtask = multi_robot_interfaces::action::ExecuteSubtask;
using ExecuteSubtaskGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteSubtask>;
using namespace std::placeholders;


class RobotActionServerNode : public rclcpp::Node // MODIFY NAME
{
public:
    RobotActionServerNode() : Node("robot_action_server") // MODIFY NAME
    { 
         
        // create the action server here
        robot_action_server_= rclcpp_action::create_server<ExecuteSubtask>(
            this,
            "execute_subtask", // the name of the server such that the client can reach it
            // to bind the goal callback function to the server
            std::bind(&RobotActionServerNode::goal_callback, this, _1, _2),
            std::bind(&RobotActionServerNode::cancel_callback, this, _1), // 1 because we only pass one argument (the goal handle) on the cancel callback
            std::bind(&RobotActionServerNode::handle_accepted_callback, this, _1) // bind the handle accepted callback function to the server
        );
        RCLCPP_INFO(this->get_logger(), "Action server has been started");
    }

private:

//create action server and store it inside this node class : RobotActionServerNode
  rclcpp_action::Server<ExecuteSubtask>::SharedPtr robot_action_server_; //it is a smart pointer that will point to a server object created later.


// create a goal callback function here
// when the client sends a goal to the action server, this callback will be invoked with a unique identifier for that goal
rclcpp_action::GoalResponse goal_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteSubtask::Goal> goal)
    {
    (void)uuid; (void)goal;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

//create a cancel callback function here 
// it is executed when a client requests to cancel an ongoing goal
rclcpp_action::CancelResponse cancel_callback(
    const std::shared_ptr<ExecuteSubtaskGoalHandle> goal_handle)
    {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;

    };

    // this function is executed when a goal is accepted by the action server
    void handle_accepted_callback(const std::shared_ptr<ExecuteSubtaskGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing the goal");
        execute_goal(goal_handle);

    }
    // 
    void execute_goal(const std::shared_ptr<ExecuteSubtaskGoalHandle> goal_handle)
    {

        std::shared_ptr<const ExecuteSubtask::Goal> goal = goal_handle->get_goal(); //Get the user provided message describing the goal.
        auto  result = std::make_shared<ExecuteSubtask::Result>();

        if(!goal)
        {
            result->success = false;
            result->error_code = ExecuteSubtask::Result::ERROR_INVALID_GOAL;
            result->message = "Invalid goal, Received null goal";
            goal_handle->succeed(result);
            return;
        }

        const std::string mission_id = goal->mission_id;
        const std::string subtask_id = goal->subtask_id;
        const std::string task_type = goal->task_type;
        const auto target_pose = goal->target_pose;
        const auto start_time = this->get_clock()->now();

        RCLCPP_INFO(
            this->get_logger(), 
            "Executing subtask '%s' of mission '%s' on robot '%s'",
            subtask_id.c_str(), mission_id.c_str(), this->get_name());

        // Check if the task type is empty
            if (task_type.empty()){
                result->success = false;
                result->error_code = ExecuteSubtask::Result::ERROR_INVALID_GOAL;
                result->message="Task type is empty";
                goal_handle->abort(result); // we pass the result when we set the final state
                return;
            }
        // allow only supported task types
        if (task_type != "navigate" && task_type != "deliver" && task_type != "inspect")    
        {
            result->success = false;
            result->error_code = ExecuteSubtask::Result::ERROR_UNSUPPORTED_TASK;
            result->message="Unsupported task type" + task_type;
            goal_handle->abort(result);
            return;
        }

        if(goal->timeout_sec <= 0.0){
            result->success = false;
            result->error_code = ExecuteSubtask::Result::ERROR_INVALID_GOAL;
            result->message = "Invalid goal, timeout_sec must be > 0";
            goal_handle->abort(result);
            return;
        }

        // --- setup a dummy Nav2 path to target ---
    // For now, we simulate navigation; later, replace this with a real Nav2 client.
    auto current_pose = target_pose;
    current_pose.pose.position.x = 0.0;
    current_pose.pose.position.y = 0.0;
    
    const double timeout_sec = goal->timeout_sec;
    const auto timeout_deadline = this->now() + rclcpp::Duration::from_seconds(timeout_sec);

    float progress = 0.0f;
    float distance_remaining = 0.0f;
    float navigation_time_sec = 0.0f;
    int16_t recoveries = 0;

    (void)distance_remaining;

    // --- simulate navigation 
    // in this simulation loop, we are publishing feedback repeatedly
    for (int step = 0; step <5; ++step){
        if(goal_handle->is_canceling()){
            result->success = false;
            result->error_code= ExecuteSubtask::Result::ERROR_CANCELED;
            result->message="goal cancelled by the coordinatoor";
            goal_handle->abort(result);
            return;
        }
        if(this->now() > timeout_deadline){
            result->success = false;
            result->error_code= ExecuteSubtask::Result::ERROR_TIMEOUT;
            result->message="Subtask timeout exceeded";
            goal_handle->abort(result);
            return;
        }


        progress = (step+1) / 5.0f;
        navigation_time_sec = (this->now() - start_time).seconds();

        std::shared_ptr<ExecuteSubtask::Feedback> feedback = std::make_shared<ExecuteSubtask::Feedback>();
        feedback->robot_name = this->get_name();
        feedback->state = ExecuteSubtask::Feedback::STATE_NAVIGATING;
        feedback->progress = progress;
        feedback->current_pose = current_pose;
        feedback->estimated_time_remaining_sec = std::max(0.0f, static_cast<float>(timeout_sec - (this->now() - start_time).seconds()));
        feedback->distance_remaining = 1.0f - progress;
        feedback->number_of_recoveries = recoveries;
        feedback->navigation_time_sec = navigation_time_sec;
        feedback->status_message = "Navigating to target pose";

        goal_handle->publish_feedback(feedback); // we pass the result when we set the final state
        //Rate means : Wait 1 second before continuing to the next iteration of this loop. This is used to slow down a loop so it does not run too fast.
        // useful for visualizing progress, simulating realistic robot operation timing
        rclcpp::Rate loop_rate(1.0); 
        loop_rate.sleep();
    }

    // check if the subtask has timed out after the navigation loop
    if(this->now() > timeout_deadline){
        result->success = false;
        result->error_code= ExecuteSubtask::Result::ERROR_TIMEOUT;
        result->message="Subtask timeout exceeded";
        goal_handle->abort(result);
        return;
    }
    // update the navigation time after the navigation loop
     navigation_time_sec = (this->now() - start_time).seconds();
    //after navigation success, do task-specific operation
    if(task_type == "navigate"){
        //finish as soon as navigation is done
        result->success = true;
        result->error_code= ExecuteSubtask::Result::ERROR_NONE;
        result->message="Navigation completed successfully";
        result->execution_time_sec = (this->now() - start_time).seconds();
        result->navigation_time_sec = navigation_time_sec;
        result->distance_remaining = 0.0f;
        result->number_of_recoveries = recoveries;
        result->final_pose = target_pose;
        goal_handle->succeed(result); // we pass the result when we set the final state
        return;
    }

    // // --- simulate task execution for other task types ---
    // for (int step = 0; step < 3; ++step) {
    //     if (goal_handle->is_canceling()) {
    //         result->success = false;
    //         result->error_code = ExecuteSubtask::Result::ERROR_CANCELED;
    //         result->message = "Canceled during task execution";
    //         goal_handle->canceled(result);
    //         return;
    //     }

    //     if (this->now() > timeout_deadline) {
    //         result->success = false;
    //         result->error_code = ExecuteSubtask::Result::ERROR_TIMEOUT;
    //         result->message = "Timeout during task execution";
    //         goal_handle->abort(result);
    //         return;
    //     }

    //     auto feedback = std::make_shared<ExecuteSubtask::Feedback>();
    //     feedback->robot_name = this->get_name();
    //     feedback->state = ExecuteSubtask::Feedback::STATE_WORKING;
    //     feedback->progress = (step + 1) / 3.0f;
    //     feedback->current_pose = target_pose;
    //     feedback->estimated_time_remaining_sec = std::max(0.0f, timeout_sec - (this->now() - start_time).seconds());
    //     feedback->distance_remaining = 0.0f;
    //     feedback->navigation_time_sec = navigation_time_sec;
    //     feedback->number_of_recoveries = recoveries;
    //     feedback->status_message = "Executing " + task_type;

    //     goal_handle->publish_feedback(feedback);

    //     rclcpp::Rate loop_rate(1.0);
    //     loop_rate.sleep();
    // }
    //  result->success = true;
    // result->error_code = ExecuteSubtask::Result::ERROR_NONE;
    // result->message = "Task " + task_type + " completed successfully";
    // result->execution_time_sec = (this->now() - start_time).seconds();
    // result->navigation_time_sec = navigation_time_sec;
    // result->distance_remaining = 0.0f;
    // result->number_of_recoveries = recoveries;
    // result->final_pose = target_pose;

    // goal_handle->succeed(result);

};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Initialize the ROS2 communication system
    auto node = std::make_shared<RobotActionServerNode>(); // Create an instance of the custom node class
    rclcpp::spin(node); // Run the ROS2 event loop for the node
    rclcpp::shutdown(); // Shut down the ROS2 communication system (stop spinning the nodes)
    return 0;
}