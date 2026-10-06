cmake_minimum_required(VERSION 3.8)
project(actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(multi_robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(robot_action_server src/robot_action_server.cpp)   
ament_target_dependencies(robot_action_server rclcpp rclcpp_action multi_robot_interfaces geometry_msgs diagnostic_msgs)

add_executable(coordinator_action_client src/coordinator_action_client.cpp)
ament_target_dependencies(coordinator_action_client rclcpp rclcpp_action multi_robot_interfaces geometry_msgs diagnostic_msgs)

install(TARGETS 
robot_action_server 
coordinator_action_client
DESTINATION lib/${PROJECT_NAME}/
) # After building, copy the executable robot_action_server into the install directory so ROS 2 can find and run it

ament_package()
