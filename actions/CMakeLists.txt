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
find_package(yaml-cpp REQUIRED)  # mission / fleet YAML loading of the DWAFP library

# DWAFP core library (Phase C): ROS-free C++ (standard library + yaml-cpp for loading only), so the
# same code is used by the unit tests, the discrete-event simulator and the ROS coordinator adapter.
# Static: it is linked into this package's own executables and tests, nothing to install.
add_library(dwafp_core STATIC
  src/workflow.cpp
  src/mission_loader.cpp
  src/impact.cpp  # C2: deviation trigger, expected state, impact analysis
  src/repair.cpp  # C3: Stage I single-edit repair candidates
  src/simulator.cpp  # C6: closed-loop simulator (the world)
  src/controllers.cpp  # C6: the compared methods as simulator controllers
  src/scenarios.cpp  # C6: scenario generator
  src/runner.cpp  # C6: experiment runner (one run, paired statistics)
)
target_include_directories(dwafp_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(dwafp_core PUBLIC cxx_std_17)
target_link_libraries(dwafp_core PRIVATE yaml-cpp)  # private: the headers do not expose yaml-cpp

add_executable(robot_action_server src/robot_action_server.cpp)   
ament_target_dependencies(robot_action_server rclcpp rclcpp_action multi_robot_interfaces geometry_msgs diagnostic_msgs)

add_executable(coordinator_action_client src/coordinator_action_client.cpp)
ament_target_dependencies(coordinator_action_client rclcpp rclcpp_action multi_robot_interfaces geometry_msgs diagnostic_msgs)

# Stage I experiments, ROS-free: links only the DWAFP library (no rclcpp).
add_executable(stage1_experiments src/stage1_experiments.cpp)
target_link_libraries(stage1_experiments dwafp_core)

# Closed-loop experiments (simulator + the five methods), ROS-free.
add_executable(dwafp_experiments src/dwafp_experiments.cpp)
target_link_libraries(dwafp_experiments dwafp_core)

install(TARGETS 
robot_action_server 
coordinator_action_client
stage1_experiments
dwafp_experiments
DESTINATION lib/${PROJECT_NAME}/
) # After building, copy the executable robot_action_server into the install directory so ROS 2 can find and run it

# Install the launch files and the fleet configuration into share/actions/,
# where `ros2 launch actions ...` and get_package_share_directory("actions") look for them.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Unit tests of the DWAFP library, run with `colcon test --packages-select actions`.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_workflow test/test_workflow.cpp)
  target_link_libraries(test_workflow dwafp_core)
  # The tests load the real config files (fleet + missions) from the source tree.
  target_compile_definitions(test_workflow PRIVATE
    ACTIONS_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

  # C2: trigger and impact analysis, same config files.
  ament_add_gtest(test_impact test/test_impact.cpp)
  target_link_libraries(test_impact dwafp_core)
  target_compile_definitions(test_impact PRIVATE
    ACTIONS_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

  # C6: closed-loop simulator.
  ament_add_gtest(test_simulator test/test_simulator.cpp)
  target_link_libraries(test_simulator dwafp_core)
  target_compile_definitions(test_simulator PRIVATE
    ACTIONS_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

  # C6: the compared methods in the simulator.
  ament_add_gtest(test_controllers test/test_controllers.cpp)
  target_link_libraries(test_controllers dwafp_core)
  target_compile_definitions(test_controllers PRIVATE
    ACTIONS_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

  # C6: scenario generator.
  ament_add_gtest(test_scenarios test/test_scenarios.cpp)
  target_link_libraries(test_scenarios dwafp_core)

  # C6: experiment runner.
  ament_add_gtest(test_runner test/test_runner.cpp)
  target_link_libraries(test_runner dwafp_core)

  # C3: Stage I repair candidates, same config files.
  ament_add_gtest(test_repair test/test_repair.cpp)
  target_link_libraries(test_repair dwafp_core)
  target_compile_definitions(test_repair PRIVATE
    ACTIONS_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
endif()

ament_package()
