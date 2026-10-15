"""Start a fleet of mock robots, one robot_action_server per robot, each in its own namespace.

Usage:
    ros2 launch actions mock_fleet.launch.py
    ros2 launch actions mock_fleet.launch.py time_scale:=10.0
    ros2 launch actions mock_fleet.launch.py fleet_config:=/path/to/my_fleet.yaml

The robots and their parameters come from a YAML file (default: config/mock_fleet.yaml of this package),
so changing the fleet means editing the YAML, not this file.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Fields a robot entry may have in the YAML file, with their default values.
# Defaults are written as floats/ints of the right type: the robot declares step_duration_sec and
# simulated_delay_sec as doubles, and ROS refuses an integer for a double parameter.
ROBOT_DEFAULTS = {
    "step_duration_sec": 1.0,
    "simulated_delay_sec": 0.0,
    "delay_at_step": 2,
}


def load_robots(config_path):
    """Read the fleet YAML file and return the list of robot entries, with defaults filled in."""
    with open(config_path, "r") as f:
        config = yaml.safe_load(f) or {}

    robots = config.get("robots", [])
    if not robots:
        raise RuntimeError(f"No robots defined in {config_path} (expected a 'robots:' list)")

    names = set()
    result = []
    for entry in robots:
        if "name" not in entry:
            raise RuntimeError(f"A robot in {config_path} has no 'name': {entry}")
        name = str(entry["name"])
        if name in names:
            # two robots in the same namespace would share one action name and clash
            raise RuntimeError(f"Robot name '{name}' is used twice in {config_path}")
        names.add(name)

        robot = {"name": name}
        for key, default in ROBOT_DEFAULTS.items():
            value = entry.get(key, default)
            # convert to the type the robot node expects (e.g. YAML `2` -> 2.0 for a double parameter)
            robot[key] = type(default)(value)
        result.append(robot)
    return result


def launch_robots(context):
    """Create one robot node per YAML entry.

    Runs in an OpaqueFunction because the launch arguments (config path, time_scale) are only
    known once the launch file is executed, and we need their values to read the YAML file.
    """
    config_path = LaunchConfiguration("fleet_config").perform(context)
    time_scale = float(LaunchConfiguration("time_scale").perform(context))
    robots = load_robots(config_path)

    nodes = []
    for robot in robots:
        nodes.append(
            Node(
                package="actions",
                executable="robot_action_server",
                # namespace: the action becomes /<name>/execute_subtask, and robot_name defaults to <name>
                namespace=robot["name"],
                # same node name in every namespace: the full names (/robot1/robot_action_server, ...) stay unique
                name="robot_action_server",
                parameters=[{
                    "time_scale": time_scale,  # same value for the whole fleet (and for the coordinator)
                    "step_duration_sec": robot["step_duration_sec"],
                    "simulated_delay_sec": robot["simulated_delay_sec"],
                    "delay_at_step": robot["delay_at_step"],
                }],
                output="screen",  # show each robot's logs in the launch terminal
            )
        )
    return nodes


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("actions"), "config", "mock_fleet.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "fleet_config",
            default_value=default_config,
            description="YAML file listing the robots and their parameters",
        ),
        DeclareLaunchArgument(
            "time_scale",
            default_value="1.0",
            description="Speed-up of the experiment for every robot (1.0 = real time); give the same value to the coordinator",
        ),
        OpaqueFunction(function=launch_robots), #the launch file needs the actual YAML path to read it, and the actual number of robots to decide how many nodes to create. OpaqueFunction delays launch_robots until the values are known;
    ])
