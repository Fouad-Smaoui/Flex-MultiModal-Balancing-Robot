"""Spawns the FLEX robot model into a running Gazebo Harmonic world.

Starts robot_state_publisher (from the flex_description xacro) and uses ros_gz_sim's
`create` executable to spawn the resulting /robot_description into Gazebo at a small
height above the ground plane so it settles under gravity — the core sanity check for
this package (see docs/architecture/simulation_architecture.md).

Usage (after flex_world.sdf / Gazebo is already running, e.g. via flex_demo.launch.py):
    ros2 launch flex_gazebo flex_spawn.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg_share = get_package_share_directory("flex_description")
    xacro_path = os.path.join(description_pkg_share, "urdf", "flex.urdf.xacro")

    spawn_z = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.30",
        description="Spawn height above ground (m) — robot settles onto wheels under gravity",
    )

    robot_description = ParameterValue(Command(["xacro ", xacro_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="flex_spawn",
        output="screen",
        arguments=[
            "-name", "flex",
            "-topic", "robot_description",
            "-z", LaunchConfiguration("spawn_z"),
        ],
    )

    return LaunchDescription(
        [
            spawn_z,
            robot_state_publisher_node,
            spawn_entity_node,
        ]
    )
