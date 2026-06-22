"""Standalone RViz visualization of the FLEX URDF/Xacro model — no Gazebo required.

Verifies the robot model in isolation: robot_state_publisher (parses the xacro into /robot_description
and publishes /tf from joint states) + joint_state_publisher_gui (lets you manually drive the wheel/
leg joints to sanity-check the kinematic tree) + RViz2 with a pre-built config.

Usage:
    ros2 launch flex_description display.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("flex_description")
    xacro_path = os.path.join(pkg_share, "urdf", "flex.urdf.xacro")
    rviz_config_path = os.path.join(pkg_share, "rviz", "flex_view.rviz")

    use_gui = DeclareLaunchArgument(
        "use_joint_state_gui",
        default_value="true",
        description="Launch joint_state_publisher_gui to manually drive joints",
    )

    robot_description = ParameterValue(
        Command(["xacro ", xacro_path]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("use_joint_state_gui")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription(
        [
            use_gui,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
