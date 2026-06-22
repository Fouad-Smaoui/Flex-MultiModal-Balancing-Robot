"""ONE-COMMAND recruiter entry point: Gazebo Harmonic + FLEX spawn + ros_gz_bridge + RViz2.

Usage:
    ros2 launch flex_gazebo flex_demo.launch.py

This is what scripts/demo.sh calls after building the workspace. It brings up:
  1. Gazebo Harmonic (gz sim) running flex_world.sdf
  2. robot_state_publisher + the FLEX robot spawned into it (flex_spawn.launch.py)
  3. ros_gz_bridge translating /imu, /cmd_vel, /clock between Gazebo and ROS2
  4. RViz2 with a pre-built view of the robot + TF tree

See docs/architecture/deployment_architecture.md "Recruiter Demo Workflow" for the full
step-by-step including what to inspect (ros2 topic list, rqt_graph, RViz).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_pkg_share = get_package_share_directory("flex_gazebo")
    description_pkg_share = get_package_share_directory("flex_description")

    world_path = os.path.join(gazebo_pkg_share, "worlds", "flex_world.sdf")
    bridge_config_path = os.path.join(gazebo_pkg_share, "config", "gz_bridge.yaml")
    rviz_config_path = os.path.join(description_pkg_share, "rviz", "flex_view.rviz")
    spawn_launch_path = os.path.join(gazebo_pkg_share, "launch", "flex_spawn.launch.py")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_path)
    )

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="flex_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config_path}],
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
            gz_sim,
            spawn,
            bridge_node,
            rviz_node,
        ]
    )
