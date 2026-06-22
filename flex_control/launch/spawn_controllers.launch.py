"""Spawns the FLEX ros2_control controllers once the controller_manager is up.

The controller_manager itself is brought up by the gz_ros2_control plugin embedded in the
URDF (flex_description/urdf/flex.gazebo.xacro), which starts as soon as the robot entity is
spawned into Gazebo. This launch file just activates the controllers on top of it, in
sequence (joint_state_broadcaster and imu_sensor_broadcaster first, since diff_drive_controller
depends on the hardware interface already being claimed/reporting cleanly).

Usage (after the robot is spawned, e.g. via flex_demo.launch.py which includes this):
    ros2 launch flex_control spawn_controllers.launch.py
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def _spawner(name: str):
    return Node(
        package="controller_manager",
        executable="spawner",
        name=f"{name}_spawner",
        arguments=[name],
        output="screen",
    )


def generate_launch_description():
    joint_state_broadcaster_spawner = _spawner("joint_state_broadcaster")
    imu_sensor_broadcaster_spawner = _spawner("imu_sensor_broadcaster")
    diff_drive_controller_spawner = _spawner("diff_drive_controller")
    leg_position_controller_spawner = _spawner("leg_position_controller")

    # Give the gz_ros2_control plugin a moment to bring up the controller_manager service
    # before the spawners try to call it.
    delayed_broadcasters = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner, imu_sensor_broadcaster_spawner],
    )
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[diff_drive_controller_spawner, leg_position_controller_spawner],
    )

    return LaunchDescription([delayed_broadcasters, delayed_controllers])
