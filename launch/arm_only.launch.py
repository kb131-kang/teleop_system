"""ROS2 Launch file: arm teleoperation only.

Usage:
    ros2 launch teleop_system arm_only.launch.py
    ros2 launch teleop_system arm_only.launch.py mode:=hardware
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="simulation"),
        DeclareLaunchArgument("sim_backend", default_value="mujoco"),

        Node(
            package="teleop_system",
            executable="arm_teleop_node",
            name="arm_teleop",
            output="screen",
            parameters=[{
                "mode": LaunchConfiguration("mode"),
                "sim_backend": LaunchConfiguration("sim_backend"),
            }],
        ),
    ])
