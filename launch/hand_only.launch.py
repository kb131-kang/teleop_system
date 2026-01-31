"""ROS2 Launch file: hand teleoperation only.

Usage:
    ros2 launch teleop_system hand_only.launch.py
    ros2 launch teleop_system hand_only.launch.py mode:=hardware
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="simulation"),

        Node(
            package="teleop_system",
            executable="hand_teleop_node",
            name="hand_teleop",
            output="screen",
            parameters=[{
                "mode": LaunchConfiguration("mode"),
            }],
        ),
    ])
