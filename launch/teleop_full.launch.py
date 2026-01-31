"""ROS2 Launch file: full teleoperation system with hardware.

Launches all modules with real VR trackers, Manus Gloves, and robot.

Usage:
    ros2 launch teleop_system teleop_full.launch.py
    ros2 launch teleop_system teleop_full.launch.py robot_ip:=192.168.0.100
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip", default_value="192.168.0.100",
            description="RB-Y1 robot IP address",
        ),
        DeclareLaunchArgument(
            "config_file", default_value="default",
            description="Configuration profile name",
        ),

        # Arm teleoperation node (hardware mode)
        Node(
            package="teleop_system",
            executable="arm_teleop_node",
            name="arm_teleop",
            output="screen",
            parameters=[{
                "mode": "hardware",
                "robot_ip": LaunchConfiguration("robot_ip"),
            }],
        ),

        # Locomotion node
        Node(
            package="teleop_system",
            executable="locomotion_node",
            name="locomotion",
            output="screen",
            parameters=[{
                "mode": "hardware",
                "robot_ip": LaunchConfiguration("robot_ip"),
            }],
        ),

        # Hand teleoperation node
        Node(
            package="teleop_system",
            executable="hand_teleop_node",
            name="hand_teleop",
            output="screen",
            parameters=[{
                "mode": "hardware",
                "robot_ip": LaunchConfiguration("robot_ip"),
            }],
        ),
    ])
