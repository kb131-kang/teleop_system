"""ROS2 Launch file: simulation-only teleoperation.

Launches all teleoperation modules with simulated devices and MuJoCo backend.
No real hardware required.

Usage:
    ros2 launch teleop_system teleop_sim.launch.py
    ros2 launch teleop_system teleop_sim.launch.py sim_backend:=mujoco
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "sim_backend", default_value="mujoco",
            description="Simulation backend (mujoco or isaac_lab)",
        ),
        DeclareLaunchArgument(
            "config_file", default_value="default",
            description="Configuration profile name",
        ),

        # Arm teleoperation node
        Node(
            package="teleop_system",
            executable="arm_teleop_node",
            name="arm_teleop",
            output="screen",
            parameters=[{
                "mode": "simulation",
                "sim_backend": LaunchConfiguration("sim_backend"),
            }],
        ),

        # Locomotion node
        Node(
            package="teleop_system",
            executable="locomotion_node",
            name="locomotion",
            output="screen",
            parameters=[{"mode": "simulation"}],
        ),

        # Hand teleoperation node
        Node(
            package="teleop_system",
            executable="hand_teleop_node",
            name="hand_teleop",
            output="screen",
            parameters=[{"mode": "simulation"}],
        ),
    ])
