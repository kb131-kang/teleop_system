"""Launch file for the complete simulation teleoperation system.

Launches both slave (MuJoCo) and master (simulated inputs) systems together.
This is the quickest way to test the full pipeline without any hardware.

Usage:
    source /opt/ros/jazzy/setup.bash
    MUJOCO_GL=glfw ros2 launch teleop_system teleop_sim_full.launch.py
    MUJOCO_GL=glfw ros2 launch teleop_system teleop_sim_full.launch.py launch_viewer:=true
    MUJOCO_GL=glfw ros2 launch teleop_system teleop_sim_full.launch.py publish_camera:=true

    # Full pipeline with camera viewer:
    MUJOCO_GL=glfw ros2 launch teleop_system teleop_sim_full.launch.py \\
        publish_camera:=true launch_camera_viewer:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory("teleop_system")
    launch_dir = os.path.join(pkg_dir, "launch")

    return LaunchDescription([
        # ── Shared launch arguments ──
        DeclareLaunchArgument(
            "launch_viewer", default_value="true",
            description="Launch MuJoCo passive viewer",
        ),
        DeclareLaunchArgument(
            "publish_camera", default_value="false",
            description="Publish RGB-D from MuJoCo head camera",
        ),
        DeclareLaunchArgument(
            "launch_camera_viewer", default_value="false",
            description="Launch RGB-D point cloud viewer on master side",
        ),
        DeclareLaunchArgument(
            "tracker_amplitude", default_value="0.08",
            description="Dummy tracker oscillation amplitude (meters)",
        ),
        DeclareLaunchArgument(
            "tracker_frequency", default_value="0.3",
            description="Dummy tracker oscillation frequency (Hz)",
        ),

        # ── Slave system (MuJoCo simulation) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "slave_mujoco.launch.py")
            ),
            launch_arguments={
                "launch_viewer": LaunchConfiguration("launch_viewer"),
                "publish_camera": LaunchConfiguration("publish_camera"),
            }.items(),
        ),

        # ── Master system (simulated inputs + teleop nodes) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "master_sim.launch.py")
            ),
            launch_arguments={
                "tracker_amplitude": LaunchConfiguration("tracker_amplitude"),
                "tracker_frequency": LaunchConfiguration("tracker_frequency"),
                "launch_viewer": LaunchConfiguration("launch_camera_viewer"),
            }.items(),
        ),
    ])
