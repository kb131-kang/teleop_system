"""ROS2 utility helpers for QoS profiles and common patterns.

Provides pre-defined QoS profiles and utility functions for
creating ROS2 publishers, subscribers, and services with consistent settings.
"""

from enum import Enum, auto


class QoSPreset(Enum):
    """Pre-defined QoS profile presets for teleoperation."""
    SENSOR_DATA = auto()
    COMMAND = auto()
    STATUS = auto()
    PARAMETER = auto()


def get_qos_profile(preset: QoSPreset):
    """Get a ROS2 QoS profile for the given preset.

    Args:
        preset: QoSPreset enum value.

    Returns:
        rclpy.qos.QoSProfile configured for the preset, or None if rclpy unavailable.
    """
    try:
        from rclpy.qos import (
            QoSProfile,
            ReliabilityPolicy,
            DurabilityPolicy,
            HistoryPolicy,
        )
    except ImportError:
        return None

    profiles = {
        QoSPreset.SENSOR_DATA: QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        ),
        QoSPreset.COMMAND: QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        ),
        QoSPreset.STATUS: QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        ),
        QoSPreset.PARAMETER: QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        ),
    }
    return profiles[preset]


class TopicNames:
    """Standard ROS2 topic names for the teleoperation system."""

    TRACKER_RIGHT_HAND = "/master/tracker/right"
    TRACKER_LEFT_HAND = "/master/tracker/left"
    TRACKER_WAIST = "/master/tracker/waist"
    TRACKER_RIGHT_FOOT = "/master/tracker/right_foot"
    TRACKER_LEFT_FOOT = "/master/tracker/left_foot"

    TRACKER_HEAD = "/master/tracker/head"

    HMD_ORIENTATION = "/master/hmd/orientation"

    HAND_LEFT_JOINTS = "/master/hand/left/joints"
    HAND_RIGHT_JOINTS = "/master/hand/right/joints"

    ARM_LEFT_CMD = "/slave/arm/left/joint_cmd"
    ARM_RIGHT_CMD = "/slave/arm/right/joint_cmd"
    TORSO_CMD = "/slave/torso/joint_cmd"

    HAND_LEFT_CMD = "/slave/hand/left/joint_cmd"
    HAND_RIGHT_CMD = "/slave/hand/right/joint_cmd"

    BASE_CMD_VEL = "/slave/base/cmd_vel"

    CAMERA_RGBD = "/slave/camera/rgbd"
    CAMERA_PAN_TILT_CMD = "/slave/camera/pan_tilt_cmd"
    CAMERA_COLOR_IMAGE = "/slave/camera/color/image_raw"
    CAMERA_DEPTH_IMAGE = "/slave/camera/depth/image_raw"
    CAMERA_INFO = "/slave/camera/camera_info"

    SYSTEM_STATUS = "/system/status"
    ESTOP_ACTIVE = "/system/estop_active"

    # MuJoCo simulation feedback
    MUJOCO_JOINT_STATES = "/mujoco/joint_states"

    # Calibration
    CALIBRATION_STATE = "/calibration/state"
    CALIBRATION_OFFSETS = "/calibration/offsets"

    # Playback control
    PLAYBACK_STATE = "/playback/state"


class ServiceNames:
    """Standard ROS2 service names for the teleoperation system."""

    SET_MODE = "/teleop/set_mode"
    ENABLE_MODULE = "/teleop/enable_module"
    CALIBRATE = "/teleop/calibrate"
    START_PLAYBACK = "/teleop/start_playback"
    INIT_POSE = "/teleop/init_pose"


def create_timer_rate(hz: float) -> float:
    """Convert frequency (Hz) to timer period (seconds).

    Args:
        hz: Desired frequency in Hz.

    Returns:
        Period in seconds.
    """
    if hz <= 0:
        raise ValueError(f"Frequency must be positive, got {hz}")
    return 1.0 / hz
