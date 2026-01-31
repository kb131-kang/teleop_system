"""Gait detection from foot tracker poses.

Converts foot tracker displacement into robot base velocity commands.
Uses the midpoint between two feet as the reference origin, with
deadzone filtering and configurable scaling.
"""

import numpy as np

from teleop_system.interfaces.master_device import IMasterTracker, Pose6D
from teleop_system.interfaces.slave_robot import VelocityCommand
from teleop_system.utils.logger import get_logger
from teleop_system.utils.transforms import quat_to_euler

logger = get_logger("gait_detector")


class GaitDetector:
    """Detects walking intent from foot tracker poses and outputs velocity commands.

    Uses the displacement of foot trackers relative to their calibrated
    reference position. Forward/lateral foot movement maps to linear velocity,
    and yaw rotation maps to angular velocity.

    No ROS2 dependency — pure control logic.
    """

    def __init__(
        self,
        left_foot_tracker: IMasterTracker,
        right_foot_tracker: IMasterTracker,
        deadzone: float = 0.02,
        linear_scale: float = 1.0,
        angular_scale: float = 1.0,
        max_linear_velocity: float = 0.5,
        max_angular_velocity: float = 1.0,
        smoothing_window: int = 5,
    ):
        """Initialize gait detector.

        Args:
            left_foot_tracker: Tracker for the left foot.
            right_foot_tracker: Tracker for the right foot.
            deadzone: Minimum displacement to register movement (meters).
            linear_scale: Scale from foot displacement to linear velocity.
            angular_scale: Scale from foot yaw change to angular velocity.
            max_linear_velocity: Maximum linear velocity output (m/s).
            max_angular_velocity: Maximum angular velocity output (rad/s).
            smoothing_window: Number of samples for moving average filter.
        """
        self._left_tracker = left_foot_tracker
        self._right_tracker = right_foot_tracker
        self._deadzone = deadzone
        self._linear_scale = linear_scale
        self._angular_scale = angular_scale
        self._max_linear_vel = max_linear_velocity
        self._max_angular_vel = max_angular_velocity
        self._smoothing_window = smoothing_window

        # Reference (calibrated) foot positions
        self._ref_left_pos: np.ndarray | None = None
        self._ref_right_pos: np.ndarray | None = None
        self._ref_midpoint: np.ndarray | None = None
        self._ref_yaw: float = 0.0

        # Smoothing buffers
        self._linear_x_buf: list[float] = []
        self._linear_y_buf: list[float] = []
        self._angular_z_buf: list[float] = []

        self._calibrated = False

    def calibrate(self) -> bool:
        """Capture current foot positions as the reference (zero-velocity) state.

        Returns:
            True if both trackers provided valid poses.
        """
        if not self._left_tracker.is_connected() or not self._right_tracker.is_connected():
            logger.warning("Cannot calibrate: one or both foot trackers disconnected")
            return False

        left_pose = self._left_tracker.get_pose()
        right_pose = self._right_tracker.get_pose()

        if not left_pose.valid or not right_pose.valid:
            logger.warning("Cannot calibrate: invalid foot poses")
            return False

        self._ref_left_pos = left_pose.position.copy()
        self._ref_right_pos = right_pose.position.copy()
        self._ref_midpoint = (self._ref_left_pos + self._ref_right_pos) / 2.0

        # Reference yaw from midpoint of foot orientations
        left_yaw = quat_to_euler(left_pose.orientation)[2]
        right_yaw = quat_to_euler(right_pose.orientation)[2]
        self._ref_yaw = (left_yaw + right_yaw) / 2.0

        self._calibrated = True
        self._linear_x_buf.clear()
        self._linear_y_buf.clear()
        self._angular_z_buf.clear()

        logger.info(
            f"Gait calibrated: midpoint={self._ref_midpoint}, yaw={self._ref_yaw:.3f}"
        )
        return True

    def update(self) -> VelocityCommand:
        """Compute velocity command from current foot positions.

        Returns:
            VelocityCommand with linear_x, linear_y, angular_z.
        """
        if not self._calibrated:
            return VelocityCommand()

        # Read current foot poses
        left_pose = self._left_tracker.get_pose() if self._left_tracker.is_connected() else Pose6D(valid=False)
        right_pose = self._right_tracker.get_pose() if self._right_tracker.is_connected() else Pose6D(valid=False)

        if not left_pose.valid or not right_pose.valid:
            return VelocityCommand()

        # Current midpoint
        midpoint = (left_pose.position + right_pose.position) / 2.0

        # Displacement from reference
        delta = midpoint - self._ref_midpoint

        # Forward (x) and lateral (y) displacement
        raw_x = delta[0]  # Forward
        raw_y = delta[1]  # Lateral

        # Yaw from foot orientations
        left_yaw = quat_to_euler(left_pose.orientation)[2]
        right_yaw = quat_to_euler(right_pose.orientation)[2]
        current_yaw = (left_yaw + right_yaw) / 2.0
        raw_yaw = current_yaw - self._ref_yaw

        # Apply deadzone
        linear_x = self._apply_deadzone(raw_x) * self._linear_scale
        linear_y = self._apply_deadzone(raw_y) * self._linear_scale
        angular_z = self._apply_deadzone(raw_yaw) * self._angular_scale

        # Clamp to limits
        linear_x = np.clip(linear_x, -self._max_linear_vel, self._max_linear_vel)
        linear_y = np.clip(linear_y, -self._max_linear_vel, self._max_linear_vel)
        angular_z = np.clip(angular_z, -self._max_angular_vel, self._max_angular_vel)

        # Smooth
        linear_x = self._smooth(self._linear_x_buf, linear_x)
        linear_y = self._smooth(self._linear_y_buf, linear_y)
        angular_z = self._smooth(self._angular_z_buf, angular_z)

        return VelocityCommand(
            linear_x=float(linear_x),
            linear_y=float(linear_y),
            angular_z=float(angular_z),
        )

    def _apply_deadzone(self, value: float) -> float:
        """Apply deadzone: values below threshold are zeroed."""
        if abs(value) < self._deadzone:
            return 0.0
        # Remove deadzone offset so motion starts from zero
        return value - np.sign(value) * self._deadzone

    def _smooth(self, buf: list[float], value: float) -> float:
        """Moving average smoothing."""
        buf.append(value)
        if len(buf) > self._smoothing_window:
            buf.pop(0)
        return sum(buf) / len(buf)

    @property
    def is_calibrated(self) -> bool:
        return self._calibrated

    def reset(self) -> None:
        """Reset calibration and buffers."""
        self._calibrated = False
        self._ref_left_pos = None
        self._ref_right_pos = None
        self._ref_midpoint = None
        self._ref_yaw = 0.0
        self._linear_x_buf.clear()
        self._linear_y_buf.clear()
        self._angular_z_buf.clear()
