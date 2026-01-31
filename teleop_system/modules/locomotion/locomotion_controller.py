"""Locomotion controller logic.

Pure control logic (no ROS2) — uses GaitDetector to convert foot tracker
poses into velocity commands for the mobile base.
"""

import numpy as np

from teleop_system.interfaces.master_device import IMasterTracker
from teleop_system.interfaces.slave_robot import IMobileBase, VelocityCommand
from teleop_system.modules.locomotion.gait_detector import GaitDetector
from teleop_system.utils.logger import get_logger

logger = get_logger("locomotion_controller")


class LocomotionController:
    """Controls robot base movement from foot tracker gait detection.

    Reads foot trackers via GaitDetector, converts to velocity commands,
    and sends them to the mobile base adapter.
    """

    def __init__(
        self,
        left_foot_tracker: IMasterTracker,
        right_foot_tracker: IMasterTracker,
        base: IMobileBase,
        deadzone: float = 0.02,
        linear_scale: float = 1.0,
        angular_scale: float = 1.0,
        max_linear_velocity: float = 0.5,
        max_angular_velocity: float = 1.0,
        smoothing_window: int = 5,
    ):
        self._base = base
        self._gait_detector = GaitDetector(
            left_foot_tracker=left_foot_tracker,
            right_foot_tracker=right_foot_tracker,
            deadzone=deadzone,
            linear_scale=linear_scale,
            angular_scale=angular_scale,
            max_linear_velocity=max_linear_velocity,
            max_angular_velocity=max_angular_velocity,
            smoothing_window=smoothing_window,
        )
        self._enabled = False

    def enable(self) -> bool:
        """Enable locomotion and calibrate foot reference positions.

        Returns:
            True if calibration succeeded.
        """
        ok = self._gait_detector.calibrate()
        if ok:
            self._enabled = True
            logger.info("LocomotionController enabled")
        else:
            logger.warning("LocomotionController enable failed: calibration error")
        return ok

    def disable(self) -> None:
        """Disable locomotion and stop the base."""
        self._enabled = False
        self._base.stop()
        logger.info("LocomotionController disabled")

    def update(self) -> VelocityCommand:
        """Run one control cycle: read feet -> detect gait -> send velocity.

        Returns:
            The VelocityCommand sent to the base (or zero if disabled).
        """
        if not self._enabled:
            return VelocityCommand()

        cmd = self._gait_detector.update()
        self._base.send_velocity(cmd.linear_x, cmd.linear_y, cmd.angular_z)
        return cmd

    def recalibrate(self) -> bool:
        """Re-calibrate foot reference positions (e.g., after user repositions)."""
        return self._gait_detector.calibrate()

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    @property
    def gait_detector(self) -> GaitDetector:
        return self._gait_detector
