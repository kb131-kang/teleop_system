"""Simulated master tracker for testing without hardware.

Generates synthetic 6DoF pose data using either:
  - Sinusoidal motion patterns (mode: 'synthetic')
  - Pre-recorded motion data replay (mode: 'mocap_file')
"""

import time

import numpy as np

from teleop_system.interfaces.master_device import IMasterTracker, Pose6D, TrackerRole
from teleop_system.utils.logger import get_logger
from teleop_system.utils.transforms import euler_to_quat

logger = get_logger("simulated_tracker")


class SimulatedTracker(IMasterTracker):
    """Simulated 6DoF tracker for testing without Vive Tracker hardware.

    Generates smooth sinusoidal motion around a configurable center pose.
    """

    def __init__(
        self,
        role: TrackerRole,
        center_position: np.ndarray | None = None,
        amplitude: float = 0.15,
        frequency: float = 0.3,
        phase_offset: float = 0.0,
    ):
        """Initialize simulated tracker.

        Args:
            role: Body part this tracker represents.
            center_position: (3,) center position in ROS2 frame (meters).
                If None, a default position based on role is used.
            amplitude: Motion amplitude in meters.
            frequency: Motion frequency in Hz.
            phase_offset: Phase offset in radians for differentiating multiple trackers.
        """
        self._role = role
        self._amplitude = amplitude
        self._frequency = frequency
        self._phase_offset = phase_offset
        self._connected = False
        self._start_time = 0.0

        # Default center positions based on role (approximate human proportions)
        default_centers = {
            TrackerRole.RIGHT_HAND: np.array([0.4, -0.3, 1.0]),
            TrackerRole.LEFT_HAND: np.array([0.4, 0.3, 1.0]),
            TrackerRole.WAIST: np.array([0.0, 0.0, 0.9]),
            TrackerRole.RIGHT_FOOT: np.array([0.0, -0.15, 0.05]),
            TrackerRole.LEFT_FOOT: np.array([0.0, 0.15, 0.05]),
            TrackerRole.HEAD: np.array([0.0, 0.0, 1.6]),
        }
        self._center = center_position if center_position is not None else default_centers[role]

    def initialize(self) -> bool:
        self._start_time = time.monotonic()
        self._connected = True
        logger.info(f"SimulatedTracker initialized: {self._role.name}")
        return True

    def get_pose(self) -> Pose6D:
        if not self._connected:
            return Pose6D(valid=False)

        t = time.monotonic() - self._start_time
        omega = 2.0 * np.pi * self._frequency
        phase = omega * t + self._phase_offset

        # Sinusoidal position offset
        dx = self._amplitude * np.sin(phase)
        dy = self._amplitude * 0.5 * np.sin(phase * 0.7 + 0.5)
        dz = self._amplitude * 0.3 * np.sin(phase * 1.3 + 1.0)
        position = self._center + np.array([dx, dy, dz])

        # Small orientation oscillation
        roll = 0.1 * np.sin(phase * 0.5)
        pitch = 0.1 * np.sin(phase * 0.3 + 0.7)
        yaw = 0.15 * np.sin(phase * 0.2 + 1.0)
        orientation = euler_to_quat(np.array([roll, pitch, yaw]))

        return Pose6D(
            position=position,
            orientation=orientation,
            timestamp=t,
            valid=True,
        )

    def is_connected(self) -> bool:
        return self._connected

    def get_role(self) -> TrackerRole:
        return self._role

    def shutdown(self) -> None:
        self._connected = False
        logger.info(f"SimulatedTracker shutdown: {self._role.name}")
