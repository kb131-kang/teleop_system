"""Simulated hand input for testing without Manus Glove hardware.

Generates synthetic finger joint angle data.
"""

import time

import numpy as np

from teleop_system.interfaces.master_device import IHandInput, HandJointState
from teleop_system.utils.logger import get_logger

logger = get_logger("simulated_hand")


class SimulatedHand(IHandInput):
    """Simulated hand input generating synthetic finger joint data.

    Produces smooth open/close cycles for all 20 joints (4 joints x 5 fingers).
    """

    def __init__(
        self,
        side: str = "right",
        frequency: float = 0.2,
        max_angle: float = 1.4,
    ):
        """Initialize simulated hand.

        Args:
            side: 'left' or 'right'.
            frequency: Open/close cycle frequency in Hz.
            max_angle: Maximum joint angle in radians (~80 degrees).
        """
        self._side = side
        self._frequency = frequency
        self._max_angle = max_angle
        self._connected = False
        self._start_time = 0.0
        self._n_joints = 20  # 4 joints x 5 fingers

    def initialize(self) -> bool:
        self._start_time = time.monotonic()
        self._connected = True
        logger.info(f"SimulatedHand initialized: {self._side}")
        return True

    def get_joint_state(self) -> HandJointState:
        if not self._connected:
            return HandJointState(valid=False)

        t = time.monotonic() - self._start_time
        omega = 2.0 * np.pi * self._frequency

        # Each finger closes with a slight phase delay
        angles = np.zeros(self._n_joints)
        for finger_idx in range(5):
            phase_delay = finger_idx * 0.3
            base_angle = self._max_angle * 0.5 * (1 + np.sin(omega * t - phase_delay))
            for joint_idx in range(4):
                idx = finger_idx * 4 + joint_idx
                # Distal joints move less than proximal
                scale = 1.0 - 0.15 * joint_idx
                angles[idx] = base_angle * scale

        return HandJointState(
            joint_angles=angles,
            timestamp=t,
            side=self._side,
            valid=True,
        )

    def is_connected(self) -> bool:
        return self._connected

    def get_side(self) -> str:
        return self._side

    def shutdown(self) -> None:
        self._connected = False
        logger.info(f"SimulatedHand shutdown: {self._side}")
