"""Hand joint retargeting from Manus Glove to DG-5F hand.

Maps 20-DOF Manus Glove ergonomics joint angles to 20-DOF TESOLLO DG-5F
robot hand joints using configurable per-finger scale factors and offsets.
"""

import numpy as np

from teleop_system.interfaces.master_device import HandJointState
from teleop_system.utils.logger import get_logger

logger = get_logger("hand_retargeting")

# Finger names in canonical order (matching config)
FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]
JOINTS_PER_FINGER = 4
TOTAL_JOINTS = len(FINGER_NAMES) * JOINTS_PER_FINGER  # 20


class HandRetargeting:
    """Retargets Manus Glove joint angles to DG-5F hand joint commands.

    Each finger has 4 joints (MCP_spread, MCP_flex, PIP, DIP).
    Retargeting applies per-joint: output = input * scale + offset,
    then clips to [0, max_angle].

    No ROS2 dependency — pure mapping logic.
    """

    def __init__(
        self,
        scale_factors: dict[str, list[float]] | None = None,
        offsets: dict[str, list[float]] | None = None,
        max_angle: float = 1.57,
        smoothing_alpha: float = 0.0,
    ):
        """Initialize retargeting.

        Args:
            scale_factors: Dict of finger_name -> [4 scale values].
                Defaults to 1.0 for all joints.
            offsets: Dict of finger_name -> [4 offset values].
                Defaults to 0.0 for all joints.
            max_angle: Maximum output angle (radians).
            smoothing_alpha: Exponential smoothing factor (0=none, 1=max).
        """
        self._max_angle = max_angle
        self._smoothing_alpha = smoothing_alpha

        # Build flat arrays (20,) for vectorized computation
        self._scales = np.ones(TOTAL_JOINTS)
        self._offsets = np.zeros(TOTAL_JOINTS)

        if scale_factors:
            for i, finger in enumerate(FINGER_NAMES):
                if finger in scale_factors:
                    vals = scale_factors[finger]
                    for j in range(min(len(vals), JOINTS_PER_FINGER)):
                        self._scales[i * JOINTS_PER_FINGER + j] = vals[j]

        if offsets:
            for i, finger in enumerate(FINGER_NAMES):
                if finger in offsets:
                    vals = offsets[finger]
                    for j in range(min(len(vals), JOINTS_PER_FINGER)):
                        self._offsets[i * JOINTS_PER_FINGER + j] = vals[j]

        # Smoothed output state
        self._prev_output: np.ndarray | None = None

    def retarget(self, glove_state: HandJointState) -> np.ndarray:
        """Map glove joint angles to robot hand joint commands.

        Args:
            glove_state: Input hand joint state from Manus Glove.

        Returns:
            (20,) array of DG-5F target joint angles in radians.
        """
        if not glove_state.valid:
            return self._prev_output if self._prev_output is not None else np.zeros(TOTAL_JOINTS)

        raw = glove_state.joint_angles[:TOTAL_JOINTS]
        if len(raw) < TOTAL_JOINTS:
            padded = np.zeros(TOTAL_JOINTS)
            padded[:len(raw)] = raw
            raw = padded

        # Apply scale and offset
        output = raw * self._scales + self._offsets

        # Clamp to valid range
        output = np.clip(output, 0.0, self._max_angle)

        # Exponential smoothing
        if self._smoothing_alpha > 0.0 and self._prev_output is not None:
            alpha = self._smoothing_alpha
            output = alpha * self._prev_output + (1.0 - alpha) * output

        self._prev_output = output.copy()
        return output

    def set_scale_factors(self, scale_factors: dict[str, list[float]]) -> None:
        """Update scale factors at runtime."""
        for i, finger in enumerate(FINGER_NAMES):
            if finger in scale_factors:
                vals = scale_factors[finger]
                for j in range(min(len(vals), JOINTS_PER_FINGER)):
                    self._scales[i * JOINTS_PER_FINGER + j] = vals[j]

    def set_offsets(self, offsets: dict[str, list[float]]) -> None:
        """Update offsets at runtime."""
        for i, finger in enumerate(FINGER_NAMES):
            if finger in offsets:
                vals = offsets[finger]
                for j in range(min(len(vals), JOINTS_PER_FINGER)):
                    self._offsets[i * JOINTS_PER_FINGER + j] = vals[j]

    def reset(self) -> None:
        """Reset smoothing state."""
        self._prev_output = None
