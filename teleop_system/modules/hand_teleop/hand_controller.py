"""Hand teleoperation controller logic.

Pure control logic (no ROS2) — reads glove input, retargets joints,
and sends commands to the robot hand.
"""

import numpy as np

from teleop_system.interfaces.master_device import IHandInput
from teleop_system.interfaces.slave_robot import ISlaveHand
from teleop_system.modules.hand_teleop.retargeting import HandRetargeting
from teleop_system.utils.logger import get_logger

logger = get_logger("hand_controller")


class HandController:
    """Controls a robot hand via retargeted glove input.

    Reads from IHandInput, maps through HandRetargeting, and sends
    to ISlaveHand. Supports velocity limiting.
    """

    def __init__(
        self,
        glove: IHandInput,
        hand: ISlaveHand,
        retargeting: HandRetargeting,
        max_joint_velocity: float = 5.0,
        dt: float = 0.01,
    ):
        """Initialize hand controller.

        Args:
            glove: Input device (Manus Glove or SimulatedHand).
            hand: Robot hand (DG-5F or simulated).
            retargeting: Joint retargeting mapper.
            max_joint_velocity: Max joint velocity (rad/s).
            dt: Control loop timestep (seconds).
        """
        self._glove = glove
        self._hand = hand
        self._retargeting = retargeting
        self._max_joint_velocity = max_joint_velocity
        self._dt = dt
        self._enabled = False
        self._prev_command: np.ndarray | None = None

    def enable(self) -> None:
        self._enabled = True
        logger.info(f"HandController enabled (side={self._glove.get_side()})")

    def disable(self) -> None:
        self._enabled = False
        logger.info(f"HandController disabled (side={self._glove.get_side()})")

    def update(self) -> np.ndarray | None:
        """Run one control cycle: read glove -> retarget -> send to hand.

        Returns:
            The joint command sent, or None if disabled/invalid.
        """
        if not self._enabled:
            return None

        if not self._glove.is_connected() or not self._hand.is_connected():
            return None

        # Read glove state
        glove_state = self._glove.get_joint_state()
        if not glove_state.valid:
            return None

        # Retarget
        target = self._retargeting.retarget(glove_state)

        # Apply velocity limiting
        if self._prev_command is not None:
            delta = target - self._prev_command
            max_delta = self._max_joint_velocity * self._dt
            delta = np.clip(delta, -max_delta, max_delta)
            target = self._prev_command + delta

        self._prev_command = target.copy()

        # Send to hand
        self._hand.send_joint_command(target)
        return target

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    @property
    def side(self) -> str:
        return self._glove.get_side()
