"""Rainbow Robotics RB-Y1 arm and base drivers.

Implements ISlaveArm and IMobileBase using the rby1-sdk Python API.
Each RBY1Arm instance controls one kinematic chain (left arm, right arm, or torso).
RBY1Base controls the omnidirectional mobile base.
"""

from __future__ import annotations

import numpy as np

from teleop_system.interfaces.slave_robot import (
    ArmSide,
    IMobileBase,
    ISlaveArm,
    JointState,
)
from teleop_system.utils.logger import get_logger

logger = get_logger("rby1_arm")

try:
    import rby1_sdk
    _RBY1_AVAILABLE = True
except ImportError:
    _RBY1_AVAILABLE = False
    logger.warning("rby1-sdk not available — RBY1Arm/RBY1Base cannot be used")


class RBY1Arm(ISlaveArm):
    """RB-Y1 arm control via rby1-sdk.

    Controls one kinematic chain (left arm, right arm, or torso)
    using position control mode.
    """

    def __init__(
        self,
        side: ArmSide,
        robot_ip: str = "192.168.0.100",
        joint_count: int = 7,
        robot: object | None = None,
    ):
        """Initialize RBY1 arm driver.

        Args:
            side: Which arm chain.
            robot_ip: RB-Y1 IP address.
            joint_count: Number of joints in this chain.
            robot: Optional shared robot SDK object.
        """
        self._side = side
        self._robot_ip = robot_ip
        self._joint_count = joint_count
        self._robot = robot
        self._connected = False
        self._last_positions = np.zeros(joint_count)

    def initialize(self) -> bool:
        if not _RBY1_AVAILABLE:
            logger.error("rby1-sdk not installed")
            return False

        try:
            if self._robot is None:
                self._robot = rby1_sdk.create_robot_a(self._robot_ip)
                self._robot.connect()

            self._connected = True
            logger.info(f"RBY1Arm initialized: {self._side.name}, {self._joint_count} joints")
            return True
        except Exception as e:
            logger.error(f"RBY1Arm init failed: {e}")
            return False

    def send_joint_command(self, joint_positions: np.ndarray) -> None:
        if not self._connected or self._robot is None:
            return

        try:
            n = min(len(joint_positions), self._joint_count)
            cmd = joint_positions[:n].tolist()

            if self._side == ArmSide.RIGHT:
                self._robot.set_joint_position_target("right_arm", cmd)
            elif self._side == ArmSide.LEFT:
                self._robot.set_joint_position_target("left_arm", cmd)
            elif self._side == ArmSide.TORSO:
                self._robot.set_joint_position_target("torso", cmd)

            self._last_positions[:n] = joint_positions[:n]
        except Exception as e:
            logger.error(f"RBY1Arm send_joint_command failed: {e}")

    def get_joint_state(self) -> JointState:
        if not self._connected or self._robot is None:
            return JointState()

        try:
            state = self._robot.get_state()
            if self._side == ArmSide.RIGHT:
                positions = np.array(state.right_arm_positions[:self._joint_count])
            elif self._side == ArmSide.LEFT:
                positions = np.array(state.left_arm_positions[:self._joint_count])
            elif self._side == ArmSide.TORSO:
                positions = np.array(state.torso_positions[:self._joint_count])
            else:
                positions = self._last_positions.copy()

            return JointState(positions=positions)
        except Exception:
            return JointState(positions=self._last_positions.copy())

    def get_joint_count(self) -> int:
        return self._joint_count

    def get_side(self) -> ArmSide:
        return self._side

    def is_connected(self) -> bool:
        return self._connected

    def shutdown(self) -> None:
        self._connected = False
        logger.info(f"RBY1Arm shutdown: {self._side.name}")


class RBY1Base(IMobileBase):
    """RB-Y1 mobile base control via rby1-sdk.

    Controls the omnidirectional AMR base with velocity commands.
    """

    def __init__(
        self,
        robot_ip: str = "192.168.0.100",
        max_linear: float = 1.0,
        max_angular: float = 1.5,
        robot: object | None = None,
    ):
        self._robot_ip = robot_ip
        self._max_linear = max_linear
        self._max_angular = max_angular
        self._robot = robot
        self._connected = False

    def initialize(self) -> bool:
        if not _RBY1_AVAILABLE:
            logger.error("rby1-sdk not installed")
            return False

        try:
            if self._robot is None:
                self._robot = rby1_sdk.create_robot_a(self._robot_ip)
                self._robot.connect()

            self._connected = True
            logger.info("RBY1Base initialized")
            return True
        except Exception as e:
            logger.error(f"RBY1Base init failed: {e}")
            return False

    def send_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        if not self._connected or self._robot is None:
            return

        try:
            lx = np.clip(linear_x, -self._max_linear, self._max_linear)
            ly = np.clip(linear_y, -self._max_linear, self._max_linear)
            az = np.clip(angular_z, -self._max_angular, self._max_angular)
            self._robot.set_base_velocity(lx, ly, az)
        except Exception as e:
            logger.error(f"RBY1Base send_velocity failed: {e}")

    def get_odometry(self) -> tuple[np.ndarray, np.ndarray]:
        if not self._connected or self._robot is None:
            return np.zeros(3), np.array([0, 0, 0, 1.0])

        try:
            state = self._robot.get_state()
            return np.array(state.base_position), np.array(state.base_orientation)
        except Exception:
            return np.zeros(3), np.array([0, 0, 0, 1.0])

    def stop(self) -> None:
        self.send_velocity(0.0, 0.0, 0.0)

    def is_connected(self) -> bool:
        return self._connected

    def shutdown(self) -> None:
        self.stop()
        self._connected = False
        logger.info("RBY1Base shutdown")
