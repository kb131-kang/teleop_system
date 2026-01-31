"""Slave robot interfaces for teleoperation output devices.

Defines abstract interfaces for:
- ISlaveArm: Robot arm joint control (RB-Y1 arm, simulated arm)
- ISlaveHand: Robot hand/gripper control (DG-5F, generic gripper)
- IMobileBase: Mobile base velocity control (RB-Y1 AMR, simulated base)
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np


class ArmSide(Enum):
    """Arm side designation."""
    LEFT = auto()
    RIGHT = auto()
    TORSO = auto()


@dataclass
class JointState:
    """Robot joint state.

    Attributes:
        positions: (N,) joint positions in radians.
        velocities: Optional (N,) joint velocities in rad/s.
        efforts: Optional (N,) joint torques in Nm.
        names: Optional joint name list.
        timestamp: Time of measurement in seconds.
    """
    positions: np.ndarray = field(default_factory=lambda: np.zeros(0))
    velocities: np.ndarray | None = None
    efforts: np.ndarray | None = None
    names: list[str] | None = None
    timestamp: float = 0.0


class ISlaveArm(ABC):
    """Abstract interface for robot arm joint control.

    Implementations: RBY1Arm (rby1-sdk), MuJoCoArm, IsaacLabArm.
    Each instance controls one kinematic chain (left arm, right arm, or torso).
    """

    @abstractmethod
    def initialize(self) -> bool:
        """Initialize robot arm connection. Returns True on success."""
        ...

    @abstractmethod
    def send_joint_command(self, joint_positions: np.ndarray) -> None:
        """Send target joint positions (radians) to the arm.

        Args:
            joint_positions: (N,) target joint angles.
        """
        ...

    @abstractmethod
    def get_joint_state(self) -> JointState:
        """Get current joint state (positions, velocities, efforts)."""
        ...

    @abstractmethod
    def get_joint_count(self) -> int:
        """Get number of joints in this arm chain."""
        ...

    @abstractmethod
    def get_side(self) -> ArmSide:
        """Get which arm chain this controls."""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """Check if the robot arm is connected and responsive."""
        ...

    @abstractmethod
    def shutdown(self) -> None:
        """Release resources and safely stop the arm."""
        ...


class ISlaveHand(ABC):
    """Abstract interface for robot hand/gripper control.

    Implementations: DG5FHand (DELTO_M_ROS2), generic gripper.
    """

    @abstractmethod
    def initialize(self) -> bool:
        """Initialize hand connection. Returns True on success."""
        ...

    @abstractmethod
    def send_joint_command(self, joint_positions: np.ndarray) -> None:
        """Send target joint positions (radians) to the hand.

        Args:
            joint_positions: (N,) target joint angles. N=20 for DG-5F.
        """
        ...

    @abstractmethod
    def get_joint_state(self) -> JointState:
        """Get current hand joint state."""
        ...

    @abstractmethod
    def get_joint_count(self) -> int:
        """Get number of joints in this hand."""
        ...

    @abstractmethod
    def get_side(self) -> str:
        """Get hand side: 'left' or 'right'."""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """Check if the hand is connected and responsive."""
        ...

    @abstractmethod
    def shutdown(self) -> None:
        """Release resources and safely stop the hand."""
        ...


@dataclass
class VelocityCommand:
    """Mobile base velocity command.

    Attributes:
        linear_x: Forward/backward velocity (m/s).
        linear_y: Left/right velocity (m/s). Non-zero for omnidirectional bases.
        angular_z: Rotational velocity (rad/s).
        timestamp: Command timestamp in seconds.
    """
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0
    timestamp: float = 0.0


class IMobileBase(ABC):
    """Abstract interface for mobile base velocity control.

    Implementations: RBY1Base (rby1-sdk), MuJoCoBase, IsaacLabBase.
    """

    @abstractmethod
    def initialize(self) -> bool:
        """Initialize mobile base connection. Returns True on success."""
        ...

    @abstractmethod
    def send_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        """Send velocity command to the mobile base.

        Args:
            linear_x: Forward/backward velocity (m/s).
            linear_y: Left/right velocity (m/s).
            angular_z: Rotational velocity (rad/s).
        """
        ...

    @abstractmethod
    def get_odometry(self) -> tuple[np.ndarray, np.ndarray]:
        """Get current odometry (position, orientation) in world frame.

        Returns:
            Tuple of (position (3,), orientation (4,) xyzw quaternion).
        """
        ...

    @abstractmethod
    def stop(self) -> None:
        """Immediately stop the base (zero velocity)."""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """Check if the base is connected and responsive."""
        ...

    @abstractmethod
    def shutdown(self) -> None:
        """Release resources and safely stop the base."""
        ...
