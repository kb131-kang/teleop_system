"""Master device interfaces for teleoperation input devices.

Defines abstract interfaces for:
- IMasterTracker: 6DoF pose tracking (Vive Tracker, simulated tracker, etc.)
- IHandInput: Hand joint angle input (Manus Glove, simulated hand, etc.)
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np


class TrackerRole(Enum):
    """Tracker body assignment role."""
    RIGHT_HAND = auto()
    LEFT_HAND = auto()
    WAIST = auto()
    RIGHT_FOOT = auto()
    LEFT_FOOT = auto()
    HEAD = auto()


@dataclass
class Pose6D:
    """6DoF pose representation.

    Attributes:
        position: (3,) xyz position in meters.
        orientation: (4,) quaternion in ROS2 convention (x, y, z, w).
        timestamp: Time of measurement in seconds.
        valid: Whether this pose reading is valid.
    """
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    timestamp: float = 0.0
    valid: bool = True


class IMasterTracker(ABC):
    """Abstract interface for a 6DoF pose tracking device.

    Implementations: ViveTracker (PyOpenVR), SimulatedTracker (dummy/mocap).
    Each instance tracks a single body part (hand, waist, foot, etc.).
    """

    @abstractmethod
    def initialize(self) -> bool:
        """Initialize device connection. Returns True on success."""
        ...

    @abstractmethod
    def get_pose(self) -> Pose6D:
        """Get the current 6DoF pose in ROS2 coordinate frame (Z-up, xyzw quat)."""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """Check if the device is currently connected and providing data."""
        ...

    @abstractmethod
    def get_role(self) -> TrackerRole:
        """Get the body-part role assigned to this tracker."""
        ...

    @abstractmethod
    def shutdown(self) -> None:
        """Release device resources."""
        ...


@dataclass
class HandJointState:
    """Hand joint state data.

    Attributes:
        joint_angles: (N,) joint angles in radians. N depends on the hand model.
            For Manus Glove ergonomics: 20 values (4 joints x 5 fingers).
        finger_tip_positions: Optional (5, 3) fingertip xyz positions.
        timestamp: Time of measurement in seconds.
        side: 'left' or 'right'.
        valid: Whether this reading is valid.
    """
    joint_angles: np.ndarray = field(default_factory=lambda: np.zeros(20))
    finger_tip_positions: np.ndarray | None = None
    timestamp: float = 0.0
    side: str = "right"
    valid: bool = True


class IHandInput(ABC):
    """Abstract interface for hand joint angle input devices.

    Implementations: ManusGlove (MANUS SDK), SimulatedHand (dummy data).
    """

    @abstractmethod
    def initialize(self) -> bool:
        """Initialize device connection. Returns True on success."""
        ...

    @abstractmethod
    def get_joint_state(self) -> HandJointState:
        """Get the current hand joint angles and optional fingertip positions."""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """Check if the device is currently connected and providing data."""
        ...

    @abstractmethod
    def get_side(self) -> str:
        """Get the hand side: 'left' or 'right'."""
        ...

    @abstractmethod
    def shutdown(self) -> None:
        """Release device resources."""
        ...
