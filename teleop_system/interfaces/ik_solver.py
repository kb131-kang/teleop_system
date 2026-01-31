"""Inverse Kinematics solver interface.

Defines the abstract interface for IK solvers that convert target end-effector
poses to joint configurations.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np

from teleop_system.interfaces.master_device import Pose6D


@dataclass
class IKResult:
    """Result from an IK solve step.

    Attributes:
        joint_positions: (N,) solved joint positions in radians.
        success: Whether the IK converged.
        error_position: Position error magnitude (meters).
        error_orientation: Orientation error magnitude (radians).
    """
    joint_positions: np.ndarray = field(default_factory=lambda: np.zeros(0))
    success: bool = True
    error_position: float = 0.0
    error_orientation: float = 0.0


class IIKSolver(ABC):
    """Abstract interface for Inverse Kinematics solvers.

    Implementations: PinkIKSolver (Pink/Pinocchio based).
    Supports differential IK with weighted tasks for multi-chain robots.
    """

    @abstractmethod
    def initialize(self, urdf_path: str, **kwargs) -> bool:
        """Initialize the IK solver with robot model.

        Args:
            urdf_path: Path to the URDF file.
            **kwargs: Solver-specific parameters (e.g., end-effector frame names).

        Returns:
            True on success.
        """
        ...

    @abstractmethod
    def solve(
        self,
        target_pose: Pose6D,
        current_joints: np.ndarray,
        dt: float = 0.01,
    ) -> IKResult:
        """Solve one IK step for a single end-effector target.

        Args:
            target_pose: Desired end-effector pose in robot base frame.
            current_joints: (N,) current joint positions in radians.
            dt: Time step for differential IK integration.

        Returns:
            IKResult with solved joint positions.
        """
        ...

    @abstractmethod
    def solve_multi(
        self,
        target_poses: dict[str, Pose6D],
        current_joints: np.ndarray,
        dt: float = 0.01,
    ) -> IKResult:
        """Solve one IK step for multiple end-effector targets simultaneously.

        Used for controlling left arm, right arm, and torso concurrently.

        Args:
            target_poses: Dict mapping chain name to target pose.
                e.g. {"left_arm": pose, "right_arm": pose, "torso": pose}
            current_joints: (N,) current joint positions for the full robot.
            dt: Time step for differential IK integration.

        Returns:
            IKResult with joint positions for all chains combined.
        """
        ...

    @abstractmethod
    def get_joint_names(self) -> list[str]:
        """Get the ordered list of joint names this solver controls."""
        ...

    @abstractmethod
    def get_joint_count(self) -> int:
        """Get total number of joints across all chains."""
        ...

    @abstractmethod
    def set_posture_target(self, posture: np.ndarray) -> None:
        """Set the null-space posture target for redundancy resolution.

        Args:
            posture: (N,) preferred joint configuration.
        """
        ...
