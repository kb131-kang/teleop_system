"""Simple proportional mapper fallback for IK.

When pin/pink are unavailable, maps tracker Cartesian position offsets
directly to joint angle targets using per-chain gain vectors.
Replicates the proportional mapping from demo_teleop_sim.py.
"""

import numpy as np

from teleop_system.interfaces.ik_solver import IIKSolver, IKResult
from teleop_system.interfaces.master_device import Pose6D
from teleop_system.utils.logger import get_logger

logger = get_logger("proportional_mapper")


class SimpleProportionalMapper(IIKSolver):
    """Fallback IK solver using proportional position-to-joint mapping.

    Maps tracker Cartesian position offsets to joint angle targets
    using per-chain gain vectors. Suitable for basic testing without
    pin/pink dependencies.
    """

    def __init__(self):
        self._nq: int = 20  # 6 torso + 7 right arm + 7 left arm
        self._joint_names: list[str] = []
        self._chain_joint_indices: dict[str, list[int]] = {}
        self._initialized = False

    def initialize(self, urdf_path: str, **kwargs) -> bool:
        """Initialize the proportional mapper.

        Args:
            urdf_path: Ignored (no model loading needed).
            **kwargs:
                nq: Total number of joints (default: 20).
                chain_joint_indices: Dict mapping chain_name -> list of indices.
                    Default: torso=[0..5], right_arm=[6..12], left_arm=[13..19].
        """
        self._nq = kwargs.get("nq", 20)
        self._chain_joint_indices = kwargs.get("chain_joint_indices", {
            "torso": list(range(0, 6)),
            "right_arm": list(range(6, 13)),
            "left_arm": list(range(13, 20)),
        })

        self._joint_names = [f"joint_{i}" for i in range(self._nq)]
        self._initialized = True
        logger.info(
            f"SimpleProportionalMapper initialized: nq={self._nq}, "
            f"chains={list(self._chain_joint_indices.keys())}"
        )
        return True

    def solve(
        self,
        target_pose: Pose6D,
        current_joints: np.ndarray,
        dt: float = 0.01,
    ) -> IKResult:
        if not self._initialized:
            return IKResult(success=False)
        chain_name = next(iter(self._chain_joint_indices), None)
        if chain_name is None:
            return IKResult(success=False)
        return self.solve_multi({chain_name: target_pose}, current_joints, dt)

    def solve_multi(
        self,
        target_poses: dict[str, Pose6D],
        current_joints: np.ndarray,
        dt: float = 0.01,
    ) -> IKResult:
        """Map each chain's target position to joint targets via proportional gains.

        Uses the same proportional mapping as demo_teleop_sim.py.
        """
        if not self._initialized or not target_poses:
            return IKResult(success=False)

        q = current_joints.copy() if len(current_joints) == self._nq else np.zeros(self._nq)

        for chain_name, pose in target_poses.items():
            if chain_name not in self._chain_joint_indices:
                continue
            indices = self._chain_joint_indices[chain_name]
            pos = pose.position

            if chain_name == "left_arm" and len(indices) >= 7:
                targets = np.zeros(7)
                targets[0] = np.clip(pos[1] * 2.0, -2.3, 2.3)
                targets[1] = np.clip((pos[2] - 1.0) * 2.0, -0.05, 3.1)
                targets[2] = np.clip(pos[0] * 1.5, -2.0, 2.0)
                targets[3] = np.clip((pos[2] - 1.0) * -1.5, -2.6, 0.01)
                for i, idx in enumerate(indices[:7]):
                    q[idx] = targets[i]

            elif chain_name == "right_arm" and len(indices) >= 7:
                targets = np.zeros(7)
                targets[0] = np.clip(pos[1] * -2.0, -2.3, 2.3)
                targets[1] = np.clip((pos[2] - 1.0) * -2.0, -3.1, 0.05)
                targets[2] = np.clip(pos[0] * 1.5, -2.0, 2.0)
                targets[3] = np.clip((pos[2] - 1.0) * -1.5, -2.6, 0.01)
                for i, idx in enumerate(indices[:7]):
                    q[idx] = targets[i]

            elif chain_name == "torso" and len(indices) >= 6:
                targets = np.zeros(6)
                targets[0] = np.clip(pos[1] * 0.3, -1.0, 1.0)
                targets[1] = np.clip((pos[2] - 0.9) * 0.5, -0.5, 0.5)
                for i, idx in enumerate(indices[:6]):
                    q[idx] = targets[i]

        return IKResult(
            joint_positions=q,
            success=True,
            error_position=0.0,
            error_orientation=0.0,
        )

    def get_joint_names(self) -> list[str]:
        return self._joint_names.copy()

    def get_joint_count(self) -> int:
        return self._nq

    def set_posture_target(self, posture: np.ndarray) -> None:
        pass  # No null-space handling in proportional mapper


def create_ik_solver(prefer_pink: bool = True) -> IIKSolver:
    """Create the best available IK solver.

    Tries PinkIKSolver first if prefer_pink=True, falls back to
    SimpleProportionalMapper when pin/pink are unavailable.
    """
    if prefer_pink:
        try:
            from teleop_system.solvers.pink_ik_solver import PinkIKSolver
            # Verify that pinocchio and pink are actually importable
            import pinocchio  # noqa: F401
            import pink  # noqa: F401
            logger.info("Using PinkIKSolver (pin/pink available)")
            return PinkIKSolver()
        except ImportError:
            logger.warning("pin/pink not available, falling back to SimpleProportionalMapper")
    return SimpleProportionalMapper()
