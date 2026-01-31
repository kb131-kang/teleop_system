"""Pink/Pinocchio-based Inverse Kinematics solver.

Implements IIKSolver using the Pink library for differential IK with
weighted tasks. Supports multi-chain solving for left arm, right arm,
and torso simultaneously via multiple FrameTasks with per-chain weights.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin
import pink

from teleop_system.interfaces.ik_solver import IIKSolver, IKResult
from teleop_system.interfaces.master_device import Pose6D
from teleop_system.utils.logger import get_logger
from teleop_system.utils.transforms import pose_to_homogeneous

logger = get_logger("pink_ik_solver")

def _detect_qp_solver() -> str:
    """Auto-detect the best available QP solver."""
    try:
        import qpsolvers
        available = qpsolvers.available_solvers
        for preferred in ("proxqp", "osqp", "daqp", "quadprog", "ecos"):
            if preferred in available:
                return preferred
        if available:
            return available[0]
    except ImportError:
        pass
    return "proxqp"


_DEFAULT_QP_SOLVER = _detect_qp_solver()


@dataclass
class ChainConfig:
    """Per-chain configuration for the IK solver.

    Attributes:
        ee_frame: End-effector frame name in URDF.
        position_cost: Weight for position tracking.
        orientation_cost: Weight for orientation tracking.
        lm_damping: Levenberg-Marquardt damping for this task.
    """
    ee_frame: str
    position_cost: float = 1.0
    orientation_cost: float = 0.5
    lm_damping: float = 1e-3


@dataclass
class ChainError:
    """Per-chain IK error.

    Attributes:
        position_error: Position error in meters.
        orientation_error: Orientation error in radians.
    """
    position_error: float = 0.0
    orientation_error: float = 0.0


class PinkIKSolver(IIKSolver):
    """IK solver using Pink (differential IK on Pinocchio).

    Supports single and multi-chain end-effector tracking with
    per-chain weight configuration and null-space posture regularization.

    For multi-chain robots (e.g., humanoid upper body), the QP optimizer
    finds joint velocities that satisfy all end-effector targets
    simultaneously. Shared joints (e.g., torso) are naturally resolved
    via task weight priorities.
    """

    def __init__(self):
        self._model: pin.Model | None = None
        self._data: pin.Data | None = None
        self._configuration: pink.Configuration | None = None
        self._frame_tasks: dict[str, pink.FrameTask] = {}
        self._posture_task: pink.PostureTask | None = None
        self._joint_names: list[str] = []
        self._initialized = False

        # Per-chain configs
        self._chain_configs: dict[str, ChainConfig] = {}
        # Per-chain error from last solve
        self._last_chain_errors: dict[str, ChainError] = {}
        # Chain name -> EE frame name mapping
        self._chain_frames: dict[str, str] = {}

        # Default config parameters (used when per-chain config not specified)
        self._default_position_cost = 1.0
        self._default_orientation_cost = 0.5
        self._posture_cost = 0.1
        self._default_lm_damping = 1e-3
        self._qp_solver = _DEFAULT_QP_SOLVER

    def initialize(self, urdf_path: str, **kwargs) -> bool:
        """Initialize the IK solver with a URDF robot model.

        Args:
            urdf_path: Path to the URDF file.
            **kwargs:
                ee_frames: Dict mapping chain name -> EE frame name.
                    e.g. {"right_arm": "right_hand_link", ...}
                chain_configs: Dict mapping chain name -> ChainConfig.
                    Overrides default weights per chain. If a chain appears
                    in ee_frames but not chain_configs, default weights apply.
                position_cost: Default position weight (default: 1.0).
                orientation_cost: Default orientation weight (default: 0.5).
                posture_cost: Null-space posture weight (default: 0.1).
                lm_damping: Default LM damping (default: 1e-3).
                qp_solver: QP solver name (default: 'proxqp').

        Returns:
            True on success.
        """
        urdf_path = str(Path(urdf_path).resolve())

        self._default_position_cost = kwargs.get("position_cost", 1.0)
        self._default_orientation_cost = kwargs.get("orientation_cost", 0.5)
        self._posture_cost = kwargs.get("posture_cost", 0.1)
        self._default_lm_damping = kwargs.get("lm_damping", 1e-3)
        self._qp_solver = kwargs.get("qp_solver", _DEFAULT_QP_SOLVER)
        ee_frames: dict[str, str] = kwargs.get("ee_frames", {})
        chain_configs: dict[str, ChainConfig] = kwargs.get("chain_configs", {})

        try:
            # Load URDF with Pinocchio
            self._model = pin.buildModelFromUrdf(urdf_path)
            self._data = self._model.createData()

            # Collect joint names (skip universe joint)
            self._joint_names = [
                self._model.names[i]
                for i in range(1, self._model.njoints)
            ]

            # Create Pink Configuration with neutral pose
            q0 = pin.neutral(self._model)
            ref_configs = self._model.referenceConfigurations
            if hasattr(ref_configs, 'get'):
                q0 = ref_configs.get("default", q0)
            elif hasattr(ref_configs, '__contains__') and "default" in ref_configs:
                q0 = ref_configs["default"]

            self._configuration = pink.Configuration(
                self._model,
                self._data,
                q0,
            )

            # Create FrameTask for each end-effector with per-chain weights
            for chain_name, frame_name in ee_frames.items():
                cfg = chain_configs.get(chain_name)
                if cfg is not None:
                    pos_cost = cfg.position_cost
                    ori_cost = cfg.orientation_cost
                    damping = cfg.lm_damping
                    self._chain_configs[chain_name] = cfg
                else:
                    pos_cost = self._default_position_cost
                    ori_cost = self._default_orientation_cost
                    damping = self._default_lm_damping
                    self._chain_configs[chain_name] = ChainConfig(
                        ee_frame=frame_name,
                        position_cost=pos_cost,
                        orientation_cost=ori_cost,
                        lm_damping=damping,
                    )

                task = pink.FrameTask(
                    frame=frame_name,
                    position_cost=pos_cost,
                    orientation_cost=ori_cost,
                    lm_damping=damping,
                )
                self._frame_tasks[chain_name] = task
                self._chain_frames[chain_name] = frame_name
                logger.info(
                    f"Created FrameTask: {chain_name} -> {frame_name} "
                    f"(pos={pos_cost}, ori={ori_cost}, damp={damping})"
                )

            # Create PostureTask for null-space regularization
            self._posture_task = pink.PostureTask(
                cost=self._posture_cost,
            )
            # Set initial posture target to current configuration
            self._posture_task.set_target_from_configuration(self._configuration)

            self._initialized = True
            logger.info(
                f"PinkIKSolver initialized: {self._model.nq} DOF, "
                f"{len(self._frame_tasks)} chains "
                f"({', '.join(self._frame_tasks.keys())})"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to initialize PinkIKSolver: {e}")
            return False

    def solve(
        self,
        target_pose: Pose6D,
        current_joints: np.ndarray,
        dt: float = 0.01,
    ) -> IKResult:
        """Solve IK for a single end-effector.

        Uses the first registered chain. For multi-chain, use solve_multi.
        """
        if not self._initialized or not self._frame_tasks:
            return IKResult(success=False)

        chain_name = next(iter(self._frame_tasks))
        return self.solve_multi(
            target_poses={chain_name: target_pose},
            current_joints=current_joints,
            dt=dt,
        )

    def solve_multi(
        self,
        target_poses: dict[str, Pose6D],
        current_joints: np.ndarray,
        dt: float = 0.01,
    ) -> IKResult:
        """Solve IK for multiple end-effectors simultaneously.

        The QP optimizer finds joint velocities that satisfy all end-effector
        targets concurrently. Shared joints (e.g., torso joints affecting both
        arms) are resolved via task weight priorities.

        Args:
            target_poses: Dict mapping chain name to target Pose6D.
            current_joints: Current joint positions (nq,).
            dt: Integration timestep.

        Returns:
            IKResult with new joint positions for the full robot.
        """
        if not self._initialized:
            return IKResult(success=False)

        try:
            # Update configuration with current joint values
            self._configuration.q = current_joints.copy()
            self._configuration.update()

            # Set targets for each frame task
            for chain_name, target in target_poses.items():
                if chain_name not in self._frame_tasks:
                    logger.warning(f"Unknown chain: {chain_name}")
                    continue
                task = self._frame_tasks[chain_name]
                T_target = pose_to_homogeneous(target.position, target.orientation)
                target_se3 = pin.SE3(T_target[:3, :3], T_target[:3, 3])
                task.set_target(target_se3)

            # Collect all tasks
            tasks = list(self._frame_tasks.values())
            if self._posture_task is not None:
                tasks.append(self._posture_task)

            # Solve IK (disable limits for robustness; limits enforced externally)
            velocity = pink.solve_ik(
                self._configuration,
                tasks,
                dt,
                solver=self._qp_solver,
                limits=[],
                safety_break=False,
            )

            # Integrate: q_new = q + v * dt
            q_new = pin.integrate(self._model, self._configuration.q, velocity * dt)

            # Compute per-chain errors
            self._configuration.q = q_new
            self._configuration.update()

            total_pos_error = 0.0
            total_ori_error = 0.0
            self._last_chain_errors.clear()

            for chain_name, target in target_poses.items():
                if chain_name in self._frame_tasks:
                    task = self._frame_tasks[chain_name]
                    error = task.compute_error(self._configuration)
                    pos_err = float(np.linalg.norm(error[:3]))
                    ori_err = float(np.linalg.norm(error[3:]))
                    self._last_chain_errors[chain_name] = ChainError(
                        position_error=pos_err,
                        orientation_error=ori_err,
                    )
                    total_pos_error += pos_err
                    total_ori_error += ori_err

            return IKResult(
                joint_positions=q_new,
                success=True,
                error_position=total_pos_error,
                error_orientation=total_ori_error,
            )

        except Exception as e:
            logger.error(f"IK solve failed: {e}")
            return IKResult(success=False)

    def get_joint_names(self) -> list[str]:
        """Get ordered joint names from the robot model."""
        return self._joint_names.copy()

    def get_joint_count(self) -> int:
        """Get the number of position DOF (nq)."""
        if self._model is None:
            return 0
        return self._model.nq

    def set_posture_target(self, posture: np.ndarray) -> None:
        """Set the null-space posture target.

        Args:
            posture: (nq,) preferred joint configuration.
        """
        if self._posture_task is not None:
            self._posture_task.set_target_from_configuration(
                pink.Configuration(self._model, self._data, posture)
            )

    def get_chain_names(self) -> list[str]:
        """Get the list of registered chain names."""
        return list(self._frame_tasks.keys())

    def get_chain_errors(self) -> dict[str, ChainError]:
        """Get per-chain errors from the last solve_multi call.

        Returns:
            Dict mapping chain name to ChainError with position/orientation errors.
        """
        return dict(self._last_chain_errors)

    def get_chain_ee_frame(self, chain_name: str) -> str | None:
        """Get the end-effector frame name for a chain.

        Args:
            chain_name: Name of the kinematic chain.

        Returns:
            Frame name string or None if chain not found.
        """
        return self._chain_frames.get(chain_name)

    def update_chain_weights(
        self,
        chain_name: str,
        position_cost: float | None = None,
        orientation_cost: float | None = None,
    ) -> bool:
        """Update the task weights for a specific chain at runtime.

        Args:
            chain_name: Name of the chain to update.
            position_cost: New position weight (None to keep current).
            orientation_cost: New orientation weight (None to keep current).

        Returns:
            True if the chain was found and updated.
        """
        if chain_name not in self._frame_tasks:
            return False

        task = self._frame_tasks[chain_name]
        cfg = self._chain_configs[chain_name]

        if position_cost is not None:
            task.position_cost = position_cost
            cfg.position_cost = position_cost
        if orientation_cost is not None:
            task.orientation_cost = orientation_cost
            cfg.orientation_cost = orientation_cost

        logger.info(
            f"Updated chain '{chain_name}' weights: "
            f"pos={cfg.position_cost}, ori={cfg.orientation_cost}"
        )
        return True

    def get_joint_index_by_name(self, joint_name: str) -> int | None:
        """Get the qpos index for a named joint.

        Args:
            joint_name: Name of the joint.

        Returns:
            Index into the q vector, or None if not found.
        """
        if self._model is None:
            return None
        try:
            joint_id = self._model.getJointId(joint_name)
            if joint_id >= self._model.njoints:
                return None
            return self._model.joints[joint_id].idx_q
        except Exception:
            return None

    def get_joint_indices_for_names(self, joint_names: list[str]) -> list[int]:
        """Get qpos indices for a list of joint names.

        Args:
            joint_names: List of joint names.

        Returns:
            List of indices into the q vector (skips names not found).
        """
        indices = []
        for name in joint_names:
            idx = self.get_joint_index_by_name(name)
            if idx is not None:
                indices.append(idx)
        return indices

    def get_ee_position(self, chain_name: str) -> np.ndarray | None:
        """Get current end-effector position for a chain.

        Args:
            chain_name: Name of the kinematic chain.

        Returns:
            (3,) position or None if chain not found.
        """
        if not self._initialized or chain_name not in self._frame_tasks:
            return None

        task = self._frame_tasks[chain_name]
        T = self._configuration.get_transform_frame_to_world(task.frame)
        return T.translation.copy()

    def get_ee_pose(self, chain_name: str) -> tuple[np.ndarray, np.ndarray] | None:
        """Get current end-effector pose (position + rotation) for a chain.

        Args:
            chain_name: Name of the kinematic chain.

        Returns:
            Tuple of (position (3,), rotation (3,3)) or None if not found.
        """
        if not self._initialized or chain_name not in self._frame_tasks:
            return None

        task = self._frame_tasks[chain_name]
        T = self._configuration.get_transform_frame_to_world(task.frame)
        return T.translation.copy(), T.rotation.copy()
