"""Arm teleoperation controller logic.

Pure control logic with no ROS2 dependency — receives tracker poses,
solves IK, and produces joint commands. Can be tested standalone.

Supports 1-3 simultaneous kinematic chains (left arm, right arm, torso)
with proper per-chain joint index mapping for dispatch.
"""

import numpy as np

from teleop_system.interfaces.ik_solver import IIKSolver, IKResult
from teleop_system.interfaces.master_device import IMasterTracker, Pose6D, TrackerRole
from teleop_system.interfaces.slave_robot import ISlaveArm, ArmSide
from teleop_system.utils.logger import get_logger

logger = get_logger("arm_controller")


# Map TrackerRole to chain name used by IK solver
_ROLE_TO_CHAIN = {
    TrackerRole.RIGHT_HAND: "right_arm",
    TrackerRole.LEFT_HAND: "left_arm",
    TrackerRole.WAIST: "torso",
}

_ROLE_TO_ARM_SIDE = {
    TrackerRole.RIGHT_HAND: ArmSide.RIGHT,
    TrackerRole.LEFT_HAND: ArmSide.LEFT,
    TrackerRole.WAIST: ArmSide.TORSO,
}

# Reverse mapping: ArmSide -> chain name
_ARM_SIDE_TO_CHAIN = {v: k for k, v in zip(
    ["right_arm", "left_arm", "torso"],
    [ArmSide.RIGHT, ArmSide.LEFT, ArmSide.TORSO],
)}


class ArmController:
    """Controls robot arms and torso via IK from tracker poses.

    Supports 1-3 simultaneous kinematic chains (left arm, right arm, torso).
    Uses differential IK (Pink) to convert Cartesian targets to joint commands.

    Joint dispatch uses per-chain index mapping: each ISlaveArm adapter
    receives only the joint subset that it controls, extracted from the
    full IK solution vector.
    """

    def __init__(
        self,
        ik_solver: IIKSolver,
        trackers: dict[TrackerRole, IMasterTracker],
        arms: dict[ArmSide, ISlaveArm],
        dt: float = 0.01,
        position_scale: float = 1.0,
        orientation_scale: float = 1.0,
        max_joint_velocity: float = 2.0,
        joint_index_map: dict[ArmSide, list[int]] | None = None,
    ):
        """Initialize arm controller.

        Args:
            ik_solver: IK solver instance (must be initialized with ee_frames).
            trackers: Dict mapping TrackerRole -> tracker device.
            arms: Dict mapping ArmSide -> slave arm.
            dt: Control loop timestep (seconds).
            position_scale: Master-to-slave position scaling factor.
            orientation_scale: Master-to-slave orientation scaling factor.
            max_joint_velocity: Joint velocity limit (rad/s).
            joint_index_map: Dict mapping ArmSide -> list of indices into the
                full q vector for that arm's joints. If None, falls back to
                sequential slicing (arm gets q[:n]).
        """
        self._ik_solver = ik_solver
        self._trackers = trackers
        self._arms = arms
        self._dt = dt
        self._position_scale = position_scale
        self._orientation_scale = orientation_scale
        self._max_joint_velocity = max_joint_velocity
        self._current_joints: np.ndarray | None = None
        self._enabled = False

        # Per-chain joint index mapping: ArmSide -> indices into full q vector
        self._joint_index_map: dict[ArmSide, list[int]] = joint_index_map or {}

        # Workspace offset: difference between tracker initial position and robot EE initial position
        self._workspace_offsets: dict[TrackerRole, np.ndarray | None] = {
            role: None for role in trackers
        }
        self._initial_tracker_poses: dict[TrackerRole, Pose6D | None] = {
            role: None for role in trackers
        }

    def enable(self) -> None:
        """Enable the controller and capture initial poses for offset calibration."""
        self._enabled = True
        # Capture initial tracker poses for workspace calibration
        for role, tracker in self._trackers.items():
            if tracker.is_connected():
                pose = tracker.get_pose()
                if pose.valid:
                    self._initial_tracker_poses[role] = pose
        logger.info("ArmController enabled")

    def disable(self) -> None:
        """Disable the controller. Stops sending commands."""
        self._enabled = False
        logger.info("ArmController disabled")

    def update(self) -> IKResult:
        """Run one control cycle: read trackers -> IK -> send commands.

        Returns:
            IKResult from the IK solver.
        """
        if not self._enabled:
            return IKResult(success=False)

        # Initialize current joints from arm feedback or zeros
        if self._current_joints is None:
            self._current_joints = self._read_joint_feedback()

        # Gather target poses for each active chain
        target_poses: dict[str, Pose6D] = {}
        for role, tracker in self._trackers.items():
            if not tracker.is_connected():
                continue
            pose = tracker.get_pose()
            if not pose.valid:
                continue

            chain_name = _ROLE_TO_CHAIN.get(role)
            if chain_name is None:
                continue

            # Apply position scaling
            scaled_pose = Pose6D(
                position=pose.position * self._position_scale,
                orientation=pose.orientation,
                timestamp=pose.timestamp,
                valid=True,
            )
            target_poses[chain_name] = scaled_pose

        if not target_poses:
            return IKResult(success=False)

        # Solve IK for all active chains simultaneously
        result = self._ik_solver.solve_multi(
            target_poses=target_poses,
            current_joints=self._current_joints,
            dt=self._dt,
        )

        if result.success:
            # Apply joint velocity limits
            joint_delta = result.joint_positions - self._current_joints
            max_delta = self._max_joint_velocity * self._dt
            joint_delta = np.clip(joint_delta, -max_delta, max_delta)
            result.joint_positions = self._current_joints + joint_delta

            # Update current joints
            self._current_joints = result.joint_positions.copy()

            # Send commands to slave arms using per-chain joint mapping
            self._dispatch_joint_commands(result.joint_positions)

        return result

    def _read_joint_feedback(self) -> np.ndarray:
        """Read current joint positions from all arms and assemble full q vector.

        If joint_index_map is configured, each arm's joint positions are
        placed at their mapped indices. Otherwise falls back to zeros.
        """
        nq = self._ik_solver.get_joint_count()
        q = np.zeros(nq)

        if self._joint_index_map:
            for side, arm in self._arms.items():
                if side in self._joint_index_map and arm.is_connected():
                    state = arm.get_joint_state()
                    indices = self._joint_index_map[side]
                    for i, idx in enumerate(indices):
                        if i < len(state.positions) and idx < nq:
                            q[idx] = state.positions[i]
        return q

    def _dispatch_joint_commands(self, full_joint_positions: np.ndarray) -> None:
        """Send the solved joint positions to the appropriate slave arms.

        Uses joint_index_map to extract each arm's joint subset from the
        full IK solution. If no mapping is configured, falls back to
        sending the full vector (the arm adapter is expected to extract
        its own joints).
        """
        for side, arm in self._arms.items():
            if not arm.is_connected():
                continue

            if side in self._joint_index_map:
                # Extract this arm's joints from the full solution
                indices = self._joint_index_map[side]
                arm_joints = np.array([
                    full_joint_positions[idx]
                    for idx in indices
                    if idx < len(full_joint_positions)
                ])
                arm.send_joint_command(arm_joints)
            else:
                # Fallback: send full vector, let adapter extract its joints
                arm.send_joint_command(full_joint_positions)

    def set_joint_state(self, joints: np.ndarray) -> None:
        """Manually set current joint state (e.g., from simulator feedback)."""
        self._current_joints = joints.copy()

    def sync_from_arms(self) -> None:
        """Read current joint state from all arm adapters and update internal state.

        Useful when the simulation or hardware has advanced and we want
        the controller's internal state to match.
        """
        self._current_joints = self._read_joint_feedback()

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    @property
    def active_chain_count(self) -> int:
        """Number of currently connected tracker channels."""
        count = 0
        for tracker in self._trackers.values():
            if tracker.is_connected():
                count += 1
        return count
