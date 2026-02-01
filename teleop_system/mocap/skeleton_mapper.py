"""Maps BVH skeleton joints to TrackerRole Pose6D data.

Converts BVH skeleton joint positions/orientations to the tracker roles
used by the teleoperation system (RIGHT_HAND, LEFT_HAND, WAIST, etc.).
Handles position normalization to align BVH data with the robot workspace.
"""

from dataclasses import dataclass, field

import numpy as np

from teleop_system.interfaces.master_device import Pose6D, TrackerRole
from teleop_system.mocap.bvh_loader import BVHData, JointFrame
from teleop_system.utils.logger import get_logger

logger = get_logger("skeleton_mapper")

# Default CMU BVH joint names for each tracker role
DEFAULT_JOINT_MAPPING = {
    TrackerRole.RIGHT_HAND: "RightHand",
    TrackerRole.LEFT_HAND: "LeftHand",
    TrackerRole.WAIST: "Hips",
    TrackerRole.RIGHT_FOOT: "RightFoot",
    TrackerRole.LEFT_FOOT: "LeftFoot",
    TrackerRole.HEAD: "Head",
}

# Default robot workspace reference positions (from SimulatedTracker)
DEFAULT_ROBOT_POSITIONS = {
    TrackerRole.RIGHT_HAND: np.array([0.4, -0.3, 1.0]),
    TrackerRole.LEFT_HAND: np.array([0.4, 0.3, 1.0]),
    TrackerRole.WAIST: np.array([0.0, 0.0, 0.9]),
    TrackerRole.RIGHT_FOOT: np.array([0.0, -0.15, 0.05]),
    TrackerRole.LEFT_FOOT: np.array([0.0, 0.15, 0.05]),
    TrackerRole.HEAD: np.array([0.0, 0.0, 1.6]),
}


@dataclass
class TrackerFrameData:
    """Per-frame tracker data for all roles.

    Attributes:
        poses: Dict mapping TrackerRole → Pose6D for this frame.
    """
    poses: dict = field(default_factory=dict)


@dataclass
class MappedMotion:
    """Complete mapped motion sequence.

    Attributes:
        frame_count: Number of frames.
        frame_time: Time per frame in seconds.
        fps: Frames per second.
        frames: List of TrackerFrameData (one per frame).
        roles: List of TrackerRole that have valid data.
    """
    frame_count: int = 0
    frame_time: float = 0.0
    fps: float = 0.0
    frames: list = field(default_factory=list)
    roles: list = field(default_factory=list)


class SkeletonMapper:
    """Maps BVH skeleton data to TrackerRole Pose6D sequences.

    Supports two normalization modes:
    - 'relative': Computes motion relative to the first frame and adds
      robot reference positions. This makes the BVH motion start at the
      robot's default pose.
    - 'absolute': Uses raw converted positions (after BVH→ROS2 transform).
    """

    def __init__(
        self,
        joint_mapping: dict[TrackerRole, str] | None = None,
        normalize_mode: str = "relative",
        robot_reference: dict[TrackerRole, np.ndarray] | None = None,
    ):
        """Initialize the skeleton mapper.

        Args:
            joint_mapping: Maps TrackerRole → BVH joint name.
                Defaults to CMU BVH joint names.
            normalize_mode: 'relative' (motion relative to first frame +
                robot reference) or 'absolute' (raw converted positions).
            robot_reference: Reference positions for each TrackerRole in
                ROS2 frame. Used when normalize_mode='relative'.
        """
        self._joint_mapping = joint_mapping or dict(DEFAULT_JOINT_MAPPING)
        self._normalize_mode = normalize_mode
        self._robot_reference = robot_reference or dict(DEFAULT_ROBOT_POSITIONS)

    def map(self, bvh_data: BVHData) -> MappedMotion:
        """Map BVH data to TrackerRole poses.

        Args:
            bvh_data: Loaded BVH data with frames in ROS2 coordinate frame.

        Returns:
            MappedMotion with per-frame Pose6D for each TrackerRole.
        """
        # Determine which roles have valid BVH joint data
        available_roles = []
        for role, joint_name in self._joint_mapping.items():
            if joint_name in bvh_data.frames and len(bvh_data.frames[joint_name]) > 0:
                available_roles.append(role)
            else:
                logger.warning(
                    f"Joint '{joint_name}' for {role.name} not found in BVH data"
                )

        if not available_roles:
            logger.error("No valid joints found for any tracker role")
            return MappedMotion(
                frame_count=bvh_data.frame_count,
                frame_time=bvh_data.frame_time,
                fps=bvh_data.fps,
            )

        # Compute first-frame positions for relative mode
        first_frame_positions = {}
        if self._normalize_mode == "relative":
            for role in available_roles:
                joint_name = self._joint_mapping[role]
                first_frame_positions[role] = bvh_data.frames[joint_name][0].position.copy()

        # Map each frame
        mapped_frames = []
        for frame_idx in range(bvh_data.frame_count):
            timestamp = frame_idx * bvh_data.frame_time
            frame_data = TrackerFrameData()

            for role in available_roles:
                joint_name = self._joint_mapping[role]
                joint_frame: JointFrame = bvh_data.frames[joint_name][frame_idx]

                if self._normalize_mode == "relative":
                    # Position relative to first frame + robot reference
                    delta = joint_frame.position - first_frame_positions[role]
                    position = self._robot_reference.get(
                        role, np.zeros(3)
                    ) + delta
                else:
                    position = joint_frame.position.copy()

                frame_data.poses[role] = Pose6D(
                    position=position,
                    orientation=joint_frame.orientation.copy(),
                    timestamp=timestamp,
                    valid=True,
                )

            mapped_frames.append(frame_data)

        logger.info(
            f"Mapped {bvh_data.frame_count} frames for "
            f"{len(available_roles)} roles: "
            f"{[r.name for r in available_roles]}"
        )

        return MappedMotion(
            frame_count=bvh_data.frame_count,
            frame_time=bvh_data.frame_time,
            fps=bvh_data.fps,
            frames=mapped_frames,
            roles=available_roles,
        )

    def get_joint_name(self, role: TrackerRole) -> str | None:
        """Get the BVH joint name mapped to a tracker role."""
        return self._joint_mapping.get(role)
