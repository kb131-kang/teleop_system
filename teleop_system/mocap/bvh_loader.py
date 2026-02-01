"""BVH motion capture file loader.

Parses BVH files using the bvhio library and extracts per-frame joint
positions and orientations. Handles coordinate conversion from BVH (Y-up)
to ROS2 (Z-up) convention.

CMU BVH coordinate system:
    X = lateral (positive = left when facing forward)
    Y = up
    Z = forward

ROS2 coordinate system:
    X = forward
    Y = left
    Z = up

Conversion: ros2_pos = (bvh_z, bvh_x, bvh_y) * scale
"""

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from teleop_system.utils.logger import get_logger

logger = get_logger("bvh_loader")


@dataclass
class JointFrame:
    """Single frame data for one joint.

    Attributes:
        position: (3,) world position in ROS2 frame (meters).
        orientation: (4,) quaternion in ROS2 convention (x, y, z, w).
    """
    position: np.ndarray
    orientation: np.ndarray


@dataclass
class BVHData:
    """Parsed BVH motion data.

    Attributes:
        joint_names: List of joint names in the skeleton.
        frame_count: Number of animation frames.
        frame_time: Time per frame in seconds.
        fps: Frames per second.
        frames: Dict mapping joint_name → list of JointFrame (one per frame).
        skeleton_parents: Dict mapping joint_name → parent joint_name (None for root).
        skeleton_offsets: Dict mapping joint_name → (3,) rest-pose offset in ROS2 meters.
    """
    joint_names: list = field(default_factory=list)
    frame_count: int = 0
    frame_time: float = 0.0
    fps: float = 0.0
    frames: dict = field(default_factory=dict)
    skeleton_parents: dict = field(default_factory=dict)
    skeleton_offsets: dict = field(default_factory=dict)


def _bvh_to_ros2_position(x: float, y: float, z: float, scale: float) -> np.ndarray:
    """Convert BVH Y-up position to ROS2 Z-up position in meters."""
    return np.array([z * scale, x * scale, y * scale])


def _bvh_to_ros2_quaternion(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Convert BVH quaternion (glm w,x,y,z Y-up) to ROS2 quaternion (xyzw Z-up).

    When permuting axes (X→Y, Y→Z, Z→X), the quaternion imaginary
    components follow the same permutation.
    BVH (w, vx, vy, vz) → ROS2 (w, vz, vx, vy) in wxyz format.
    In ROS2 xyzw format: (vz, vx, vy, w).
    """
    return np.array([z, x, y, w])


def _collect_joint_names(joint) -> list:
    """Recursively collect all joint names from the hierarchy."""
    names = [joint.Name]
    for child in joint.Children:
        names.extend(_collect_joint_names(child))
    return names


def _find_joint(joint, name: str):
    """Recursively find a joint by name in the hierarchy."""
    if joint.Name == name:
        return joint
    for child in joint.Children:
        result = _find_joint(child, name)
        if result is not None:
            return result
    return None


def _collect_parent_map(joint, parent_name=None) -> dict:
    """Build parent map: joint_name → parent_name."""
    parents = {joint.Name: parent_name}
    for child in joint.Children:
        parents.update(_collect_parent_map(child, joint.Name))
    return parents


def load_bvh(
    file_path: str | Path,
    scale: float = 0.056,
) -> BVHData:
    """Load a BVH file and convert to ROS2 coordinate frame.

    Args:
        file_path: Path to the BVH file.
        scale: Scale factor to convert BVH units to meters.
            Default 0.056 works well for CMU BVH data (places hips
            at approximately 0.95m height).

    Returns:
        BVHData with all frames converted to ROS2 convention.

    Raises:
        FileNotFoundError: If the BVH file doesn't exist.
        ImportError: If bvhio is not installed.
    """
    file_path = Path(file_path)
    if not file_path.exists():
        raise FileNotFoundError(f"BVH file not found: {file_path}")

    try:
        import bvhio
    except ImportError:
        raise ImportError(
            "bvhio is required for BVH loading. Install with: pip install bvhio"
        )

    # Get frame metadata
    bvh_container = bvhio.readAsBvh(str(file_path))
    frame_count = bvh_container.FrameCount
    frame_time = bvh_container.FrameTime
    fps = 1.0 / frame_time if frame_time > 0 else 0.0

    # Load hierarchy for pose access
    root = bvhio.readAsHierarchy(str(file_path))

    # Collect joint info
    joint_names = _collect_joint_names(root)
    parent_map = _collect_parent_map(root)

    # Collect rest-pose offsets
    root.loadRestPose()
    skeleton_offsets = {}
    for jname in joint_names:
        joint = _find_joint(root, jname)
        if joint is not None:
            pos = joint.PositionWorld
            skeleton_offsets[jname] = _bvh_to_ros2_position(pos.x, pos.y, pos.z, scale)

    # Extract per-frame data for all joints
    frames = {name: [] for name in joint_names}

    for frame_idx in range(frame_count):
        root.loadPose(frame_idx)
        for jname in joint_names:
            joint = _find_joint(root, jname)
            if joint is not None:
                pos = joint.PositionWorld
                rot = joint.RotationWorld

                position = _bvh_to_ros2_position(pos.x, pos.y, pos.z, scale)
                orientation = _bvh_to_ros2_quaternion(rot.w, rot.x, rot.y, rot.z)

                frames[jname].append(JointFrame(
                    position=position,
                    orientation=orientation,
                ))
            else:
                frames[jname].append(JointFrame(
                    position=np.zeros(3),
                    orientation=np.array([0.0, 0.0, 0.0, 1.0]),
                ))

    logger.info(
        f"Loaded BVH: {file_path.name}, "
        f"{frame_count} frames @ {fps:.0f}fps, "
        f"{len(joint_names)} joints"
    )

    return BVHData(
        joint_names=joint_names,
        frame_count=frame_count,
        frame_time=frame_time,
        fps=fps,
        frames=frames,
        skeleton_parents=parent_map,
        skeleton_offsets=skeleton_offsets,
    )


def load_bvh_lazy(
    file_path: str | Path,
    scale: float = 0.056,
    joints: list[str] | None = None,
) -> BVHData:
    """Load a BVH file, extracting only specified joints for efficiency.

    Args:
        file_path: Path to the BVH file.
        scale: Scale factor to convert BVH units to meters.
        joints: List of joint names to extract. If None, extracts all joints.

    Returns:
        BVHData with requested joints' frames converted to ROS2 convention.
    """
    file_path = Path(file_path)
    if not file_path.exists():
        raise FileNotFoundError(f"BVH file not found: {file_path}")

    try:
        import bvhio
    except ImportError:
        raise ImportError(
            "bvhio is required for BVH loading. Install with: pip install bvhio"
        )

    bvh_container = bvhio.readAsBvh(str(file_path))
    frame_count = bvh_container.FrameCount
    frame_time = bvh_container.FrameTime
    fps = 1.0 / frame_time if frame_time > 0 else 0.0

    root = bvhio.readAsHierarchy(str(file_path))

    all_joint_names = _collect_joint_names(root)
    parent_map = _collect_parent_map(root)

    target_joints = joints if joints is not None else all_joint_names

    # Validate requested joints exist
    missing = [j for j in target_joints if j not in all_joint_names]
    if missing:
        logger.warning(f"Joints not found in BVH: {missing}")
        target_joints = [j for j in target_joints if j in all_joint_names]

    # Rest-pose offsets
    root.loadRestPose()
    skeleton_offsets = {}
    for jname in all_joint_names:
        joint = _find_joint(root, jname)
        if joint is not None:
            pos = joint.PositionWorld
            skeleton_offsets[jname] = _bvh_to_ros2_position(pos.x, pos.y, pos.z, scale)

    # Extract frames for target joints only
    frames = {name: [] for name in target_joints}

    for frame_idx in range(frame_count):
        root.loadPose(frame_idx)
        for jname in target_joints:
            joint = _find_joint(root, jname)
            if joint is not None:
                pos = joint.PositionWorld
                rot = joint.RotationWorld
                position = _bvh_to_ros2_position(pos.x, pos.y, pos.z, scale)
                orientation = _bvh_to_ros2_quaternion(rot.w, rot.x, rot.y, rot.z)
                frames[jname].append(JointFrame(
                    position=position,
                    orientation=orientation,
                ))

    logger.info(
        f"Loaded BVH (lazy): {file_path.name}, "
        f"{frame_count} frames @ {fps:.0f}fps, "
        f"{len(target_joints)}/{len(all_joint_names)} joints"
    )

    return BVHData(
        joint_names=all_joint_names,
        frame_count=frame_count,
        frame_time=frame_time,
        fps=fps,
        frames=frames,
        skeleton_parents=parent_map,
        skeleton_offsets=skeleton_offsets,
    )
