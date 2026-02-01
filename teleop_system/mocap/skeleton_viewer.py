"""Matplotlib 3D skeleton viewer for BVH motion data.

Displays BVH skeleton as a stick figure with joints. Supports
frame-by-frame playback with interactive slider control.
Highlights TrackerRole-mapped joints in a different color.
"""

from pathlib import Path

import numpy as np

from teleop_system.mocap.bvh_loader import BVHData
from teleop_system.mocap.skeleton_mapper import DEFAULT_JOINT_MAPPING
from teleop_system.utils.logger import get_logger

logger = get_logger("skeleton_viewer")


# Bone connections for CMU BVH skeleton (parent → child)
CMU_BONES = [
    ("Hips", "LHipJoint"),
    ("LHipJoint", "LeftUpLeg"),
    ("LeftUpLeg", "LeftLeg"),
    ("LeftLeg", "LeftFoot"),
    ("LeftFoot", "LeftToeBase"),
    ("Hips", "RHipJoint"),
    ("RHipJoint", "RightUpLeg"),
    ("RightUpLeg", "RightLeg"),
    ("RightLeg", "RightFoot"),
    ("RightFoot", "RightToeBase"),
    ("Hips", "LowerBack"),
    ("LowerBack", "Spine"),
    ("Spine", "Spine1"),
    ("Spine1", "Neck"),
    ("Neck", "Neck1"),
    ("Neck1", "Head"),
    ("Spine1", "LeftShoulder"),
    ("LeftShoulder", "LeftArm"),
    ("LeftArm", "LeftForeArm"),
    ("LeftForeArm", "LeftHand"),
    ("LeftHand", "LeftFingerBase"),
    ("LeftFingerBase", "LeftHandIndex1"),
    ("LeftHand", "LThumb"),
    ("Spine1", "RightShoulder"),
    ("RightShoulder", "RightArm"),
    ("RightArm", "RightForeArm"),
    ("RightForeArm", "RightHand"),
    ("RightHand", "RightFingerBase"),
    ("RightFingerBase", "RightHandIndex1"),
    ("RightHand", "RThumb"),
]


def view_skeleton(
    bvh_data: BVHData,
    start_frame: int = 0,
    highlight_tracker_joints: bool = True,
    bone_connections: list | None = None,
    figsize: tuple = (10, 8),
):
    """Display interactive 3D skeleton viewer.

    Args:
        bvh_data: Loaded BVH data (in ROS2 coordinates).
        start_frame: Initial frame to display.
        highlight_tracker_joints: Whether to highlight TrackerRole joints.
        bone_connections: List of (parent, child) bone pairs. If None, uses CMU_BONES.
        figsize: Figure size.
    """
    try:
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Slider
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        logger.error("matplotlib is required for skeleton viewer")
        return

    bones = bone_connections or CMU_BONES

    # Filter to bones where both joints exist in the data
    valid_bones = [
        (p, c) for p, c in bones
        if p in bvh_data.frames and c in bvh_data.frames
    ]

    # Tracker joint names for highlighting
    tracker_joints = set()
    if highlight_tracker_joints:
        for role, jname in DEFAULT_JOINT_MAPPING.items():
            tracker_joints.add(jname)

    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection="3d")
    plt.subplots_adjust(bottom=0.15)

    # Slider axis
    ax_slider = plt.axes([0.15, 0.02, 0.7, 0.03])
    slider = Slider(
        ax_slider, "Frame",
        0, max(1, bvh_data.frame_count - 1),
        valinit=start_frame, valstep=1,
    )

    def update(frame_idx):
        frame_idx = int(frame_idx)
        ax.clear()

        # Collect joint positions for this frame
        joint_positions = {}
        for jname in bvh_data.frames:
            if frame_idx < len(bvh_data.frames[jname]):
                joint_positions[jname] = bvh_data.frames[jname][frame_idx].position

        # Draw bones
        for parent, child in valid_bones:
            if parent in joint_positions and child in joint_positions:
                p1 = joint_positions[parent]
                p2 = joint_positions[child]
                ax.plot(
                    [p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
                    "b-", linewidth=2, alpha=0.7,
                )

        # Draw joints
        for jname, pos in joint_positions.items():
            if jname in tracker_joints:
                ax.scatter(pos[0], pos[1], pos[2], c="red", s=60, marker="o", zorder=5)
            else:
                ax.scatter(pos[0], pos[1], pos[2], c="blue", s=20, marker="o")

        # Set axis labels and limits (ROS2: X=forward, Y=left, Z=up)
        ax.set_xlabel("X (forward)")
        ax.set_ylabel("Y (left)")
        ax.set_zlabel("Z (up)")
        ax.set_title(f"Frame {frame_idx}/{bvh_data.frame_count - 1}")

        # Auto-scale with equal aspect
        all_pos = np.array(list(joint_positions.values()))
        if len(all_pos) > 0:
            center = np.mean(all_pos, axis=0)
            max_range = np.max(np.ptp(all_pos, axis=0)) / 2
            max_range = max(max_range, 0.1)
            ax.set_xlim(center[0] - max_range, center[0] + max_range)
            ax.set_ylim(center[1] - max_range, center[1] + max_range)
            ax.set_zlim(center[2] - max_range, center[2] + max_range)

        fig.canvas.draw_idle()

    slider.on_changed(update)
    update(start_frame)
    plt.show()


def save_skeleton_frames(
    bvh_data: BVHData,
    output_dir: str | Path,
    frame_step: int = 10,
    bone_connections: list | None = None,
    figsize: tuple = (8, 6),
):
    """Save skeleton frames as PNG images.

    Args:
        bvh_data: Loaded BVH data.
        output_dir: Directory to save images.
        frame_step: Save every Nth frame.
        bone_connections: Bone connections.
        figsize: Figure size.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        logger.error("matplotlib is required")
        return

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    bones = bone_connections or CMU_BONES
    valid_bones = [
        (p, c) for p, c in bones
        if p in bvh_data.frames and c in bvh_data.frames
    ]

    for frame_idx in range(0, bvh_data.frame_count, frame_step):
        fig = plt.figure(figsize=figsize)
        ax = fig.add_subplot(111, projection="3d")

        joint_positions = {}
        for jname in bvh_data.frames:
            if frame_idx < len(bvh_data.frames[jname]):
                joint_positions[jname] = bvh_data.frames[jname][frame_idx].position

        for parent, child in valid_bones:
            if parent in joint_positions and child in joint_positions:
                p1 = joint_positions[parent]
                p2 = joint_positions[child]
                ax.plot(
                    [p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
                    "b-", linewidth=2,
                )

        for jname, pos in joint_positions.items():
            ax.scatter(pos[0], pos[1], pos[2], c="blue", s=15)

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title(f"Frame {frame_idx}")

        plt.savefig(output_dir / f"frame_{frame_idx:05d}.png", dpi=100)
        plt.close(fig)

    logger.info(f"Saved {bvh_data.frame_count // frame_step} frames to {output_dir}")
