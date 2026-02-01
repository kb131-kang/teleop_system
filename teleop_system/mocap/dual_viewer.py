"""Side-by-side skeleton and robot state viewer.

Displays BVH human skeleton alongside robot joint state for visual
comparison of how well the teleop pipeline reproduces human motion.
"""

from pathlib import Path

import numpy as np

from teleop_system.mocap.bvh_loader import BVHData
from teleop_system.mocap.skeleton_mapper import MappedMotion, DEFAULT_JOINT_MAPPING
from teleop_system.mocap.skeleton_viewer import CMU_BONES
from teleop_system.interfaces.master_device import TrackerRole
from teleop_system.utils.logger import get_logger

logger = get_logger("dual_viewer")


def view_dual(
    bvh_data: BVHData,
    mapped_motion: MappedMotion,
    recorded_output: dict | None = None,
    start_frame: int = 0,
    figsize: tuple = (16, 8),
):
    """Display side-by-side skeleton and tracker/output comparison.

    Left panel: Full BVH skeleton with TrackerRole joints highlighted.
    Right panel: Mapped tracker positions with optional recorded output overlay.

    Args:
        bvh_data: Loaded BVH data (full skeleton).
        mapped_motion: Mapped motion from SkeletonMapper.
        recorded_output: Optional dict of channel → (N, 3) output positions.
        start_frame: Initial frame.
        figsize: Figure size.
    """
    try:
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Slider
    except ImportError:
        logger.error("matplotlib is required for dual viewer")
        return

    valid_bones = [
        (p, c) for p, c in CMU_BONES
        if p in bvh_data.frames and c in bvh_data.frames
    ]

    tracker_joints = {jname for role, jname in DEFAULT_JOINT_MAPPING.items()}

    role_colors = {
        TrackerRole.RIGHT_HAND: "red",
        TrackerRole.LEFT_HAND: "blue",
        TrackerRole.WAIST: "green",
        TrackerRole.RIGHT_FOOT: "orange",
        TrackerRole.LEFT_FOOT: "purple",
        TrackerRole.HEAD: "cyan",
    }

    fig = plt.figure(figsize=figsize)
    ax_skel = fig.add_subplot(121, projection="3d")
    ax_tracker = fig.add_subplot(122, projection="3d")
    plt.subplots_adjust(bottom=0.12)

    ax_slider = plt.axes([0.15, 0.02, 0.7, 0.03])
    slider = Slider(
        ax_slider, "Frame",
        0, max(1, bvh_data.frame_count - 1),
        valinit=start_frame, valstep=1,
    )

    def update(frame_idx):
        frame_idx = int(frame_idx)
        ax_skel.clear()
        ax_tracker.clear()

        # --- Left panel: Full skeleton ---
        joint_positions = {}
        for jname in bvh_data.frames:
            if frame_idx < len(bvh_data.frames[jname]):
                joint_positions[jname] = bvh_data.frames[jname][frame_idx].position

        for parent, child in valid_bones:
            if parent in joint_positions and child in joint_positions:
                p1 = joint_positions[parent]
                p2 = joint_positions[child]
                ax_skel.plot(
                    [p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
                    "b-", linewidth=2, alpha=0.6,
                )

        for jname, pos in joint_positions.items():
            color = "red" if jname in tracker_joints else "blue"
            size = 50 if jname in tracker_joints else 15
            ax_skel.scatter(pos[0], pos[1], pos[2], c=color, s=size)

        ax_skel.set_xlabel("X (fwd)")
        ax_skel.set_ylabel("Y (left)")
        ax_skel.set_zlabel("Z (up)")
        ax_skel.set_title(f"Human Skeleton - Frame {frame_idx}")

        # --- Right panel: Tracker positions ---
        if frame_idx < len(mapped_motion.frames):
            frame_data = mapped_motion.frames[frame_idx]
            for role, pose in frame_data.poses.items():
                if pose.valid:
                    color = role_colors.get(role, "gray")
                    ax_tracker.scatter(
                        pose.position[0], pose.position[1], pose.position[2],
                        c=color, s=100, marker="o", label=role.name,
                        edgecolors="black", linewidth=1,
                    )
                    ax_tracker.text(
                        pose.position[0], pose.position[1], pose.position[2] + 0.05,
                        role.name, fontsize=7, ha="center",
                    )

        # Overlay recorded output if available
        if recorded_output is not None:
            for channel, positions in recorded_output.items():
                if frame_idx < len(positions):
                    pos = positions[frame_idx]
                    ax_tracker.scatter(
                        pos[0], pos[1], pos[2],
                        c="gray", s=50, marker="^", alpha=0.6,
                    )

        ax_tracker.set_xlabel("X (fwd)")
        ax_tracker.set_ylabel("Y (left)")
        ax_tracker.set_zlabel("Z (up)")
        ax_tracker.set_title(f"Tracker Positions - Frame {frame_idx}")

        # Auto-scale both panels
        for ax_panel in [ax_skel, ax_tracker]:
            ax_panel.set_xlim(-1, 1.5)
            ax_panel.set_ylim(-1, 1)
            ax_panel.set_zlim(0, 2)

        fig.canvas.draw_idle()

    slider.on_changed(update)
    update(start_frame)
    plt.show()


def save_comparison_plot(
    input_positions: np.ndarray,
    output_positions: np.ndarray,
    labels: tuple[str, str] = ("Input", "Output"),
    title: str = "Trajectory Comparison",
    output_path: str | Path = "comparison.png",
):
    """Save a static comparison plot of input vs output trajectories.

    Args:
        input_positions: (N, 3) input trajectory.
        output_positions: (N, 3) output trajectory.
        labels: Labels for input and output.
        title: Plot title.
        output_path: Path to save the plot.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        logger.error("matplotlib is required")
        return

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axis_labels = ["X (forward)", "Y (left)", "Z (up)"]

    for i, (ax, label) in enumerate(zip(axes, axis_labels)):
        ax.plot(input_positions[:, i], label=labels[0], alpha=0.8)
        ax.plot(output_positions[:, i], label=labels[1], alpha=0.8, linestyle="--")
        ax.set_ylabel(f"{label} (m)")
        ax.legend()
        ax.grid(True, alpha=0.3)

    axes[0].set_title(title)
    axes[-1].set_xlabel("Frame")

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close(fig)
    logger.info(f"Comparison plot saved to {output_path}")
