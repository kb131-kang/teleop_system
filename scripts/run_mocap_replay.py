#!/usr/bin/env python3
"""Standalone BVH motion replay and analysis (no ROS2 required).

Loads a BVH file, maps it to tracker data, feeds it through the
teleop controllers, records output, and generates metrics + plots.

Usage:
    python3 scripts/run_mocap_replay.py --bvh data/bvh/cmu/002/02_01.bvh
    python3 scripts/run_mocap_replay.py --bvh data/bvh/cmu/002/02_01.bvh --view
    python3 scripts/run_mocap_replay.py --bvh data/bvh/cmu/002/02_01.bvh --output report.json
"""

import argparse
import sys
from pathlib import Path

# Add project root to path so `teleop_system` is importable when running as script
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np


def main():
    parser = argparse.ArgumentParser(description="BVH motion replay and analysis")
    parser.add_argument(
        "--bvh", type=str, required=True,
        help="Path to BVH file",
    )
    parser.add_argument(
        "--scale", type=float, default=0.056,
        help="BVH → meters scale factor (default: 0.056)",
    )
    parser.add_argument(
        "--normalize", type=str, default="relative",
        choices=["relative", "absolute"],
        help="Position normalization mode",
    )
    parser.add_argument(
        "--output", type=str, default=None,
        help="Output path for metrics report (JSON)",
    )
    parser.add_argument(
        "--view", action="store_true",
        help="Open skeleton viewer after analysis",
    )
    parser.add_argument(
        "--dual-view", action="store_true",
        help="Open dual viewer (skeleton + tracker)",
    )
    parser.add_argument(
        "--save-plots", type=str, default=None,
        help="Directory to save trajectory plots",
    )

    args = parser.parse_args()

    # Resolve BVH path
    bvh_path = Path(args.bvh)
    if not bvh_path.exists():
        print(f"Error: BVH file not found: {bvh_path}")
        sys.exit(1)

    print(f"Loading BVH: {bvh_path}")

    # Load and map
    from teleop_system.mocap.bvh_loader import load_bvh, load_bvh_lazy
    from teleop_system.mocap.skeleton_mapper import SkeletonMapper
    from teleop_system.mocap.metrics import (
        compute_tracking_error,
        compute_workspace_utilization,
        compute_smoothness,
        generate_report,
    )
    from teleop_system.interfaces.master_device import TrackerRole

    # Load full skeleton for viewing, lazy for analysis
    target_joints = ["Hips", "Head", "LeftHand", "RightHand", "LeftFoot", "RightFoot"]
    bvh_data_lazy = load_bvh_lazy(str(bvh_path), scale=args.scale, joints=target_joints)

    mapper = SkeletonMapper(normalize_mode=args.normalize)
    mapped_motion = mapper.map(bvh_data_lazy)

    print(f"Frames: {mapped_motion.frame_count}")
    print(f"FPS: {mapped_motion.fps:.0f}")
    print(f"Duration: {mapped_motion.frame_count * mapped_motion.frame_time:.1f}s")
    print(f"Roles: {[r.name for r in mapped_motion.roles]}")

    # Extract position trajectories for analysis
    metrics = {}
    role_names = {
        TrackerRole.RIGHT_HAND: "right_hand",
        TrackerRole.LEFT_HAND: "left_hand",
        TrackerRole.WAIST: "waist",
        TrackerRole.RIGHT_FOOT: "right_foot",
        TrackerRole.LEFT_FOOT: "left_foot",
        TrackerRole.HEAD: "head",
    }

    for role in mapped_motion.roles:
        name = role_names.get(role, role.name)
        positions = np.array([
            f.poses[role].position for f in mapped_motion.frames
            if role in f.poses
        ])

        if len(positions) < 4:
            continue

        # Workspace utilization
        ws = compute_workspace_utilization(positions)
        metrics[f"{name}_workspace"] = ws

        # Motion smoothness
        smooth = compute_smoothness(positions, dt=mapped_motion.frame_time)
        metrics[f"{name}_smoothness"] = smooth

        print(f"\n--- {name} ---")
        print(f"  Range: X={ws['range_xyz'][0]:.3f}m, Y={ws['range_xyz'][1]:.3f}m, Z={ws['range_xyz'][2]:.3f}m")
        print(f"  Smoothness (dim. jerk): {smooth['dimensionless_jerk']:.1f}")

    # Generate report
    if args.output:
        report = generate_report(metrics, output_path=args.output)
        print(f"\nReport saved: {args.output}")
    else:
        report = generate_report(metrics)
        print(f"\n{report}")

    # Save trajectory plots
    if args.save_plots:
        from teleop_system.mocap.dual_viewer import save_comparison_plot

        plots_dir = Path(args.save_plots)
        plots_dir.mkdir(parents=True, exist_ok=True)

        for role in mapped_motion.roles:
            name = role_names.get(role, role.name)
            positions = np.array([
                f.poses[role].position for f in mapped_motion.frames
                if role in f.poses
            ])
            if len(positions) > 0:
                # Plot position vs time
                save_comparison_plot(
                    positions, positions,
                    labels=("Position", "Position"),
                    title=f"{name} Trajectory",
                    output_path=plots_dir / f"{name}_trajectory.png",
                )
        print(f"\nPlots saved to: {plots_dir}")

    # Open viewers
    if args.view:
        print("\nOpening skeleton viewer...")
        bvh_data_full = load_bvh(str(bvh_path), scale=args.scale)
        from teleop_system.mocap.skeleton_viewer import view_skeleton
        view_skeleton(bvh_data_full)

    if args.dual_view:
        print("\nOpening dual viewer...")
        bvh_data_full = load_bvh(str(bvh_path), scale=args.scale)
        from teleop_system.mocap.dual_viewer import view_dual
        view_dual(bvh_data_full, mapped_motion)


if __name__ == "__main__":
    main()
