#!/usr/bin/env python3
"""Parameter sweep tool for optimizing teleoperation tuning.

Tests different parameter combinations against BVH motion data and
compares metrics to find optimal settings.

Usage:
    python3 scripts/run_parameter_sweep.py --bvh data/bvh/cmu/002/02_01.bvh
    python3 scripts/run_parameter_sweep.py --bvh data/bvh/cmu/002/02_01.bvh --output sweep_results.json
"""

import argparse
import itertools
import json
import sys
from pathlib import Path

# Add project root to path so `teleop_system` is importable when running as script
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np


# Default parameter ranges to sweep
DEFAULT_SWEEP_PARAMS = {
    "scale": [0.04, 0.050, 0.056, 0.065, 0.075],
}


def run_single_config(
    bvh_path: str,
    scale: float,
    normalize_mode: str = "relative",
) -> dict:
    """Run replay with a single parameter configuration and return metrics.

    Args:
        bvh_path: Path to BVH file.
        scale: BVH→meters scale factor.
        normalize_mode: Position normalization mode.

    Returns:
        Dict of metric values.
    """
    from teleop_system.mocap.bvh_loader import load_bvh_lazy
    from teleop_system.mocap.skeleton_mapper import SkeletonMapper
    from teleop_system.mocap.metrics import (
        compute_workspace_utilization,
        compute_smoothness,
    )
    from teleop_system.interfaces.master_device import TrackerRole

    target_joints = ["Hips", "Head", "LeftHand", "RightHand", "LeftFoot", "RightFoot"]
    bvh_data = load_bvh_lazy(bvh_path, scale=scale, joints=target_joints)
    mapper = SkeletonMapper(normalize_mode=normalize_mode)
    mapped = mapper.map(bvh_data)

    results = {"scale": scale, "normalize_mode": normalize_mode}

    role_names = {
        TrackerRole.RIGHT_HAND: "right_hand",
        TrackerRole.LEFT_HAND: "left_hand",
        TrackerRole.WAIST: "waist",
        TrackerRole.HEAD: "head",
    }

    for role in [TrackerRole.RIGHT_HAND, TrackerRole.WAIST, TrackerRole.HEAD]:
        if role not in mapped.roles:
            continue

        name = role_names[role]
        positions = np.array([
            f.poses[role].position for f in mapped.frames
            if role in f.poses
        ])

        if len(positions) < 4:
            continue

        ws = compute_workspace_utilization(positions)
        smooth = compute_smoothness(positions, dt=mapped.frame_time)

        results[f"{name}_range_x"] = ws["range_xyz"][0]
        results[f"{name}_range_y"] = ws["range_xyz"][1]
        results[f"{name}_range_z"] = ws["range_xyz"][2]
        results[f"{name}_jerk"] = smooth["dimensionless_jerk"]

        # Position statistics
        results[f"{name}_mean_z"] = float(np.mean(positions[:, 2]))

    return results


def main():
    parser = argparse.ArgumentParser(description="Parameter sweep for teleop tuning")
    parser.add_argument(
        "--bvh", type=str, required=True,
        help="Path to BVH file",
    )
    parser.add_argument(
        "--output", type=str, default=None,
        help="Output JSON file for sweep results",
    )
    parser.add_argument(
        "--scales", type=float, nargs="+",
        default=None,
        help="Scale values to sweep (overrides defaults)",
    )

    args = parser.parse_args()

    bvh_path = Path(args.bvh)
    if not bvh_path.exists():
        print(f"Error: BVH file not found: {bvh_path}")
        sys.exit(1)

    scales = args.scales or DEFAULT_SWEEP_PARAMS["scale"]

    print(f"Parameter Sweep")
    print(f"BVH: {bvh_path}")
    print(f"Scales: {scales}")
    print(f"Total configs: {len(scales)}")
    print()

    all_results = []

    for i, scale in enumerate(scales):
        print(f"[{i + 1}/{len(scales)}] scale={scale}")
        try:
            result = run_single_config(str(bvh_path), scale=scale)
            all_results.append(result)

            # Print key metrics
            for key in sorted(result.keys()):
                if key in ("scale", "normalize_mode"):
                    continue
                print(f"  {key}: {result[key]:.4f}")
            print()
        except Exception as e:
            print(f"  Error: {e}")
            print()

    # Print comparison table
    if all_results:
        print("\n" + "=" * 70)
        print("COMPARISON TABLE")
        print("=" * 70)

        # Get all metric keys
        metric_keys = sorted(set(
            k for r in all_results for k in r.keys()
            if k not in ("scale", "normalize_mode")
        ))

        # Header
        header = f"{'scale':>8}"
        for key in metric_keys[:6]:  # Limit columns
            short = key.replace("right_hand_", "rh_").replace("waist_", "w_").replace("head_", "h_")
            header += f"  {short:>12}"
        print(header)
        print("-" * len(header))

        # Rows
        for r in all_results:
            row = f"{r['scale']:>8.4f}"
            for key in metric_keys[:6]:
                val = r.get(key, float("nan"))
                row += f"  {val:>12.4f}"
            print(row)

    # Save results
    if args.output:
        with open(args.output, "w") as f:
            json.dump(all_results, f, indent=2)
        print(f"\nResults saved: {args.output}")


if __name__ == "__main__":
    main()
