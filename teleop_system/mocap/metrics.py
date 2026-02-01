"""Motion replay metrics and analysis tools.

Provides functions for evaluating how well the teleoperation pipeline
reproduces motion capture input data. Metrics include tracking error,
velocity saturation, workspace utilization, and motion smoothness.
"""

import json
from pathlib import Path

import numpy as np

from teleop_system.utils.logger import get_logger

logger = get_logger("mocap_metrics")


def compute_tracking_error(
    input_positions: np.ndarray,
    output_positions: np.ndarray,
) -> dict:
    """Compute position tracking error between input and output trajectories.

    Args:
        input_positions: (N, 3) input trajectory positions.
        output_positions: (N, 3) output trajectory positions.

    Returns:
        Dict with 'rmse', 'max_error', 'mean_error', 'per_frame_error' keys.
    """
    assert input_positions.shape == output_positions.shape, \
        f"Shape mismatch: {input_positions.shape} vs {output_positions.shape}"

    errors = np.linalg.norm(input_positions - output_positions, axis=1)

    return {
        "rmse": float(np.sqrt(np.mean(errors ** 2))),
        "max_error": float(np.max(errors)),
        "mean_error": float(np.mean(errors)),
        "per_frame_error": errors,
    }


def compute_orientation_error(
    input_quats: np.ndarray,
    output_quats: np.ndarray,
) -> dict:
    """Compute orientation tracking error between quaternion trajectories.

    Uses geodesic distance on SO(3): angle = 2 * arccos(|q1 · q2|).

    Args:
        input_quats: (N, 4) input quaternions (xyzw).
        output_quats: (N, 4) output quaternions (xyzw).

    Returns:
        Dict with 'rmse_rad', 'rmse_deg', 'max_error_deg', 'mean_error_deg'.
    """
    assert input_quats.shape == output_quats.shape

    dots = np.abs(np.sum(input_quats * output_quats, axis=1))
    dots = np.clip(dots, 0.0, 1.0)
    angles = 2.0 * np.arccos(dots)

    return {
        "rmse_rad": float(np.sqrt(np.mean(angles ** 2))),
        "rmse_deg": float(np.degrees(np.sqrt(np.mean(angles ** 2)))),
        "max_error_deg": float(np.degrees(np.max(angles))),
        "mean_error_deg": float(np.degrees(np.mean(angles))),
        "per_frame_error_rad": angles,
    }


def compute_velocity_saturation(
    velocities: np.ndarray,
    max_velocity: float,
    threshold: float = 0.95,
) -> dict:
    """Compute fraction of frames where velocity is near the limit.

    Args:
        velocities: (N,) or (N, D) velocity values.
        max_velocity: Maximum allowed velocity.
        threshold: Fraction of max_velocity considered "saturated".

    Returns:
        Dict with 'saturation_ratio', 'mean_velocity', 'max_velocity_observed'.
    """
    if velocities.ndim > 1:
        speed = np.linalg.norm(velocities, axis=1)
    else:
        speed = np.abs(velocities)

    saturated = np.sum(speed >= threshold * max_velocity)

    return {
        "saturation_ratio": float(saturated / max(len(speed), 1)),
        "mean_velocity": float(np.mean(speed)),
        "max_velocity_observed": float(np.max(speed)),
        "limit": float(max_velocity),
    }


def compute_workspace_utilization(
    positions: np.ndarray,
    workspace_bounds: tuple[np.ndarray, np.ndarray] | None = None,
) -> dict:
    """Compute workspace coverage of a trajectory.

    Args:
        positions: (N, 3) trajectory positions.
        workspace_bounds: (min_xyz, max_xyz) workspace limits. If None,
            uses the trajectory's own bounding box.

    Returns:
        Dict with 'range_xyz', 'volume', 'center', 'min', 'max'.
    """
    pos_min = np.min(positions, axis=0)
    pos_max = np.max(positions, axis=0)
    pos_range = pos_max - pos_min
    center = (pos_min + pos_max) / 2

    volume = float(np.prod(pos_range))

    result = {
        "range_xyz": pos_range.tolist(),
        "volume": volume,
        "center": center.tolist(),
        "min": pos_min.tolist(),
        "max": pos_max.tolist(),
    }

    if workspace_bounds is not None:
        ws_min, ws_max = workspace_bounds
        ws_range = ws_max - ws_min
        ws_volume = float(np.prod(ws_range))
        result["utilization_ratio"] = volume / max(ws_volume, 1e-10)

    return result


def compute_smoothness(trajectory: np.ndarray, dt: float) -> dict:
    """Compute motion smoothness using jerk metric.

    Lower jerk = smoother motion. Uses dimensionless jerk metric:
    DJ = sqrt(0.5 * integral(jerk^2) * duration^5 / path_length^2)

    Args:
        trajectory: (N, 3) position trajectory.
        dt: Time step between frames.

    Returns:
        Dict with 'dimensionless_jerk', 'mean_jerk', 'max_jerk'.
    """
    if len(trajectory) < 4:
        return {
            "dimensionless_jerk": 0.0,
            "mean_jerk": 0.0,
            "max_jerk": 0.0,
        }

    # Velocity (first derivative)
    vel = np.diff(trajectory, axis=0) / dt
    # Acceleration (second derivative)
    acc = np.diff(vel, axis=0) / dt
    # Jerk (third derivative)
    jerk = np.diff(acc, axis=0) / dt

    jerk_magnitude = np.linalg.norm(jerk, axis=1)

    # Path length
    displacements = np.diff(trajectory, axis=0)
    path_length = np.sum(np.linalg.norm(displacements, axis=1))

    # Duration
    duration = len(trajectory) * dt

    # Dimensionless jerk
    jerk_integral = np.sum(jerk_magnitude ** 2) * dt
    if path_length > 1e-6:
        dj = np.sqrt(0.5 * jerk_integral * duration ** 5 / path_length ** 2)
    else:
        dj = 0.0

    return {
        "dimensionless_jerk": float(dj),
        "mean_jerk": float(np.mean(jerk_magnitude)),
        "max_jerk": float(np.max(jerk_magnitude)),
    }


def generate_report(
    metrics: dict,
    output_path: str | Path | None = None,
) -> str:
    """Generate a human-readable metrics report.

    Args:
        metrics: Dict of metric name → metric dict.
        output_path: Optional file path to save JSON report.

    Returns:
        Formatted report string.
    """
    lines = ["=" * 60, "Motion Replay Metrics Report", "=" * 60, ""]

    for name, values in metrics.items():
        lines.append(f"--- {name} ---")
        for key, val in values.items():
            if isinstance(val, np.ndarray):
                continue  # Skip per-frame arrays in report
            if isinstance(val, float):
                lines.append(f"  {key}: {val:.6f}")
            elif isinstance(val, list):
                lines.append(f"  {key}: {val}")
            else:
                lines.append(f"  {key}: {val}")
        lines.append("")

    report = "\n".join(lines)

    if output_path is not None:
        output_path = Path(output_path)
        # Save JSON (excluding numpy arrays)
        serializable = {}
        for name, values in metrics.items():
            serializable[name] = {
                k: v for k, v in values.items()
                if not isinstance(v, np.ndarray)
            }
        with open(output_path, "w") as f:
            json.dump(serializable, f, indent=2)
        logger.info(f"Report saved to {output_path}")

    return report
