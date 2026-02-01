"""Tests for motion replay metrics."""

import numpy as np
import pytest


class TestTrackingError:
    """Tests for tracking error computation."""

    def test_module_imports(self):
        from teleop_system.mocap.metrics import (
            compute_tracking_error,
            compute_orientation_error,
            compute_velocity_saturation,
            compute_workspace_utilization,
            compute_smoothness,
            generate_report,
        )
        assert compute_tracking_error is not None

    def test_zero_error(self):
        from teleop_system.mocap.metrics import compute_tracking_error

        positions = np.random.randn(100, 3)
        result = compute_tracking_error(positions, positions)

        assert result["rmse"] == pytest.approx(0.0, abs=1e-10)
        assert result["max_error"] == pytest.approx(0.0, abs=1e-10)
        assert result["mean_error"] == pytest.approx(0.0, abs=1e-10)

    def test_known_error(self):
        from teleop_system.mocap.metrics import compute_tracking_error

        input_pos = np.zeros((10, 3))
        output_pos = np.ones((10, 3))

        result = compute_tracking_error(input_pos, output_pos)

        expected_error = np.sqrt(3.0)  # |[1,1,1]| = sqrt(3)
        assert result["rmse"] == pytest.approx(expected_error, rel=1e-5)
        assert result["max_error"] == pytest.approx(expected_error, rel=1e-5)

    def test_per_frame_error_shape(self):
        from teleop_system.mocap.metrics import compute_tracking_error

        n = 50
        result = compute_tracking_error(
            np.random.randn(n, 3),
            np.random.randn(n, 3),
        )

        assert result["per_frame_error"].shape == (n,)


class TestOrientationError:
    """Tests for orientation error computation."""

    def test_identical_quaternions(self):
        from teleop_system.mocap.metrics import compute_orientation_error

        quats = np.tile([0, 0, 0, 1], (20, 1)).astype(float)
        result = compute_orientation_error(quats, quats)

        assert result["rmse_deg"] == pytest.approx(0.0, abs=1e-5)

    def test_opposite_quaternions_equivalent(self):
        """q and -q represent the same rotation."""
        from teleop_system.mocap.metrics import compute_orientation_error

        q1 = np.tile([0, 0, 0, 1], (10, 1)).astype(float)
        q2 = -q1  # Negated quaternion = same rotation

        result = compute_orientation_error(q1, q2)
        assert result["rmse_deg"] == pytest.approx(0.0, abs=1e-5)

    def test_90_degree_rotation(self):
        from teleop_system.mocap.metrics import compute_orientation_error

        # Identity
        q1 = np.tile([0, 0, 0, 1], (5, 1)).astype(float)
        # 90° around Z: quat = (0, 0, sin(45°), cos(45°))
        s = np.sin(np.pi / 4)
        c = np.cos(np.pi / 4)
        q2 = np.tile([0, 0, s, c], (5, 1)).astype(float)

        result = compute_orientation_error(q1, q2)
        assert result["mean_error_deg"] == pytest.approx(90.0, abs=1.0)


class TestVelocitySaturation:
    """Tests for velocity saturation computation."""

    def test_no_saturation(self):
        from teleop_system.mocap.metrics import compute_velocity_saturation

        velocities = np.full(100, 0.5)
        result = compute_velocity_saturation(velocities, max_velocity=10.0)

        assert result["saturation_ratio"] == pytest.approx(0.0)

    def test_full_saturation(self):
        from teleop_system.mocap.metrics import compute_velocity_saturation

        velocities = np.full(100, 9.6)  # > 0.95 * 10.0
        result = compute_velocity_saturation(velocities, max_velocity=10.0)

        assert result["saturation_ratio"] == pytest.approx(1.0)

    def test_partial_saturation(self):
        from teleop_system.mocap.metrics import compute_velocity_saturation

        velocities = np.concatenate([
            np.full(50, 1.0),   # Below threshold
            np.full(50, 9.8),   # Above threshold
        ])
        result = compute_velocity_saturation(velocities, max_velocity=10.0)

        assert result["saturation_ratio"] == pytest.approx(0.5)


class TestWorkspaceUtilization:
    """Tests for workspace utilization computation."""

    def test_single_point(self):
        from teleop_system.mocap.metrics import compute_workspace_utilization

        positions = np.tile([1, 2, 3], (10, 1)).astype(float)
        result = compute_workspace_utilization(positions)

        assert result["volume"] == pytest.approx(0.0)

    def test_unit_cube(self):
        from teleop_system.mocap.metrics import compute_workspace_utilization

        # Trajectory spanning a unit cube
        positions = np.array([
            [0, 0, 0],
            [1, 1, 1],
        ], dtype=float)
        result = compute_workspace_utilization(positions)

        assert result["volume"] == pytest.approx(1.0)
        np.testing.assert_allclose(result["range_xyz"], [1.0, 1.0, 1.0])

    def test_utilization_ratio(self):
        from teleop_system.mocap.metrics import compute_workspace_utilization

        positions = np.array([[0, 0, 0], [0.5, 0.5, 0.5]], dtype=float)
        bounds = (np.array([0, 0, 0]), np.array([1, 1, 1]))
        result = compute_workspace_utilization(positions, workspace_bounds=bounds)

        assert result["utilization_ratio"] == pytest.approx(0.125)


class TestSmoothness:
    """Tests for motion smoothness computation."""

    def test_constant_position(self):
        from teleop_system.mocap.metrics import compute_smoothness

        trajectory = np.tile([1, 2, 3], (100, 1)).astype(float)
        result = compute_smoothness(trajectory, dt=0.01)

        assert result["mean_jerk"] == pytest.approx(0.0, abs=1e-5)

    def test_linear_motion(self):
        """Linear motion should have zero jerk."""
        from teleop_system.mocap.metrics import compute_smoothness

        t = np.linspace(0, 1, 100)
        trajectory = np.column_stack([t, np.zeros(100), np.zeros(100)])
        result = compute_smoothness(trajectory, dt=0.01)

        assert result["mean_jerk"] == pytest.approx(0.0, abs=1e-3)

    def test_short_trajectory(self):
        from teleop_system.mocap.metrics import compute_smoothness

        trajectory = np.array([[0, 0, 0], [1, 0, 0]], dtype=float)
        result = compute_smoothness(trajectory, dt=0.01)

        # Should handle gracefully with < 4 points
        assert result["dimensionless_jerk"] == 0.0


class TestReport:
    """Tests for report generation."""

    def test_generate_report_string(self):
        from teleop_system.mocap.metrics import generate_report

        metrics = {
            "tracking_error": {
                "rmse": 0.05,
                "max_error": 0.12,
            },
            "smoothness": {
                "dimensionless_jerk": 15.3,
            },
        }

        report = generate_report(metrics)
        assert "tracking_error" in report
        assert "0.05" in report

    def test_generate_report_saves_json(self, tmp_path):
        from teleop_system.mocap.metrics import generate_report
        import json

        output = tmp_path / "report.json"
        metrics = {"test": {"value": 42.0}}

        generate_report(metrics, output_path=output)

        assert output.exists()
        with open(output) as f:
            loaded = json.load(f)
        assert loaded["test"]["value"] == 42.0
