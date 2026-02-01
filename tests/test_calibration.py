"""Tests for the A-Pose calibration system."""

import time
from unittest.mock import patch

import numpy as np
import pytest

from teleop_system.calibration.pose_calibrator import (
    CalibrationConfig,
    CalibrationState,
    PoseCalibrator,
    TrackerCalibrationResult,
)
from teleop_system.interfaces.master_device import Pose6D, TrackerRole


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def default_config():
    """Config with short timings for fast tests."""
    return CalibrationConfig(
        countdown_sec=0.05,
        capture_duration_sec=0.1,
        capture_rate_hz=100.0,
        reference_positions={
            TrackerRole.RIGHT_HAND: np.array([0.25, -0.35, 0.85]),
            TrackerRole.LEFT_HAND: np.array([-0.25, -0.35, 0.85]),
            TrackerRole.WAIST: np.array([0.0, 0.0, 0.95]),
            TrackerRole.RIGHT_FOOT: np.array([0.15, 0.0, 0.0]),
            TrackerRole.LEFT_FOOT: np.array([-0.15, 0.0, 0.0]),
            TrackerRole.HEAD: np.array([0.0, 0.0, 1.55]),
        },
    )


@pytest.fixture
def calibrator(default_config):
    return PoseCalibrator(default_config)


def _make_poses(positions: dict[TrackerRole, np.ndarray]) -> dict[TrackerRole, Pose6D]:
    """Create Pose6D dict from position dict."""
    return {
        role: Pose6D(position=pos.copy(), valid=True)
        for role, pos in positions.items()
    }


# ---------------------------------------------------------------------------
# State machine tests
# ---------------------------------------------------------------------------

class TestCalibrationStateMachine:
    """Test state transitions of PoseCalibrator."""

    def test_initial_state_is_idle(self, calibrator):
        assert calibrator.state == CalibrationState.IDLE

    def test_start_transitions_to_waiting(self, calibrator):
        assert calibrator.start_calibration()
        assert calibrator.state == CalibrationState.WAITING

    def test_cannot_start_while_capturing(self, calibrator):
        calibrator.start_calibration(countdown_sec=0.0)
        # Force into CAPTURING by updating past countdown
        time.sleep(0.01)
        calibrator.update({})
        assert calibrator.state == CalibrationState.CAPTURING
        assert not calibrator.start_calibration()

    def test_can_restart_after_calibrated(self, default_config):
        cal = PoseCalibrator(default_config)
        # Run through full cycle
        _run_full_calibration(cal, default_config)
        assert cal.state == CalibrationState.CALIBRATED
        # Should be able to restart
        assert cal.start_calibration()
        assert cal.state == CalibrationState.WAITING

    def test_can_restart_after_error(self, default_config):
        cal = PoseCalibrator(default_config)
        cal.start_calibration(countdown_sec=0.0)
        time.sleep(0.01)
        # Update with empty poses to trigger capture
        cal.update({})
        assert cal.state == CalibrationState.CAPTURING
        # Wait for capture to end with insufficient samples
        time.sleep(0.15)
        cal.update({})
        assert cal.state == CalibrationState.ERROR
        # Should be able to restart
        assert cal.start_calibration()
        assert cal.state == CalibrationState.WAITING

    def test_reset_returns_to_idle(self, calibrator):
        calibrator.start_calibration()
        calibrator.reset()
        assert calibrator.state == CalibrationState.IDLE
        assert calibrator.get_offsets() == {}

    def test_full_cycle(self, default_config):
        cal = PoseCalibrator(default_config)
        _run_full_calibration(cal, default_config)
        assert cal.state == CalibrationState.CALIBRATED


class TestCountdown:
    """Test countdown and timing behavior."""

    def test_countdown_remaining(self, calibrator):
        calibrator.start_calibration(countdown_sec=1.0)
        remaining = calibrator.countdown_remaining
        assert 0.8 < remaining <= 1.0

    def test_countdown_zero_when_not_waiting(self, calibrator):
        assert calibrator.countdown_remaining == 0.0

    def test_progress_increases(self, calibrator):
        assert calibrator.progress == 0.0
        calibrator.start_calibration(countdown_sec=0.05)
        time.sleep(0.03)
        p1 = calibrator.progress
        assert p1 > 0.0


# ---------------------------------------------------------------------------
# Offset computation tests
# ---------------------------------------------------------------------------

class TestOffsetComputation:
    """Test calibration offset calculation."""

    def test_offset_is_reference_minus_captured(self, default_config):
        cal = PoseCalibrator(default_config)

        # Simulate known captured positions (offset from reference by known amount)
        captured_offset = np.array([0.1, 0.2, 0.3])
        poses = {}
        for role, ref in default_config.reference_positions.items():
            poses[role] = Pose6D(position=ref - captured_offset, valid=True)

        _run_full_calibration(cal, default_config, override_poses=poses)

        offsets = cal.get_offsets()
        for role in default_config.reference_positions:
            np.testing.assert_allclose(
                offsets[role], captured_offset, atol=1e-10,
                err_msg=f"Offset mismatch for {role.name}",
            )

    def test_zero_offset_when_at_reference(self, default_config):
        cal = PoseCalibrator(default_config)

        # Captured positions exactly match reference
        poses = _make_poses(default_config.reference_positions)
        _run_full_calibration(cal, default_config, override_poses=poses)

        offsets = cal.get_offsets()
        for role in default_config.reference_positions:
            np.testing.assert_allclose(
                offsets[role], np.zeros(3), atol=1e-10,
            )

    def test_apply_offset(self, default_config):
        cal = PoseCalibrator(default_config)
        captured_offset = np.array([0.05, -0.1, 0.15])
        poses = {}
        for role, ref in default_config.reference_positions.items():
            poses[role] = Pose6D(position=ref - captured_offset, valid=True)

        _run_full_calibration(cal, default_config, override_poses=poses)

        # Apply offset should return reference position
        test_pos = np.array([1.0, 2.0, 3.0])
        calibrated = cal.apply_offset(TrackerRole.RIGHT_HAND, test_pos)
        expected = test_pos + captured_offset
        np.testing.assert_allclose(calibrated, expected, atol=1e-10)

    def test_apply_offset_uncalibrated_returns_copy(self, calibrator):
        pos = np.array([1.0, 2.0, 3.0])
        result = calibrator.apply_offset(TrackerRole.RIGHT_HAND, pos)
        np.testing.assert_array_equal(result, pos)
        # Should be a copy, not the same object
        assert result is not pos

    def test_results_contain_sample_count(self, default_config):
        cal = PoseCalibrator(default_config)
        _run_full_calibration(cal, default_config)
        results = cal.get_results()
        for role, result in results.items():
            assert isinstance(result, TrackerCalibrationResult)
            assert result.sample_count >= 5
            assert result.std_dev >= 0.0


class TestErrorHandling:
    """Test error conditions."""

    def test_insufficient_samples_causes_error(self, default_config):
        cal = PoseCalibrator(default_config)
        cal.start_calibration(countdown_sec=0.0)
        time.sleep(0.01)
        # Update once to transition to CAPTURING, but don't provide enough samples
        cal.update({})
        assert cal.state == CalibrationState.CAPTURING

        # Wait for capture to end
        time.sleep(0.15)
        cal.update({})
        assert cal.state == CalibrationState.ERROR
        assert "Insufficient" in cal.error_message

    def test_invalid_poses_not_captured(self, default_config):
        cal = PoseCalibrator(default_config)
        cal.start_calibration(countdown_sec=0.0)
        time.sleep(0.01)
        cal.update({})  # → CAPTURING

        # Feed invalid poses
        invalid_poses = {
            role: Pose6D(position=np.zeros(3), valid=False)
            for role in default_config.reference_positions
        }
        for _ in range(20):
            cal.update(invalid_poses)

        time.sleep(0.15)
        cal.update(invalid_poses)
        assert cal.state == CalibrationState.ERROR

    def test_partial_poses_cause_error(self, default_config):
        """If some roles have data but others don't, should error."""
        cal = PoseCalibrator(default_config)
        cal.start_calibration(countdown_sec=0.0)
        time.sleep(0.01)
        cal.update({})  # → CAPTURING

        # Only provide poses for one role
        partial_poses = {
            TrackerRole.RIGHT_HAND: Pose6D(
                position=np.array([0.25, -0.35, 0.85]), valid=True
            )
        }
        for _ in range(20):
            cal.update(partial_poses)

        time.sleep(0.15)
        cal.update(partial_poses)
        assert cal.state == CalibrationState.ERROR


# ---------------------------------------------------------------------------
# Config loading tests
# ---------------------------------------------------------------------------

class TestCalibrationConfig:
    """Test configuration loading."""

    def test_default_config_has_all_roles(self):
        config = CalibrationConfig._default()
        expected_roles = {
            TrackerRole.RIGHT_HAND, TrackerRole.LEFT_HAND,
            TrackerRole.WAIST, TrackerRole.RIGHT_FOOT,
            TrackerRole.LEFT_FOOT, TrackerRole.HEAD,
        }
        assert set(config.reference_positions.keys()) == expected_roles

    def test_from_yaml(self, tmp_path):
        yaml_content = """
a_pose_reference:
  right_hand: [0.3, -0.4, 0.9]
  left_hand: [-0.3, -0.4, 0.9]
  waist: [0.0, 0.0, 1.0]
  right_foot: [0.2, 0.0, 0.0]
  left_foot: [-0.2, 0.0, 0.0]
  head: [0.0, 0.0, 1.6]

calibration:
  countdown_sec: 5.0
  capture_duration_sec: 2.0
  capture_rate_hz: 50
"""
        yaml_file = tmp_path / "test_config.yaml"
        yaml_file.write_text(yaml_content)

        config = CalibrationConfig.from_yaml(yaml_file)
        assert config.countdown_sec == 5.0
        assert config.capture_duration_sec == 2.0
        assert config.capture_rate_hz == 50.0
        np.testing.assert_allclose(
            config.reference_positions[TrackerRole.RIGHT_HAND],
            [0.3, -0.4, 0.9],
        )

    def test_from_yaml_missing_file_uses_defaults(self):
        config = CalibrationConfig.from_yaml("/nonexistent/path.yaml")
        assert config.countdown_sec == 3.0
        assert len(config.reference_positions) == 6


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _run_full_calibration(
    cal: PoseCalibrator,
    config: CalibrationConfig,
    override_poses: dict[TrackerRole, Pose6D] | None = None,
) -> None:
    """Run a complete calibration cycle with the given calibrator."""
    if override_poses is None:
        # Use reference positions as captured (zero offset)
        override_poses = _make_poses(config.reference_positions)

    cal.start_calibration(countdown_sec=0.01)
    time.sleep(0.02)
    cal.update(override_poses)  # WAITING → CAPTURING

    # Feed enough samples during capture
    for _ in range(30):
        cal.update(override_poses)
        time.sleep(0.005)

    # Wait for capture to end and trigger compute
    time.sleep(0.1)
    cal.update(override_poses)
