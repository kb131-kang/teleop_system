"""A-Pose calibration system for tracker-to-robot offset computation.

Implements a state machine that captures tracker positions while the
operator holds an A-Pose, computes position offsets to align tracker
coordinates with robot reference positions.

State machine: IDLE -> WAITING -> CAPTURING -> COMPUTING -> CALIBRATED | ERROR
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path

import numpy as np
import yaml

from teleop_system.interfaces.master_device import Pose6D, TrackerRole
from teleop_system.utils.logger import get_logger

logger = get_logger("pose_calibrator")


class CalibrationState(Enum):
    """Calibration state machine states."""
    IDLE = auto()
    WAITING = auto()       # Countdown before capture
    CAPTURING = auto()     # Recording tracker positions
    COMPUTING = auto()     # Computing offsets
    CALIBRATED = auto()    # Offsets ready
    ERROR = auto()         # Calibration failed


@dataclass
class CalibrationConfig:
    """Configuration for the calibration process."""
    countdown_sec: float = 3.0
    capture_duration_sec: float = 1.0
    capture_rate_hz: float = 100.0
    reference_positions: dict[TrackerRole, np.ndarray] = field(default_factory=dict)

    @classmethod
    def from_yaml(cls, path: str | Path) -> CalibrationConfig:
        """Load configuration from a YAML file."""
        path = Path(path)
        if not path.exists():
            logger.warning(f"Config file not found: {path}, using defaults")
            return cls._default()

        with open(path) as f:
            data = yaml.safe_load(f)

        ref = data.get("a_pose_reference", {})
        cal = data.get("calibration", {})

        role_map = {
            "right_hand": TrackerRole.RIGHT_HAND,
            "left_hand": TrackerRole.LEFT_HAND,
            "waist": TrackerRole.WAIST,
            "right_foot": TrackerRole.RIGHT_FOOT,
            "left_foot": TrackerRole.LEFT_FOOT,
            "head": TrackerRole.HEAD,
        }

        reference_positions = {}
        for key, role in role_map.items():
            if key in ref:
                reference_positions[role] = np.array(ref[key], dtype=np.float64)

        return cls(
            countdown_sec=cal.get("countdown_sec", 3.0),
            capture_duration_sec=cal.get("capture_duration_sec", 1.0),
            capture_rate_hz=cal.get("capture_rate_hz", 100.0),
            reference_positions=reference_positions,
        )

    @classmethod
    def _default(cls) -> CalibrationConfig:
        """Create default configuration with reasonable reference positions."""
        return cls(
            reference_positions={
                TrackerRole.RIGHT_HAND: np.array([0.25, -0.35, 0.85]),
                TrackerRole.LEFT_HAND: np.array([-0.25, -0.35, 0.85]),
                TrackerRole.WAIST: np.array([0.0, 0.0, 0.95]),
                TrackerRole.RIGHT_FOOT: np.array([0.15, 0.0, 0.0]),
                TrackerRole.LEFT_FOOT: np.array([-0.15, 0.0, 0.0]),
                TrackerRole.HEAD: np.array([0.0, 0.0, 1.55]),
            },
        )


@dataclass
class TrackerCalibrationResult:
    """Result of calibration for a single tracker."""
    role: TrackerRole
    offset: np.ndarray          # Position offset to apply
    captured_mean: np.ndarray   # Mean captured position
    reference: np.ndarray       # Robot reference position
    sample_count: int           # Number of samples captured
    std_dev: float              # Standard deviation of captured positions


class PoseCalibrator:
    """A-Pose calibration: captures tracker positions and computes offsets.

    Usage:
        calibrator = PoseCalibrator(config)
        calibrator.start_calibration()
        # In loop:
        calibrator.update(current_poses)
        # When state == CALIBRATED:
        offsets = calibrator.get_offsets()
    """

    def __init__(self, config: CalibrationConfig | None = None):
        if config is None:
            config = CalibrationConfig._default()
        self._config = config
        self._state = CalibrationState.IDLE
        self._error_msg = ""

        # Timing
        self._countdown_sec = config.countdown_sec
        self._capture_duration_sec = config.capture_duration_sec
        self._state_start_time: float = 0.0

        # Captured data: role -> list of position arrays
        self._captured: dict[TrackerRole, list[np.ndarray]] = {}

        # Results
        self._results: dict[TrackerRole, TrackerCalibrationResult] = {}
        self._offsets: dict[TrackerRole, np.ndarray] = {}

    @property
    def state(self) -> CalibrationState:
        return self._state

    @property
    def error_message(self) -> str:
        return self._error_msg

    @property
    def progress(self) -> float:
        """Return progress as 0.0-1.0 based on current state."""
        now = time.monotonic()
        elapsed = now - self._state_start_time

        if self._state == CalibrationState.IDLE:
            return 0.0
        elif self._state == CalibrationState.WAITING:
            return min(elapsed / self._countdown_sec, 1.0) * 0.3
        elif self._state == CalibrationState.CAPTURING:
            return 0.3 + min(elapsed / self._capture_duration_sec, 1.0) * 0.5
        elif self._state == CalibrationState.COMPUTING:
            return 0.8
        elif self._state == CalibrationState.CALIBRATED:
            return 1.0
        else:  # ERROR
            return 0.0

    @property
    def countdown_remaining(self) -> float:
        """Seconds remaining in countdown, or 0 if not in WAITING state."""
        if self._state != CalibrationState.WAITING:
            return 0.0
        elapsed = time.monotonic() - self._state_start_time
        return max(0.0, self._countdown_sec - elapsed)

    def start_calibration(self, countdown_sec: float | None = None) -> bool:
        """Start the calibration process.

        Args:
            countdown_sec: Override countdown duration. None uses config value.

        Returns:
            True if calibration started successfully.
        """
        if self._state not in (CalibrationState.IDLE, CalibrationState.CALIBRATED,
                                CalibrationState.ERROR):
            logger.warning(f"Cannot start calibration in state {self._state.name}")
            return False

        if countdown_sec is not None:
            self._countdown_sec = countdown_sec
        else:
            self._countdown_sec = self._config.countdown_sec

        self._captured = {role: [] for role in self._config.reference_positions}
        self._results = {}
        self._offsets = {}
        self._error_msg = ""

        self._state = CalibrationState.WAITING
        self._state_start_time = time.monotonic()
        logger.info(f"Calibration started, countdown: {self._countdown_sec:.1f}s")
        return True

    def update(self, current_poses: dict[TrackerRole, Pose6D]) -> CalibrationState:
        """Tick the calibration state machine.

        Call this at the tracker update rate with the latest poses.

        Args:
            current_poses: Current tracker poses keyed by role.

        Returns:
            Current calibration state after update.
        """
        now = time.monotonic()
        elapsed = now - self._state_start_time

        if self._state == CalibrationState.WAITING:
            if elapsed >= self._countdown_sec:
                self._state = CalibrationState.CAPTURING
                self._state_start_time = now
                logger.info("Countdown complete, capturing poses...")

        elif self._state == CalibrationState.CAPTURING:
            # Record valid poses
            for role in self._captured:
                if role in current_poses and current_poses[role].valid:
                    self._captured[role].append(
                        current_poses[role].position.copy()
                    )

            if elapsed >= self._capture_duration_sec:
                self._state = CalibrationState.COMPUTING
                self._state_start_time = now
                logger.info("Capture complete, computing offsets...")
                self._compute_offsets()

        return self._state

    def _compute_offsets(self) -> None:
        """Compute calibration offsets from captured data."""
        min_samples = 5  # Minimum samples for valid calibration

        for role, samples in self._captured.items():
            if len(samples) < min_samples:
                self._state = CalibrationState.ERROR
                self._error_msg = (
                    f"Insufficient samples for {role.name}: "
                    f"got {len(samples)}, need {min_samples}"
                )
                logger.error(self._error_msg)
                return

            positions = np.array(samples)
            mean_pos = positions.mean(axis=0)
            std_dev = np.linalg.norm(positions.std(axis=0))
            reference = self._config.reference_positions[role]
            offset = reference - mean_pos

            self._results[role] = TrackerCalibrationResult(
                role=role,
                offset=offset,
                captured_mean=mean_pos,
                reference=reference,
                sample_count=len(samples),
                std_dev=std_dev,
            )
            self._offsets[role] = offset

        self._state = CalibrationState.CALIBRATED
        logger.info(
            f"Calibration complete. Roles calibrated: "
            f"{[r.name for r in self._offsets]}"
        )

        for role, result in self._results.items():
            logger.info(
                f"  {role.name}: offset={result.offset}, "
                f"std={result.std_dev:.4f}, samples={result.sample_count}"
            )

    def get_offsets(self) -> dict[TrackerRole, np.ndarray]:
        """Get computed position offsets.

        Returns:
            Dict mapping TrackerRole to 3D position offset arrays.
            Empty if not calibrated.
        """
        return dict(self._offsets)

    def get_results(self) -> dict[TrackerRole, TrackerCalibrationResult]:
        """Get detailed calibration results per tracker."""
        return dict(self._results)

    def apply_offset(self, role: TrackerRole, position: np.ndarray) -> np.ndarray:
        """Apply calibration offset to a tracker position.

        Args:
            role: Tracker role.
            position: Raw tracker position (3,).

        Returns:
            Calibrated position with offset applied.
        """
        if role in self._offsets:
            return position + self._offsets[role]
        return position.copy()

    def reset(self) -> None:
        """Reset calibrator to IDLE state, clearing all data."""
        self._state = CalibrationState.IDLE
        self._captured = {}
        self._results = {}
        self._offsets = {}
        self._error_msg = ""
        self._state_start_time = 0.0
        logger.info("Calibration reset")
