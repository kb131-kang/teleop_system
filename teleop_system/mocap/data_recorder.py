"""Data recorder for capturing teleoperation input/output during replay.

Records master tracker input, slave joint commands, and base velocity
commands to numpy archive (.npz) files for offline analysis. Can be
used both as a standalone recorder and as part of the ROS2 pipeline.
"""

import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from teleop_system.utils.logger import get_logger

logger = get_logger("data_recorder")


@dataclass
class RecordingBuffer:
    """In-memory buffer for recorded data.

    Attributes:
        timestamps: List of timestamps.
        data: Dict of channel_name → list of numpy arrays.
    """
    timestamps: list = field(default_factory=list)
    data: dict = field(default_factory=dict)


class DataRecorder:
    """Records teleoperation data channels to numpy archives.

    Usage (standalone, no ROS2):
        recorder = DataRecorder()
        recorder.start()
        for frame in frames:
            recorder.record("input/right_hand_pos", position)
            recorder.record("input/right_hand_quat", orientation)
            recorder.record("output/arm_right_cmd", joint_cmd)
            recorder.tick()
        recorder.save("recording.npz")

    Each recorded channel becomes a 2D array in the .npz file:
    channel_name → (N_frames, D) where D is the data dimension.
    """

    def __init__(self):
        self._buffer = RecordingBuffer()
        self._recording = False
        self._start_time = 0.0
        self._frame_count = 0

    def start(self) -> None:
        """Start recording."""
        self._buffer = RecordingBuffer()
        self._start_time = time.monotonic()
        self._recording = True
        self._frame_count = 0
        logger.info("Recording started")

    def stop(self) -> None:
        """Stop recording."""
        self._recording = False
        logger.info(f"Recording stopped: {self._frame_count} frames")

    def is_recording(self) -> bool:
        return self._recording

    def record(self, channel: str, data: np.ndarray) -> None:
        """Record a data sample to a named channel.

        Args:
            channel: Channel name (e.g., 'input/right_hand_pos').
            data: Data array to record.
        """
        if not self._recording:
            return

        if channel not in self._buffer.data:
            self._buffer.data[channel] = []

        self._buffer.data[channel].append(data.copy() if isinstance(data, np.ndarray) else np.array(data))

    def tick(self) -> None:
        """Mark end of a frame (records timestamp)."""
        if not self._recording:
            return
        self._buffer.timestamps.append(time.monotonic() - self._start_time)
        self._frame_count += 1

    def save(self, output_path: str | Path) -> Path:
        """Save recorded data to a numpy archive (.npz).

        Args:
            output_path: Path for the output file.

        Returns:
            Path to the saved file.
        """
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        save_dict = {
            "timestamps": np.array(self._buffer.timestamps),
        }

        for channel, samples in self._buffer.data.items():
            if samples:
                # Stack samples into 2D array
                try:
                    save_dict[channel] = np.stack(samples)
                except ValueError:
                    logger.warning(
                        f"Channel '{channel}' has inconsistent shapes, "
                        f"saving as object array"
                    )
                    save_dict[channel] = np.array(samples, dtype=object)

        np.savez_compressed(output_path, **save_dict)
        logger.info(
            f"Saved recording: {output_path} "
            f"({self._frame_count} frames, {len(self._buffer.data)} channels)"
        )
        return output_path

    def get_channel_data(self, channel: str) -> np.ndarray | None:
        """Get recorded data for a channel as a stacked array."""
        if channel not in self._buffer.data:
            return None
        samples = self._buffer.data[channel]
        if not samples:
            return None
        return np.stack(samples)

    def get_timestamps(self) -> np.ndarray:
        """Get recorded timestamps."""
        return np.array(self._buffer.timestamps)

    def get_channels(self) -> list[str]:
        """Get list of recorded channel names."""
        return list(self._buffer.data.keys())

    @property
    def frame_count(self) -> int:
        return self._frame_count
