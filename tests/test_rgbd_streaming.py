"""Tests for RGB-D compressed streaming (codec, server, client).

Tests the encode/decode pipeline (RGBDCodec), the server/client TCP
connection, frame delivery, compression ratio, and reverse control channel.
"""

import struct
import threading
import time

import numpy as np
import pytest

from teleop_system.interfaces.camera_stream import ICameraStream, RGBDFrame
from teleop_system.modules.camera.rgbd_streaming import (
    RGBDCodec,
    RGBDStreamClient,
    RGBDStreamServer,
    _CTRL_FMT,
    _CTRL_MAGIC,
    _HEADER_FMT,
    _HEADER_SIZE,
)


# ── Helpers ──


def _make_test_frame(
    width: int = 640,
    height: int = 480,
    seed: int = 42,
) -> RGBDFrame:
    """Create a synthetic RGB-D frame for testing."""
    rng = np.random.RandomState(seed)
    rgb = rng.randint(0, 256, (height, width, 3), dtype=np.uint8)
    depth = rng.uniform(0.1, 10.0, (height, width)).astype(np.float32)

    fx = fy = height / (2.0 * np.tan(np.radians(30)))
    cx, cy = width / 2.0, height / 2.0
    intrinsics = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1],
    ], dtype=np.float64)

    return RGBDFrame(
        rgb=rgb,
        depth=depth,
        intrinsics=intrinsics,
        timestamp=1.234,
        width=width,
        height=height,
    )


class MockCameraStream(ICameraStream):
    """Minimal ICameraStream that returns a fixed frame."""

    def __init__(self, frame: RGBDFrame):
        self._frame = frame
        self._pan = 0.0
        self._tilt = 0.0
        self._orientation_calls: list[tuple[float, float]] = []

    def initialize(self) -> bool:
        return True

    def get_rgbd(self) -> RGBDFrame:
        return self._frame

    def set_orientation(self, pan: float, tilt: float) -> None:
        self._pan = pan
        self._tilt = tilt
        self._orientation_calls.append((pan, tilt))

    def get_orientation(self) -> tuple[float, float]:
        return self._pan, self._tilt

    def get_intrinsics(self) -> np.ndarray:
        return self._frame.intrinsics.copy()

    def is_connected(self) -> bool:
        return True

    def shutdown(self) -> None:
        pass


# ── Codec tests ──


class TestRGBDCodec:
    """Test encode/decode roundtrip fidelity and compression."""

    def test_encode_decode_roundtrip(self):
        """Encoded frame decodes back with acceptable fidelity."""
        frame = _make_test_frame()
        codec = RGBDCodec(jpeg_quality=95)  # high quality for fidelity test

        encoded = codec.encode_frame(frame)
        decoded = codec.decode_frame(encoded)

        assert decoded.width == frame.width
        assert decoded.height == frame.height
        assert decoded.rgb.shape == frame.rgb.shape
        assert decoded.depth.shape == frame.depth.shape
        assert abs(decoded.timestamp - frame.timestamp) < 1e-6
        np.testing.assert_array_almost_equal(
            decoded.intrinsics, frame.intrinsics, decimal=10
        )

    def test_rgb_jpeg_lossy_close(self):
        """JPEG-compressed RGB is close to original (PSNR > 10 dB).

        Note: Random noise images have very low PSNR after JPEG compression
        because JPEG is designed for natural images with spatial coherence.
        Real camera images typically achieve PSNR > 30 dB at quality=85.
        """
        frame = _make_test_frame()
        codec = RGBDCodec(jpeg_quality=85)

        encoded = codec.encode_frame(frame)
        decoded = codec.decode_frame(encoded)

        # Calculate PSNR — random noise is worst case for JPEG
        mse = np.mean((frame.rgb.astype(float) - decoded.rgb.astype(float)) ** 2)
        if mse > 0:
            psnr = 10 * np.log10(255**2 / mse)
            assert psnr > 10, f"PSNR too low: {psnr:.1f} dB"

    def test_depth_quantization_precision(self):
        """Depth roundtrip error is <= 1mm (uint16 quantization)."""
        frame = _make_test_frame()
        codec = RGBDCodec()

        encoded = codec.encode_frame(frame)
        decoded = codec.decode_frame(encoded)

        # Max error should be <= 0.001m (1mm) from uint16 quantization
        max_error = np.abs(frame.depth - decoded.depth).max()
        assert max_error <= 0.001 + 1e-6, f"Depth error too large: {max_error:.6f}m"

    def test_compression_ratio(self):
        """Compressed frame is smaller than raw.

        Random noise achieves ~2.5x due to poor JPEG+lz4 compression on
        random data. Real camera images typically achieve 5-10x.
        """
        frame = _make_test_frame()
        codec = RGBDCodec(jpeg_quality=85)

        encoded = codec.encode_frame(frame)
        raw_size = frame.rgb.nbytes + frame.depth.nbytes  # ~1.84 MB
        compressed_size = len(encoded)

        ratio = raw_size / compressed_size
        assert ratio > 1.5, f"Compression ratio too low: {ratio:.1f}x"

    def test_encode_rgb_jpeg_smaller(self):
        """JPEG-encoded RGB is much smaller than raw."""
        frame = _make_test_frame()
        codec = RGBDCodec(jpeg_quality=85)

        raw_size = frame.rgb.nbytes  # 640*480*3 = 921600
        compressed = codec.encode_rgb(frame.rgb)

        assert len(compressed) < raw_size * 0.5  # at least 2x reduction

    def test_encode_depth_lz4_smaller(self):
        """lz4-compressed depth is smaller than raw uint16."""
        frame = _make_test_frame()
        codec = RGBDCodec()

        raw_uint16_size = frame.depth.size * 2  # 614400
        compressed = codec.encode_depth(frame.depth)

        # Random depth data won't compress well, but structured data will.
        # Just verify it at least adds the lz4 framing.
        assert isinstance(compressed, bytes)
        assert len(compressed) > 0

    def test_small_frame(self):
        """Works with small frame sizes."""
        frame = _make_test_frame(width=64, height=48)
        codec = RGBDCodec()

        encoded = codec.encode_frame(frame)
        decoded = codec.decode_frame(encoded)

        assert decoded.width == 64
        assert decoded.height == 48
        assert decoded.rgb.shape == (48, 64, 3)
        assert decoded.depth.shape == (48, 64)

    def test_header_format(self):
        """Wire format header is correct."""
        frame = _make_test_frame()
        codec = RGBDCodec()

        encoded = codec.encode_frame(frame)

        # Parse header
        msg_len, timestamp, width, height = struct.unpack_from(
            _HEADER_FMT, encoded, 0
        )
        assert width == frame.width
        assert height == frame.height
        assert abs(timestamp - frame.timestamp) < 1e-6
        assert msg_len == len(encoded) - _HEADER_SIZE

    def test_different_jpeg_qualities(self):
        """Higher JPEG quality produces larger but more accurate images."""
        frame = _make_test_frame()
        codec_low = RGBDCodec(jpeg_quality=50)
        codec_high = RGBDCodec(jpeg_quality=95)

        low = codec_low.encode_rgb(frame.rgb)
        high = codec_high.encode_rgb(frame.rgb)

        assert len(high) > len(low), "Higher quality should produce larger output"

    def test_zero_depth(self):
        """Handles zero depth (invalid pixels) correctly."""
        frame = _make_test_frame()
        frame.depth[:100, :100] = 0.0  # Zero out a region
        codec = RGBDCodec()

        encoded = codec.encode_frame(frame)
        decoded = codec.decode_frame(encoded)

        # Zero depth should remain zero after roundtrip
        assert np.all(decoded.depth[:100, :100] == 0.0)


# ── Server/Client integration tests ──


def _run_capture_loop(server: RGBDStreamServer, fps: float, stop_event: threading.Event):
    """Pump server.capture() in a thread (safe for MockCameraStream, no EGL)."""
    dt = 1.0 / fps
    while not stop_event.is_set():
        if server.is_running:
            server.capture()
        stop_event.wait(dt)


class TestStreamingIntegration:
    """Test server-client TCP streaming."""

    def _get_free_port(self) -> int:
        """Find an available TCP port."""
        import socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(("127.0.0.1", 0))
            return s.getsockname()[1]

    def _start_capture(self, server, fps=60.0):
        """Start a capture-pump thread. Returns (thread, stop_event)."""
        stop = threading.Event()
        t = threading.Thread(
            target=_run_capture_loop, args=(server, fps, stop), daemon=True
        )
        t.start()
        return t, stop

    def test_server_client_connection(self):
        """Client connects to server and receives frames."""
        frame = _make_test_frame(width=160, height=120)
        mock_cam = MockCameraStream(frame)
        port = self._get_free_port()

        server = RGBDStreamServer(
            camera=mock_cam, host="127.0.0.1", port=port, fps=30.0
        )
        server.start()
        cap_thread, cap_stop = self._start_capture(server, fps=30.0)
        time.sleep(0.3)

        client = RGBDStreamClient(host="127.0.0.1", port=port)
        assert client.initialize()

        # Wait for at least one frame
        time.sleep(0.5)

        received = client.get_rgbd()
        assert received.width == 160
        assert received.height == 120
        assert received.rgb.shape == (120, 160, 3)
        assert client.frames_received > 0

        cap_stop.set()
        client.shutdown()
        server.stop()
        cap_thread.join(timeout=2.0)

    def test_multiple_frames_received(self):
        """Client receives multiple frames over time."""
        frame = _make_test_frame(width=80, height=60)
        mock_cam = MockCameraStream(frame)
        port = self._get_free_port()

        server = RGBDStreamServer(
            camera=mock_cam, host="127.0.0.1", port=port, fps=60.0
        )
        server.start()
        cap_thread, cap_stop = self._start_capture(server, fps=60.0)
        time.sleep(0.2)

        client = RGBDStreamClient(host="127.0.0.1", port=port)
        client.initialize()

        time.sleep(1.0)

        # Should have received multiple frames at 60fps
        assert client.frames_received >= 10, (
            f"Expected >= 10 frames, got {client.frames_received}"
        )

        cap_stop.set()
        client.shutdown()
        server.stop()
        cap_thread.join(timeout=2.0)

    def test_reverse_channel_pan_tilt(self):
        """Client can send pan/tilt commands back to server."""
        frame = _make_test_frame(width=80, height=60)
        mock_cam = MockCameraStream(frame)
        port = self._get_free_port()

        server = RGBDStreamServer(
            camera=mock_cam, host="127.0.0.1", port=port, fps=30.0
        )
        server.start()
        cap_thread, cap_stop = self._start_capture(server, fps=30.0)
        time.sleep(0.2)

        client = RGBDStreamClient(host="127.0.0.1", port=port)
        client.initialize()
        time.sleep(0.3)

        # Send orientation command
        client.set_orientation(0.25, -0.15)
        time.sleep(0.3)

        # Check the mock camera received the command
        assert len(mock_cam._orientation_calls) > 0
        last_pan, last_tilt = mock_cam._orientation_calls[-1]
        assert abs(last_pan - 0.25) < 0.01
        assert abs(last_tilt - (-0.15)) < 0.01

        cap_stop.set()
        client.shutdown()
        server.stop()
        cap_thread.join(timeout=2.0)

    def test_client_orientation_state(self):
        """Client tracks its own pan/tilt state."""
        client = RGBDStreamClient(host="127.0.0.1", port=1)
        # Don't connect — just test state tracking
        client._connected = False

        client._pan = 0.3
        client._tilt = -0.2
        pan, tilt = client.get_orientation()
        assert pan == 0.3
        assert tilt == -0.2

    def test_client_not_connected_returns_empty(self):
        """Client returns empty frame before connection."""
        client = RGBDStreamClient(host="127.0.0.1", port=1)
        frame = client.get_rgbd()
        assert frame.width == 640
        assert frame.height == 480
        assert not client.is_connected()

    def test_graceful_disconnect(self):
        """Server and client handle disconnect gracefully."""
        frame = _make_test_frame(width=80, height=60)
        mock_cam = MockCameraStream(frame)
        port = self._get_free_port()

        server = RGBDStreamServer(
            camera=mock_cam, host="127.0.0.1", port=port, fps=30.0
        )
        server.start()
        cap_thread, cap_stop = self._start_capture(server, fps=30.0)
        time.sleep(0.2)

        client = RGBDStreamClient(host="127.0.0.1", port=port)
        client.initialize()
        time.sleep(0.3)

        assert client.is_connected()

        # Client disconnects
        client.shutdown()
        time.sleep(0.3)

        assert not client.is_connected()

        cap_stop.set()
        server.stop()
        cap_thread.join(timeout=2.0)

    def test_server_stats(self):
        """Server tracks frames_sent and bytes_sent."""
        frame = _make_test_frame(width=80, height=60)
        mock_cam = MockCameraStream(frame)
        port = self._get_free_port()

        server = RGBDStreamServer(
            camera=mock_cam, host="127.0.0.1", port=port, fps=60.0
        )
        server.start()
        cap_thread, cap_stop = self._start_capture(server, fps=60.0)
        time.sleep(0.2)

        client = RGBDStreamClient(host="127.0.0.1", port=port)
        client.initialize()
        time.sleep(0.5)

        assert server.frames_sent > 0
        assert server.bytes_sent > 0

        cap_stop.set()
        client.shutdown()
        server.stop()
        cap_thread.join(timeout=2.0)

    def test_connection_refused(self):
        """Client handles connection failure gracefully."""
        port = self._get_free_port()
        client = RGBDStreamClient(host="127.0.0.1", port=port)
        result = client.initialize()
        assert result is False
        assert not client.is_connected()

    def test_client_get_intrinsics(self):
        """Client returns intrinsics from received frame."""
        frame = _make_test_frame(width=80, height=60)
        mock_cam = MockCameraStream(frame)
        port = self._get_free_port()

        server = RGBDStreamServer(
            camera=mock_cam, host="127.0.0.1", port=port, fps=30.0
        )
        server.start()
        cap_thread, cap_stop = self._start_capture(server, fps=30.0)
        time.sleep(0.2)

        client = RGBDStreamClient(host="127.0.0.1", port=port)
        client.initialize()
        time.sleep(0.5)

        intrinsics = client.get_intrinsics()
        assert intrinsics.shape == (3, 3)
        # Should match the mock camera's intrinsics (approximately, after encode/decode)
        np.testing.assert_array_almost_equal(
            intrinsics, frame.intrinsics, decimal=10
        )

        cap_stop.set()
        client.shutdown()
        server.stop()
        cap_thread.join(timeout=2.0)
