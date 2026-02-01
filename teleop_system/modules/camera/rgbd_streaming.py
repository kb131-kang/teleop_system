"""Compressed RGB-D streaming over TCP for wireless teleoperation.

Provides server and client classes for streaming RGB-D frames between
the slave (robot with camera) and master (VR operator) over a network.

Compression strategy:
  - RGB: JPEG via Pillow (~20:1 compression)
  - Depth: uint16 quantization (mm) + lz4 (~3:1 compression)
  - Total: ~250 KB/frame vs ~2.1 MB raw = ~8x reduction

Wire protocol (per frame):
  [4B msg_len][8B timestamp][4B width][4B height]
  [4B rgb_len][rgb_jpeg_bytes]
  [4B depth_len][depth_lz4_bytes]
  [72B intrinsics_raw]

Reverse channel (client -> server, for pan/tilt control):
  [4B 'CTRL'][4B pan_float][4B tilt_float]

Usage:
    # Server (robot side):
    server = RGBDStreamServer(camera=sim_camera, host="0.0.0.0", port=9876)
    server.start()

    # Client (operator side):
    client = RGBDStreamClient(host="robot_ip", port=9876)
    client.initialize()
    frame = client.get_rgbd()  # ICameraStream-compatible
"""

from __future__ import annotations

import io
import socket
import struct
import threading
import time

import numpy as np

from teleop_system.interfaces.camera_stream import ICameraStream, RGBDFrame
from teleop_system.utils.logger import get_logger

logger = get_logger("rgbd_streaming")

try:
    from PIL import Image

    _PILLOW_AVAILABLE = True
except ImportError:
    _PILLOW_AVAILABLE = False
    logger.warning("Pillow not available — JPEG compression disabled")

try:
    import lz4.frame

    _LZ4_AVAILABLE = True
except ImportError:
    _LZ4_AVAILABLE = False
    logger.warning("lz4 not available — depth compression disabled")


# ── Wire protocol constants ──
_HEADER_FMT = "!I d I I"  # msg_len, timestamp, width, height
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)  # 4+8+4+4 = 20 bytes
_LEN_FMT = "!I"  # length prefix for variable-length fields
_LEN_SIZE = 4
_INTRINSICS_SIZE = 72   # 9 x float64
_EXTRINSICS_SIZE = 128  # 16 x float64
_CTRL_MAGIC = b"CTRL"
_CTRL_FMT = "!4s f f"  # magic, pan, tilt
_CTRL_SIZE = struct.calcsize(_CTRL_FMT)  # 4+4+4 = 12 bytes


class RGBDCodec:
    """Encode/decode RGB-D frames for network transport.

    RGB is compressed as JPEG (lossy, configurable quality).
    Depth is quantized to uint16 (mm) and compressed with lz4.
    Falls back to raw transport if compression libraries are unavailable.
    """

    def __init__(self, jpeg_quality: int = 85):
        self._jpeg_quality = jpeg_quality

    def encode_frame(self, frame: RGBDFrame) -> bytes:
        """Encode an RGBDFrame into bytes for transmission.

        Returns:
            Packed bytes: header + rgb_data + depth_data + intrinsics.
        """
        rgb_bytes = self.encode_rgb(frame.rgb)
        depth_bytes = self.encode_depth(frame.depth)
        intrinsics_bytes = frame.intrinsics.astype(np.float64).tobytes()
        extrinsics_bytes = frame.extrinsics.astype(np.float64).tobytes()

        # Total payload after header
        payload_len = (
            _LEN_SIZE + len(rgb_bytes)
            + _LEN_SIZE + len(depth_bytes)
            + _INTRINSICS_SIZE
            + _EXTRINSICS_SIZE
        )

        header = struct.pack(
            _HEADER_FMT,
            payload_len,
            frame.timestamp,
            frame.width,
            frame.height,
        )

        parts = [
            header,
            struct.pack(_LEN_FMT, len(rgb_bytes)),
            rgb_bytes,
            struct.pack(_LEN_FMT, len(depth_bytes)),
            depth_bytes,
            intrinsics_bytes,
            extrinsics_bytes,
        ]
        return b"".join(parts)

    def decode_frame(self, data: bytes) -> RGBDFrame:
        """Decode bytes back into an RGBDFrame.

        Args:
            data: Full message bytes (header + payload).
        """
        offset = 0

        # Header
        msg_len, timestamp, width, height = struct.unpack_from(
            _HEADER_FMT, data, offset
        )
        offset += _HEADER_SIZE

        # RGB
        (rgb_len,) = struct.unpack_from(_LEN_FMT, data, offset)
        offset += _LEN_SIZE
        rgb_bytes = data[offset : offset + rgb_len]
        offset += rgb_len

        # Depth
        (depth_len,) = struct.unpack_from(_LEN_FMT, data, offset)
        offset += _LEN_SIZE
        depth_bytes = data[offset : offset + depth_len]
        offset += depth_len

        # Intrinsics
        intrinsics_bytes = data[offset : offset + _INTRINSICS_SIZE]
        offset += _INTRINSICS_SIZE

        # Extrinsics
        extrinsics_bytes = data[offset : offset + _EXTRINSICS_SIZE]

        rgb = self.decode_rgb(rgb_bytes, width, height)
        depth = self.decode_depth(depth_bytes, width, height)
        intrinsics = np.frombuffer(intrinsics_bytes, dtype=np.float64).reshape(3, 3)
        extrinsics = np.frombuffer(extrinsics_bytes, dtype=np.float64).reshape(4, 4)

        return RGBDFrame(
            rgb=rgb,
            depth=depth,
            intrinsics=intrinsics,
            extrinsics=extrinsics,
            timestamp=timestamp,
            width=width,
            height=height,
        )

    def encode_rgb(self, rgb: np.ndarray) -> bytes:
        """Compress RGB image as JPEG."""
        if _PILLOW_AVAILABLE:
            img = Image.fromarray(rgb)
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=self._jpeg_quality)
            return buf.getvalue()
        else:
            return rgb.tobytes()

    def decode_rgb(self, data: bytes, width: int, height: int) -> np.ndarray:
        """Decompress JPEG bytes to RGB array."""
        if _PILLOW_AVAILABLE:
            img = Image.open(io.BytesIO(data))
            return np.array(img)
        else:
            return np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)

    def encode_depth(self, depth: np.ndarray) -> bytes:
        """Quantize depth to uint16 (mm) and compress with lz4."""
        # Convert meters to millimeters, clip to uint16 range
        depth_mm = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
        raw = depth_mm.tobytes()
        if _LZ4_AVAILABLE:
            return lz4.frame.compress(raw)
        else:
            return raw

    def decode_depth(self, data: bytes, width: int, height: int) -> np.ndarray:
        """Decompress and convert uint16 mm back to float32 meters."""
        if _LZ4_AVAILABLE:
            raw = lz4.frame.decompress(data)
        else:
            raw = data
        depth_mm = np.frombuffer(raw, dtype=np.uint16).reshape(height, width)
        return depth_mm.astype(np.float32) * 0.001


def _recv_exact(sock: socket.socket, n: int) -> bytes | None:
    """Receive exactly n bytes from socket, or None on disconnect."""
    chunks = []
    remaining = n
    while remaining > 0:
        chunk = sock.recv(min(remaining, 65536))
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


class RGBDStreamServer:
    """TCP server that streams compressed RGB-D frames from an ICameraStream.

    Runs on the slave (robot) side. Accepts one client connection and
    sends compressed frames captured via the ``capture()`` method.

    Thread safety:
        ``capture()`` must be called from the main thread (where the
        rendering context lives — e.g. EGL for MuJoCo). The background
        thread only handles TCP accept/send and the reverse control channel.

    Args:
        camera: Source ICameraStream (SimCameraStream or RealSenseCamera).
        host: Bind address (default "0.0.0.0" for all interfaces).
        port: TCP port.
        fps: Target frame rate for streaming (used only for send pacing).
        jpeg_quality: JPEG compression quality (1-100).
    """

    def __init__(
        self,
        camera: ICameraStream,
        host: str = "0.0.0.0",
        port: int = 9876,
        fps: float = 15.0,
        jpeg_quality: int = 85,
    ):
        self._camera = camera
        self._host = host
        self._port = port
        self._fps = fps
        self._codec = RGBDCodec(jpeg_quality=jpeg_quality)

        self._server_sock: socket.socket | None = None
        self._client_sock: socket.socket | None = None
        self._running = False
        self._send_thread: threading.Thread | None = None
        self._recv_thread: threading.Thread | None = None
        self._lock = threading.Lock()

        # Encoded frame buffer (written by main thread, read by send thread)
        self._encoded_frame: bytes | None = None
        self._frame_event = threading.Event()

        # Stats
        self._frames_sent = 0
        self._bytes_sent = 0

    def start(self) -> None:
        """Start the server: bind, listen, and begin the send loop."""
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((self._host, self._port))
        self._server_sock.listen(1)
        self._server_sock.settimeout(1.0)
        self._running = True

        self._send_thread = threading.Thread(
            target=self._server_loop, daemon=True, name="rgbd-server"
        )
        self._send_thread.start()
        logger.info(f"RGBDStreamServer listening on {self._host}:{self._port}")

    def capture(self) -> None:
        """Capture a frame from the camera and encode it for sending.

        **Must be called from the main thread** (the thread that owns the
        rendering context, e.g. EGL). The encoded bytes are stored in an
        internal buffer that the background send thread picks up.
        """
        frame = self._camera.get_rgbd()
        encoded = self._codec.encode_frame(frame)
        with self._lock:
            self._encoded_frame = encoded
        self._frame_event.set()

    def _server_loop(self) -> None:
        """Background loop: accept connections and send captured frames."""
        while self._running:
            # Accept client
            try:
                client, addr = self._server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            logger.info(f"Client connected: {addr}")
            with self._lock:
                self._client_sock = client
            client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            # Start reverse channel receiver
            self._recv_thread = threading.Thread(
                target=self._recv_loop, args=(client,), daemon=True,
                name="rgbd-server-recv",
            )
            self._recv_thread.start()

            # Send frames as they become available from capture()
            try:
                while self._running:
                    # Wait for a new frame from the main thread
                    if not self._frame_event.wait(timeout=0.5):
                        continue
                    self._frame_event.clear()

                    with self._lock:
                        encoded = self._encoded_frame
                    if encoded is None:
                        continue

                    try:
                        client.sendall(encoded)
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        break

                    self._frames_sent += 1
                    self._bytes_sent += len(encoded)
            except Exception as e:
                logger.error(f"Server streaming error: {e}")
            finally:
                logger.info(f"Client disconnected: {addr}")
                with self._lock:
                    self._client_sock = None
                client.close()

    def _recv_loop(self, client: socket.socket) -> None:
        """Receive control messages (pan/tilt) from client."""
        try:
            while self._running:
                data = _recv_exact(client, _CTRL_SIZE)
                if data is None:
                    break

                magic, pan, tilt = struct.unpack(_CTRL_FMT, data)
                if magic == _CTRL_MAGIC:
                    self._camera.set_orientation(pan, tilt)
        except (OSError, struct.error):
            pass

    def stop(self) -> None:
        """Stop the server and close connections."""
        self._running = False
        with self._lock:
            if self._client_sock:
                try:
                    self._client_sock.close()
                except OSError:
                    pass
                self._client_sock = None
        if self._server_sock:
            try:
                self._server_sock.close()
            except OSError:
                pass
        if self._send_thread:
            self._send_thread.join(timeout=3.0)
        logger.info(
            f"RGBDStreamServer stopped. "
            f"Sent {self._frames_sent} frames, {self._bytes_sent / 1e6:.1f} MB"
        )

    @property
    def frames_sent(self) -> int:
        return self._frames_sent

    @property
    def bytes_sent(self) -> int:
        return self._bytes_sent

    @property
    def is_running(self) -> bool:
        return self._running


class RGBDStreamClient(ICameraStream):
    """TCP client that receives compressed RGB-D frames from a StreamServer.

    Runs on the master (operator) side. Implements ICameraStream so it can
    be used as a drop-in replacement for SimCameraStream or RealSenseCamera.

    Args:
        host: Server address (robot IP or "localhost").
        port: TCP port matching the server.
        jpeg_quality: Expected JPEG quality (for codec init only).
    """

    def __init__(
        self,
        host: str = "localhost",
        port: int = 9876,
        jpeg_quality: int = 85,
    ):
        self._host = host
        self._port = port
        self._codec = RGBDCodec(jpeg_quality=jpeg_quality)

        self._sock: socket.socket | None = None
        self._connected = False
        self._running = False
        self._recv_thread: threading.Thread | None = None

        # Double-buffered frame (atomic swap under GIL)
        self._front_frame: RGBDFrame = RGBDFrame()
        self._back_frame: RGBDFrame = RGBDFrame()
        self._frame_lock = threading.Lock()

        # Pan/tilt state
        self._pan = 0.0
        self._tilt = 0.0

        # Stats
        self._frames_received = 0
        self._bytes_received = 0

    def initialize(self) -> bool:
        """Connect to the stream server."""
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.connect((self._host, self._port))
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._connected = True
            self._running = True

            self._recv_thread = threading.Thread(
                target=self._recv_loop, daemon=True, name="rgbd-client-recv"
            )
            self._recv_thread.start()

            logger.info(f"RGBDStreamClient connected to {self._host}:{self._port}")
            return True
        except (OSError, ConnectionRefusedError) as e:
            logger.error(f"Failed to connect to {self._host}:{self._port}: {e}")
            return False

    def _recv_loop(self) -> None:
        """Background thread: receive and decode frames."""
        try:
            while self._running:
                # Read header
                header_data = _recv_exact(self._sock, _HEADER_SIZE)
                if header_data is None:
                    break

                msg_len, timestamp, width, height = struct.unpack(
                    _HEADER_FMT, header_data
                )

                # Read payload
                payload = _recv_exact(self._sock, msg_len)
                if payload is None:
                    break

                # Reconstruct full message for decode
                full_msg = header_data + payload
                frame = self._codec.decode_frame(full_msg)

                # Atomic swap (GIL-safe)
                with self._frame_lock:
                    self._front_frame = frame

                self._frames_received += 1
                self._bytes_received += _HEADER_SIZE + msg_len

        except (OSError, struct.error) as e:
            if self._running:
                logger.error(f"Client receive error: {e}")
        finally:
            self._connected = False
            logger.info("RGBDStreamClient disconnected")

    def get_rgbd(self) -> RGBDFrame:
        """Get the latest received frame."""
        with self._frame_lock:
            return self._front_frame

    def set_orientation(self, pan: float, tilt: float) -> None:
        """Send pan/tilt command back to server (reverse channel)."""
        self._pan = pan
        self._tilt = tilt
        if self._sock and self._connected:
            try:
                msg = struct.pack(_CTRL_FMT, _CTRL_MAGIC, pan, tilt)
                self._sock.sendall(msg)
            except (BrokenPipeError, OSError):
                pass

    def get_orientation(self) -> tuple[float, float]:
        return self._pan, self._tilt

    def get_intrinsics(self) -> np.ndarray:
        with self._frame_lock:
            return self._front_frame.intrinsics.copy()

    def is_connected(self) -> bool:
        return self._connected

    def shutdown(self) -> None:
        """Disconnect from the server."""
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        if self._recv_thread:
            self._recv_thread.join(timeout=3.0)
        self._connected = False
        logger.info(
            f"RGBDStreamClient shutdown. "
            f"Received {self._frames_received} frames, "
            f"{self._bytes_received / 1e6:.1f} MB"
        )

    @property
    def frames_received(self) -> int:
        return self._frames_received

    @property
    def bytes_received(self) -> int:
        return self._bytes_received
