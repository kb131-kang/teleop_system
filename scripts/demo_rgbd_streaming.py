#!/usr/bin/env python3
"""RGB-D streaming demo: server, client, or local (both-in-one).

Demonstrates compressed RGB-D streaming over TCP between a slave (robot
with camera) and master (operator with viewer).

Modes:
  server  — MuJoCo simulation + StreamServer (runs on robot side)
  client  — StreamClient + PointCloudViewer (runs on operator side)
  local   — Both server and client in one process (testing/development)

Usage:
    # Terminal 1 (robot): start server with MuJoCo simulation
    MUJOCO_GL=egl python3 scripts/demo_rgbd_streaming.py --mode server

    # Terminal 2 (operator): connect and view point cloud
    python3 scripts/demo_rgbd_streaming.py --mode client --host localhost

    # Single process (development):
    python3 scripts/demo_rgbd_streaming.py --mode local

Controls (in client/local mode):
    Left-drag:   Rotate view + robot camera
    Right-drag:  Pan view
    Scroll:      Zoom in/out
    R:           Reset view
    Q/ESC:       Quit
"""

import argparse
import os
import sys
import time
from pathlib import Path

# MuJoCo rendering backend
if "MUJOCO_GL" not in os.environ:
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        os.environ["MUJOCO_GL"] = "glfw"
    else:
        os.environ["MUJOCO_GL"] = "egl"

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np

from teleop_system.modules.camera.rgbd_streaming import (
    RGBDCodec,
    RGBDStreamClient,
    RGBDStreamServer,
)


def run_server(args):
    """Run the streaming server with MuJoCo simulation."""
    from teleop_system.simulators.mujoco_sim import MuJoCoSimulator
    from teleop_system.simulators.sim_camera_stream import SimCameraStream

    print("\n" + "=" * 70)
    print("  RGB-D STREAM SERVER")
    print("=" * 70)

    # Initialize MuJoCo
    sim = MuJoCoSimulator()
    mjcf_path = str(PROJECT_ROOT / "models" / "rby1" / "model_teleop.xml")
    success = sim.initialize({
        "mjcf_path": mjcf_path,
        "timestep": 0.002,
        "render": True,
        "render_width": args.render_width,
        "render_height": args.render_height,
    })
    if not success:
        print("  [ERROR] MuJoCo initialization failed")
        return 1

    print(f"  MuJoCo: {len(sim.get_joint_names())} joints, "
          f"render={args.render_width}x{args.render_height}")

    # Camera stream
    cam = SimCameraStream(simulator=sim)
    cam.initialize()

    # Warm up physics
    for _ in range(200):
        sim.step()

    # Start stream server
    server = RGBDStreamServer(
        camera=cam,
        host=args.host,
        port=args.port,
        fps=args.stream_fps,
        jpeg_quality=args.jpeg_quality,
    )
    server.start()

    # Estimate compression ratio
    raw_frame = cam.get_rgbd()
    codec = RGBDCodec(jpeg_quality=args.jpeg_quality)
    encoded = codec.encode_frame(raw_frame)
    raw_size = raw_frame.rgb.nbytes + raw_frame.depth.nbytes
    compressed_size = len(encoded)
    ratio = raw_size / compressed_size

    print(f"  Streaming: {args.host}:{args.port} @ {args.stream_fps} Hz")
    print(f"  JPEG quality: {args.jpeg_quality}")
    print(f"  Compression: {raw_size:,} B -> {compressed_size:,} B "
          f"({ratio:.1f}x reduction)")
    print(f"  Bandwidth: ~{compressed_size * args.stream_fps / 1e6:.2f} MB/s")
    print(f"\n  Waiting for client... (Ctrl+C to stop)")
    print("=" * 70 + "\n")

    # Main loop: step physics + capture frames on the main thread
    # (camera.get_rgbd() calls MuJoCo render which requires the EGL context
    # that was created on this thread — cannot be called from a background thread)
    dt_physics = 0.002
    dt_step = 1.0 / 60.0
    dt_capture = 1.0 / args.stream_fps
    steps_per_cycle = max(1, int(dt_step / dt_physics))
    last_capture = 0.0

    try:
        while server.is_running:
            now = time.monotonic()

            for _ in range(steps_per_cycle):
                sim.step()

            # Capture + encode on main thread at stream FPS
            if now - last_capture >= dt_capture:
                server.capture()
                last_capture = now

            time.sleep(max(0.001, dt_step - (time.monotonic() - now)))

            # Print periodic stats
            if server.frames_sent > 0 and server.frames_sent % 100 == 0:
                mb = server.bytes_sent / 1e6
                print(f"  Sent {server.frames_sent} frames, {mb:.1f} MB")
    except KeyboardInterrupt:
        print("\n  Stopping server...")

    server.stop()
    cam.shutdown()
    sim.shutdown()
    print("  Done.")
    return 0


def run_client(args):
    """Run the streaming client with point cloud viewer."""
    from teleop_system.modules.camera.pointcloud_generator import PointCloudGenerator
    from teleop_system.modules.camera.pointcloud_viewer import PointCloudViewer

    print("\n" + "=" * 70)
    print("  RGB-D STREAM CLIENT")
    print("=" * 70)

    # Connect to server
    client = RGBDStreamClient(
        host=args.host,
        port=args.port,
        jpeg_quality=args.jpeg_quality,
    )
    if not client.initialize():
        print(f"  [ERROR] Cannot connect to {args.host}:{args.port}")
        return 1

    print(f"  Connected to {args.host}:{args.port}")

    # Point cloud generator
    pcg = PointCloudGenerator(
        voxel_size=args.voxel_size,
        max_depth=args.max_depth,
        min_depth=0.1,
        use_open3d=True,
    )

    # Viewer with camera sync (drives robot camera via reverse channel)
    viewer = PointCloudViewer(
        width=960,
        height=720,
        title="RGB-D Stream Viewer",
        point_size=args.point_size,
        camera=client,  # mouse orbit drives robot camera
    )
    if not viewer.initialize():
        print("  [ERROR] Failed to initialize viewer")
        client.shutdown()
        return 1

    # Set initial camera view
    viewer._cam_distance = 8.0
    viewer._cam_yaw = 180.0
    viewer._cam_pitch = -20.0
    viewer.set_camera(client)  # update reference point

    print(f"  Viewer: mouse orbit drives robot camera (pan/tilt)")
    print(f"  Point cloud: voxel={args.voxel_size}m, max_depth={args.max_depth}m")
    print(f"\n  Controls: Left-drag=Rotate+Robot, Right-drag=Pan, "
          f"Scroll=Zoom, R=Reset, Q=Quit")
    print("=" * 70 + "\n")

    dt_capture = 1.0 / args.stream_fps
    last_capture = 0.0
    capture_count = 0

    try:
        while viewer.is_running() and client.is_connected():
            now = time.monotonic()

            if now - last_capture >= dt_capture:
                frame = client.get_rgbd()
                if frame.rgb.any():
                    pc = pcg.generate(frame)
                    capture_count += 1

                    if pc["count"] > 0:
                        viewer.update_points(pc["points"], pc["colors"])

                    pan, tilt = client.get_orientation()
                    viewer.set_stats(
                        f"pan={pan:.2f} tilt={tilt:.2f}  "
                        f"frames={capture_count}  "
                        f"rx={client.bytes_received/1e6:.1f}MB"
                    )

                last_capture = now

            viewer.render()
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n  Stopped by user")

    # Summary
    print(f"\n  Frames received: {client.frames_received}")
    print(f"  Data received: {client.bytes_received / 1e6:.1f} MB")

    viewer.shutdown()
    client.shutdown()
    print("  Done.")
    return 0


def run_local(args):
    """Run server and client in the same process for testing."""
    from teleop_system.modules.camera.pointcloud_generator import PointCloudGenerator
    from teleop_system.modules.camera.pointcloud_viewer import PointCloudViewer
    from teleop_system.simulators.mujoco_sim import MuJoCoSimulator
    from teleop_system.simulators.sim_camera_stream import SimCameraStream

    print("\n" + "=" * 70)
    print("  RGB-D STREAMING — LOCAL MODE (server + client in-process)")
    print("=" * 70)

    # Initialize MuJoCo
    sim = MuJoCoSimulator()
    mjcf_path = str(PROJECT_ROOT / "models" / "rby1" / "model_teleop.xml")
    success = sim.initialize({
        "mjcf_path": mjcf_path,
        "timestep": 0.002,
        "render": True,
        "render_width": args.render_width,
        "render_height": args.render_height,
    })
    if not success:
        print("  [ERROR] MuJoCo initialization failed")
        return 1

    cam = SimCameraStream(simulator=sim)
    cam.initialize()

    # Warm up
    for _ in range(200):
        sim.step()

    # Start server on localhost
    server = RGBDStreamServer(
        camera=cam,
        host="127.0.0.1",
        port=args.port,
        fps=args.stream_fps,
        jpeg_quality=args.jpeg_quality,
    )
    server.start()

    # Brief delay for server to start listening
    time.sleep(0.5)

    # Connect client
    client = RGBDStreamClient(
        host="127.0.0.1",
        port=args.port,
        jpeg_quality=args.jpeg_quality,
    )
    if not client.initialize():
        print("  [ERROR] Client failed to connect")
        server.stop()
        sim.shutdown()
        return 1

    # Point cloud generator
    pcg = PointCloudGenerator(
        voxel_size=args.voxel_size,
        max_depth=args.max_depth,
        min_depth=0.1,
        use_open3d=True,
    )

    # Viewer with camera sync
    viewer = PointCloudViewer(
        width=960,
        height=720,
        title="RGB-D Stream (Local)",
        point_size=args.point_size,
        camera=client,
    )
    if not viewer.initialize():
        print("  [ERROR] Failed to initialize viewer")
        client.shutdown()
        server.stop()
        sim.shutdown()
        return 1

    viewer._cam_distance = 8.0
    viewer._cam_yaw = 180.0
    viewer._cam_pitch = -20.0
    viewer.set_camera(client)

    # Estimate compression
    raw_frame = cam.get_rgbd()
    codec = RGBDCodec(jpeg_quality=args.jpeg_quality)
    encoded = codec.encode_frame(raw_frame)
    raw_size = raw_frame.rgb.nbytes + raw_frame.depth.nbytes
    compressed_size = len(encoded)

    print(f"  MuJoCo: {len(sim.get_joint_names())} joints")
    print(f"  Streaming: localhost:{args.port} @ {args.stream_fps} Hz")
    print(f"  Compression: {raw_size/1e3:.0f} KB -> {compressed_size/1e3:.0f} KB "
          f"({raw_size/compressed_size:.1f}x)")
    print(f"  Bandwidth: ~{compressed_size * args.stream_fps / 1e6:.2f} MB/s")
    print(f"\n  Controls: Left-drag=Rotate+Robot, Right-drag=Pan, "
          f"Scroll=Zoom, R=Reset, Q=Quit")
    print("=" * 70 + "\n")

    dt_physics = 1.0 / 60.0
    steps_per_cycle = max(1, int(dt_physics / 0.002))
    dt_capture = 1.0 / args.stream_fps
    last_capture = 0.0
    capture_count = 0
    start_time = time.monotonic()

    try:
        while viewer.is_running():
            now = time.monotonic()

            # Step physics
            for _ in range(steps_per_cycle):
                sim.step()

            # Capture + encode on main thread (EGL context lives here)
            if now - last_capture >= dt_capture:
                server.capture()

                frame = client.get_rgbd()
                if frame.rgb.any():
                    pc = pcg.generate(frame)
                    capture_count += 1

                    if pc["count"] > 0:
                        viewer.update_points(pc["points"], pc["colors"])

                    pan, tilt = client.get_orientation()
                    viewer.set_stats(
                        f"pan={pan:.2f} tilt={tilt:.2f}  "
                        f"frames={capture_count}  "
                        f"tx={server.bytes_sent/1e6:.1f}MB  "
                        f"rx={client.bytes_received/1e6:.1f}MB"
                    )
                last_capture = now

            viewer.render()
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n  Stopped by user")

    elapsed = time.monotonic() - start_time
    print(f"\n  Duration: {elapsed:.1f}s")
    print(f"  Server: {server.frames_sent} frames, {server.bytes_sent/1e6:.1f} MB")
    print(f"  Client: {client.frames_received} frames, {client.bytes_received/1e6:.1f} MB")

    viewer.shutdown()
    client.shutdown()
    server.stop()
    cam.shutdown()
    sim.shutdown()
    print("  Done.")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="RGB-D streaming demo (server/client/local)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--mode", choices=["server", "client", "local"],
                        default="local", help="Run mode (default: local)")
    parser.add_argument("--host", type=str, default="0.0.0.0",
                        help="Server bind/connect address")
    parser.add_argument("--port", type=int, default=9876,
                        help="TCP port")
    parser.add_argument("--stream-fps", type=float, default=15.0,
                        help="Stream frame rate (Hz)")
    parser.add_argument("--jpeg-quality", type=int, default=85,
                        help="JPEG compression quality (1-100)")
    parser.add_argument("--render-width", type=int, default=640,
                        help="Camera render width")
    parser.add_argument("--render-height", type=int, default=480,
                        help="Camera render height")
    parser.add_argument("--voxel-size", type=float, default=0.02,
                        help="Voxel downsampling (0=none)")
    parser.add_argument("--max-depth", type=float, default=10.0,
                        help="Max depth for point cloud (meters)")
    parser.add_argument("--point-size", type=float, default=2.0,
                        help="OpenGL point size")
    args = parser.parse_args()

    if args.mode == "server":
        return run_server(args)
    elif args.mode == "client":
        return run_client(args)
    else:
        return run_local(args)


if __name__ == "__main__":
    sys.exit(main())
