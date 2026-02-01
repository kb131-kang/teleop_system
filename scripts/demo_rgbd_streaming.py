#!/usr/bin/env python3
"""RGB-D streaming demo: server, client, local, ros2-viewer, or ros2-server.

Demonstrates compressed RGB-D streaming over TCP between a slave (robot
with camera) and master (operator with viewer), or via ROS2 topics.

Modes:
  server      — Standalone MuJoCo simulation + StreamServer (single-machine testing)
  client      — StreamClient + PointCloudViewer (runs on operator side)
  local       — Both server and client in one process (testing/development)
  ros2-viewer — Subscribe to ROS2 RGB-D topics + PointCloudViewer (same machine)
  ros2-server — Bridge ROS2 camera topics to TCP streaming (cross-machine)

For cross-machine streaming with full robot sync, use ros2-server + client:

    # Slave machine: MuJoCo bridge + ros2-server
    ros2 launch teleop_system slave_mujoco.launch.py publish_camera:=true
    python3 scripts/demo_rgbd_streaming.py --mode ros2-server --host 0.0.0.0

    # Master machine: client viewer
    python3 scripts/demo_rgbd_streaming.py --mode client --host <slave-ip>

For same-machine viewing:

    # MuJoCo bridge with camera
    ros2 launch teleop_system slave_mujoco.launch.py publish_camera:=true

    # Viewer subscribing directly to ROS2 topics
    python3 scripts/demo_rgbd_streaming.py --mode ros2-viewer

Other modes:

    # Standalone server (own MuJoCo, no master sync):
    MUJOCO_GL=egl python3 scripts/demo_rgbd_streaming.py --mode server

    # Single process (development):
    python3 scripts/demo_rgbd_streaming.py --mode local

Controls (in client/local/ros2-viewer mode):
    Left-drag:   Rotate view + robot camera
    R:           Reset view
    Q/ESC:       Quit
"""

import argparse
import os
import sys
import threading
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

    # Viewer in first-person mode: shows camera-frame point cloud
    # (like looking through the robot's eyes)
    viewer = PointCloudViewer(
        width=960,
        height=720,
        title="RGB-D Stream Viewer (First-Person)",
        point_size=args.point_size,
        camera=client,  # mouse drag drives robot camera pan/tilt
        first_person=True,
    )
    if not viewer.initialize():
        print("  [ERROR] Failed to initialize viewer")
        client.shutdown()
        return 1

    viewer.set_camera(client)

    # Optional ROS2 head orientation sync
    head_sync = None
    if args.ros2_sync:
        from teleop_system.modules.camera.ros2_head_sync import ROS2HeadSync

        head_sync = ROS2HeadSync(camera=client)
        if head_sync.start():
            print("  ROS2 head sync: ENABLED (/mujoco/joint_states -> TCP)")
        else:
            print("  ROS2 head sync: FAILED (falling back to mouse-only)")
            head_sync = None

    print(f"  Viewer: first-person (robot camera POV)")
    print(f"  Point cloud: voxel={args.voxel_size}m, max_depth={args.max_depth}m")
    print(f"\n  Controls: Left-drag=Move robot camera, R=Reset, Q=Quit")
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
                    # Keep points in camera frame (don't transform to world).
                    # The first-person viewer displays camera-frame points directly.
                    frame.extrinsics = np.eye(4)
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

    if head_sync:
        head_sync.stop()
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

    # Viewer in first-person mode (robot camera POV)
    viewer = PointCloudViewer(
        width=960,
        height=720,
        title="RGB-D Stream (Local, First-Person)",
        point_size=args.point_size,
        camera=client,
        first_person=True,
    )
    if not viewer.initialize():
        print("  [ERROR] Failed to initialize viewer")
        client.shutdown()
        server.stop()
        sim.shutdown()
        return 1

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
    print(f"\n  Controls: Left-drag=Move robot camera, R=Reset, Q=Quit")
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
                    # Keep points in camera frame for first-person view
                    frame.extrinsics = np.eye(4)
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


def _ros2_spin_loop(node):
    """Background ROS2 spin loop."""
    import rclpy

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)


def run_ros2_viewer(args):
    """Run viewer subscribing to ROS2 RGB-D topics."""
    import rclpy

    from teleop_system.modules.camera.pointcloud_generator import PointCloudGenerator
    from teleop_system.modules.camera.pointcloud_viewer import PointCloudViewer
    from teleop_system.modules.camera.ros2_rgbd_subscriber import ROS2RGBDSubscriber

    print("\n" + "=" * 70)
    print("  RGB-D ROS2 VIEWER")
    print("=" * 70)

    try:
        if not rclpy.ok():
            rclpy.init()
    except RuntimeError:
        rclpy.init()

    # Create ROS2 RGB-D subscriber (ICameraStream)
    subscriber = ROS2RGBDSubscriber(
        color_topic=args.color_topic or "/slave/camera/color/image_raw",
        depth_topic=args.depth_topic or "/slave/camera/depth/image_raw",
        info_topic=args.info_topic or "/slave/camera/camera_info",
        joint_states_topic="/mujoco/joint_states",
        pan_tilt_topic="/slave/camera/pan_tilt_cmd",
    )
    if not subscriber.initialize():
        print("  [ERROR] Failed to initialize ROS2 RGB-D subscriber")
        rclpy.shutdown()
        return 1

    print(f"  Subscribing to ROS2 topics:")
    print(f"    Color: {args.color_topic or '/slave/camera/color/image_raw'}")
    print(f"    Depth: {args.depth_topic or '/slave/camera/depth/image_raw'}")
    print(f"    Info:  {args.info_topic or '/slave/camera/camera_info'}")
    print(f"    Head:  /mujoco/joint_states")

    # Point cloud generator
    pcg = PointCloudGenerator(
        voxel_size=args.voxel_size,
        max_depth=args.max_depth,
        min_depth=0.1,
        use_open3d=True,
    )

    # Viewer in first-person mode with camera sync
    viewer = PointCloudViewer(
        width=960,
        height=720,
        title="ROS2 RGB-D Viewer (First-Person)",
        point_size=args.point_size,
        camera=subscriber,
        first_person=True,
    )
    if not viewer.initialize():
        print("  [ERROR] Failed to initialize viewer")
        subscriber.shutdown()
        rclpy.shutdown()
        return 1

    viewer.set_camera(subscriber)

    # Spin ROS2 in background thread
    spin_thread = threading.Thread(
        target=_ros2_spin_loop, args=(subscriber._node,),
        daemon=True, name="ros2-viewer-spin",
    )
    spin_thread.start()

    print(f"  Point cloud: voxel={args.voxel_size}m, max_depth={args.max_depth}m")
    print(f"\n  Controls: Left-drag=Move robot camera, R=Reset, Q=Quit")
    print(f"  Waiting for data... (Ctrl+C to stop)")
    print("=" * 70 + "\n")

    dt_capture = 1.0 / args.stream_fps
    last_capture = 0.0
    capture_count = 0

    try:
        while viewer.is_running():
            now = time.monotonic()

            if now - last_capture >= dt_capture:
                frame = subscriber.get_rgbd()
                if frame.rgb is not None and frame.rgb.any():
                    frame.extrinsics = np.eye(4)
                    pc = pcg.generate(frame)
                    capture_count += 1

                    if pc["count"] > 0:
                        viewer.update_points(pc["points"], pc["colors"])

                    pan, tilt = subscriber.get_orientation()
                    viewer.set_stats(
                        f"pan={pan:.2f} tilt={tilt:.2f}  "
                        f"frames={capture_count}"
                    )
                last_capture = now

            viewer.render()
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n  Stopped by user")

    print(f"\n  Frames displayed: {capture_count}")

    viewer.shutdown()
    subscriber.shutdown()
    rclpy.shutdown()
    print("  Done.")
    return 0


def run_ros2_server(args):
    """Run a TCP streaming server that reads from ROS2 camera topics.

    Unlike ``run_server()`` which creates its own MuJoCo simulation, this mode
    subscribes to the MuJoCo bridge's ROS2 camera topics and forwards frames
    over TCP to remote clients.  This keeps a single MuJoCo instance (the
    bridge) as the authoritative simulation, so the robot body/head movement
    from the master system is correctly reflected in the streamed images.

    When a client sends orientation via the TCP reverse channel, the server
    publishes it to ``/slave/camera/pan_tilt_cmd`` so the bridge moves the
    head joints, which in turn produces updated camera images.
    """
    import rclpy

    from teleop_system.modules.camera.ros2_rgbd_subscriber import ROS2RGBDSubscriber

    print("\n" + "=" * 70)
    print("  RGB-D ROS2 → TCP STREAM SERVER")
    print("=" * 70)

    try:
        if not rclpy.ok():
            rclpy.init()
    except RuntimeError:
        rclpy.init()

    # ROS2 subscriber acts as ICameraStream — receives images from the bridge
    subscriber = ROS2RGBDSubscriber(
        color_topic=args.color_topic or "/slave/camera/color/image_raw",
        depth_topic=args.depth_topic or "/slave/camera/depth/image_raw",
        info_topic=args.info_topic or "/slave/camera/camera_info",
        joint_states_topic="/mujoco/joint_states",
        pan_tilt_topic="/slave/camera/pan_tilt_cmd",
    )
    if not subscriber.initialize():
        print("  [ERROR] Failed to initialize ROS2 RGB-D subscriber")
        rclpy.shutdown()
        return 1

    # TCP streaming server — uses the subscriber as its camera source
    server = RGBDStreamServer(
        camera=subscriber,
        host=args.host,
        port=args.port,
        fps=args.stream_fps,
        jpeg_quality=args.jpeg_quality,
    )
    server.start()

    # Spin ROS2 in background thread so subscriptions receive data
    spin_thread = threading.Thread(
        target=_ros2_spin_loop, args=(subscriber._node,),
        daemon=True, name="ros2-server-spin",
    )
    spin_thread.start()

    print(f"  Subscribing to ROS2 camera topics:")
    print(f"    Color: {args.color_topic or '/slave/camera/color/image_raw'}")
    print(f"    Depth: {args.depth_topic or '/slave/camera/depth/image_raw'}")
    print(f"    Info:  {args.info_topic or '/slave/camera/camera_info'}")
    print(f"  TCP streaming on {args.host}:{args.port} @ {args.stream_fps} Hz")
    print(f"  JPEG quality: {args.jpeg_quality}")
    print(f"  Client pan/tilt → /slave/camera/pan_tilt_cmd (head control)")
    print(f"\n  Waiting for client... (Ctrl+C to stop)")
    print("=" * 70 + "\n")

    dt_capture = 1.0 / args.stream_fps
    last_capture = 0.0

    try:
        while server.is_running:
            now = time.monotonic()

            if now - last_capture >= dt_capture:
                # Only capture when we have data from ROS2
                if subscriber.is_connected():
                    server.capture()
                last_capture = now

            time.sleep(0.001)

            # Print periodic stats
            if server.frames_sent > 0 and server.frames_sent % 100 == 0:
                mb = server.bytes_sent / 1e6
                print(f"  Sent {server.frames_sent} frames, {mb:.1f} MB")
    except KeyboardInterrupt:
        print("\n  Stopping server...")

    server.stop()
    subscriber.shutdown()
    rclpy.shutdown()
    print("  Done.")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="RGB-D streaming demo (server/client/local)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--mode",
                        choices=["server", "client", "local", "ros2-viewer", "ros2-server"],
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
    parser.add_argument("--ros2-sync", action="store_true",
                        help="Enable ROS2 head sync in client mode "
                             "(forwards /mujoco/joint_states head orientation to server)")
    parser.add_argument("--color-topic", type=str, default=None,
                        help="ROS2 color image topic (ros2-viewer/ros2-server mode)")
    parser.add_argument("--depth-topic", type=str, default=None,
                        help="ROS2 depth image topic (ros2-viewer/ros2-server mode)")
    parser.add_argument("--info-topic", type=str, default=None,
                        help="ROS2 camera info topic (ros2-viewer/ros2-server mode)")
    args = parser.parse_args()

    if args.mode == "server":
        return run_server(args)
    elif args.mode == "client":
        return run_client(args)
    elif args.mode == "ros2-viewer":
        return run_ros2_viewer(args)
    elif args.mode == "ros2-server":
        return run_ros2_server(args)
    else:
        return run_local(args)


if __name__ == "__main__":
    sys.exit(main())
