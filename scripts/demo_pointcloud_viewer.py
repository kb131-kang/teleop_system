#!/usr/bin/env python3
"""Real-time point cloud streaming demo with 3D viewer.

Runs the full camera pipeline and visualizes the point cloud in real-time:
  SimulatedTracker(HEAD) -> CameraController -> SimCameraStream
  -> MuJoCo physics -> RGB-D rendering -> PointCloudGenerator
  -> OpenGL point cloud viewer (GLFW window)

The head continuously moves via simulated HMD orientation,
and the point cloud updates in real-time as the view changes.

MuJoCo uses EGL for off-screen RGB-D rendering while the viewer
uses a separate GLFW window for display (no context conflict).

Usage:
    python3 scripts/demo_pointcloud_viewer.py
    python3 scripts/demo_pointcloud_viewer.py --with-robot-viewer
    python3 scripts/demo_pointcloud_viewer.py --duration 60
    python3 scripts/demo_pointcloud_viewer.py --camera-fps 30

Controls (in point cloud window):
    Left-drag:   Rotate view
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

# MuJoCo uses EGL for off-screen rendering; GLFW is used only for the viewer window.
# This must be set BEFORE importing mujoco.
if "MUJOCO_GL" not in os.environ:
    os.environ["MUJOCO_GL"] = "egl"

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np

from teleop_system.interfaces.master_device import TrackerRole
from teleop_system.modules.camera.camera_controller import CameraController
from teleop_system.modules.camera.pointcloud_generator import PointCloudGenerator
from teleop_system.modules.camera.pointcloud_viewer import PointCloudViewer
from teleop_system.simulators.mujoco_sim import MuJoCoSimulator
from teleop_system.simulators.sim_camera_stream import SimCameraStream
from teleop_system.simulators.simulated_tracker import SimulatedTracker


def main():
    parser = argparse.ArgumentParser(
        description="Real-time point cloud streaming demo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--duration", type=float, default=0,
                        help="Duration in seconds (0 = unlimited)")
    parser.add_argument("--camera-fps", type=float, default=15.0,
                        help="RGB-D capture rate (Hz)")
    parser.add_argument("--control-fps", type=float, default=30.0,
                        help="Head control rate (Hz)")
    parser.add_argument("--render-width", type=int, default=640,
                        help="Camera render width")
    parser.add_argument("--render-height", type=int, default=480,
                        help="Camera render height")
    parser.add_argument("--voxel-size", type=float, default=0.02,
                        help="Voxel downsampling size in meters (0=none)")
    parser.add_argument("--max-depth", type=float, default=10.0,
                        help="Max depth for point cloud (meters)")
    parser.add_argument("--point-size", type=float, default=2.0,
                        help="OpenGL point size")
    parser.add_argument("--with-robot-viewer", action="store_true",
                        help="Also launch MuJoCo passive viewer (shows robot)")
    parser.add_argument("--hmd-frequency", type=float, default=0.15,
                        help="Simulated HMD oscillation frequency (Hz)")
    parser.add_argument("--hmd-amplitude", type=float, default=0.08,
                        help="Simulated HMD oscillation amplitude")
    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("  REAL-TIME POINT CLOUD STREAMING DEMO")
    print("=" * 70)

    # ── 1. Initialize MuJoCo ──
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
        print("  [ERROR] Failed to initialize MuJoCo")
        return 1

    print(f"  MuJoCo: {len(sim.get_joint_names())} joints, "
          f"render={args.render_width}x{args.render_height}")

    # ── 2. Camera stream ──
    cam = SimCameraStream(simulator=sim)
    cam.initialize()

    # ── 3. HMD tracker ──
    tracker = SimulatedTracker(
        role=TrackerRole.HEAD,
        amplitude=args.hmd_amplitude,
        frequency=args.hmd_frequency,
    )
    tracker.initialize()

    # ── 4. Camera controller ──
    controller = CameraController(
        tracker=tracker,
        camera=cam,
        smoothing_alpha=0.3,
        max_angular_velocity=2.0,
        dt=1.0 / args.control_fps,
    )
    controller.enable()

    # ── 5. Point cloud generator ──
    pcg = PointCloudGenerator(
        voxel_size=args.voxel_size,
        max_depth=args.max_depth,
        min_depth=0.1,
        use_open3d=True,  # falls back to numpy automatically
    )

    # ── 6. Point cloud viewer ──
    viewer = PointCloudViewer(
        width=960,
        height=720,
        title="Head Camera Point Cloud",
        point_size=args.point_size,
    )
    if not viewer.initialize():
        print("  [ERROR] Failed to initialize viewer")
        sim.shutdown()
        return 1

    # Set initial camera view
    viewer._cam_distance = 8.0
    viewer._cam_yaw = 180.0
    viewer._cam_pitch = -20.0

    # ── 7. Optional robot viewer ──
    # Note: --with-robot-viewer requires MUJOCO_GL=glfw (not default EGL).
    # Run with: MUJOCO_GL=glfw python3 scripts/demo_pointcloud_viewer.py --with-robot-viewer
    robot_viewer = None
    if args.with_robot_viewer:
        robot_viewer = sim.launch_passive_viewer()
        if robot_viewer:
            print("  Robot viewer launched")
        else:
            print("  Warning: Robot viewer failed (need MUJOCO_GL=glfw)")

    # ── Timing setup ──
    dt_control = 1.0 / args.control_fps
    dt_camera = 1.0 / args.camera_fps
    dt_physics = 0.002
    steps_per_control = max(1, int(dt_control / dt_physics))

    print(f"  Head control: {args.control_fps} Hz")
    print(f"  Camera capture: {args.camera_fps} Hz")
    print(f"  Point cloud: voxel={args.voxel_size}m, max_depth={args.max_depth}m")
    print(f"\n  Viewer controls: Left-drag=Rotate, Right-drag=Pan, "
          f"Scroll=Zoom, R=Reset, Q=Quit")
    print("=" * 70 + "\n")

    # ── Main loop ──
    start_time = time.monotonic()
    last_control_time = start_time
    last_camera_time = start_time
    control_count = 0
    capture_count = 0

    try:
        while viewer.is_running():
            now = time.monotonic()
            elapsed = now - start_time

            if args.duration > 0 and elapsed >= args.duration:
                break

            # Check robot viewer
            if robot_viewer is not None and not robot_viewer.is_running():
                break

            # ── Control loop (30 Hz) ──
            if now - last_control_time >= dt_control:
                result = controller.update()
                control_count += 1
                last_control_time = now

                # Step physics
                for _ in range(steps_per_control):
                    sim.step()

                # Sync robot viewer
                if robot_viewer is not None:
                    sim.sync_viewer()

            # ── Camera capture + point cloud update (15 Hz) ──
            if now - last_camera_time >= dt_camera:
                frame = cam.get_rgbd()
                pc = pcg.generate(frame)
                capture_count += 1
                last_camera_time = now

                if pc["count"] > 0:
                    viewer.update_points(pc["points"], pc["colors"])

                # Update stats
                pan, tilt = cam.get_orientation()
                viewer.set_stats(
                    f"pan={pan:.2f} tilt={tilt:.2f}  "
                    f"cap={capture_count}  "
                    f"depth=[{frame.depth.min():.1f},{frame.depth.max():.1f}]"
                )

            # ── Render viewer (as fast as possible) ──
            viewer.render()

            # Small sleep to not burn 100% CPU
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n  Stopped by user")

    # ── Summary ──
    elapsed = time.monotonic() - start_time
    print(f"\n  Duration:    {elapsed:.1f}s")
    print(f"  Controls:    {control_count} ({control_count/max(elapsed,1):.1f} Hz)")
    print(f"  Captures:    {capture_count} ({capture_count/max(elapsed,1):.1f} Hz)")

    pan, tilt = cam.get_orientation()
    print(f"  Final pan:   {pan:.4f} rad")
    print(f"  Final tilt:  {tilt:.4f} rad")

    # Cleanup
    viewer.shutdown()
    tracker.shutdown()
    cam.shutdown()
    sim.shutdown()

    print("  Done.\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
