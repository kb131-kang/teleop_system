#!/usr/bin/env python3
"""Standalone camera/head teleoperation test with MuJoCo viewer.

Tests the full pipeline WITHOUT ROS2:
  SimulatedTracker(HEAD) -> CameraController -> SimCameraStream -> MuJoCo ctrl[22:24]
  -> MuJoCo physics -> get_camera_rgbd("head_camera") -> PointCloudGenerator -> verify

Usage:
    MUJOCO_GL=glfw python3 scripts/test_camera_teleop_standalone.py
    MUJOCO_GL=glfw python3 scripts/test_camera_teleop_standalone.py --no-viewer
    MUJOCO_GL=glfw python3 scripts/test_camera_teleop_standalone.py --duration 30
    MUJOCO_GL=egl  python3 scripts/test_camera_teleop_standalone.py --no-viewer --no-pointcloud
"""

import argparse
import sys
import time
from pathlib import Path

# Add project root
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np

from teleop_system.interfaces.master_device import TrackerRole
from teleop_system.modules.camera.camera_controller import CameraController
from teleop_system.modules.camera.pointcloud_generator import PointCloudGenerator
from teleop_system.simulators.mujoco_sim import MuJoCoSimulator
from teleop_system.simulators.sim_camera_stream import SimCameraStream
from teleop_system.simulators.simulated_tracker import SimulatedTracker


def create_camera_teleop_pipeline():
    """Create the full camera teleoperation pipeline.

    Returns:
        Tuple of (simulator, camera_controller, sim_camera, tracker, pointcloud_gen)
    """
    # 1. Initialize MuJoCo simulator with rendering enabled
    sim = MuJoCoSimulator()
    mjcf_path = str(PROJECT_ROOT / "models" / "rby1" / "model_teleop.xml")
    success = sim.initialize({
        "mjcf_path": mjcf_path,
        "timestep": 0.002,
        "render": True,
        "render_width": 640,
        "render_height": 480,
    })
    if not success:
        raise RuntimeError(f"Failed to initialize MuJoCo from {mjcf_path}")

    print(f"  MuJoCo initialized: {len(sim.get_joint_names())} joints")

    # 2. Create SimCameraStream wrapping the simulator
    sim_camera = SimCameraStream(
        simulator=sim,
        camera_name="head_camera",
        pan_ctrl_index=22,
        tilt_ctrl_index=23,
    )
    sim_camera.initialize()
    print(f"  SimCameraStream initialized: camera='head_camera'")

    # 3. Create SimulatedTracker for HEAD (HMD)
    tracker = SimulatedTracker(
        role=TrackerRole.HEAD,
        amplitude=0.08,
        frequency=0.15,
    )
    tracker.initialize()
    print(f"  HMD tracker: HEAD role, freq=0.15Hz, amplitude=0.08")

    # 4. Create CameraController
    controller = CameraController(
        tracker=tracker,
        camera=sim_camera,
        smoothing_alpha=0.3,
        max_angular_velocity=2.0,
        dt=1.0 / 30.0,
    )

    # 5. Create PointCloudGenerator (numpy fallback)
    pcg = PointCloudGenerator(
        voxel_size=0.0,
        max_depth=5.0,
        min_depth=0.1,
        use_open3d=True,
    )

    return sim, controller, sim_camera, tracker, pcg


def run_test(duration: float = 20.0, launch_viewer: bool = True, enable_pointcloud: bool = True):
    """Run the camera teleoperation test.

    Args:
        duration: Test duration in seconds.
        launch_viewer: Whether to launch the MuJoCo passive viewer.
        enable_pointcloud: Whether to generate point clouds (slower).
    """
    print("\n" + "=" * 70)
    print("  CAMERA/HEAD TELEOPERATION STANDALONE TEST")
    print("=" * 70)

    sim, controller, sim_camera, tracker, pcg = create_camera_teleop_pipeline()

    # Launch viewer
    viewer_handle = None
    if launch_viewer:
        viewer_handle = sim.launch_passive_viewer()
        if viewer_handle is not None:
            print("  Passive viewer launched (close window to stop)")
        else:
            print("  Warning: Failed to launch viewer")

    # Enable controller
    controller.enable()
    print("  Camera controller enabled")

    print(f"\n  Running for {duration}s (Ctrl+C to stop)...")
    print(f"  {'Time':>6s}  {'Pan':>8s}  {'Tilt':>8s}  "
          f"{'RGB shape':>14s}  {'Depth range':>16s}  {'Points':>8s}")
    print("  " + "-" * 70)

    dt_control = 1.0 / 30.0   # 30 Hz control loop
    dt_physics = 0.002         # 500 Hz physics
    steps_per_control = int(dt_control / dt_physics)

    start_time = time.monotonic()
    control_count = 0
    cmd_count = 0
    total_points = 0

    try:
        while True:
            elapsed = time.monotonic() - start_time
            if elapsed >= duration:
                break

            # Check if viewer was closed
            if viewer_handle is not None and not viewer_handle.is_running():
                print("\n  Viewer closed by user")
                break

            # Run camera controller
            result = controller.update()
            control_count += 1

            if result is not None:
                cmd_count += 1

            # Step physics multiple times per control cycle
            for _ in range(steps_per_control):
                sim.step()

            # Sync viewer
            if viewer_handle is not None:
                sim.sync_viewer()

            # Print status every 2 seconds
            if control_count % 60 == 0:
                pan, tilt = sim_camera.get_orientation()

                # Render RGB-D
                frame = sim_camera.get_rgbd()
                rgb_shape = f"{frame.rgb.shape}" if frame.rgb is not None else "None"
                depth_range = "N/A"
                if frame.depth is not None and frame.depth.size > 0:
                    depth_range = f"[{frame.depth.min():.2f}, {frame.depth.max():.2f}]"

                # Optional point cloud generation
                pc_count = "-"
                if enable_pointcloud:
                    pc = pcg.generate(frame)
                    pc_count = str(pc["count"])
                    total_points += pc["count"]

                print(
                    f"  {elapsed:6.1f}s  {pan:8.4f}  {tilt:8.4f}  "
                    f"{rgb_shape:>14s}  {depth_range:>16s}  {pc_count:>8s}"
                )

            # Sleep to maintain control rate
            next_time = start_time + control_count * dt_control
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n  Stopped by user")

    # Summary
    elapsed = time.monotonic() - start_time
    print("\n  " + "=" * 70)
    print(f"  SUMMARY")
    print(f"    Duration:        {elapsed:.1f}s")
    print(f"    Control cycles:  {control_count}")
    print(f"    Commands sent:   {cmd_count}/{control_count} "
          f"({100*cmd_count/max(1,control_count):.1f}%)")
    pan, tilt = sim_camera.get_orientation()
    print(f"    Final pan:       {pan:.4f} rad")
    print(f"    Final tilt:      {tilt:.4f} rad")
    if enable_pointcloud:
        print(f"    Total points:    {total_points}")

    # Final RGB-D verification
    frame = sim_camera.get_rgbd()
    print(f"    Final RGB:       shape={frame.rgb.shape}, mean={frame.rgb.mean():.1f}")
    print(f"    Final depth:     shape={frame.depth.shape}, "
          f"range=[{frame.depth.min():.2f}, {frame.depth.max():.2f}]")
    print(f"    Intrinsics fx:   {frame.intrinsics[0,0]:.1f}")
    print("  " + "=" * 70)

    # Cleanup
    tracker.shutdown()
    sim_camera.shutdown()
    sim.shutdown()

    return cmd_count > 0


def main():
    parser = argparse.ArgumentParser(description="Standalone camera/head teleop test")
    parser.add_argument("--duration", type=float, default=20.0,
                        help="Test duration in seconds")
    parser.add_argument("--no-viewer", action="store_true",
                        help="Run without MuJoCo viewer")
    parser.add_argument("--no-pointcloud", action="store_true",
                        help="Skip point cloud generation (faster)")
    args = parser.parse_args()

    success = run_test(
        duration=args.duration,
        launch_viewer=not args.no_viewer,
        enable_pointcloud=not args.no_pointcloud,
    )

    if success:
        print("\n  [PASS] Camera/head teleoperation test completed successfully")
    else:
        print("\n  [FAIL] No camera commands generated")
        sys.exit(1)


if __name__ == "__main__":
    main()
