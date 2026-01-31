#!/usr/bin/env python3
"""Standalone locomotion teleoperation test with MuJoCo viewer.

Tests the full pipeline WITHOUT ROS2:
  SimulatedTracker (feet) -> LocomotionController -> MuJoCoBase -> MuJoCo physics -> Viewer

Usage:
    MUJOCO_GL=glfw python3 scripts/test_locomotion_standalone.py
    MUJOCO_GL=glfw python3 scripts/test_locomotion_standalone.py --no-viewer
    MUJOCO_GL=glfw python3 scripts/test_locomotion_standalone.py --duration 30
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
from teleop_system.modules.locomotion.locomotion_controller import LocomotionController
from teleop_system.simulators.mujoco_sim import MuJoCoSimulator, MuJoCoBase
from teleop_system.simulators.simulated_tracker import SimulatedTracker


def create_locomotion_pipeline():
    """Create the full locomotion pipeline.

    Returns:
        Tuple of (simulator, controller, trackers_dict)
    """
    # 1. Initialize MuJoCo simulator
    sim = MuJoCoSimulator()
    mjcf_path = str(PROJECT_ROOT / "models" / "rby1" / "model_teleop.xml")
    success = sim.initialize({
        "mjcf_path": mjcf_path,
        "timestep": 0.002,
        "render": False,
    })
    if not success:
        raise RuntimeError(f"Failed to initialize MuJoCo from {mjcf_path}")

    print(f"  MuJoCo initialized: {len(sim.get_joint_names())} joints")

    # 2. Create MuJoCoBase adapter
    base = MuJoCoBase(simulator=sim)

    # 3. Create simulated foot trackers (anti-phase for walking)
    trackers = {
        TrackerRole.RIGHT_FOOT: SimulatedTracker(
            role=TrackerRole.RIGHT_FOOT,
            amplitude=0.05,
            frequency=0.5,
            phase_offset=0.0,
        ),
        TrackerRole.LEFT_FOOT: SimulatedTracker(
            role=TrackerRole.LEFT_FOOT,
            amplitude=0.05,
            frequency=0.5,
            phase_offset=np.pi,
        ),
    }
    for tracker in trackers.values():
        tracker.initialize()

    print(f"  Trackers: {[r.name for r in trackers]}")

    # 4. Create LocomotionController
    controller = LocomotionController(
        left_foot_tracker=trackers[TrackerRole.LEFT_FOOT],
        right_foot_tracker=trackers[TrackerRole.RIGHT_FOOT],
        base=base,
        deadzone=0.02,
        linear_scale=1.0,
        angular_scale=1.0,
        max_linear_velocity=0.5,
        max_angular_velocity=1.0,
        smoothing_window=5,
    )

    return sim, controller, trackers


def run_test(duration: float = 20.0, launch_viewer: bool = True):
    """Run the locomotion teleoperation test.

    Args:
        duration: Test duration in seconds.
        launch_viewer: Whether to launch the MuJoCo passive viewer.
    """
    print("\n" + "=" * 60)
    print("  LOCOMOTION TELEOPERATION STANDALONE TEST")
    print("=" * 60)

    sim, controller, trackers = create_locomotion_pipeline()

    # Launch viewer
    viewer_handle = None
    if launch_viewer:
        viewer_handle = sim.launch_passive_viewer()
        if viewer_handle is not None:
            print("  Passive viewer launched (close window to stop)")
        else:
            print("  Warning: Failed to launch viewer")

    # Enable controller (calibrates foot positions)
    ok = controller.enable()
    print(f"  Controller enabled: {ok}")
    if not ok:
        print("  WARNING: Calibration failed, will retry after trackers provide data")
        # Trackers may need a moment to generate valid poses
        time.sleep(0.1)
        ok = controller.enable()
        print(f"  Retry calibration: {ok}")

    print(f"\n  Running for {duration}s (Ctrl+C to stop)...")
    print(f"  {'Time':>6s}  {'lin_x':>8s}  {'lin_y':>8s}  {'ang_z':>8s}  {'Pos X':>8s}  {'Pos Y':>8s}  {'Yaw':>8s}")
    print("  " + "-" * 60)

    dt_control = 0.02   # 50 Hz control loop
    dt_physics = 0.002  # 500 Hz physics
    steps_per_control = int(dt_control / dt_physics)

    start_time = time.monotonic()
    control_count = 0
    nonzero_cmd_count = 0

    try:
        while True:
            elapsed = time.monotonic() - start_time
            if elapsed >= duration:
                break

            # Check if viewer was closed
            if viewer_handle is not None and not viewer_handle.is_running():
                print("\n  Viewer closed by user")
                break

            # Run LocomotionController update
            cmd = controller.update()
            control_count += 1

            if cmd.linear_x != 0.0 or cmd.linear_y != 0.0 or cmd.angular_z != 0.0:
                nonzero_cmd_count += 1

            # Step physics multiple times per control cycle
            for _ in range(steps_per_control):
                sim.step()

            # Sync viewer
            if viewer_handle is not None:
                sim.sync_viewer()

            # Print status every 2 seconds
            if control_count % 100 == 0:
                state = sim.get_state()
                pos = state.base_position
                ori = state.base_orientation
                # Extract yaw from quaternion (approximate)
                from teleop_system.utils.transforms import quat_to_euler
                euler = quat_to_euler(ori)
                yaw = euler[2]

                print(
                    f"  {elapsed:6.1f}s  {cmd.linear_x:8.4f}  {cmd.linear_y:8.4f}  "
                    f"{cmd.angular_z:8.4f}  {pos[0]:8.3f}  {pos[1]:8.3f}  {yaw:8.3f}"
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
    state = sim.get_state()
    pos = state.base_position

    print("\n  " + "=" * 60)
    print(f"  SUMMARY")
    print(f"    Duration:         {elapsed:.1f}s")
    print(f"    Control cycles:   {control_count}")
    print(f"    Non-zero cmds:    {nonzero_cmd_count}/{control_count} "
          f"({100*nonzero_cmd_count/max(1,control_count):.1f}%)")
    print(f"    Final position:   x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f}")
    print(f"    Moved from origin: {np.linalg.norm(pos[:2]):.3f} m")
    print("  " + "=" * 60)

    # Cleanup
    for tracker in trackers.values():
        tracker.shutdown()
    sim.shutdown()

    return nonzero_cmd_count > 0


def main():
    parser = argparse.ArgumentParser(description="Standalone locomotion test")
    parser.add_argument("--duration", type=float, default=20.0,
                        help="Test duration in seconds")
    parser.add_argument("--no-viewer", action="store_true",
                        help="Run without MuJoCo viewer")
    args = parser.parse_args()

    success = run_test(
        duration=args.duration,
        launch_viewer=not args.no_viewer,
    )

    if success:
        print("\n  [PASS] Locomotion test completed successfully")
    else:
        print("\n  [FAIL] No non-zero velocity commands generated")
        sys.exit(1)


if __name__ == "__main__":
    main()
