#!/usr/bin/env python3
"""Standalone hand teleoperation test with MuJoCo viewer.

Tests the full pipeline WITHOUT ROS2:
  SimulatedHand -> HandController -> HandRetargeting -> MuJoCoHand -> MuJoCo physics -> Viewer

Usage:
    MUJOCO_GL=glfw python3 scripts/test_hand_teleop_standalone.py
    MUJOCO_GL=glfw python3 scripts/test_hand_teleop_standalone.py --no-viewer
    MUJOCO_GL=glfw python3 scripts/test_hand_teleop_standalone.py --duration 30
"""

import argparse
import sys
import time
from pathlib import Path

# Add project root
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np

from teleop_system.modules.hand_teleop.hand_controller import HandController
from teleop_system.modules.hand_teleop.retargeting import HandRetargeting
from teleop_system.simulators.mujoco_sim import MuJoCoSimulator, MuJoCoHand
from teleop_system.simulators.simulated_hand import SimulatedHand

# MuJoCo ctrl indices (from demo_teleop_sim.py)
CTRL_RIGHT_GRIPPER = 24
CTRL_LEFT_GRIPPER = 25

# Gripper finger qpos indices (from demo_teleop_sim.py)
QPOS_RIGHT_GRIPPER = [22, 23]  # gripper_finger_r1, r2
QPOS_LEFT_GRIPPER = [31, 32]   # gripper_finger_l1, l2


def create_hand_teleop_pipeline():
    """Create the full hand teleoperation pipeline.

    Returns:
        Tuple of (simulator, left_controller, right_controller, gloves_dict)
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

    # 2. Create MuJoCoHand adapters (ISlaveHand for gripper control)
    right_hand = MuJoCoHand(
        simulator=sim,
        side="right",
        ctrl_index=CTRL_RIGHT_GRIPPER,
        qpos_indices=QPOS_RIGHT_GRIPPER,
    )
    left_hand = MuJoCoHand(
        simulator=sim,
        side="left",
        ctrl_index=CTRL_LEFT_GRIPPER,
        qpos_indices=QPOS_LEFT_GRIPPER,
    )

    # 3. Create simulated gloves (different frequencies for visual distinction)
    right_glove = SimulatedHand(side="right", frequency=0.3, max_angle=1.4)
    left_glove = SimulatedHand(side="left", frequency=0.2, max_angle=1.4)
    right_glove.initialize()
    left_glove.initialize()

    print(f"  Gloves: right (freq=0.3Hz), left (freq=0.2Hz)")

    # 4. Create retargeting (identity mapping by default)
    right_rt = HandRetargeting(smoothing_alpha=0.3)
    left_rt = HandRetargeting(smoothing_alpha=0.3)

    # 5. Create HandControllers
    right_ctrl = HandController(
        glove=right_glove,
        hand=right_hand,
        retargeting=right_rt,
        max_joint_velocity=5.0,
        dt=0.01,
    )
    left_ctrl = HandController(
        glove=left_glove,
        hand=left_hand,
        retargeting=left_rt,
        max_joint_velocity=5.0,
        dt=0.01,
    )

    return sim, left_ctrl, right_ctrl, {"left": left_glove, "right": right_glove}


def run_test(duration: float = 20.0, launch_viewer: bool = True):
    """Run the hand teleoperation test.

    Args:
        duration: Test duration in seconds.
        launch_viewer: Whether to launch the MuJoCo passive viewer.
    """
    print("\n" + "=" * 60)
    print("  HAND TELEOPERATION STANDALONE TEST")
    print("=" * 60)

    sim, left_ctrl, right_ctrl, gloves = create_hand_teleop_pipeline()

    # Launch viewer
    viewer_handle = None
    if launch_viewer:
        viewer_handle = sim.launch_passive_viewer()
        if viewer_handle is not None:
            print("  Passive viewer launched (close window to stop)")
        else:
            print("  Warning: Failed to launch viewer")

    # Enable controllers
    left_ctrl.enable()
    right_ctrl.enable()
    print("  Both hand controllers enabled")

    print(f"\n  Running for {duration}s (Ctrl+C to stop)...")
    print(f"  {'Time':>6s}  {'L ctrl':>8s}  {'R ctrl':>8s}  "
          f"{'L grip qpos':>20s}  {'R grip qpos':>20s}")
    print("  " + "-" * 70)

    dt_control = 0.01   # 100 Hz control loop
    dt_physics = 0.002  # 500 Hz physics
    steps_per_control = int(dt_control / dt_physics)

    start_time = time.monotonic()
    control_count = 0
    cmd_count = 0

    try:
        while True:
            elapsed = time.monotonic() - start_time
            if elapsed >= duration:
                break

            # Check if viewer was closed
            if viewer_handle is not None and not viewer_handle.is_running():
                print("\n  Viewer closed by user")
                break

            # Run both controllers
            left_result = left_ctrl.update()
            right_result = right_ctrl.update()
            control_count += 1

            if left_result is not None or right_result is not None:
                cmd_count += 1

            # Step physics multiple times per control cycle
            for _ in range(steps_per_control):
                sim.step()

            # Sync viewer
            if viewer_handle is not None:
                sim.sync_viewer()

            # Print status every 2 seconds
            if control_count % 200 == 0:
                state = sim.get_state()
                l_grip = state.joint_positions[31:33] if len(state.joint_positions) > 33 else np.zeros(2)
                r_grip = state.joint_positions[22:24] if len(state.joint_positions) > 24 else np.zeros(2)
                l_ctrl_val = sim._data.ctrl[CTRL_LEFT_GRIPPER]
                r_ctrl_val = sim._data.ctrl[CTRL_RIGHT_GRIPPER]
                print(
                    f"  {elapsed:6.1f}s  {l_ctrl_val:8.3f}  {r_ctrl_val:8.3f}  "
                    f"{np.array2string(l_grip, precision=4):>20s}  "
                    f"{np.array2string(r_grip, precision=4):>20s}"
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
    print("\n  " + "=" * 60)
    print(f"  SUMMARY")
    print(f"    Duration:        {elapsed:.1f}s")
    print(f"    Control cycles:  {control_count}")
    print(f"    Commands sent:   {cmd_count}/{control_count} "
          f"({100*cmd_count/max(1,control_count):.1f}%)")
    print(f"    Final L ctrl:    {sim._data.ctrl[CTRL_LEFT_GRIPPER]:.4f}")
    print(f"    Final R ctrl:    {sim._data.ctrl[CTRL_RIGHT_GRIPPER]:.4f}")
    print("  " + "=" * 60)

    # Cleanup
    for glove in gloves.values():
        glove.shutdown()
    sim.shutdown()

    return cmd_count > 0


def main():
    parser = argparse.ArgumentParser(description="Standalone hand teleop test")
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
        print("\n  [PASS] Hand teleoperation test completed successfully")
    else:
        print("\n  [FAIL] No hand commands generated")
        sys.exit(1)


if __name__ == "__main__":
    main()
