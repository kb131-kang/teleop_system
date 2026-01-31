#!/usr/bin/env python3
"""Standalone arm teleoperation test with MuJoCo viewer.

Tests the full pipeline WITHOUT ROS2:
  SimulatedTracker -> ArmController -> PinkIKSolver -> MuJoCoArm -> MuJoCo physics -> Viewer

If PinkIKSolver is not available, falls back to SimpleProportionalMapper.

Usage:
    MUJOCO_GL=glfw python3 scripts/test_arm_teleop_standalone.py
    MUJOCO_GL=glfw python3 scripts/test_arm_teleop_standalone.py --no-viewer
    MUJOCO_GL=glfw python3 scripts/test_arm_teleop_standalone.py --duration 30
    MUJOCO_GL=glfw python3 scripts/test_arm_teleop_standalone.py --use-proportional
"""

import argparse
import os
import sys
import time
from pathlib import Path

# Add project root
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import mujoco
import numpy as np

from teleop_system.interfaces.master_device import TrackerRole
from teleop_system.interfaces.slave_robot import ArmSide
from teleop_system.modules.arm_teleop.arm_controller import ArmController
from teleop_system.simulators.mujoco_sim import MuJoCoSimulator, MuJoCoArm
from teleop_system.simulators.simulated_tracker import SimulatedTracker
from teleop_system.solvers.proportional_mapper import create_ik_solver, SimpleProportionalMapper

# ── MuJoCo ctrl index mapping ──
CTRL_TORSO = slice(2, 8)
CTRL_RIGHT_ARM = slice(8, 15)
CTRL_LEFT_ARM = slice(15, 22)


def create_arm_teleop_pipeline(use_proportional: bool = False):
    """Create the full arm teleoperation pipeline.

    Returns:
        Tuple of (simulator, arm_controller, trackers_dict)
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

    # 2. Create MuJoCoArm adapters (maps ISlaveArm to MuJoCo ctrl indices)
    right_arm = MuJoCoArm(
        simulator=sim,
        side=ArmSide.RIGHT,
        joint_indices=list(range(CTRL_RIGHT_ARM.start, CTRL_RIGHT_ARM.stop)),
    )
    left_arm = MuJoCoArm(
        simulator=sim,
        side=ArmSide.LEFT,
        joint_indices=list(range(CTRL_LEFT_ARM.start, CTRL_LEFT_ARM.stop)),
    )
    torso_arm = MuJoCoArm(
        simulator=sim,
        side=ArmSide.TORSO,
        joint_indices=list(range(CTRL_TORSO.start, CTRL_TORSO.stop)),
    )

    # 3. Create IK solver
    ik_solver = create_ik_solver(prefer_pink=not use_proportional)
    use_pink = False

    if isinstance(ik_solver, SimpleProportionalMapper):
        ik_solver.initialize("", nq=20, chain_joint_indices={
            "torso": list(range(0, 6)),
            "right_arm": list(range(6, 13)),
            "left_arm": list(range(13, 20)),
        })
        print(f"  IK solver: SimpleProportionalMapper (fallback)")
    else:
        urdf_path = str(PROJECT_ROOT / "models" / "rby1" / "urdf" / "model_pinocchio.urdf")
        success = ik_solver.initialize(
            urdf_path,
            ee_frames={
                "right_arm": "ee_right",
                "left_arm": "ee_left",
                "torso": "link_torso_5",
            },
        )
        if success:
            print(f"  IK solver: PinkIKSolver ({ik_solver.get_joint_count()} DOF)")
            use_pink = True
        else:
            print(f"  PinkIKSolver init failed, falling back to proportional")
            ik_solver = SimpleProportionalMapper()
            ik_solver.initialize("", nq=20, chain_joint_indices={
                "torso": list(range(0, 6)),
                "right_arm": list(range(6, 13)),
                "left_arm": list(range(13, 20)),
            })

    # 4. Create simulated trackers
    trackers = {
        TrackerRole.RIGHT_HAND: SimulatedTracker(
            role=TrackerRole.RIGHT_HAND,
            amplitude=0.08,
            frequency=0.3,
            phase_offset=0.0,
        ),
        TrackerRole.LEFT_HAND: SimulatedTracker(
            role=TrackerRole.LEFT_HAND,
            amplitude=0.08,
            frequency=0.3,
            phase_offset=np.pi / 2,
        ),
        TrackerRole.WAIST: SimulatedTracker(
            role=TrackerRole.WAIST,
            amplitude=0.04,
            frequency=0.15,
        ),
    }
    for tracker in trackers.values():
        tracker.initialize()

    print(f"  Trackers: {[r.name for r in trackers]}")

    # 5. Create ArmController with solver-appropriate joint index map
    if use_pink:
        # PinkIKSolver uses pinocchio q-vector indices
        joint_index_map = {
            ArmSide.TORSO: [4, 5, 6, 7, 8, 9],
            ArmSide.RIGHT: [21, 22, 23, 24, 25, 26, 27],
            ArmSide.LEFT: [12, 13, 14, 15, 16, 17, 18],
        }
    else:
        # SimpleProportionalMapper uses compact 20-DOF indices
        joint_index_map = {
            ArmSide.TORSO: list(range(0, 6)),
            ArmSide.RIGHT: list(range(6, 13)),
            ArmSide.LEFT: list(range(13, 20)),
        }

    controller = ArmController(
        ik_solver=ik_solver,
        trackers=trackers,
        arms={
            ArmSide.RIGHT: right_arm,
            ArmSide.LEFT: left_arm,
            ArmSide.TORSO: torso_arm,
        },
        dt=0.01,
        joint_index_map=joint_index_map,
    )

    return sim, controller, trackers


def run_test(duration: float = 20.0, launch_viewer: bool = True, use_proportional: bool = False):
    """Run the arm teleoperation test.

    Args:
        duration: Test duration in seconds.
        launch_viewer: Whether to launch the MuJoCo passive viewer.
        use_proportional: Force proportional mapper instead of PinkIKSolver.
    """
    print("\n" + "=" * 60)
    print("  ARM TELEOPERATION STANDALONE TEST")
    print("=" * 60)

    sim, controller, trackers = create_arm_teleop_pipeline(use_proportional)

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
    print(f"\n  Running for {duration}s (Ctrl+C to stop)...")
    print(f"  {'Time':>6s}  {'IK OK':>5s}  {'PosErr':>8s}  {'Right Arm [0:3]':>24s}  {'Left Arm [0:3]':>24s}")
    print("  " + "-" * 75)

    dt_control = 0.01   # 100 Hz control loop
    dt_physics = 0.002  # 500 Hz physics
    steps_per_control = int(dt_control / dt_physics)

    start_time = time.monotonic()
    control_count = 0
    ik_success_count = 0

    try:
        while True:
            elapsed = time.monotonic() - start_time
            if elapsed >= duration:
                break

            # Check if viewer was closed
            if viewer_handle is not None and not viewer_handle.is_running():
                print("\n  Viewer closed by user")
                break

            # Run ArmController update
            result = controller.update()
            control_count += 1

            if result.success:
                ik_success_count += 1

            # Step physics multiple times per control cycle
            for _ in range(steps_per_control):
                sim.step()

            # Sync viewer
            if viewer_handle is not None:
                sim.sync_viewer()

            # Print status every 2 seconds
            if control_count % 200 == 0:
                state = sim.get_state()
                right_q = state.joint_positions[15:22] if len(state.joint_positions) > 22 else np.zeros(7)
                left_q = state.joint_positions[24:31] if len(state.joint_positions) > 31 else np.zeros(7)
                print(
                    f"  {elapsed:6.1f}s  {'YES' if result.success else ' NO':>5s}  "
                    f"{result.error_position:8.4f}  "
                    f"{np.array2string(right_q[:3], precision=3, suppress_small=True):>24s}  "
                    f"{np.array2string(left_q[:3], precision=3, suppress_small=True):>24s}"
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
    print(f"    IK successes:    {ik_success_count}/{control_count} "
          f"({100*ik_success_count/max(1,control_count):.1f}%)")

    # Final joint state
    state = sim.get_state()
    print(f"    Final qpos size: {len(state.joint_positions)}")
    print(f"    Right arm:       {state.joint_positions[15:22].round(3)}")
    print(f"    Left arm:        {state.joint_positions[24:31].round(3)}")
    print(f"    Torso:           {state.joint_positions[9:15].round(3)}")
    print("  " + "=" * 60)

    # Cleanup
    for tracker in trackers.values():
        tracker.shutdown()
    sim.shutdown()

    return ik_success_count > 0


def main():
    parser = argparse.ArgumentParser(description="Standalone arm teleop test")
    parser.add_argument("--duration", type=float, default=20.0,
                        help="Test duration in seconds")
    parser.add_argument("--no-viewer", action="store_true",
                        help="Run without MuJoCo viewer")
    parser.add_argument("--use-proportional", action="store_true",
                        help="Force SimpleProportionalMapper instead of PinkIKSolver")
    args = parser.parse_args()

    success = run_test(
        duration=args.duration,
        launch_viewer=not args.no_viewer,
        use_proportional=args.use_proportional,
    )

    if success:
        print("\n  [PASS] Arm teleoperation test completed successfully")
    else:
        print("\n  [FAIL] No successful IK solves")
        sys.exit(1)


if __name__ == "__main__":
    main()
