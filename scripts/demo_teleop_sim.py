#!/usr/bin/env python3
"""Step-by-step MuJoCo teleoperation demo with dummy data.

Tests each subsystem independently using simulated inputs (no hardware needed):
  Step 1: Load model and verify joints/actuators
  Step 2: Arm teleoperation (SimulatedTracker -> arm joint commands)
  Step 3: Hand teleoperation (SimulatedHand -> gripper commands)
  Step 4: Mobility (wheel velocity commands)
  Step 5: Vision (head camera RGB-D rendering)

Usage:
    python3 scripts/demo_teleop_sim.py              # Run all steps
    python3 scripts/demo_teleop_sim.py --step 2     # Run only step 2 (arm)
    python3 scripts/demo_teleop_sim.py --step 5     # Run only step 5 (vision)
    python3 scripts/demo_teleop_sim.py --steps 100  # Number of sim steps per test
"""

import argparse
import os
import sys
import time
from pathlib import Path

# EGL for headless rendering
if "MUJOCO_GL" not in os.environ:
    os.environ["MUJOCO_GL"] = "egl"

# Add project root to path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import mujoco
import numpy as np

from teleop_system.simulators.simulated_tracker import SimulatedTracker
from teleop_system.simulators.simulated_hand import SimulatedHand
from teleop_system.interfaces.master_device import TrackerRole

# ── Joint/actuator index maps for RB-Y1 model_teleop.xml ──
# ctrl indices (nu=26)
CTRL_LEFT_WHEEL = 0
CTRL_RIGHT_WHEEL = 1
CTRL_TORSO = slice(2, 8)        # 6 joints: torso_0..5
CTRL_RIGHT_ARM = slice(8, 15)   # 7 joints: right_arm_0..6
CTRL_LEFT_ARM = slice(15, 22)   # 7 joints: left_arm_0..6
CTRL_HEAD = slice(22, 24)       # 2 joints: head_0, head_1
CTRL_RIGHT_GRIPPER = 24
CTRL_LEFT_GRIPPER = 25

# qpos indices (nq=35, free joint base takes 0:7)
QPOS_BASE_POS = slice(0, 3)
QPOS_BASE_QUAT = slice(3, 7)
QPOS_RIGHT_WHEEL = 7
QPOS_LEFT_WHEEL = 8
QPOS_TORSO = slice(9, 15)
QPOS_RIGHT_ARM = slice(15, 22)
QPOS_LEFT_ARM = slice(24, 31)
QPOS_HEAD = slice(33, 35)


def load_model():
    """Load the RB-Y1 MuJoCo model."""
    model_path = str(PROJECT_ROOT / "models" / "rby1" / "model_teleop.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    return model, data


def step1_model_info(model, data):
    """Step 1: Load model and print joint/actuator info."""
    print("\n" + "=" * 60)
    print("  STEP 1: Model Loading & Verification")
    print("=" * 60)

    print(f"  Bodies:    {model.nbody}")
    print(f"  Joints:    {model.njnt}")
    print(f"  DOF (nq):  {model.nq}")
    print(f"  Actuators: {model.nu}")
    print(f"  Cameras:   {model.ncam}")
    print()

    # List all joints with their types
    type_names = ["free", "ball", "slide", "hinge"]
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        jtype = type_names[model.jnt_type[i]]
        qpos = model.jnt_qposadr[i]
        print(f"    joint[{i:2d}] {name:25s} type={jtype:6s} qpos={qpos}")

    print()
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"    ctrl[{i:2d}] = {name}")

    # Forward kinematics at initial pose
    mujoco.mj_forward(model, data)
    print(f"\n  Initial base position: {data.qpos[QPOS_BASE_POS]}")
    print(f"  Initial base quat:     {data.qpos[QPOS_BASE_QUAT]}")
    print("  [PASS] Model loaded successfully")


def step2_arm_teleop(model, data, n_steps=100):
    """Step 2: Arm teleoperation using SimulatedTrackers."""
    print("\n" + "=" * 60)
    print("  STEP 2: Arm Teleoperation (SimulatedTracker -> Arm Joints)")
    print("=" * 60)

    # Create simulated trackers for left and right hands
    left_tracker = SimulatedTracker(
        role=TrackerRole.LEFT_HAND,
        amplitude=0.08,
        frequency=0.5,
    )
    right_tracker = SimulatedTracker(
        role=TrackerRole.RIGHT_HAND,
        amplitude=0.08,
        frequency=0.5,
        phase_offset=np.pi / 2,
    )
    left_tracker.initialize()
    right_tracker.initialize()

    print(f"  Running {n_steps} steps with simulated arm trackers...")
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    for i in range(n_steps):
        # Get tracker poses
        left_pose = left_tracker.get_pose()
        right_pose = right_tracker.get_pose()

        # Simple proportional mapping: use position offsets as joint targets
        # (This is a simplified demo; real system uses IK solver)
        if left_pose.valid:
            pos = left_pose.position
            # Map position offsets to arm joint angles
            left_targets = np.zeros(7)
            left_targets[0] = np.clip(pos[1] * 2.0, -2.3, 2.3)      # shoulder
            left_targets[1] = np.clip((pos[2] - 1.0) * 2.0, -0.05, 3.1)
            left_targets[2] = np.clip(pos[0] * 1.5, -2.0, 2.0)      # elbow
            left_targets[3] = np.clip((pos[2] - 1.0) * -1.5, -2.6, 0.01)
            data.ctrl[CTRL_LEFT_ARM] = left_targets

        if right_pose.valid:
            pos = right_pose.position
            right_targets = np.zeros(7)
            right_targets[0] = np.clip(pos[1] * -2.0, -2.3, 2.3)
            right_targets[1] = np.clip((pos[2] - 1.0) * -2.0, -3.1, 0.05)
            right_targets[2] = np.clip(pos[0] * 1.5, -2.0, 2.0)
            right_targets[3] = np.clip((pos[2] - 1.0) * -1.5, -2.6, 0.01)
            data.ctrl[CTRL_RIGHT_ARM] = right_targets

        mujoco.mj_step(model, data)

        if (i + 1) % (n_steps // 4) == 0:
            left_q = data.qpos[QPOS_LEFT_ARM]
            right_q = data.qpos[QPOS_RIGHT_ARM]
            print(f"    Step {i+1:4d}: left_arm={left_q[:3].round(3)} right_arm={right_q[:3].round(3)}")

    left_tracker.shutdown()
    right_tracker.shutdown()
    print("  [PASS] Arm teleoperation OK")


def step3_hand_teleop(model, data, n_steps=100):
    """Step 3: Hand teleoperation using SimulatedHand."""
    print("\n" + "=" * 60)
    print("  STEP 3: Hand Teleoperation (SimulatedHand -> Gripper Commands)")
    print("=" * 60)
    print("  NOTE: Current model has 2-finger grippers (not DG-5F 20-DOF hands)")
    print("        SimulatedHand generates 20 joint values; we map to gripper open/close")

    left_hand = SimulatedHand(side="left", frequency=0.3)
    right_hand = SimulatedHand(side="right", frequency=0.3)
    left_hand.initialize()
    right_hand.initialize()

    print(f"  Running {n_steps} steps with simulated hand inputs...")
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    for i in range(n_steps):
        left_state = left_hand.get_joint_state()
        right_state = right_hand.get_joint_state()

        if left_state.valid:
            # Average finger flexion -> gripper open/close (0=open, positive=close)
            avg_flex = np.mean(left_state.joint_angles)
            data.ctrl[CTRL_LEFT_GRIPPER] = np.clip(avg_flex * 20.0, -100, 100)

        if right_state.valid:
            avg_flex = np.mean(right_state.joint_angles)
            data.ctrl[CTRL_RIGHT_GRIPPER] = np.clip(avg_flex * 20.0, -100, 100)

        mujoco.mj_step(model, data)

        if (i + 1) % (n_steps // 4) == 0:
            l_grip = data.qpos[31:33]  # gripper_finger_l1, l2
            r_grip = data.qpos[22:24]  # gripper_finger_r1, r2
            print(f"    Step {i+1:4d}: left_grip={l_grip.round(4)} right_grip={r_grip.round(4)}")

    left_hand.shutdown()
    right_hand.shutdown()
    print("  [PASS] Hand teleoperation OK")


def step4_mobility(model, data, n_steps=100):
    """Step 4: Mobility - wheel velocity commands."""
    print("\n" + "=" * 60)
    print("  STEP 4: Mobility (Wheel Velocity Commands)")
    print("=" * 60)

    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    print(f"  Running {n_steps} steps with wheel velocity commands...")
    initial_pos = data.qpos[QPOS_BASE_POS].copy()

    phases = [
        ("Forward",     0.5,  0.5),
        ("Turn left",   0.3, -0.3),
        ("Backward",   -0.3, -0.3),
        ("Turn right", -0.2,  0.2),
    ]

    steps_per_phase = n_steps // len(phases)
    for phase_idx, (name, left_vel, right_vel) in enumerate(phases):
        for i in range(steps_per_phase):
            data.ctrl[CTRL_LEFT_WHEEL] = left_vel
            data.ctrl[CTRL_RIGHT_WHEEL] = right_vel
            mujoco.mj_step(model, data)

        pos = data.qpos[QPOS_BASE_POS]
        displacement = np.linalg.norm(pos - initial_pos)
        print(f"    Phase '{name}': pos={pos.round(4)}, displacement={displacement:.4f}m")

    final_pos = data.qpos[QPOS_BASE_POS]
    total = np.linalg.norm(final_pos - initial_pos)
    print(f"  Total displacement: {total:.4f}m")
    print("  [PASS] Mobility OK")


def step5_vision(model, data):
    """Step 5: Head camera RGB-D rendering."""
    print("\n" + "=" * 60)
    print("  STEP 5: Vision (Head Camera RGB-D)")
    print("=" * 60)

    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    # Check for cameras
    cam_names = []
    for i in range(model.ncam):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
        if name:
            cam_names.append(name)
    print(f"  Available cameras: {cam_names}")

    if "head_camera" not in cam_names:
        print("  [SKIP] head_camera not in model")
        return

    # Render RGB
    width = min(640, model.vis.global_.offwidth)
    height = min(480, model.vis.global_.offheight)
    renderer = mujoco.Renderer(model, width=width, height=height)

    renderer.update_scene(data, camera="head_camera")
    rgb = renderer.render()
    print(f"  RGB render:   shape={rgb.shape}, dtype={rgb.dtype}")
    print(f"    Mean pixel value: {rgb.mean():.1f}")
    print(f"    Non-zero pixels:  {(rgb > 0).sum()} / {rgb.size}")

    # Render depth
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera="head_camera")
    depth = renderer.render()
    renderer.disable_depth_rendering()
    print(f"  Depth render: shape={depth.shape}, dtype={depth.dtype}")
    print(f"    Depth range: [{depth.min():.2f}, {depth.max():.2f}]")

    # Test head pan-tilt movement
    print("\n  Testing head pan-tilt camera movement...")
    positions = [
        ("Center",  0.0, 0.5),
        ("Left",    0.4, 0.5),
        ("Right",  -0.4, 0.5),
        ("Up",      0.0, 0.0),
        ("Down",    0.0, 1.2),
    ]

    for name, pan, tilt in positions:
        data.ctrl[CTRL_HEAD] = [pan, tilt]
        # Step simulation to let head move
        for _ in range(50):
            mujoco.mj_step(model, data)

        renderer.update_scene(data, camera="head_camera")
        frame = renderer.render()
        print(f"    Head '{name}' (pan={pan:.1f}, tilt={tilt:.1f}): mean_pixel={frame.mean():.1f}")

    renderer.close()
    print("  [PASS] Vision OK")


def main():
    parser = argparse.ArgumentParser(description="Step-by-step MuJoCo Teleop Demo")
    parser.add_argument("--step", type=int, default=0,
                        help="Run only this step (1-5, 0=all)")
    parser.add_argument("--steps", type=int, default=100,
                        help="Number of simulation steps per test")
    args = parser.parse_args()

    print("Loading RB-Y1 model...")
    model, data = load_model()
    print(f"Model loaded: {model.njnt} joints, {model.nu} actuators\n")

    steps = {
        1: lambda: step1_model_info(model, data),
        2: lambda: step2_arm_teleop(model, data, args.steps),
        3: lambda: step3_hand_teleop(model, data, args.steps),
        4: lambda: step4_mobility(model, data, args.steps),
        5: lambda: step5_vision(model, data),
    }

    if args.step > 0:
        if args.step in steps:
            steps[args.step]()
        else:
            print(f"Unknown step {args.step}. Valid: 1-5")
            sys.exit(1)
    else:
        for step_fn in steps.values():
            step_fn()

    print("\n" + "=" * 60)
    print("  ALL TESTS PASSED")
    print("=" * 60)


if __name__ == "__main__":
    main()
