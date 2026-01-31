#!/usr/bin/env python3
"""MuJoCo Viewer: Load and visualize the RB-Y1 robot model.

This is the simplest way to see the robot in MuJoCo. It opens an interactive
viewer where you can inspect the model, drag joints, and view the robot.

Usage:
    python3 scripts/demo_mujoco_viewer.py                    # Interactive viewer
    python3 scripts/demo_mujoco_viewer.py --render-only       # Offscreen render (no GUI)
    python3 scripts/demo_mujoco_viewer.py --model path/to.xml # Custom model

Viewer Controls:
    Left mouse drag     : Rotate view
    Right mouse drag    : Pan view
    Scroll              : Zoom in/out
    Double-click body   : Track that body
    Ctrl+Right drag     : Apply force to body
    Tab                 : Cycle cameras
    Space               : Pause/resume simulation
    Backspace           : Reset to initial pose
    Ctrl+A              : Toggle visualization of all elements
"""

import argparse
import os
import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# Use EGL for headless rendering if no display or MUJOCO_GL not explicitly set
if "MUJOCO_GL" not in os.environ:
    os.environ["MUJOCO_GL"] = "egl"

import mujoco
import mujoco.viewer


def get_default_model_path() -> str:
    """Get the default MJCF model path relative to project root."""
    project_root = Path(__file__).resolve().parent.parent
    return str(project_root / "models" / "rby1" / "model_teleop.xml")


def print_model_info(model: mujoco.MjModel):
    """Print a summary of the loaded model."""
    print("\n" + "=" * 60)
    print("  RB-Y1 Robot Model (MuJoCo)")
    print("=" * 60)
    print(f"  Bodies:    {model.nbody}")
    print(f"  Joints:    {model.njnt}")
    print(f"  DOF (nq):  {model.nq}")
    print(f"  Actuators: {model.nu}")
    print(f"  Cameras:   {model.ncam}")
    print()

    # Enumerate all joints grouped by prefix
    joint_groups = {}
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name is None:
            name = f"joint_{i}"

        # Determine group
        if name.startswith("torso_"):
            group = "Torso"
        elif name.startswith("left_arm_"):
            group = "Left Arm"
        elif name.startswith("right_arm_"):
            group = "Right Arm"
        elif name.startswith("head_"):
            group = "Head"
        elif "wheel" in name:
            group = "Wheels"
        elif "gripper" in name or "finger" in name:
            group = "Grippers"
        elif name == "base":
            group = "Base (free joint)"
        else:
            group = "Other"

        if group not in joint_groups:
            joint_groups[group] = []
        joint_groups[group].append((i, name))

    for group_name, joints in joint_groups.items():
        names = [n for _, n in joints]
        print(f"  {group_name} ({len(joints)} joints):")
        if len(names) <= 6:
            print(f"    {', '.join(names)}")
        else:
            print(f"    {', '.join(names[:4])}, ... ({len(names)} total)")

    # Enumerate actuators
    print()
    print(f"  Actuators ({model.nu}):")
    act_groups = {}
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if name is None:
            name = f"actuator_{i}"

        if "wheel" in name:
            group = "Wheels (velocity)"
        elif "link" in name or "torso" in name:
            group = "Torso (position)"
        elif "right_arm" in name:
            group = "Right Arm (position)"
        elif "left_arm" in name:
            group = "Left Arm (position)"
        elif "head" in name:
            group = "Head (position)"
        elif "finger" in name:
            group = "Grippers (motor)"
        else:
            group = "Other"

        if group not in act_groups:
            act_groups[group] = []
        act_groups[group].append(name)

    for group_name, acts in act_groups.items():
        print(f"    {group_name}: {len(acts)}")

    # Cameras
    print()
    cameras = []
    for i in range(model.ncam):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
        cameras.append(name or f"camera_{i}")
    print(f"  Cameras: {', '.join(cameras) if cameras else '(none)'}")
    print("=" * 60)


def render_offscreen(model: mujoco.MjModel, data: mujoco.MjData):
    """Render a single frame offscreen and print info."""
    # Use the model's offscreen buffer size (respects <visual><global offwidth/offheight>)
    width = min(1280, model.vis.global_.offwidth)
    height = min(720, model.vis.global_.offheight)
    print(f"\nOffscreen buffer: {model.vis.global_.offwidth}x{model.vis.global_.offheight}")
    print(f"Rendering at: {width}x{height}")
    renderer = mujoco.Renderer(model, width=width, height=height)
    mujoco.mj_forward(model, data)

    # Default camera render
    renderer.update_scene(data)
    rgb = renderer.render()
    print(f"\nDefault camera render: {rgb.shape} (H={rgb.shape[0]}, W={rgb.shape[1]})")

    # Head camera render (if available)
    cam_names = []
    for i in range(model.ncam):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
        if name:
            cam_names.append(name)

    if "head_camera" in cam_names:
        renderer.update_scene(data, camera="head_camera")
        head_rgb = renderer.render()
        print(f"Head camera render: {head_rgb.shape}")

        # Depth render from head camera
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera="head_camera")
        depth = renderer.render()
        renderer.disable_depth_rendering()
        print(f"Depth render: {depth.shape}, range=[{depth.min():.2f}, {depth.max():.2f}]")
    else:
        print("  (head_camera not found in model)")

    print("\nOffscreen rendering OK.")
    if cam_names:
        print(f"Available cameras: {', '.join(cam_names)}")


def main():
    parser = argparse.ArgumentParser(description="MuJoCo Viewer for RB-Y1 Robot")
    parser.add_argument(
        "--model", type=str, default=None,
        help="Path to MJCF/URDF model file (default: models/rby1/model_teleop.xml)",
    )
    parser.add_argument(
        "--render-only", action="store_true",
        help="Do offscreen render test instead of opening viewer",
    )
    args = parser.parse_args()

    model_path = args.model or get_default_model_path()
    print(f"Loading model: {model_path}")

    try:
        model = mujoco.MjModel.from_xml_path(model_path)
    except Exception as e:
        print(f"ERROR: Failed to load model: {e}")
        sys.exit(1)

    data = mujoco.MjData(model)
    print_model_info(model)

    if args.render_only:
        render_offscreen(model, data)
        return

    print("\nLaunching interactive viewer...")
    print("  Tip: Press Tab to switch cameras, Space to pause")
    print("  Tip: Ctrl+Right-drag on a body to push it")
    print("  Tip: Double-click a body to track it")
    print()

    try:
        mujoco.viewer.launch(model, data)
    except Exception as e:
        print(f"\nViewer failed (headless environment?): {e}")
        print("Try: python3 scripts/demo_mujoco_viewer.py --render-only")
        sys.exit(1)


if __name__ == "__main__":
    main()
