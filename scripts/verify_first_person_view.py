#!/usr/bin/env python3
"""Verify first-person camera orientation is correct.

Captures RGB-D frames from the robot's head camera, generates point
clouds in camera frame, and prints diagnostic analysis. Also saves
the raw RGB image and optionally shows the first-person viewer.

Expected behavior (robot looking straight ahead):
  - Camera frame: Z = depth (positive, forward), X = right, Y = down
  - Floor points should have large Y (downward) and large Z (forward)
  - Points should be roughly centered around X=0

Usage:
    # Headless analysis only (saves RGB image to disk):
    MUJOCO_GL=egl python3 scripts/verify_first_person_view.py

    # With first-person viewer:
    python3 scripts/verify_first_person_view.py --viewer

    # Test with a colored box placed in front of the robot:
    python3 scripts/verify_first_person_view.py --viewer --add-box
"""

import argparse
import os
import sys
import time
from pathlib import Path

if "MUJOCO_GL" not in os.environ:
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        os.environ["MUJOCO_GL"] = "glfw"
    else:
        os.environ["MUJOCO_GL"] = "egl"

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np


def add_test_box(sim):
    """Add a bright red box 1.5m in front of the robot using MuJoCo visual markers.

    Since we can't add collision geometry at runtime (needed for depth),
    we modify the model XML to include a box and reinitialize.
    """
    import mujoco
    import tempfile

    # Read the original model XML
    mjcf_path = str(PROJECT_ROOT / "models" / "rby1" / "model_teleop.xml")
    with open(mjcf_path) as f:
        xml_content = f.read()

    # Inject a bright red box in front of the robot.
    # The robot is at origin, facing +X. The head camera is ~1.46m up.
    # Place a box at (1.5, 0, 0.5) in MuJoCo Z-up coords: 1.5m in front, 0.5m up.
    box_xml = """
    <body name="test_box" pos="1.5 0 0.5">
      <geom name="test_box_geom" type="box" size="0.15 0.15 0.15"
            rgba="1 0 0 1" contype="0" conaffinity="0"/>
    </body>
"""
    # Insert the box into the worldbody (before the closing </worldbody>)
    # The included rby1 file has the worldbody. We need to add to the top-level.
    # Add it after the include statement.
    insert_point = '<include file="./rby1_v1.1_patched.xml"/>'
    if insert_point in xml_content:
        # We need to add a worldbody section at the top level with the box
        box_section = f"""
  <worldbody>
    {box_xml}
  </worldbody>
"""
        xml_content = xml_content.replace(
            insert_point,
            insert_point + box_section,
        )

    # Write modified XML to temp file and reload
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".xml", dir=str(PROJECT_ROOT / "models" / "rby1"),
        delete=False,
    ) as f:
        f.write(xml_content)
        temp_path = f.name

    return temp_path


def main():
    parser = argparse.ArgumentParser(description="Verify camera first-person orientation")
    parser.add_argument("--viewer", action="store_true",
                        help="Show first-person viewer (requires display)")
    parser.add_argument("--add-box", action="store_true",
                        help="Add a red test box in front of the robot")
    parser.add_argument("--pan", type=float, default=0.0,
                        help="Initial camera pan angle (radians)")
    parser.add_argument("--tilt", type=float, default=0.0,
                        help="Initial camera tilt angle (radians)")
    args = parser.parse_args()

    from teleop_system.modules.camera.pointcloud_generator import PointCloudGenerator
    from teleop_system.simulators.mujoco_sim import MuJoCoSimulator
    from teleop_system.simulators.sim_camera_stream import SimCameraStream

    print("\n" + "=" * 70)
    print("  FIRST-PERSON CAMERA ORIENTATION VERIFICATION")
    print("=" * 70)

    # Initialize MuJoCo
    sim = MuJoCoSimulator()
    mjcf_path = str(PROJECT_ROOT / "models" / "rby1" / "model_teleop.xml")

    temp_model = None
    if args.add_box:
        temp_model = add_test_box(sim)
        mjcf_path = temp_model
        print("  Added red test box at (1.5, 0, 0.5) in MuJoCo world")

    success = sim.initialize({
        "mjcf_path": mjcf_path,
        "timestep": 0.002,
        "render": True,
        "render_width": 640,
        "render_height": 480,
    })
    if not success:
        print("  [ERROR] MuJoCo initialization failed")
        return 1

    cam = SimCameraStream(simulator=sim)
    cam.initialize()

    # Set initial pan/tilt
    if args.pan != 0.0 or args.tilt != 0.0:
        cam.set_orientation(args.pan, args.tilt)
        print(f"  Camera: pan={args.pan:.3f} rad, tilt={args.tilt:.3f} rad")

    # Warm up physics
    for _ in range(500):
        sim.step()

    # Capture frame
    frame = cam.get_rgbd()
    print(f"\n  Frame: {frame.width}x{frame.height}")
    print(f"  RGB range: [{frame.rgb.min()}, {frame.rgb.max()}]")
    print(f"  Depth range: [{frame.depth.min():.3f}, {frame.depth.max():.3f}] m")

    # Print extrinsics (camera → viewer world)
    print(f"\n  Extrinsics (camera → world):")
    R = frame.extrinsics[:3, :3]
    t = frame.extrinsics[:3, 3]
    print(f"    Position (viewer Y-up world): [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}]")
    print(f"    Rotation matrix:\n      {R[0]}\n      {R[1]}\n      {R[2]}")

    # Save RGB image
    try:
        from PIL import Image
        rgb_path = str(PROJECT_ROOT / "head_camera_rgb.png")
        Image.fromarray(frame.rgb).save(rgb_path)
        print(f"\n  Saved RGB image: {rgb_path}")
        print("  (Check this image to see what the camera sees)")
    except ImportError:
        print("\n  [WARN] Pillow not available, cannot save RGB image")

    # Generate point cloud in camera frame (no world transform)
    pcg = PointCloudGenerator(voxel_size=0.0, max_depth=10.0, min_depth=0.1)
    frame_cam = RGBDFrame(
        rgb=frame.rgb,
        depth=frame.depth,
        intrinsics=frame.intrinsics,
        extrinsics=np.eye(4),  # keep in camera frame
        timestamp=frame.timestamp,
        width=frame.width,
        height=frame.height,
    )
    pc = pcg.generate(frame_cam)

    print(f"\n  Point cloud (camera frame, OpenCV convention):")
    print(f"    Count: {pc['count']:,}")
    if pc["count"] > 0:
        pts = pc["points"]
        print(f"    X range: [{pts[:, 0].min():.3f}, {pts[:, 0].max():.3f}] (right+)")
        print(f"    Y range: [{pts[:, 1].min():.3f}, {pts[:, 1].max():.3f}] (down+)")
        print(f"    Z range: [{pts[:, 2].min():.3f}, {pts[:, 2].max():.3f}] (forward+)")
        print(f"    Centroid: ({pts[:, 0].mean():.3f}, {pts[:, 1].mean():.3f}, {pts[:, 2].mean():.3f})")

        # Sanity checks
        checks = []

        # Z should be positive (points are in front of camera)
        z_positive = (pts[:, 2] > 0).sum() / len(pts) * 100
        checks.append(("Z > 0 (in front)", z_positive, z_positive > 95))

        # X should be centered around 0 (symmetric scene)
        x_mean = abs(pts[:, 0].mean())
        checks.append(("X centered (|mean| < 0.5)", x_mean, x_mean < 0.5))

        # Floor points: should have positive Y (below camera) for downward-looking camera
        y_positive = (pts[:, 1] > 0).sum() / len(pts) * 100
        checks.append(("Y > 0 (below camera)", y_positive, y_positive > 30))

        print(f"\n  Orientation checks:")
        all_pass = True
        for name, val, ok in checks:
            status = "PASS" if ok else "FAIL"
            if not ok:
                all_pass = False
            if isinstance(val, float) and val > 1:
                print(f"    [{status}] {name}: {val:.1f}%")
            else:
                print(f"    [{status}] {name}: {val:.3f}")

        if all_pass:
            print(f"\n  ALL CHECKS PASSED - Camera orientation is correct")
        else:
            print(f"\n  SOME CHECKS FAILED - Camera orientation may be wrong")

        if args.add_box:
            # Check if red box is visible
            red_mask = (pc["colors"][:, 0] > 0.6) & (pc["colors"][:, 1] < 0.3) & (pc["colors"][:, 2] < 0.3)
            red_count = red_mask.sum()
            if red_count > 0:
                red_pts = pts[red_mask]
                print(f"\n  Red box detection:")
                print(f"    Red points: {red_count}")
                print(f"    Red centroid: ({red_pts[:, 0].mean():.3f}, "
                      f"{red_pts[:, 1].mean():.3f}, {red_pts[:, 2].mean():.3f})")
                print(f"    [PASS] Red box visible in camera frame")
            else:
                print(f"\n  Red box detection:")
                print(f"    [FAIL] No red points found — box may not be in view")

    # Show first-person viewer
    if args.viewer:
        from teleop_system.modules.camera.pointcloud_viewer import PointCloudViewer

        print(f"\n  Opening first-person viewer...")
        print(f"  Controls: Left-drag=Move camera, R=Reset, Q=Quit")

        viewer = PointCloudViewer(
            width=960,
            height=720,
            title="First-Person Camera View",
            point_size=2.0,
            camera=cam,
            first_person=True,
        )
        if not viewer.initialize():
            print("  [ERROR] Failed to initialize viewer")
        else:
            viewer.set_camera(cam)
            dt_capture = 1.0 / 15.0
            last_capture = 0.0
            count = 0

            while viewer.is_running():
                now = time.monotonic()

                for _ in range(30):
                    sim.step()

                if now - last_capture >= dt_capture:
                    frame = cam.get_rgbd()
                    # Keep in camera frame
                    frame.extrinsics = np.eye(4)
                    pc = pcg.generate(frame)
                    count += 1

                    if pc["count"] > 0:
                        viewer.update_points(pc["points"], pc["colors"])

                    pan, tilt = cam.get_orientation()
                    viewer.set_stats(
                        f"pan={pan:.2f} tilt={tilt:.2f}  pts={pc['count']:,}"
                    )
                    last_capture = now

                viewer.render()
                time.sleep(0.001)

            viewer.shutdown()

    # Cleanup
    if temp_model and os.path.exists(temp_model):
        os.unlink(temp_model)
    cam.shutdown()
    sim.shutdown()
    print("  Done.")
    return 0


# Need RGBDFrame for the camera-frame copy
from teleop_system.interfaces.camera_stream import RGBDFrame

if __name__ == "__main__":
    sys.exit(main())
