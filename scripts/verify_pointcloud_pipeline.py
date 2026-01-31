#!/usr/bin/env python3
"""Headless point cloud pipeline verification.

Runs the full streaming pipeline and renders point cloud snapshots
as 2D images (bird's-eye + side views) using Pillow. Works in headless
environments without a display server.

Pipeline tested:
  SimulatedTracker(HEAD) -> CameraController -> SimCameraStream
  -> MuJoCo physics -> RGB-D rendering -> PointCloudGenerator
  -> 2D projection renders + PLY files

Usage:
    MUJOCO_GL=egl python3 scripts/verify_pointcloud_pipeline.py
    MUJOCO_GL=egl python3 scripts/verify_pointcloud_pipeline.py --output-dir output/pc_verify
    MUJOCO_GL=egl python3 scripts/verify_pointcloud_pipeline.py --frames 10
"""

import argparse
import os
import sys
import time
from pathlib import Path

if "MUJOCO_GL" not in os.environ:
    os.environ["MUJOCO_GL"] = "egl"

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
from PIL import Image

from teleop_system.interfaces.master_device import TrackerRole
from teleop_system.modules.camera.camera_controller import CameraController
from teleop_system.modules.camera.pointcloud_generator import PointCloudGenerator
from teleop_system.simulators.mujoco_sim import MuJoCoSimulator
from teleop_system.simulators.sim_camera_stream import SimCameraStream
from teleop_system.simulators.simulated_tracker import SimulatedTracker


def render_pointcloud_topdown(
    points: np.ndarray,
    colors: np.ndarray,
    width: int = 640,
    height: int = 640,
    x_range: tuple = (-5, 5),
    z_range: tuple = (-5, 5),
) -> Image.Image:
    """Render top-down (bird's-eye) view of point cloud as a Pillow image.

    Projects onto the XZ plane (Y is up in MuJoCo world frame).
    """
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = [20, 20, 30]  # dark background

    if len(points) == 0:
        return Image.fromarray(img)

    x = points[:, 0]
    z = points[:, 2]

    # Map to pixel coordinates
    px = ((x - x_range[0]) / (x_range[1] - x_range[0]) * (width - 1)).astype(int)
    pz = ((z - z_range[0]) / (z_range[1] - z_range[0]) * (height - 1)).astype(int)

    # Filter to valid pixels
    mask = (px >= 0) & (px < width) & (pz >= 0) & (pz < height)
    px, pz = px[mask], pz[mask]
    c = (colors[mask] * 255).clip(0, 255).astype(np.uint8)

    # Draw points (y-axis flipped for image coords)
    img[height - 1 - pz, px] = c

    # Draw crosshair at origin
    ox = int((0 - x_range[0]) / (x_range[1] - x_range[0]) * (width - 1))
    oz = int((0 - z_range[0]) / (z_range[1] - z_range[0]) * (height - 1))
    if 0 <= ox < width and 0 <= oz < height:
        for d in range(-10, 11):
            if 0 <= ox + d < width:
                img[height - 1 - oz, ox + d] = [255, 50, 50]
            if 0 <= oz + d < height:
                img[height - 1 - (oz + d), ox] = [50, 255, 50]

    return Image.fromarray(img)


def render_pointcloud_side(
    points: np.ndarray,
    colors: np.ndarray,
    width: int = 640,
    height: int = 480,
    x_range: tuple = (-5, 5),
    y_range: tuple = (-2, 4),
) -> Image.Image:
    """Render side view (XY plane) of point cloud as a Pillow image.

    Projects onto the XY plane (looking along -Z).
    """
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = [20, 20, 30]

    if len(points) == 0:
        return Image.fromarray(img)

    x = points[:, 0]
    y = points[:, 1]

    px = ((x - x_range[0]) / (x_range[1] - x_range[0]) * (width - 1)).astype(int)
    py = ((y - y_range[0]) / (y_range[1] - y_range[0]) * (height - 1)).astype(int)

    mask = (px >= 0) & (px < width) & (py >= 0) & (py < height)
    px, py = px[mask], py[mask]
    c = (colors[mask] * 255).clip(0, 255).astype(np.uint8)

    # Y-flip for image coords (Y-up -> row 0 at top)
    img[height - 1 - py, px] = c

    # Draw ground line (y=0)
    gy = int((0 - y_range[0]) / (y_range[1] - y_range[0]) * (height - 1))
    if 0 <= gy < height:
        img[height - 1 - gy, :] = [40, 40, 50]

    return Image.fromarray(img)


def save_pointcloud_ply(points: np.ndarray, colors: np.ndarray, path: str) -> None:
    """Save point cloud as binary PLY file."""
    n = len(points)
    rgb_uint8 = (colors * 255).clip(0, 255).astype(np.uint8)

    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    )

    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        for i in range(n):
            f.write(np.float32(points[i]).tobytes())
            f.write(rgb_uint8[i].tobytes())


def main():
    parser = argparse.ArgumentParser(
        description="Headless point cloud pipeline verification",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--output-dir", type=str, default="output/pc_verify",
                        help="Output directory for images and PLY files")
    parser.add_argument("--frames", type=int, default=5,
                        help="Number of point cloud frames to capture")
    parser.add_argument("--render-width", type=int, default=640,
                        help="Camera render width")
    parser.add_argument("--render-height", type=int, default=480,
                        help="Camera render height")
    parser.add_argument("--control-fps", type=float, default=30.0,
                        help="Control loop rate (Hz)")
    parser.add_argument("--max-depth", type=float, default=10.0,
                        help="Max depth for point cloud (meters)")
    parser.add_argument("--voxel-size", type=float, default=0.02,
                        help="Voxel downsampling (0 = none)")
    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("  POINT CLOUD PIPELINE VERIFICATION (headless)")
    print("=" * 70)

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

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
        print("  [ERROR] MuJoCo initialization failed")
        return 1

    print(f"  MuJoCo: {len(sim.get_joint_names())} joints, "
          f"render={args.render_width}x{args.render_height}")

    # ── 2. Camera stream ──
    cam = SimCameraStream(simulator=sim)
    cam.initialize()

    # ── 3. HMD tracker ──
    tracker = SimulatedTracker(
        role=TrackerRole.HEAD,
        amplitude=0.08,
        frequency=0.15,
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
        use_open3d=True,  # falls back to numpy
    )

    # ── Warm up: run physics for 200 steps ──
    for _ in range(200):
        sim.step()

    dt_control = 1.0 / args.control_fps
    steps_per_control = max(1, int(dt_control / 0.002))

    print(f"  Output: {out_dir.resolve()}")
    print(f"  Frames to capture: {args.frames}")
    print(f"  Max depth: {args.max_depth}m, Voxel: {args.voxel_size}m")
    print()

    # ── 6. Capture frames with different head orientations ──
    all_pass = True

    for frame_idx in range(args.frames):
        # Run control + physics for 30 cycles (1s at 30Hz)
        for _ in range(30):
            controller.update()
            for _ in range(steps_per_control):
                sim.step()

        # Capture RGB-D frame
        t0 = time.monotonic()
        rgbd = cam.get_rgbd()
        pc = pcg.generate(rgbd)
        dt = time.monotonic() - t0

        pan, tilt = cam.get_orientation()
        n_points = pc["count"]

        print(f"  Frame {frame_idx}: pan={pan:+.3f} tilt={tilt:+.3f}  "
              f"points={n_points:,}  gen={dt*1000:.1f}ms  "
              f"depth=[{rgbd.depth.min():.1f},{rgbd.depth.max():.1f}]")

        if n_points == 0:
            print(f"    [WARN] No points generated!")
            all_pass = False
            continue

        points = pc["points"]
        colors = pc["colors"]

        # Save RGB image
        rgb_img = Image.fromarray(rgbd.rgb)
        rgb_img.save(str(out_dir / f"frame{frame_idx}_rgb.png"))

        # Save depth as grayscale PNG
        depth_norm = np.clip(rgbd.depth / args.max_depth, 0, 1)
        depth_img = Image.fromarray((depth_norm * 255).astype(np.uint8))
        depth_img.save(str(out_dir / f"frame{frame_idx}_depth.png"))

        # Render point cloud views
        topdown = render_pointcloud_topdown(
            points, colors, x_range=(-8, 8), z_range=(-8, 8),
        )
        topdown.save(str(out_dir / f"frame{frame_idx}_topdown.png"))

        side = render_pointcloud_side(
            points, colors, x_range=(-8, 8), y_range=(-2, 6),
        )
        side.save(str(out_dir / f"frame{frame_idx}_side.png"))

        # Save PLY (subsample if large)
        if n_points > 50000:
            idx = np.random.choice(n_points, 50000, replace=False)
            save_pointcloud_ply(
                points[idx], colors[idx],
                str(out_dir / f"frame{frame_idx}.ply"),
            )
        else:
            save_pointcloud_ply(
                points, colors,
                str(out_dir / f"frame{frame_idx}.ply"),
            )

        # Validate
        checks = [
            ("points > 0", n_points > 0),
            ("RGB non-blank", rgbd.rgb.astype(float).std() > 5.0),
            ("depth valid", (rgbd.depth > 0.1).sum() > 100),
            ("Z range ok", points[:, 2].max() - points[:, 2].min() > 0.1),
        ]
        for name, passed in checks:
            if not passed:
                print(f"    [FAIL] {name}")
                all_pass = False

    # ── 7. Streaming throughput test ──
    print("\n" + "-" * 70)
    print("  STREAMING THROUGHPUT")
    print("-" * 70)

    n_cycles = 50
    t0 = time.monotonic()
    for _ in range(n_cycles):
        controller.update()
        for _ in range(steps_per_control):
            sim.step()
        rgbd = cam.get_rgbd()
        pc = pcg.generate(rgbd)
    dt = time.monotonic() - t0

    fps = n_cycles / dt
    print(f"  Cycles:    {n_cycles}")
    print(f"  Total:     {dt:.2f}s")
    print(f"  Pipeline:  {fps:.1f} Hz (control + physics + RGB-D + pointcloud)")
    print(f"  Last PC:   {pc['count']:,} points")
    if fps >= 15.0:
        print(f"  [PASS] >= 15 Hz target")
    else:
        print(f"  [WARN] Below 15 Hz target")
        all_pass = False

    # ── 8. Create composite image ──
    print("\n" + "-" * 70)
    print("  COMPOSITE IMAGE")
    print("-" * 70)

    # Load all frame images and compose a grid
    frames_per_row = min(args.frames, 5)
    tile_w, tile_h = 320, 240
    rows = 4  # rgb, depth, topdown, side
    composite = Image.new("RGB", (tile_w * frames_per_row, tile_h * rows), (20, 20, 30))

    for i in range(min(args.frames, frames_per_row)):
        for row, suffix in enumerate(["rgb", "depth", "topdown", "side"]):
            path = out_dir / f"frame{i}_{suffix}.png"
            if path.exists():
                tile = Image.open(str(path)).resize((tile_w, tile_h))
                if tile.mode != "RGB":
                    tile = tile.convert("RGB")
                composite.paste(tile, (i * tile_w, row * tile_h))

    composite_path = out_dir / "composite.png"
    composite.save(str(composite_path))
    print(f"  Saved: {composite_path}")

    # ── Summary ──
    print("\n" + "=" * 70)
    pan, tilt = cam.get_orientation()
    print(f"  Final pan:  {pan:.4f} rad")
    print(f"  Final tilt: {tilt:.4f} rad")
    print(f"  Files: {out_dir.resolve()}/")

    # Cleanup
    tracker.shutdown()
    cam.shutdown()
    sim.shutdown()

    if all_pass:
        print("\n  ALL CHECKS PASSED")
    else:
        print("\n  SOME CHECKS FAILED")
    print("=" * 70 + "\n")

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
