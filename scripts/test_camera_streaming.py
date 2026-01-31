#!/usr/bin/env python3
"""Camera RGB-D streaming verification script.

Tests and visualizes the RGB-D rendering and point cloud pipeline:
  1. Renders head_camera at multiple head orientations
  2. Saves RGB + depth images to disk
  3. Generates point clouds and saves as PLY files
  4. Prints detailed statistics for each frame

Usage:
    MUJOCO_GL=egl python3 scripts/test_camera_streaming.py
    MUJOCO_GL=egl python3 scripts/test_camera_streaming.py --output-dir /tmp/camera_test
    MUJOCO_GL=egl python3 scripts/test_camera_streaming.py --no-save  # stats only
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

import mujoco
import numpy as np

from teleop_system.interfaces.camera_stream import RGBDFrame
from teleop_system.modules.camera.pointcloud_generator import PointCloudGenerator
from teleop_system.simulators.mujoco_sim import MuJoCoSimulator
from teleop_system.simulators.sim_camera_stream import SimCameraStream


def save_rgb_ppm(rgb: np.ndarray, path: str) -> None:
    """Save RGB image as PPM (no external dependency)."""
    h, w = rgb.shape[:2]
    with open(path, "wb") as f:
        f.write(f"P6\n{w} {h}\n255\n".encode())
        f.write(rgb.tobytes())


def save_depth_pgm(depth: np.ndarray, path: str, max_depth: float = 10.0) -> None:
    """Save depth as PGM (normalized to 0-65535 range)."""
    h, w = depth.shape[:2]
    normalized = np.clip(depth / max_depth, 0, 1)
    uint16 = (normalized * 65535).astype(np.uint16)
    with open(path, "wb") as f:
        f.write(f"P5\n{w} {h}\n65535\n".encode())
        f.write(uint16.byteswap().tobytes())


def save_pointcloud_ply(points: np.ndarray, colors: np.ndarray, path: str) -> None:
    """Save point cloud as PLY (ASCII format)."""
    n = len(points)
    with open(path, "w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        rgb_uint8 = (colors * 255).clip(0, 255).astype(np.uint8)
        for i in range(n):
            f.write(f"{points[i,0]:.4f} {points[i,1]:.4f} {points[i,2]:.4f} "
                    f"{rgb_uint8[i,0]} {rgb_uint8[i,1]} {rgb_uint8[i,2]}\n")


def print_frame_stats(label: str, frame: RGBDFrame) -> None:
    """Print detailed statistics for an RGB-D frame."""
    print(f"\n  [{label}]")
    print(f"    RGB:   shape={frame.rgb.shape}, dtype={frame.rgb.dtype}, "
          f"mean={frame.rgb.mean():.1f}, range=[{frame.rgb.min()}, {frame.rgb.max()}]")
    print(f"    Depth: shape={frame.depth.shape}, dtype={frame.depth.dtype}, "
          f"range=[{frame.depth.min():.3f}, {frame.depth.max():.3f}]")

    valid_depth = frame.depth[(frame.depth > 0.1) & (frame.depth < 50.0)]
    if len(valid_depth) > 0:
        print(f"    Valid depth pixels: {len(valid_depth)}/{frame.depth.size} "
              f"({100*len(valid_depth)/frame.depth.size:.1f}%)")
        print(f"    Valid depth range: [{valid_depth.min():.3f}, {valid_depth.max():.3f}] m")
        print(f"    Valid depth mean:  {valid_depth.mean():.3f} m")
    else:
        print(f"    Valid depth pixels: 0 (all out of range)")

    print(f"    Intrinsics: fx={frame.intrinsics[0,0]:.1f}, fy={frame.intrinsics[1,1]:.1f}, "
          f"cx={frame.intrinsics[0,2]:.1f}, cy={frame.intrinsics[1,2]:.1f}")
    print(f"    Timestamp: {frame.timestamp:.4f}s")

    # Check for non-uniform content (image is not blank)
    rgb_std = frame.rgb.astype(float).std()
    print(f"    RGB std (>0 = non-blank): {rgb_std:.1f}")


def main():
    parser = argparse.ArgumentParser(description="Camera RGB-D streaming test")
    parser.add_argument("--output-dir", type=str, default="output/camera_test",
                        help="Directory to save output files")
    parser.add_argument("--no-save", action="store_true",
                        help="Skip saving files (print stats only)")
    parser.add_argument("--width", type=int, default=640,
                        help="Render width")
    parser.add_argument("--height", type=int, default=480,
                        help="Render height")
    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("  CAMERA RGB-D STREAMING VERIFICATION")
    print("=" * 70)

    # Initialize simulator
    sim = MuJoCoSimulator()
    mjcf_path = str(PROJECT_ROOT / "models" / "rby1" / "model_teleop.xml")
    success = sim.initialize({
        "mjcf_path": mjcf_path,
        "timestep": 0.002,
        "render": True,
        "render_width": args.width,
        "render_height": args.height,
    })
    assert success, "MuJoCo initialization failed"

    cam = SimCameraStream(simulator=sim)
    cam.initialize()
    pcg = PointCloudGenerator(use_open3d=False, max_depth=50.0, min_depth=0.1)

    # Prepare output directory
    if not args.no_save:
        out_dir = Path(args.output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        print(f"\n  Output directory: {out_dir.resolve()}")

    # Step simulation forward
    for _ in range(100):
        sim.step()

    # ── Test 1: Render at different head orientations ──
    print("\n" + "-" * 70)
    print("  TEST 1: Rendering at different head orientations")
    print("-" * 70)

    orientations = [
        ("center",    0.0,  0.5),
        ("left",      0.4,  0.5),
        ("right",    -0.4,  0.5),
        ("up",        0.0,  0.0),
        ("down",      0.0,  1.2),
    ]

    frames = {}
    for name, pan, tilt in orientations:
        cam.set_orientation(pan, tilt)
        # Step to let joints settle
        for _ in range(50):
            sim.step()

        frame = cam.get_rgbd()
        frames[name] = frame
        print_frame_stats(f"{name} (pan={pan:.1f}, tilt={tilt:.1f})", frame)

        if not args.no_save:
            save_rgb_ppm(frame.rgb, str(out_dir / f"rgb_{name}.ppm"))
            save_depth_pgm(frame.depth, str(out_dir / f"depth_{name}.pgm"))

    # ── Test 2: Verify different orientations produce different images ──
    print("\n" + "-" * 70)
    print("  TEST 2: Image difference across orientations")
    print("-" * 70)

    base = frames["center"].rgb.astype(float)
    for name, frame in frames.items():
        if name == "center":
            continue
        diff = np.abs(frame.rgb.astype(float) - base)
        mean_diff = diff.mean()
        max_diff = diff.max()
        changed_pix = (diff.sum(axis=2) > 10).sum()
        total_pix = frame.rgb.shape[0] * frame.rgb.shape[1]
        print(f"    center vs {name:8s}: mean_diff={mean_diff:6.1f}, "
              f"max_diff={max_diff:5.0f}, changed={changed_pix}/{total_pix} "
              f"({100*changed_pix/total_pix:.1f}%)")

    # ── Test 3: Point cloud generation ──
    print("\n" + "-" * 70)
    print("  TEST 3: Point cloud generation")
    print("-" * 70)

    cam.set_orientation(0.0, 0.5)
    for _ in range(50):
        sim.step()
    frame = cam.get_rgbd()

    t0 = time.monotonic()
    pc = pcg.generate(frame)
    dt = time.monotonic() - t0

    print(f"    Points:    {pc['count']}")
    print(f"    Shape:     {pc['points'].shape}")
    print(f"    X range:   [{pc['points'][:,0].min():.2f}, {pc['points'][:,0].max():.2f}] m")
    print(f"    Y range:   [{pc['points'][:,1].min():.2f}, {pc['points'][:,1].max():.2f}] m")
    print(f"    Z range:   [{pc['points'][:,2].min():.2f}, {pc['points'][:,2].max():.2f}] m")
    print(f"    Gen time:  {dt*1000:.1f} ms")
    print(f"    Throughput: {pc['count']/dt/1e6:.1f}M points/sec")

    if not args.no_save:
        # Save downsampled PLY (full cloud can be huge)
        n = pc["count"]
        if n > 50000:
            idx = np.random.choice(n, 50000, replace=False)
            pts = pc["points"][idx]
            cols = pc["colors"][idx]
        else:
            pts = pc["points"]
            cols = pc["colors"]
        save_pointcloud_ply(pts, cols, str(out_dir / "pointcloud.ply"))
        print(f"    Saved PLY: {out_dir / 'pointcloud.ply'} ({len(pts)} points)")

    # ── Test 4: Streaming throughput ──
    print("\n" + "-" * 70)
    print("  TEST 4: RGB-D streaming throughput")
    print("-" * 70)

    n_frames = 30
    t0 = time.monotonic()
    for i in range(n_frames):
        sim.step()
        frame = cam.get_rgbd()
    dt = time.monotonic() - t0

    fps = n_frames / dt
    rgb_bytes = frame.rgb.nbytes
    depth_bytes = frame.depth.nbytes
    bandwidth_mbps = (rgb_bytes + depth_bytes) * fps / 1e6

    print(f"    Frames:    {n_frames}")
    print(f"    Time:      {dt:.3f}s")
    print(f"    FPS:       {fps:.1f}")
    print(f"    RGB size:  {rgb_bytes/1024:.0f} KB/frame")
    print(f"    Depth size: {depth_bytes/1024:.0f} KB/frame")
    print(f"    Bandwidth: {bandwidth_mbps:.1f} MB/s")

    target_fps = 15.0
    if fps >= target_fps:
        print(f"    [PASS] Exceeds {target_fps} fps target")
    else:
        print(f"    [WARN] Below {target_fps} fps target (rendering at {fps:.1f} fps)")

    # ── Summary ──
    print("\n" + "=" * 70)
    print("  SUMMARY")
    print("=" * 70)
    all_pass = True

    checks = [
        ("RGB non-blank", frames["center"].rgb.astype(float).std() > 5.0),
        ("Depth non-zero", (frames["center"].depth > 0.1).sum() > 100),
        ("Intrinsics valid", frames["center"].intrinsics[0, 0] > 10),
        ("Orientations differ", any(
            np.abs(frames[n].rgb.astype(float) - base).mean() > 1.0
            for n in ["left", "right", "up", "down"]
        )),
        ("Point cloud > 0", pc["count"] > 0),
        (f"FPS >= {target_fps}", fps >= target_fps),
    ]

    for name, passed in checks:
        status = "PASS" if passed else "FAIL"
        if not passed:
            all_pass = False
        print(f"    [{status}] {name}")

    if not args.no_save:
        print(f"\n  Files saved to: {out_dir.resolve()}")
        print(f"    - rgb_*.ppm: View with 'display' or any image viewer")
        print(f"    - depth_*.pgm: View with 'display' (brighter = farther)")
        print(f"    - pointcloud.ply: View with 'meshlab' or Open3D viewer")

    # Cleanup
    cam.shutdown()
    sim.shutdown()

    print("\n" + "=" * 70)
    if all_pass:
        print("  ALL CHECKS PASSED")
    else:
        print("  SOME CHECKS FAILED")
    print("=" * 70 + "\n")

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
