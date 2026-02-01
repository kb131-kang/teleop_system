#!/usr/bin/env python3
"""Launch the complete teleoperation system (slave + master) without ROS2 launch.

Starts both the MuJoCo slave system and the simulated master system as
subprocesses. This is an alternative to
`ros2 launch teleop_system teleop_sim_full.launch.py`.

All 8+ nodes:
  Slave:  MuJoCo ROS2 Bridge (+ optional TCP streaming server)
  Master: Dummy Tracker/Glove/HMD Publishers + Arm/Locomotion/Hand/Camera Teleop
          (+ optional RGB-D point cloud viewer)

Usage:
    python3 scripts/launch_all.py
    MUJOCO_GL=glfw python3 scripts/launch_all.py --launch-viewer
    python3 scripts/launch_all.py --publish-camera
    python3 scripts/launch_all.py --publish-camera --launch-camera-viewer
"""

import argparse
import signal
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def main():
    parser = argparse.ArgumentParser(
        description="Launch complete teleop system (slave + master)"
    )
    parser.add_argument("--launch-viewer", action="store_true",
                        help="Launch MuJoCo passive viewer")
    parser.add_argument("--publish-camera", action="store_true",
                        help="Publish RGB-D camera as ROS2 topics")
    parser.add_argument("--launch-camera-viewer", action="store_true",
                        help="Launch RGB-D point cloud viewer (requires --publish-camera)")
    parser.add_argument("--tracker-amplitude", type=float, default=0.08,
                        help="Dummy tracker oscillation amplitude (meters)")
    parser.add_argument("--tracker-frequency", type=float, default=0.3,
                        help="Dummy tracker oscillation frequency (Hz)")
    args = parser.parse_args()

    # Auto-enable publish_camera when camera viewer is requested
    if args.launch_camera_viewer and not args.publish_camera:
        args.publish_camera = True
        print("  Note: --publish-camera auto-enabled for camera viewer")

    node_specs = [
        # ── Slave system ──
        {
            "name": "MuJoCo Bridge (Slave)",
            "module": "teleop_system.simulators.mujoco_ros2_bridge",
            "extra_args": (
                (["--launch-viewer"] if args.launch_viewer else [])
                + (["--publish-camera"] if args.publish_camera else [])
            ),
        },
        # ── Master system: dummy publishers ──
        {
            "name": "Dummy Tracker Publisher",
            "module": "teleop_system.simulators.dummy_tracker_pub",
        },
        {
            "name": "Dummy Glove Publisher",
            "module": "teleop_system.simulators.dummy_glove_pub",
        },
        {
            "name": "Dummy HMD Publisher",
            "module": "teleop_system.simulators.dummy_hmd_pub",
        },
        # ── Master system: teleop nodes ──
        {
            "name": "Arm Teleop Node",
            "module": "teleop_system.modules.arm_teleop.arm_teleop_node",
        },
        {
            "name": "Locomotion Node",
            "module": "teleop_system.modules.locomotion.locomotion_node",
        },
        {
            "name": "Hand Teleop Node",
            "module": "teleop_system.modules.hand_teleop.hand_teleop_node",
        },
        {
            "name": "Camera Teleop Node",
            "module": "teleop_system.modules.camera.camera_node",
        },
    ]

    # Optional RGB-D point cloud viewer (master side)
    if args.launch_camera_viewer:
        node_specs.append({
            "name": "RGB-D Viewer",
            "script": "scripts/demo_rgbd_streaming.py",
            "extra_args": ["--mode", "ros2-viewer"],
        })

    print("=" * 60)
    print("  Complete Teleoperation System")
    print("  (MuJoCo Slave + Simulated Master)")
    print("=" * 60)
    print(f"  Viewer:         {'enabled' if args.launch_viewer else 'disabled'}")
    print(f"  Camera:         {'publishing' if args.publish_camera else 'disabled'}")
    print(f"  Camera viewer:  {'enabled' if args.launch_camera_viewer else 'disabled'}")
    print(f"  Nodes:          {len(node_specs)}")
    print("  Press Ctrl+C to stop all")
    print()

    processes = []
    for spec in node_specs:
        if "module" in spec:
            cmd = [sys.executable, "-m", spec["module"]] + spec.get("extra_args", [])
        else:
            cmd = [sys.executable, spec["script"]] + spec.get("extra_args", [])
        print(f"  Starting {spec['name']}...")
        proc = subprocess.Popen(cmd, cwd=str(PROJECT_ROOT))
        processes.append((spec["name"], proc))
        time.sleep(0.3)  # Brief delay between launches

    print(f"\n  All {len(processes)} nodes running.")
    print()

    def shutdown(signum, frame):
        print("\nShutting down all nodes...")
        for name, proc in reversed(processes):
            print(f"  Stopping {name}...")
            proc.terminate()
        for name, proc in processes:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        print("  All nodes stopped.")

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        while True:
            for name, proc in processes:
                ret = proc.poll()
                if ret is not None:
                    print(f"\n  {name} exited with code {ret}")
                    shutdown(None, None)
                    return
            time.sleep(0.5)
    except KeyboardInterrupt:
        shutdown(None, None)


if __name__ == "__main__":
    main()
