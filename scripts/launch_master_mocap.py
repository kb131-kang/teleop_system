#!/usr/bin/env python3
"""Launch the master system with BVH motion capture replay (without ROS2 launch).

Starts the BVH replay publisher and teleop processing nodes as subprocesses.
This is an alternative to `ros2 launch teleop_system master_mocap.launch.py`.

Nodes launched:
  1. BVH Replay Publisher (replaces Dummy Tracker/Glove/HMD Publishers)
  2. Arm Teleop Node (tracker -> IK -> joint commands)
  3. Locomotion Node (foot tracker -> base velocity)
  4. Hand Teleop Node (glove -> gripper commands)
  5. Camera Teleop Node (HMD -> head pan/tilt)
  6. (Optional) RGB-D Viewer (first-person camera view)

Usage:
    python3 scripts/launch_master_mocap.py --bvh data/bvh/cmu/002/02_01.bvh
    python3 scripts/launch_master_mocap.py --bvh data/bvh/cmu/002/02_01.bvh --playback-speed 0.5
    python3 scripts/launch_master_mocap.py --bvh data/bvh/cmu/002/02_01.bvh --launch-viewer
"""

import argparse
import signal
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def main():
    parser = argparse.ArgumentParser(description="Launch master system (BVH mocap replay)")
    parser.add_argument("--bvh", type=str, required=True,
                        help="Path to BVH motion capture file")
    parser.add_argument("--rate-hz", type=float, default=120.0,
                        help="Replay publishing rate (Hz)")
    parser.add_argument("--playback-speed", type=float, default=1.0,
                        help="Playback speed multiplier")
    parser.add_argument("--loop", action="store_true", default=True,
                        help="Loop playback (default: true)")
    parser.add_argument("--no-loop", action="store_true",
                        help="Disable loop playback")
    parser.add_argument("--scale", type=float, default=0.056,
                        help="BVH to meters scale factor")
    parser.add_argument("--normalize-mode", type=str, default="relative",
                        choices=["relative", "absolute"],
                        help="Position normalization mode")
    parser.add_argument("--launch-viewer", action="store_true",
                        help="Launch RGB-D point cloud viewer (first-person camera)")
    parser.add_argument("--viewer-mode", type=str, default="ros2-viewer",
                        choices=["ros2-viewer", "client"],
                        help="Viewer mode: ros2-viewer (same machine) or client (TCP)")
    parser.add_argument("--viewer-host", type=str, default="localhost",
                        help="TCP streaming host for viewer-mode=client (slave IP)")
    parser.add_argument("--viewer-port", type=int, default=9876,
                        help="TCP streaming port for viewer-mode=client")
    parser.add_argument("--no-gui", action="store_true",
                        help="Disable GUI control panel")
    parser.add_argument("--auto-start", action="store_true",
                        help="Start playback immediately (default: wait for start command)")
    args = parser.parse_args()

    loop = not args.no_loop

    # Resolve BVH path
    bvh_path = Path(args.bvh)
    if not bvh_path.is_absolute():
        bvh_path = PROJECT_ROOT / bvh_path
    if not bvh_path.exists():
        print(f"Error: BVH file not found: {bvh_path}")
        sys.exit(1)

    # Build ROS2 parameter arguments for bvh_replay_publisher
    bvh_ros_args = [
        "--ros-args",
        "-p", f"bvh_file:={bvh_path}",
        "-p", f"rate_hz:={args.rate_hz}",
        "-p", f"playback_speed:={args.playback_speed}",
        "-p", f"loop:={str(loop).lower()}",
        "-p", f"scale:={args.scale}",
        "-p", f"normalize_mode:={args.normalize_mode}",
        "-p", f"auto_start:={str(args.auto_start).lower()}",
    ]

    node_specs = [
        {
            "name": "BVH Replay Publisher",
            "module": "teleop_system.mocap.bvh_replay_publisher",
            "extra_args": bvh_ros_args,
        },
        {
            "name": "Calibration Node",
            "module": "teleop_system.calibration.calibration_node",
        },
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

    if not args.no_gui:
        node_specs.append({
            "name": "GUI Control Panel",
            "module": "teleop_system.gui.gui_node",
        })

    if args.launch_viewer:
        viewer_args = ["--mode", args.viewer_mode]
        if args.viewer_mode == "client":
            viewer_args += ["--host", args.viewer_host, "--port", str(args.viewer_port)]
        node_specs.append({
            "name": f"RGB-D Viewer ({args.viewer_mode})",
            "script": "scripts/demo_rgbd_streaming.py",
            "extra_args": viewer_args,
        })

    print("=" * 60)
    print("  Master System (BVH Motion Capture Replay)")
    print("=" * 60)
    print(f"  BVH file:           {bvh_path.name}")
    print(f"  Playback speed:     {args.playback_speed}x")
    print(f"  Loop:               {loop}")
    print(f"  Scale:              {args.scale}")
    print(f"  Normalize mode:     {args.normalize_mode}")
    print(f"  Viewer:             {'enabled' if args.launch_viewer else 'disabled'}")
    print(f"  GUI:                {'disabled' if args.no_gui else 'enabled'}")
    print(f"  Nodes to launch:    {len(node_specs)}")
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
        time.sleep(0.2)  # Brief delay between launches

    print(f"\n  All {len(processes)} nodes running.")
    print()

    def shutdown(signum, frame):
        print("\nShutting down master system...")
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
        # Wait for any process to exit
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
