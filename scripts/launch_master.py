#!/usr/bin/env python3
"""Launch the master system (simulated VR inputs + teleop nodes) without ROS2 launch.

Starts all dummy publishers and teleop processing nodes as subprocesses.
This is an alternative to `ros2 launch teleop_system master_sim.launch.py`.

Nodes launched:
  1. Dummy Tracker Publisher (simulated VR tracker data)
  2. Dummy Glove Publisher (simulated hand glove data)
  3. Dummy HMD Publisher (simulated head orientation)
  4. Arm Teleop Node (tracker -> IK -> joint commands)
  5. Locomotion Node (foot tracker -> base velocity)
  6. Hand Teleop Node (glove -> gripper commands)
  7. Camera Teleop Node (HMD -> head pan/tilt)
  8. (Optional) RGB-D Viewer (first-person camera view)

Usage:
    python3 scripts/launch_master.py
    python3 scripts/launch_master.py --tracker-amplitude 0.10
    python3 scripts/launch_master.py --launch-viewer
"""

import argparse
import signal
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def main():
    parser = argparse.ArgumentParser(description="Launch master system (simulated)")
    parser.add_argument("--tracker-amplitude", type=float, default=0.08,
                        help="Dummy tracker oscillation amplitude (meters)")
    parser.add_argument("--tracker-frequency", type=float, default=0.3,
                        help="Dummy tracker oscillation frequency (Hz)")
    parser.add_argument("--launch-viewer", action="store_true",
                        help="Launch ROS2 RGB-D point cloud viewer (first-person camera)")
    args = parser.parse_args()

    node_specs = [
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

    if args.launch_viewer:
        node_specs.append({
            "name": "RGB-D Viewer",
            "script": "scripts/demo_rgbd_streaming.py",
            "extra_args": ["--mode", "ros2-viewer"],
        })

    print("=" * 60)
    print("  Master System (Simulated VR Inputs)")
    print("=" * 60)
    print(f"  Tracker amplitude:  {args.tracker_amplitude} m")
    print(f"  Tracker frequency:  {args.tracker_frequency} Hz")
    print(f"  Viewer:             {'enabled' if args.launch_viewer else 'disabled'}")
    print(f"  Nodes to launch:    {len(node_specs)}")
    print("  Press Ctrl+C to stop all")
    print()

    processes = []
    for spec in node_specs:
        if "module" in spec:
            cmd = [sys.executable, "-m", spec["module"]]
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
            proc.wait(timeout=5)
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
