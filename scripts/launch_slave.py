#!/usr/bin/env python3
"""Launch the slave system (MuJoCo simulation) without ROS2 launch.

Starts the MuJoCo ROS2 Bridge as a subprocess. This is an alternative
to `ros2 launch teleop_system slave_mujoco.launch.py` for environments
where colcon/ros2 launch is not set up.

Optionally starts a TCP streaming server (ros2-server mode) that bridges
ROS2 camera topics to TCP for remote viewers.

Usage:
    python3 scripts/launch_slave.py
    python3 scripts/launch_slave.py --launch-viewer
    python3 scripts/launch_slave.py --publish-camera --camera-fps 30
    python3 scripts/launch_slave.py --publish-camera --launch-streaming
"""

import argparse
import signal
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def main():
    parser = argparse.ArgumentParser(description="Launch slave system (MuJoCo)")
    parser.add_argument("--launch-viewer", action="store_true",
                        help="Launch MuJoCo passive viewer")
    parser.add_argument("--publish-camera", action="store_true",
                        help="Publish RGB-D camera as ROS2 topics")
    parser.add_argument("--camera-fps", type=float, default=15.0,
                        help="Camera publish rate (Hz)")
    parser.add_argument("--mjcf-path", type=str,
                        default="models/rby1/model_teleop.xml",
                        help="Path to MuJoCo XML model")
    parser.add_argument("--launch-streaming", action="store_true",
                        help="Launch TCP streaming server (ros2-server) for remote viewers")
    parser.add_argument("--streaming-port", type=int, default=9876,
                        help="TCP streaming server port")
    args = parser.parse_args()

    # Force publish_camera when streaming is enabled
    if args.launch_streaming and not args.publish_camera:
        args.publish_camera = True
        print("  Note: --publish-camera auto-enabled for streaming")

    bridge_cmd = [
        sys.executable, "-m", "teleop_system.simulators.mujoco_ros2_bridge",
    ]
    if args.launch_viewer:
        bridge_cmd.append("--launch-viewer")
    if args.publish_camera:
        bridge_cmd.append("--publish-camera")

    print("=" * 60)
    print("  Slave System (MuJoCo Simulation)")
    print("=" * 60)
    print(f"  Viewer:    {'enabled' if args.launch_viewer else 'disabled'}")
    print(f"  Camera:    {'publishing at {:.0f} Hz'.format(args.camera_fps) if args.publish_camera else 'disabled'}")
    print(f"  Streaming: {'port {}'.format(args.streaming_port) if args.launch_streaming else 'disabled'}")
    print(f"  Model:     {args.mjcf_path}")
    print("  Press Ctrl+C to stop")
    print()

    processes = []

    # Start MuJoCo bridge
    print("  Starting MuJoCo Bridge...")
    bridge_proc = subprocess.Popen(bridge_cmd, cwd=str(PROJECT_ROOT))
    processes.append(("MuJoCo Bridge", bridge_proc))

    # Optionally start TCP streaming server
    if args.launch_streaming:
        time.sleep(1.0)  # Let bridge start publishing before server subscribes
        streaming_cmd = [
            sys.executable, "scripts/demo_rgbd_streaming.py",
            "--mode", "ros2-server",
            "--host", "0.0.0.0",
            "--port", str(args.streaming_port),
        ]
        print("  Starting TCP Streaming Server...")
        streaming_proc = subprocess.Popen(streaming_cmd, cwd=str(PROJECT_ROOT))
        processes.append(("TCP Streaming Server", streaming_proc))

    print(f"\n  All {len(processes)} process(es) running.")
    print()

    def shutdown(signum, frame):
        print("\nShutting down slave system...")
        for name, proc in reversed(processes):
            print(f"  Stopping {name}...")
            proc.terminate()
        for name, proc in processes:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        print("  All processes stopped.")

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
