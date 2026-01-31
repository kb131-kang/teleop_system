#!/usr/bin/env python3
"""Launch the MuJoCo ROS2 Bridge node.

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 scripts/run_mujoco_bridge.py
    MUJOCO_GL=glfw python3 scripts/run_mujoco_bridge.py --launch-viewer
"""

import sys
from pathlib import Path

# Add project root
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop_system.simulators.mujoco_ros2_bridge import main

if __name__ == "__main__":
    main()
