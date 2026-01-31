#!/usr/bin/env python3
"""Main entry point for the RB-Y1 Teleoperation System.

Usage:
    python scripts/run_teleop.py                    # Default: simulation mode
    python scripts/run_teleop.py --mode hardware    # Hardware mode
    python scripts/run_teleop.py --sim-backend mujoco
    python scripts/run_teleop.py --modules arm locomotion
"""

import argparse
import signal
import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from teleop_system.utils.config_loader import load_config, merge_configs
from teleop_system.utils.logger import get_logger

logger = get_logger("main")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RB-Y1 Teleoperation System")
    parser.add_argument(
        "--mode",
        choices=["simulation", "hardware"],
        default="simulation",
        help="Operating mode (default: simulation)",
    )
    parser.add_argument(
        "--sim-backend",
        choices=["mujoco", "isaac_lab"],
        default="mujoco",
        help="Simulation backend (default: mujoco)",
    )
    parser.add_argument(
        "--modules",
        nargs="+",
        choices=["arm", "locomotion", "hand", "camera"],
        default=["arm", "locomotion", "hand", "camera"],
        help="Modules to enable",
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Disable GUI control panel",
    )
    parser.add_argument(
        "--log-level",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        default="INFO",
        help="Logging level",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Load configuration
    cfg = load_config("default", overrides={
        "system": {"mode": args.mode},
        "simulation": {"backend": args.sim_backend},
    })

    logger.info(f"RB-Y1 Teleoperation System starting...")
    logger.info(f"  Mode: {args.mode}")
    logger.info(f"  Simulation backend: {args.sim_backend}")
    logger.info(f"  Active modules: {args.modules}")

    # Module enable/disable based on args
    module_flags = {
        "arm_teleop": "arm" in args.modules,
        "locomotion": "locomotion" in args.modules,
        "hand_teleop": "hand" in args.modules,
        "camera": "camera" in args.modules,
    }

    for module_name, enabled in module_flags.items():
        logger.info(f"  [{'+' if enabled else '-'}] {module_name}")

    # TODO: Phase 2+ will add actual module initialization here
    # For now, validate that configuration loads correctly
    logger.info("Configuration loaded successfully.")
    logger.info(f"  Robot URDF: {cfg.robot.urdf_path}")
    logger.info(f"  Left arm joints: {cfg.robot.left_arm_joints}")
    logger.info(f"  Right arm joints: {cfg.robot.right_arm_joints}")
    logger.info(f"  Torso joints: {cfg.robot.torso_joints}")
    logger.info("System ready. (Phase 1 — skeleton only)")


if __name__ == "__main__":
    main()
