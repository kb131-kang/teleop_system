"""Unit tests for configuration loading."""

import pytest
from pathlib import Path

from teleop_system.utils.config_loader import (
    get_config_dir,
    get_project_root,
    load_config,
    load_teleop_config,
    load_hardware_config,
    load_simulation_config,
    merge_configs,
)


class TestConfigPaths:
    def test_config_dir_exists(self):
        assert get_config_dir().exists()

    def test_project_root_exists(self):
        assert get_project_root().exists()

    def test_default_yaml_exists(self):
        assert (get_config_dir() / "default.yaml").exists()


class TestLoadConfig:
    def test_load_default(self):
        cfg = load_config("default")
        assert "system" in cfg
        assert "robot" in cfg
        assert "modules" in cfg
        assert cfg.system.mode in ("simulation", "hardware")

    def test_load_with_overrides(self):
        cfg = load_config("default", overrides={"system": {"mode": "hardware"}})
        assert cfg.system.mode == "hardware"


class TestTeleopConfig:
    def test_load_arm_config(self):
        cfg = load_teleop_config("arm")
        assert "arm_teleop" in cfg
        assert cfg.arm_teleop.rate_hz == 100
        assert cfg.arm_teleop.ik.solver == "pink"

    def test_load_locomotion_config(self):
        cfg = load_teleop_config("locomotion")
        assert "locomotion" in cfg
        assert cfg.locomotion.rate_hz == 50

    def test_load_hand_config(self):
        cfg = load_teleop_config("hand")
        assert "hand_teleop" in cfg
        assert cfg.hand_teleop.rate_hz == 100


class TestHardwareConfig:
    def test_load_rby1_config(self):
        cfg = load_hardware_config("rby1")
        assert "rby1" in cfg
        assert cfg.rby1.joints.left_arm_count == 7
        assert cfg.rby1.joints.right_arm_count == 7

    def test_load_vive_tracker_config(self):
        cfg = load_hardware_config("vive_tracker")
        assert "vive_tracker" in cfg
        assert cfg.vive_tracker.rate_hz == 100

    def test_load_dg5f_config(self):
        cfg = load_hardware_config("dg5f")
        assert "dg5f" in cfg
        assert cfg.dg5f.joints_per_hand == 20


class TestSimulationConfig:
    def test_load_mujoco_config(self):
        cfg = load_simulation_config("mujoco")
        assert "mujoco" in cfg
        assert cfg.mujoco.timestep == 0.002

    def test_load_isaac_lab_config(self):
        cfg = load_simulation_config("isaac_lab")
        assert "isaac_lab" in cfg
        assert cfg.isaac_lab.enable_pinocchio is True


class TestMergeConfigs:
    def test_merge_override(self):
        cfg1 = {"a": 1, "b": 2}
        cfg2 = {"b": 3, "c": 4}
        merged = merge_configs(cfg1, cfg2)
        assert merged.a == 1
        assert merged.b == 3
        assert merged.c == 4
