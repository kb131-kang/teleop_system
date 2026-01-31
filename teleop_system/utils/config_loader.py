"""Hydra-based configuration loader.

Provides centralized configuration loading from YAML files
in the config/ directory. Supports hierarchical overrides.
"""

from pathlib import Path
from typing import Any

from omegaconf import DictConfig, OmegaConf
import yaml


# Project root: two levels up from this file (utils/ -> teleop_system/ -> project root)
_PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
_CONFIG_DIR = _PROJECT_ROOT / "config"


def get_config_dir() -> Path:
    """Get the absolute path to the config/ directory."""
    return _CONFIG_DIR


def get_project_root() -> Path:
    """Get the absolute path to the project root directory."""
    return _PROJECT_ROOT


def load_yaml(file_path: str | Path) -> dict:
    """Load a YAML file and return as dict.

    Args:
        file_path: Path to the YAML file (absolute or relative to config dir).

    Returns:
        Parsed YAML content as dict.
    """
    path = Path(file_path)
    if not path.is_absolute():
        path = _CONFIG_DIR / path
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def load_config(
    config_name: str = "default",
    overrides: dict[str, Any] | None = None,
) -> DictConfig:
    """Load configuration by name with optional overrides.

    Loads config/{config_name}.yaml and merges with any specified overrides.

    Args:
        config_name: Name of the config file (without .yaml extension).
        overrides: Optional dict of override values to merge.

    Returns:
        OmegaConf DictConfig with resolved values.
    """
    base_config = load_yaml(f"{config_name}.yaml")
    cfg = OmegaConf.create(base_config)

    if overrides:
        override_cfg = OmegaConf.create(overrides)
        cfg = OmegaConf.merge(cfg, override_cfg)

    return cfg


def load_teleop_config(
    teleop_module: str,
    overrides: dict[str, Any] | None = None,
) -> DictConfig:
    """Load a teleop module configuration.

    Args:
        teleop_module: Module name (e.g., 'arm', 'locomotion', 'hand').
        overrides: Optional dict of override values.

    Returns:
        OmegaConf DictConfig for the specified module.
    """
    return load_config(f"teleop/{teleop_module}", overrides)


def load_hardware_config(
    hardware_name: str,
    overrides: dict[str, Any] | None = None,
) -> DictConfig:
    """Load a hardware device configuration.

    Args:
        hardware_name: Device name (e.g., 'rby1', 'vive_tracker', 'manus_glove').
        overrides: Optional dict of override values.

    Returns:
        OmegaConf DictConfig for the specified hardware.
    """
    return load_config(f"hardware/{hardware_name}", overrides)


def load_simulation_config(
    sim_backend: str,
    overrides: dict[str, Any] | None = None,
) -> DictConfig:
    """Load a simulator backend configuration.

    Args:
        sim_backend: Simulator name (e.g., 'mujoco', 'isaac_lab').
        overrides: Optional dict of override values.

    Returns:
        OmegaConf DictConfig for the specified simulator.
    """
    return load_config(f"simulation/{sim_backend}", overrides)


def merge_configs(*configs: DictConfig | dict) -> DictConfig:
    """Merge multiple configurations, later configs take precedence.

    Args:
        *configs: DictConfig or dict objects to merge.

    Returns:
        Merged OmegaConf DictConfig.
    """
    result = OmegaConf.create({})
    for cfg in configs:
        if isinstance(cfg, dict):
            cfg = OmegaConf.create(cfg)
        result = OmegaConf.merge(result, cfg)
    return result
