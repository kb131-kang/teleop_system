"""Module-level logger factory.

Provides consistent logging configuration across all teleoperation modules.
"""

import logging
import sys

_LOGGERS: dict[str, logging.Logger] = {}

_DEFAULT_FORMAT = "[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s"
_DEFAULT_LEVEL = logging.INFO


def get_logger(
    name: str,
    level: int | None = None,
    fmt: str | None = None,
) -> logging.Logger:
    """Get or create a named logger with consistent formatting.

    Args:
        name: Logger name, typically the module name (e.g., 'arm_teleop').
        level: Optional log level override (default: INFO).
        fmt: Optional format string override.

    Returns:
        Configured logging.Logger instance.
    """
    if name in _LOGGERS:
        return _LOGGERS[name]

    logger = logging.getLogger(f"teleop.{name}")
    logger.setLevel(level or _DEFAULT_LEVEL)

    if not logger.handlers:
        handler = logging.StreamHandler(sys.stdout)
        handler.setLevel(level or _DEFAULT_LEVEL)
        formatter = logging.Formatter(fmt or _DEFAULT_FORMAT)
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    logger.propagate = False
    _LOGGERS[name] = logger
    return logger


def set_global_level(level: int) -> None:
    """Set log level for all existing teleop loggers.

    Args:
        level: logging.DEBUG, logging.INFO, logging.WARNING, etc.
    """
    for logger in _LOGGERS.values():
        logger.setLevel(level)
        for handler in logger.handlers:
            handler.setLevel(level)


def get_ros2_logger(node_name: str):
    """Get a ROS2-compatible logger from a node.

    Falls back to standard Python logger if rclpy is not available.

    Args:
        node_name: ROS2 node name.

    Returns:
        ROS2 logger or standard Python logger.
    """
    try:
        import rclpy.logging
        return rclpy.logging.get_logger(node_name)
    except ImportError:
        return get_logger(node_name)
