from setuptools import setup, find_packages

package_name = "teleop_system"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    install_requires=[
        "numpy>=1.26.0",
        "scipy>=1.12.0",
        "pyyaml>=6.0",
        "hydra-core>=1.3.0",
        "omegaconf>=2.3.0",
        "transforms3d>=0.4.1",
    ],
    extras_require={
        "sim": ["mujoco>=3.4.0"],
        "ik": ["pin>=2.7.0", "pin-pink>=3.4.0"],
        "vr": ["openvr>=2.12.1401"],
        "viz": ["open3d>=0.19.0"],
        "gui": ["dearpygui>=2.1.1"],
        "all": [
            "mujoco>=3.4.0",
            "pin>=2.7.0",
            "pin-pink>=3.4.0",
            "openvr>=2.12.1401",
            "open3d>=0.19.0",
            "dearpygui>=2.1.1",
        ],
        "test": ["pytest>=7.0.0", "pytest-cov>=4.0.0"],
    },
    entry_points={
        "console_scripts": [
            "run_teleop=scripts.run_teleop:main",
            "mujoco_bridge=teleop_system.simulators.mujoco_ros2_bridge:main",
            "dummy_tracker_pub=teleop_system.simulators.dummy_tracker_pub:main",
            "dummy_glove_pub=teleop_system.simulators.dummy_glove_pub:main",
            "vive_tracker_pub=teleop_system.devices.vive_tracker_pub:main",
            "arm_teleop_node=teleop_system.modules.arm_teleop.arm_teleop_node:main",
            "locomotion_node=teleop_system.modules.locomotion.locomotion_node:main",
            "hand_teleop_node=teleop_system.modules.hand_teleop.hand_teleop_node:main",
            "camera_teleop_node=teleop_system.modules.camera.camera_node:main",
            "dummy_hmd_pub=teleop_system.simulators.dummy_hmd_pub:main",
        ],
    },
    python_requires=">=3.10",
    author="Teleop System Team",
    description="RB-Y1 Teleoperation System with VR-based master control",
)
