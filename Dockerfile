# ==============================================================================
# RB-Y1 Teleoperation System — Development Docker Image
#
# Base: ROS2 Jazzy on Ubuntu 24.04
#
# Build:
#   docker build -t teleop-system .
#
# Run (simulation only, no GUI):
#   docker run --rm -it teleop-system
#
# Run (with GUI — X11 forwarding):
#   docker run --rm -it \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     --network host \
#     teleop-system
#
# Run (with GPU — NVIDIA):
#   docker run --rm -it --gpus all \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     --network host \
#     teleop-system
# ==============================================================================

FROM ros:jazzy-ros-base-noble

LABEL maintainer="Teleop System Team"
LABEL description="RB-Y1 Teleoperation System with VR-based master control"

# Prevent interactive prompts during apt install
ENV DEBIAN_FRONTEND=noninteractive
# Pip config for Docker: allow root, allow breaking system packages
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV PIP_ROOT_USER_ACTION=ignore

# ── System dependencies ──────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    python3-pip \
    python3-dev \
    git \
    wget \
    # ROS2 packages
    ros-jazzy-std-srvs \
    ros-jazzy-nav-msgs \
    ros-jazzy-rclpy \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-lifecycle-msgs \
    # System libs for GUI / rendering
    libgl1-mesa-glx \
    libglib2.0-0 \
    libegl1-mesa-dev \
    libglfw3-dev \
    x11-utils \
    # Cleanup
    && rm -rf /var/lib/apt/lists/*

# ── Python dependencies (core) ───────────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    numpy>=1.26.0 \
    scipy>=1.12.0 \
    pyyaml>=6.0 \
    hydra-core>=1.3.0 \
    omegaconf>=2.3.0 \
    pytest>=7.0.0

# ── Python dependencies (simulation) ─────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    mujoco>=3.4.0 \
    lz4>=4.0.0 \
    pillow>=10.0.0

# ── Python dependencies (IK solver) ──────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    pin>=2.7.0 \
    pin-pink>=3.4.0 \
    || echo "IK solver packages failed to install — IK tests will be skipped"

# ── Python dependencies (motion capture) ──────────────────────────────────────
RUN pip3 install --no-cache-dir \
    bvhio>=1.5.0 \
    "matplotlib>=3.9.0" \
    || echo "Mocap packages failed to install — mocap tests will be skipped"

# ── Python dependencies (GUI — optional, may fail on headless) ────────────────
RUN pip3 install --no-cache-dir \
    dearpygui>=2.1.1 \
    || echo "Dear PyGui failed to install — GUI will be disabled"

# ── Copy project ──────────────────────────────────────────────────────────────
WORKDIR /ros2_ws/src/teleop_system
COPY . .

# ── Build ROS2 workspace ─────────────────────────────────────────────────────
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --packages-select teleop_system"

# ── Source workspace on entry ─────────────────────────────────────────────────
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# ── Default command ──────────────────────────────────────────────────────────
WORKDIR /ros2_ws/src/teleop_system
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && python3 -m pytest tests/ -v && echo '=== All tests passed ==='"]
