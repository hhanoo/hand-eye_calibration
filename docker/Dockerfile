# =========================================================================
# Base Image (NVIDIA CUDA + Ubuntu)
# =========================================================================
# - https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda
ARG UBUNTU_RELEASE=22.04
ARG CUDA_VERSION=12.5.1
ARG CUDA_RELEASE=12.5
ARG CUDA_PURPOSE=devel
ARG CUDNN_VERSION=""

FROM nvcr.io/nvidia/cuda:${CUDA_VERSION}-${CUDNN_VERSION}${CUDA_PURPOSE}-ubuntu${UBUNTU_RELEASE}

# - Build dependencies
ARG DEBIAN_FRONTEND=noninteractive

# - NVIDIA runtime settings
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    XDG_RUNTIME_DIR=/tmp/runtime-root

# =========================================================================
# Locale and Font
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    fonts-noto-color-emoji \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# =========================================================================
# Essential Packages
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-transport-https \
    apt-utils \
    ca-certificates \
    curl \
    git \
    gnupg2 \
    lsb-release \
    software-properties-common \
    sudo \
    wget \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# ROS2
# =========================================================================
# - ROS2 repository setup
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

# - ROS2 setup
ARG ROS_VERSION=humble
ENV ROS_DISTRO=${ROS_VERSION}

# - ROS2 Desktop + dev tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# - ROS2 Control
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# - ROS2 CycloneDDS RMW (preferred for large sensor messages)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# - ROS2 Vision / Image transport (generic, shared by GUI node & camera drivers)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-image-publisher \
    ros-${ROS_DISTRO}-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*

# - ROS2 Robot Description / Visualization
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# Vision Sensor: Intel RealSense (D415 / D435 / D455 etc.)
# =========================================================================
# - Driver installed via apt (precompiled binary)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-realsense2-* \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# Vision Sensor: Orbbec (Femto Mega/Bolt, Gemini 2 / 330 series etc.)
# =========================================================================
# - OrbbecSDK_ROS2 is built from source via VCS; install build/runtime deps
# - ros-* runtime msgs : diagnostic-updater/msgs, statistics-msgs, backward-ros
# - C/C++ build deps    : libgflags-dev, nlohmann-json3-dev,
#                         libdw-dev (backward-ros), libssl-dev,
#                         libgoogle-glog-dev
# - GL runtime          : mesa-utils, libgl1 (SDK sample/preview)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-diagnostic-msgs \
    ros-${ROS_DISTRO}-statistics-msgs \
    ros-${ROS_DISTRO}-backward-ros \
    libgflags-dev \
    nlohmann-json3-dev \
    libdw-dev \
    libssl-dev \
    libgoogle-glog-dev \
    mesa-utils \
    libgl1 \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# Python
# =========================================================================
# - Python base
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-dev \
    python3-pip \
    python3-vcstool \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# - pip / setuptools (ROS2-compatible versions)
RUN python3 -m pip install --no-cache-dir --upgrade \
    pip \
    setuptools==58.2.0 \
    packaging==21.3 \
    wheel

# - Python dependencies for Hand-Eye Calibration
RUN python3 -m pip install --no-cache-dir \
    'numpy>=1.21.0,<2.0' \
    scipy \
    sympy \
    opencv-contrib-python-headless==4.10.0.84 \
    PyQt5 \
    PyQt5-sip \
    matplotlib

# =========================================================================
# Development Tools
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    clangd \
    cmake \
    gdb \
    gdbserver \
    pkg-config \
    terminator \
    vim \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# GUI / OpenGL (for PyQt5 display)
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libxcb-glx0 \
    libxrender1 \
    libxi6 \
    libxtst6 \
    libxrandr2 \
    libxinerama1 \
    libxcursor1 \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# C++ Build Dependencies
# =========================================================================
# - libeigen3-dev    : hand_eye_calibration, dsr_pose_reader
# - libopencv-dev    : hand_eye_calibration (ArUco)
# - libusb-1.0-0-dev : realsense2-camera, OrbbecSDK_ROS2
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev \
    libopencv-dev \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    udev \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# Runtime Setup
# =========================================================================
# - USB device access groups (plugdev/dialout/video for vision sensors)
RUN groupadd -f plugdev && \
    groupadd -f dialout && \
    usermod -aG plugdev root && \
    usermod -aG dialout root && \
    usermod -aG video root

# - rosdep initialization
RUN rosdep init && rosdep update

# - CUDA PATH
ENV PATH=/usr/local/cuda/bin:${PATH} \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

# =========================================================================
# Workspace Setup
# =========================================================================
# - Workspace
WORKDIR /ros2_ws

# - Commands
COPY ./commands.sh /etc/profile.d/commands.sh
RUN chmod 644 /etc/profile.d/commands.sh && \
    echo 'source /etc/profile.d/commands.sh' >> /etc/bash.bashrc

# - CycloneDDS configuration
COPY ./cyclonedds.xml /etc/cyclonedds.xml

# - Entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# - Default command
CMD ["bash"]
