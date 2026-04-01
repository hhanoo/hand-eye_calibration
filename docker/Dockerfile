# Ubuntu Release -----------------------------------------------------------
ARG UBUNTU_RELEASE=22.04

# CUDA Release ------------------------------------------------------------
ARG CUDA_VERSION=12.5.1
ARG CUDA_RELEASE=12.5
ARG CUDA_PURPOSE=devel
ARG CUDNN_VERSION=""

# CUDA Docker Image Download -----------------------------------------------
FROM nvcr.io/nvidia/cuda:${CUDA_VERSION}-${CUDNN_VERSION}${CUDA_PURPOSE}-ubuntu${UBUNTU_RELEASE}

# Non-interactive mode, NVIDIA settings ------------------------------------
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# Locale and Font Settings -------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    fonts-noto-color-emoji \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# Essential Packages -------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-transport-https \
    apt-utils \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    git \
    gnupg2 \
    lsb-release \
    python3-pip \
    software-properties-common \
    sudo \
    terminator \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

# ROS2 Humble Repository Setup --------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

# ROS2 Humble Installation ------------------------------------------------
ARG ROS_VERSION=humble
ENV ROS_DISTRO=${ROS_VERSION}
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# ROS2 Control Dependencies -----------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# Intel RealSense SDK ------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-realsense2-* \
    && rm -rf /var/lib/apt/lists/*

# Vision ROS2 Packages (cv_bridge, tf2, image_transport) -------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-rclpy \
    && rm -rf /var/lib/apt/lists/*

# Robot Description and Visualization -------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Tool Dependencies (USB for RealSense) -----------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Vision Dependencies (C++) -----------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# GUI / OpenGL / GPU Packages (for PyQt5 display) -------------------------
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
    mesa-utils \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Python base --------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-dev \
    python3-vcstool \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install ROS2-compatible setuptools -----------------------
RUN python3 -m pip install --no-cache-dir --upgrade \
    pip \
    setuptools==58.2.0 \
    packaging==21.3 \
    wheel

# Python Dependencies for Hand-Eye Calibration ----------------------------
RUN python3 -m pip install --no-cache-dir \
    'numpy>=1.21.0,<2.0' \
    scipy \
    sympy \
    opencv-contrib-python-headless==4.10.0.84 \
    PyQt5 \
    PyQt5-sip \
    matplotlib

# USB device access groups -------------------------------------------------
RUN groupadd -f plugdev && \
    groupadd -f dialout && \
    usermod -aG plugdev root && \
    usermod -aG dialout root && \
    usermod -aG video root

# rosdep initialization ---------------------------------------------------
RUN rosdep init && rosdep update

# CUDA PATH environment variables -----------------------------------------
RUN echo "export PATH=/usr/local/cuda/bin:\$PATH" >> /root/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib64:\$LD_LIBRARY_PATH" >> /root/.bashrc

# Setup workspace ----------------------------------------------------------
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

# Aliases ---------------------------------------------------------------
COPY ./aliases.sh /etc/profile.d/aliases.sh
RUN chmod 644 /etc/profile.d/aliases.sh && \
    echo 'source /etc/profile.d/aliases.sh' >> /etc/bash.bashrc

# Entrypoint ---------------------------------------------------------------
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Default command ----------------------------------------------------------
CMD ["bash"]
