#!/bin/bash
set -e

# RMW / DDS configuration (CycloneDDS for large sensor messages).
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -f /etc/cyclonedds.xml ]; then
    export CYCLONEDDS_URI=file:///etc/cyclonedds.xml
fi

# Source ROS2 environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"
