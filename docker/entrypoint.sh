#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Write aliases to a file so the exec'd shell can source them
ALIAS_FILE="/etc/hand_eye_cal_aliases.sh"
cat > "$ALIAS_FILE" << 'ALIASEOF'
# Sourced by .bashrc so aliases work in interactive shell
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
alias camera='ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 rgb_camera.color_profile:=1280x720x30 align_depth.enable:=true'
alias calibrate='ros2 run hand_eye_calibration gui_node'
alias calibrate_launch='ros2 launch hand_eye_calibration calibration.launch.py'
ALIASEOF

# Ensure root's .bashrc sources the alias file (once)
if [ -w /root/.bashrc ]; then
    grep -q 'hand_eye_cal_aliases' /root/.bashrc 2>/dev/null || echo '[ -f /etc/hand_eye_cal_aliases.sh ] && source /etc/hand_eye_cal_aliases.sh' >> /root/.bashrc
fi

exec "$@"
