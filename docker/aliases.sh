# ROS env
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash

# Aliases
alias camera='ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 rgb_camera.color_profile:=1280x720x30 align_depth.enable:=true'
alias gui='ros2 run hand_eye_calibration gui_node'
alias launch='ros2 launch hand_eye_calibration calibration.launch.py'
alias doosan='ros2 launch dsr_pose_reader dsr_pose_reader.launch.py'
alias build='cd /ros2_ws && colcon build --symlink-install && source install/setup.bash'
alias cmd_help='echo; echo "[hand_eye_calibration] Commands: camera, gui, launch, doosan, build"'

case $- in
    *i*) cmd_help ;;
esac