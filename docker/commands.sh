# ===== ROS env =====
[ -f /opt/ros/humble/setup.bash ]      && source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ]     && source /ros2_ws/install/setup.bash
[ -f /ros2_ws/docker/config.sh ]       && source /ros2_ws/docker/config.sh

# ===== Common helpers =====
source-ros-ws() {
    [ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
}

source-config() {
    [ -f /ros2_ws/docker/config.sh ] && source /ros2_ws/docker/config.sh
}

# ===== Build =====
# if you want to show details, use `--event-handlers console_direct+`
build() {
    cd /ros2_ws || return 1
    colcon build \
        --symlink-install \
        --cmake-args \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            -DCMAKE_BUILD_TYPE=Release "$@"
    source-ros-ws
    source-config
}

build-debug() {
    # Build with debug symbols, keep optimization (RelWithDebInfo)
    build -DCMAKE_BUILD_TYPE=RelWithDebInfo "$@"
}

# ===== Debug =====
# Run a node under gdbserver so host VSCode (cppdbg) can attach on :3000.
debug-doosan() {
    source-ros-ws
    gdbserver :3000 /ros2_ws/build/dsr_pose_reader/pose_reader_node "$@"
}

# ===== Camera launchers =====
camera-realsense() {
    source-ros-ws
    ros2 launch realsense2_camera rs_launch.py \
        depth_module.depth_profile:=1280x720x30 \
        rgb_camera.color_profile:=1280x720x30 \
        align_depth.enable:=true \
        "$@"
}

camera-orbbec() {
    source-ros-ws
    source-config
    ros2 launch orbbec_camera "${ORBBEC_MODEL:-femto_bolt}.launch.py" \
        color_width:="${ORBBEC_COLOR_WIDTH:-1280}" \
        color_height:="${ORBBEC_COLOR_HEIGHT:-720}" \
        color_fps:="${ORBBEC_COLOR_FPS:-30}" \
        "$@"
}

# ===== GUI launchers =====
gui-realsense() {
    source-ros-ws
    ros2 run hand_eye_calibration gui_node "$@"
}

gui-orbbec() {
    source-ros-ws
    ros2 run hand_eye_calibration gui_node --ros-args \
        -p image_topic:=/camera/color/image_raw \
        -p camera_info_topic:=/camera/color/camera_info \
        "$@"
}

# ===== Robot launchers =====
doosan() {
    source-ros-ws
    ros2 launch dsr_pose_reader dsr_pose_reader.launch.py "$@"
}

# ===== Help =====
cmd-help() {
    printf "\n[hand_eye_calibration] Commands:\n\n"

    printf "  Build:\n"
    printf "    %-18s - %s\n" "build"             "colcon build (Release) + source overlay"
    printf "    %-18s - %s\n" "build-debug"       "build with debug symbols (RelWithDebInfo)"
    printf "\n"

    printf "  Debug (gdbserver :3000, host VSCode F5 attach):\n"
    printf "    %-18s - %s\n" "debug-doosan"      "Run pose_reader_node under gdbserver"
    printf "\n"

    printf "  Camera launchers:\n"
    printf "    %-18s - %s\n" "camera-realsense"  "Intel RealSense (D415 / D435)"
    printf "    %-18s - %s\n" "camera-orbbec"     "Orbbec camera             [ORBBEC_MODEL, ORBBEC_COLOR_*]"
    printf "\n"

    printf "  GUI launchers:\n"
    printf "    %-18s - %s\n" "gui-realsense"     "Hand-eye calibration GUI (RealSense topics)"
    printf "    %-18s - %s\n" "gui-orbbec"        "Hand-eye calibration GUI (Orbbec topics)"
    printf "\n"

    printf "  Robot launchers:\n"
    printf "    %-18s - %s\n" "doosan"            "Doosan pose reader (dsr_pose_reader)"
    printf "\n"

    printf "  Config / Help:\n"
    printf "    %-18s - %s\n" "source-config"     "Reload /ros2_ws/docker/config.sh"
    printf "    %-18s - %s\n" "cmd-help"          "Show this help"
    printf "\n"

    printf "  Current config (from /ros2_ws/docker/config.sh):\n"
    printf "    ORBBEC_MODEL=%s\n"        "${ORBBEC_MODEL}"
    printf "    ORBBEC_COLOR_WIDTH=%s\n"  "${ORBBEC_COLOR_WIDTH}"
    printf "    ORBBEC_COLOR_HEIGHT=%s\n" "${ORBBEC_COLOR_HEIGHT}"
    printf "    ORBBEC_COLOR_FPS=%s\n"    "${ORBBEC_COLOR_FPS}"
    echo
}

# ===== Show help on interactive shell =====
case $- in
    *i*) cmd-help ;;
esac
