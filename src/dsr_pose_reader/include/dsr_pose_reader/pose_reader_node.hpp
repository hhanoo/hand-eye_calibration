#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "DRFL.h"

namespace dsr_pose_reader {

class PoseReaderNode : public rclcpp::Node {
   public:
    PoseReaderNode();
    ~PoseReaderNode() override;

   private:
    void connect_to_robot();
    void publish_timer_cb();

    // Convert Euler ZYZ (deg) + position (mm) to PoseStamped (m, quaternion)
    static geometry_msgs::msg::PoseStamped to_quaternion_pose(
        const float pos[6], const float rot[3][3],
        const std::string& frame_id, const rclcpp::Time& stamp);

    // DRFL monitoring callback (static, called from DRFL thread)
    static void on_monitoring_data(const LPMONITORING_DATA pData);

    // DRFL instance
    DRAFramework::CDRFL drfl_;

    // Latest pose from monitoring callback
    static std::mutex        s_mtx_;
    static float             s_tcp_pos_[6];        // (x, y, z, a, b, c) [mm, deg]
    static float             s_rot_matrix_[3][3];  // rotation matrix from DRFL
    static std::atomic<bool> s_data_ready_;

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr                                  timer_;

    // Parameters
    std::string robot_ip_;
    int         robot_port_;
    std::string base_frame_;
    std::string ee_frame_;
    double      publish_rate_;
    bool        publish_tf_;
};

}  // namespace dsr_pose_reader
