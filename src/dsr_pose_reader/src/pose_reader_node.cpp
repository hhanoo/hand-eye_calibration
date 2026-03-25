#include "dsr_pose_reader/pose_reader_node.hpp"

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace dsr_pose_reader {

// Static members
std::mutex        PoseReaderNode::s_mtx_;
float             PoseReaderNode::s_tcp_pos_[6]       = {};
float             PoseReaderNode::s_rot_matrix_[3][3] = {};
std::atomic<bool> PoseReaderNode::s_data_ready_{false};

PoseReaderNode::PoseReaderNode()
    : Node("dsr_pose_reader") {
    // Declare parameters
    this->declare_parameter("robot_ip", "192.168.137.100");
    this->declare_parameter("robot_port", 12345);
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("ee_frame", "tool0");
    this->declare_parameter("publish_rate", 30.0);
    this->declare_parameter("publish_tf", true);

    robot_ip_     = this->get_parameter("robot_ip").as_string();
    robot_port_   = this->get_parameter("robot_port").as_int();
    base_frame_   = this->get_parameter("base_frame").as_string();
    ee_frame_     = this->get_parameter("ee_frame").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    publish_tf_   = this->get_parameter("publish_tf").as_bool();

    // Create publisher and TF broadcaster
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("tcp_pose", 10);
    if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    connect_to_robot();

    // Publish timer
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_      = this->create_wall_timer(
             std::chrono::duration_cast<std::chrono::nanoseconds>(period),
             std::bind(&PoseReaderNode::publish_timer_cb, this));

    RCLCPP_INFO(this->get_logger(), "Publishing TCP pose at %.1f Hz on '%s'",
                publish_rate_, pose_pub_->get_topic_name());
}

PoseReaderNode::~PoseReaderNode() {
    timer_.reset();
    drfl_.CloseConnection();
    RCLCPP_INFO(this->get_logger(), "Disconnected from Doosan controller");
}

void PoseReaderNode::connect_to_robot() {
    RCLCPP_INFO(this->get_logger(), "Connecting to Doosan controller at %s:%d (read-only)...",
                robot_ip_.c_str(), robot_port_);

    // Register monitoring callback BEFORE connecting
    drfl_.SetOnMonitoringData(PoseReaderNode::on_monitoring_data);

    if (!drfl_.OpenConnection(robot_ip_, static_cast<unsigned int>(robot_port_))) {
        RCLCPP_FATAL(this->get_logger(), "Failed to connect to %s:%d", robot_ip_.c_str(), robot_port_);
        rclcpp::shutdown();
        return;
    }

    // NOTE: ManageAccessControl() is intentionally NOT called.
    // This keeps the connection read-only so pendant control is preserved.

    RCLCPP_INFO(this->get_logger(), "Connected (read-only, pendant control preserved)");
}

void PoseReaderNode::on_monitoring_data(const LPMONITORING_DATA pData) {
    if (!pData)
        return;

    std::lock_guard<std::mutex> lock(s_mtx_);

    // _fActualPos[0] = tool position, [1] = flange position
    for (int i = 0; i < 6; ++i) {
        s_tcp_pos_[i] = pData->_tCtrl._tTask._fActualPos[0][i];
    }
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            s_rot_matrix_[r][c] = pData->_tCtrl._tTask._fRotationMatrix[r][c];
        }
    }

    s_data_ready_.store(true);
}

void PoseReaderNode::publish_timer_cb() {
    if (!s_data_ready_.load())
        return;

    float pos[6];
    float rot[3][3];
    {
        std::lock_guard<std::mutex> lock(s_mtx_);
        std::memcpy(pos, s_tcp_pos_, sizeof(pos));
        std::memcpy(rot, s_rot_matrix_, sizeof(rot));
    }

    auto now = this->get_clock()->now();
    auto msg = to_quaternion_pose(pos, rot, base_frame_, now);
    pose_pub_->publish(msg);

    // Broadcast TF
    if (publish_tf_ && tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header                  = msg.header;
        tf.child_frame_id          = ee_frame_;
        tf.transform.translation.x = msg.pose.position.x;
        tf.transform.translation.y = msg.pose.position.y;
        tf.transform.translation.z = msg.pose.position.z;
        tf.transform.rotation      = msg.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
    }
}

geometry_msgs::msg::PoseStamped PoseReaderNode::to_quaternion_pose(
    const float pos[6], const float rot[3][3],
    const std::string& frame_id, const rclcpp::Time& stamp) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = frame_id;

    // Position: mm -> m
    msg.pose.position.x = static_cast<double>(pos[0]) * 0.001;
    msg.pose.position.y = static_cast<double>(pos[1]) * 0.001;
    msg.pose.position.z = static_cast<double>(pos[2]) * 0.001;

    // Rotation matrix -> quaternion
    double R[3][3];
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            R[r][c] = static_cast<double>(rot[r][c]);

    double trace = R[0][0] + R[1][1] + R[2][2];
    double qw, qx, qy, qz;

    if (trace > 0.0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        qw       = 0.25 / s;
        qx       = (R[2][1] - R[1][2]) * s;
        qy       = (R[0][2] - R[2][0]) * s;
        qz       = (R[1][0] - R[0][1]) * s;
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
        qw       = (R[2][1] - R[1][2]) / s;
        qx       = 0.25 * s;
        qy       = (R[0][1] + R[1][0]) / s;
        qz       = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
        qw       = (R[0][2] - R[2][0]) / s;
        qx       = (R[0][1] + R[1][0]) / s;
        qy       = 0.25 * s;
        qz       = (R[1][2] + R[2][1]) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
        qw       = (R[1][0] - R[0][1]) / s;
        qx       = (R[0][2] + R[2][0]) / s;
        qy       = (R[1][2] + R[2][1]) / s;
        qz       = 0.25 * s;
    }

    // Normalize
    double norm            = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    msg.pose.orientation.w = qw / norm;
    msg.pose.orientation.x = qx / norm;
    msg.pose.orientation.y = qy / norm;
    msg.pose.orientation.z = qz / norm;

    return msg;
}

}  // namespace dsr_pose_reader

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dsr_pose_reader::PoseReaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
