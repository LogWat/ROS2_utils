#include "utils/livox_imu_collector.hpp"
#include <rclcpp/rclcpp.hpp>

namespace utils {

LivoxImuCollector::LivoxImuCollector(const rclcpp::NodeOptions &options)
    : Node("livox_imu_collector", options)
{
    // パラメータの取得
    this->declare_parameter<std::string>("imu_sub_topic", "imu");
    this->declare_parameter<std::string>("imu_pub_topic", "imu_corrected");
    this->declare_parameter<std::string>("imu_frame_id", "imu");
    this->declare_parameter<double>("diff_roll", 0.0);
    this->declare_parameter<double>("diff_pitch", 0.0);
    this->declare_parameter<double>("diff_yaw", 0.0);
    this->declare_parameter<double>("diff_x", 0.0);
    this->declare_parameter<double>("diff_y", 0.0);
    this->declare_parameter<double>("diff_z", 0.0);
    this->declare_parameter<double>("acc_coff", 9.80665); // 重力加速度 (LIVOX ONLY)
    this->declare_parameter<double>("gyro_coff", 1.0);

    this->get_parameter("imu_sub_topic", imu_sub_topic_);
    this->get_parameter("imu_pub_topic", imu_pub_topic_);
    this->get_parameter("imu_frame_id", imu_frame_id_);
    this->get_parameter("diff_roll", diff_roll_);
    this->get_parameter("diff_pitch", diff_pitch_);
    this->get_parameter("diff_yaw", diff_yaw_);
    this->get_parameter("diff_x", diff_x_);
    this->get_parameter("diff_y", diff_y_);
    this->get_parameter("diff_z", diff_z_);
    this->get_parameter("acc_coff", acc_coff_);
    this->get_parameter("gyro_coff", gyro_coff_);

    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_pub_topic_, qos_imu);
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_sub_topic_, qos_imu, std::bind(&LivoxImuCollector::callback, this, std::placeholders::_1));

    // LiDAR間の変換行列の設定
    tf2::Quaternion q;
    q.setRPY(diff_roll_, diff_pitch_, diff_yaw_);
    tf2::Vector3 v(diff_x_, diff_y_, diff_z_);
    imu_diff_transform_.setRotation(q);
    imu_diff_transform_.setOrigin(v);
}

void LivoxImuCollector::callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    sensor_msgs::msg::Imu imu_corrected = *msg;
    imu_corrected.header.frame_id = imu_frame_id_;
    imu_corrected.header.stamp = msg->header.stamp;

    // 加速度の補正
    tf2::Vector3 acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    acc = imu_diff_transform_.getBasis() * acc;
    imu_corrected.linear_acceleration.x = acc.x() * acc_coff_;
    imu_corrected.linear_acceleration.y = acc.y() * acc_coff_;
    imu_corrected.linear_acceleration.z = acc.z() * acc_coff_;

    // 角速度の補正
    tf2::Vector3 gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    gyro = imu_diff_transform_.getBasis() * gyro;
    imu_corrected.angular_velocity.x = gyro.x() * gyro_coff_;
    imu_corrected.angular_velocity.y = gyro.y() * gyro_coff_;
    imu_corrected.angular_velocity.z = gyro.z() * gyro_coff_;

    pub_imu_->publish(imu_corrected);
}

} // namespace utils

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(utils::LivoxImuCollector)