#include "utils/vel2odom.hpp"
#include <rclcpp/rclcpp.hpp>

namespace utils {
Vel2Odom::Vel2Odom(const rclcpp::NodeOptions & options) 
    : Node("vel2odom", options),
      tf_broadcaster_(*this),
        prev_time_(this->now()),
        odom_check_time_(this->now())
{
    // パラメータの取得
    this->declare_parameter<std::string>("vel_topic", "vel");
    this->declare_parameter<std::string>("gyro_topic", "gyro");
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("base_frame_id", "base_link");
    this->declare_parameter<double>("odom_pub_rate", 10.0);

    this->get_parameter("vel_topic", vel_topic_);
    this->get_parameter("gyro_topic", gyro_topic_);
    this->get_parameter("odom_frame_id", odom_frame_id_);
    this->get_parameter("base_frame_id", base_frame_id_);
    this->get_parameter("odom_pub_rate", odom_pub_rate_);

    // Publisherの設定
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Subscriberの設定
    sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(vel_topic_, 10, std::bind(&Vel2Odom::cb_vel, this, std::placeholders::_1));
    sub_gyro_ = this->create_subscription<sensor_msgs::msg::Imu>(gyro_topic_, 10, std::bind(&Vel2Odom::cb_gyro, this, std::placeholders::_1));

    tf_broadcaster_ = tf2_ros::TransformBroadcaster(*this);

    // 初期化
    init_odom();

}

void Vel2Odom::cb_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    vel_ = msg;
}

void Vel2Odom::cb_gyro(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (vel_ == nullptr) {
        return;
    }
    // 現在時刻の取得
    rclcpp::Time current_time = this->now();

    // 過剰なodomのpublishを防ぐ
    rclcpp::Duration t = current_time - odom_check_time_;
    if (t.seconds() < 1.0 / odom_pub_rate_) {
        return;
    }
    odom_check_time_ = current_time;

    // 前回時刻からの経過時間の取得
    rclcpp::Duration dt = current_time - prev_time_;

    double vx = vel_->linear.x;
    double vw = -msg->angular_velocity.y;
    double vroll = msg->angular_velocity.z;
    double vpitch = -msg->angular_velocity.x;
    geometry_msgs::msg::Quaternion q = prev_odom_->pose.pose.orientation;
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
    double dx = vx * cos(yaw) * dt.seconds();
    double dy = vx * sin(yaw) * dt.seconds();
    double dth = vw * dt.seconds();
    double droll = vroll * dt.seconds();
    double dpitch = vpitch * dt.seconds();
    prev_odom_->header.stamp = current_time;
    prev_odom_->pose.pose.position.x += dx;
    prev_odom_->pose.pose.position.y += dy;
    // rpy to quaternion
    tf2::Quaternion q_new;
    q_new.setRPY(roll + droll, pitch + dpitch, yaw + dth);
    prev_odom_->pose.pose.orientation.x = q_new.x();
    prev_odom_->pose.pose.orientation.y = q_new.y();
    prev_odom_->pose.pose.orientation.z = q_new.z();
    prev_odom_->pose.pose.orientation.w = q_new.w();
    prev_odom_->twist.twist.linear.x = vx;
    prev_odom_->twist.twist.angular.z = vw;

    pub_odom_->publish(*prev_odom_);
    prev_time_ = current_time;

    // tfのbroadcast
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current_time;
    transform.header.frame_id = odom_frame_id_;
    transform.child_frame_id = base_frame_id_;
    transform.transform.translation.x = prev_odom_->pose.pose.position.x;
    transform.transform.translation.y = prev_odom_->pose.pose.position.y;
    transform.transform.translation.z = prev_odom_->pose.pose.position.z;
    transform.transform.rotation = prev_odom_->pose.pose.orientation;
    tf_broadcaster_.sendTransform(transform);
}


} // namespace utils

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(utils::Vel2Odom)