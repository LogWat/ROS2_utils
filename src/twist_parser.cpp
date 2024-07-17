#include "utils/twist_parser.hpp"
#include <rclcpp/rclcpp.hpp>

namespace utils {
TwistParser::TwistParser(const rclcpp::NodeOptions &options)
    : Node("twist_parser", options)
{
    this->declare_parameter("twist_raw_topic", "/twist");
    this->declare_parameter("twist_with_cov_topic", "/twist_with_cov");
    this->declare_parameter("pose_coff", 1.25);

    this->get_parameter("twist_raw_topic", twist_topic_);
    this->get_parameter("twist_with_cov_topic", pose_topic_);
    this->get_parameter("pose_coff", pose_coff_);

    vel_ = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();

    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
        twist_topic_, qos_vel_,
        std::bind(&TwistParser::callback, this, std::placeholders::_1));
    pub_pose_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        pose_topic_, qos_vel_);

    std::thread(std::bind(&TwistParser::pub_loop, this)).detach();
    RCLCPP_INFO(this->get_logger(), "TwistParser has been started.");
}

void TwistParser::callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    geometry_msgs::msg::TwistWithCovarianceStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "base_link";
    pose.twist.twist.linear.x = msg->linear.x * pose_coff_;
    pose.twist.twist.linear.y = msg->linear.y * pose_coff_;
    pose.twist.twist.linear.z = msg->linear.z * pose_coff_;
    pose.twist.twist.angular.x = msg->angular.x * pose_coff_;
    pose.twist.twist.angular.y = msg->angular.y * pose_coff_;
    pose.twist.twist.angular.z = msg->angular.z * pose_coff_;
    pose.twist.covariance[0] = 0.01;
    pose.twist.covariance[7] = 0.01;
    pose.twist.covariance[14] = 0.01;
    pose.twist.covariance[21] = 0.01;
    pose.twist.covariance[28] = 0.01;
    pose.twist.covariance[35] = 0.01;

    vel_ = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>(pose);
}

void TwistParser::pub_loop() {
    rclcpp::Rate rate(200);
    while (rclcpp::ok()) {
        if (vel_) pub_pose_->publish(*vel_.get());
        rate.sleep();
    }
}

} // namespace utils

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(utils::TwistParser)