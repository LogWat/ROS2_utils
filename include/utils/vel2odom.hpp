#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <string>

namespace utils {
class Vel2Odom : public rclcpp::Node
{
public:
    Vel2Odom(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    Vel2Odom(
        const std::string &name_space,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void cb_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
    void cb_gyro(const sensor_msgs::msg::Imu::SharedPtr msg);

    void init_odom() {
        prev_odom_ = std::make_shared<nav_msgs::msg::Odometry>();
        prev_odom_->header.frame_id = odom_frame_id_;
        prev_odom_->child_frame_id = base_frame_id_;
        prev_odom_->pose.pose.position.z = 0.0;
        prev_odom_->pose.pose.orientation.x = 0.0;
        prev_odom_->pose.pose.orientation.y = 0.0;
        prev_odom_->pose.pose.orientation.z = 0.0;
        prev_odom_->pose.pose.orientation.w = 1.0;
        prev_odom_->twist.twist.linear.x = 0.0;
        prev_odom_->twist.twist.linear.y = 0.0;
        prev_odom_->twist.twist.linear.z = 0.0;
        prev_odom_->twist.twist.angular.x = 0.0;
        prev_odom_->twist.twist.angular.y = 0.0;
        prev_odom_->twist.twist.angular.z = 0.0;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_gyro_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    std::shared_ptr<nav_msgs::msg::Odometry> prev_odom_;
    std::shared_ptr<geometry_msgs::msg::Twist> vel_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    rclcpp::Time prev_time_, odom_check_time_;
    std::string odom_frame_id_, base_frame_id_;
    std::string vel_topic_, gyro_topic_;
    double odom_pub_rate_;
};

} // namespace utils