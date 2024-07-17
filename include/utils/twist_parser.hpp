#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

rmw_qos_profile_t qos_profile_imu{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

auto qos_vel_ = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_imu.history,
        qos_profile_imu.depth
    ),
    qos_profile_imu);

namespace utils {

class TwistParser : public rclcpp::Node
{
public:
    TwistParser(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    TwistParser(
        const std::string &name_space,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
    );

private:
    void callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void pub_loop();

    std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> vel_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_pose_;
    std::string twist_topic_, pose_topic_;
    double pose_coff_;
};

} // namespace utils