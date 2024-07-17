#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>

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

auto qos_imu = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_imu.history,
        qos_profile_imu.depth
    ),
    qos_profile_imu);

namespace utils {

class LivoxImuCollector : public rclcpp::Node
{
public:
    LivoxImuCollector(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    LivoxImuCollector(
        const std::string &name_space,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
    );

private:
    void callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    std::string imu_sub_topic_, imu_pub_topic_;
    std::string imu_frame_id_;
    double diff_roll_, diff_pitch_, diff_yaw_;
    double diff_x_, diff_y_, diff_z_;
    tf2::Transform imu_diff_transform_;
    double acc_coff_, gyro_coff_;
};

} // namespace utils