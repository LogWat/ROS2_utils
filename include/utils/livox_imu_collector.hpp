#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>

namespace utils {

class LivoxIMUCollector : public rclcpp::Node
{
public:
    LivoxIMUCollector(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    LivoxIMUCollector(
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