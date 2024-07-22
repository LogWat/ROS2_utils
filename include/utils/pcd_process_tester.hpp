#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>
#include <tuple>
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl_conversions/pcl_conversions.h>

rmw_qos_profile_t qos_profile_lidar{
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

auto qos_lidar = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_lidar.history,
        qos_profile_lidar.depth
    ),
    qos_profile_lidar);

namespace utils {
class PcdProcessTester : public rclcpp::Node
{
public:
    PcdProcessTester(const rclcpp::NodeOptions &options);
    PcdProcessTester(
        const std::string &name_space,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
    );

private:
    // 法線推定 & 特徴量計算
    std::tuple<pcl::PointCloud<pcl::Normal>::Ptr, pcl::PointCloud<pcl::FPFHSignature33>::Ptr> preprocess_pcd(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const float voxel_size
    );

    // 外れ値除去 (平滑化)
    pcl::PointCloud<pcl::PointXYZ>::Ptr remove_outlier(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const float mean_k,
        const float std_dev
    );



};

} // namespace utils
