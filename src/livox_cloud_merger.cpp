#include "livox_cloud_merger.hpp"

#include <rclcpp/rclcpp.hpp>

namespace utils {

LivoxCloudMerger::LivoxCloudMerger(const rclcpp::NodeOptions & options)
    : Node("livox_cloud_merger", options),
      ring_(0)
    {
        // パラメータの取得
        this->declare_parameter<std::string>("pointcloud1_topic", "pointcloud1");
        this->declare_parameter<std::string>("pointcloud2_topic", "pointcloud2");
        this->declare_parameter<std::string>("output_topic", "pointcloud");
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->declare_parameter<double>("lidar_diffx", 0.0);
        this->declare_parameter<double>("lidar_diffy", 0.0);
        this->declare_parameter<double>("lidar_diffz", 0.0);
        this->declare_parameter<double>("lidar_diffroll", 0.0);
        this->declare_parameter<double>("lidar_diffpitch", 0.0);
        this->declare_parameter<double>("lidar_diffyaw", 0.0);

        this->get_parameter("pointcloud1_topic", pointcloud1_topic_);
        this->get_parameter("pointcloud2_topic", pointcloud2_topic_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("lidar_diffx", lidar_diffx_);
        this->get_parameter("lidar_diffy", lidar_diffy_);
        this->get_parameter("lidar_diffz", lidar_diffz_);
        this->get_parameter("lidar_diffroll", lidar_diffroll_);
        this->get_parameter("lidar_diffpitch", lidar_diffpitch_);
        this->get_parameter("lidar_diffyaw", lidar_diffyaw_);

        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

        pointcloud1_sub_.subscribe(this, pointcloud1_topic_);
        pointcloud2_sub_.subscribe(this, pointcloud2_topic_);

        sync_.reset(new Sync(SyncPolicy(10), pointcloud1_sub_, pointcloud2_sub_));
        sync_->registerCallback(&LivoxCloudMerger::callback, this);

        // LiDAR間の変換行列の設定
        Eigen::Affine3f translation(Eigen::Translation3f(lidar_diffx_, lidar_diffy_, lidar_diffz_));
        Eigen::Affine3f rotation(Eigen::AngleAxisf(lidar_diffroll_, Eigen::Vector3f::UnitX()) *
                                 Eigen::AngleAxisf(lidar_diffpitch_, Eigen::Vector3f::UnitY()) *
                                 Eigen::AngleAxisf(lidar_diffyaw_, Eigen::Vector3f::UnitZ()));
        lidar_diff_transform_ = translation * rotation;
    }

}; // namespace respkg

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(utils::LivoxCloudMerger)