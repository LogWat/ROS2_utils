#include "utils/livox_cloud_merger.hpp"

#include <rclcpp/rclcpp.hpp>

namespace utils {

LivoxCloudMerger::LivoxCloudMerger(const rclcpp::NodeOptions & options)
    : Node("livox_cloud_merger", options)
    {
        // パラメータの取得
        this->declare_parameter<std::string>("pointcloud1_topic", "pointcloud1"); // 1が下
        this->declare_parameter<std::string>("pointcloud2_topic", "pointcloud2");
        this->declare_parameter<std::string>("output_topic", "pointcloud");
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->declare_parameter<double>("lidar_diffz", 0.0);
        this->declare_parameter<int>("ring_num", 10);

        this->get_parameter("pointcloud1_topic", pointcloud1_topic_);
        this->get_parameter("pointcloud2_topic", pointcloud2_topic_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("lidar_diffz", lidar_diffz_);
        this->get_parameter("ring_num", ring_num_);

        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

        pointcloud1_sub_.subscribe(this, pointcloud1_topic_);
        pointcloud2_sub_.subscribe(this, pointcloud2_topic_);

        sync_.reset(new Sync(SyncPolicy(10), pointcloud1_sub_, pointcloud2_sub_));
        sync_->registerCallback(&LivoxCloudMerger::callback, this);

        // 仮想LiDARへの変換行列の設定
        lidar_diff_transform_1_.setIdentity();
        lidar_diff_transform_2_.setIdentity();
        lidar_diff_transform_1_.translation() = Eigen::Vector3f(0.0, 0.0, 0.0);
        lidar_diff_transform_2_.translation() = Eigen::Vector3f(0.0, 0.0, lidar_diffz_);
        lidar_diff_transform_2_.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
    }

}; // namespace respkg

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(utils::LivoxCloudMerger)