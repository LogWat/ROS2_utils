#include "pcd_process_utils.hpp"
#include <rclcpp/rclcpp.hpp>

namespace utils {

PcdProcessUtils::PcdProcessUtils(const rclcpp::NodeOptions &options)
    : Node("pcd_process_utils", options), is_imu_T_set_(false),
    roll_(0.0), pitch_(0.0), yaw_(0.0)
{
    // パラメータの取得
    this->declare_parameter<std::string>("lidar_sub_topic", "input");
    this->declare_parameter<std::string>("lidar_pub_topic", "output");
    this->declare_parameter<double>("voxel_size", 0.1);
    this->declare_parameter<double>("mean_k", 50);
    this->declare_parameter<double>("stddev_mul_thresh", 1.0);
    this->declare_parameter<double>("search_radius", 0.1);
    this->declare_parameter<double>("distance_threshold", 0.01);
    this->declare_parameter<double>("leaf_size", 0.1);
    this->declare_parameter<int>("pcd_queue_size", 10);

    this->get_parameter("lidar_sub_topic", lidar_sub_topic_);
    this->get_parameter("lidar_pub_topic", lidar_pub_topic_);
    this->get_parameter("voxel_size", voxel_size_);
    this->get_parameter("mean_k", mean_k_);
    this->get_parameter("stddev_mul_thresh", stddev_mul_thresh_);
    this->get_parameter("search_radius", search_radius_);
    this->get_parameter("distance_threshold", distance_threshold_);
    this->get_parameter("leaf_size", leaf_size_);
    this->get_parameter("pcd_queue_size", pcd_queue_size_);

    last_time_ = std::make_shared<rclcpp::Time>(this->get_clock()->now());

    // tf
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        lidar_pub_topic_, qos_lidar
    );
    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_sub_topic_, 
        qos_lidar, 
        std::bind(&PcdProcessUtils::cb_lidar, this, std::placeholders::_1)
    );
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_sub_topic_,
        rclcpp::QoS(10),
        std::bind(&PcdProcessUtils::cb_imu, this, std::placeholders::_1)
    );
}

void PcdProcessUtils::cb_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    pcd_queue_.get()->push_back(cloud);
    if (pcd_queue_.get()->size() > pcd_queue_size_) {
        pcd_queue_.get()->erase(pcd_queue_.get()->begin());
    }

    // voxel grid filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    downsample_by_voxel(cloud, voxel_size_, cloud_filtered);
    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    estimate_normals(cloud_filtered, search_radius_, normals);
    // segment by normals
    std::vector<pcl::PointIndices> cluster_indices;
    normals_clustering(
        cloud_filtered, normals,
        50, 1000,              // min, max cluster size
        50,                    // number of neighbors
        0.1,                   // smoothness threshold
        0.1,                   // curvature threshold
        cluster_indices
    );
    // segmentごとにintensity で色付け
    
}

void PcdProcessUtils::cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (is_livox_imu_) {
        msg.get()->linear_acceleration.x *= 9.80665;
        msg.get()->linear_acceleration.y *= 9.80665;
        msg.get()->linear_acceleration.z *= 9.80665;
    }
    // ついでにtf (lidar->base_link) これは1回だけでいい
    if (!is_imu_T_set_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for TF...");
        try {
            auto transform = tf_buffer_->lookupTransform(
                "base_link", msg.get()->header.frame_id, tf2::TimePointZero
            );
            tf2::fromMsg(transform.transform, imu_T_);
            tf2::fromMsg(transform, imu_T_tf_);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }
        is_imu_T_set_ = true;
    }
    // imuの剛体変換 (imu_link->base_link)
    tf2::Vector3 acc(msg.get()->linear_acceleration.x, msg.get()->linear_acceleration.y, msg.get()->linear_acceleration.z);
    acc = imu_T_tf_.getBasis() * acc;
    msg.get()->linear_acceleration.x = acc.x();
    msg.get()->linear_acceleration.y = acc.y();
    msg.get()->linear_acceleration.z = acc.z();
    tf2::Vector3 gyro(msg.get()->angular_velocity.x, msg.get()->angular_velocity.y, msg.get()->angular_velocity.z);
    gyro = imu_T_tf_.getBasis() * gyro;
    msg.get()->angular_velocity.x = gyro.x();
    msg.get()->angular_velocity.y = gyro.y();
    msg.get()->angular_velocity.z = gyro.z();

    // queueに入るのは剛体変換後(=base_linkにおける)のimu情報
    imu_queue_.get()->push_back(msg);
    if (imu_queue_.get()->size() > imu_queue_size_) {
        imu_queue_.get()->erase(imu_queue_.get()->begin());
    }

    // imuから算出される"robotの"傾き
    // (剛体変換後のimu情報を使われるのでbase_linkにおける傾きが求まる)
    // EKFによる加速度を使った姿勢推定
    auto now = imu_queue_.get()->back().get()->header.stamp;
    auto dt = (now.sec - last_time_.get()->seconds());

}



}; // namespace utils

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(utils::PcdProcessUtils)
