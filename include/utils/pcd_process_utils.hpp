#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <string>
#include <tuple>
#include <vector>
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/surface/mls.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
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
class PcdProcessUtils : public rclcpp::Node
{
public:
    PcdProcessUtils(const rclcpp::NodeOptions &options);
    PcdProcessUtils(
        const std::string &name_space,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
    );

private:
    // voxel grid downsampling
    template <typename T>
    void downsample_by_voxel(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const T leaf_size,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered
    ) {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.filter(*cloud_filtered);
    }

    // 法線推定 & 特徴量計算
    std::tuple<pcl::PointCloud<pcl::Normal>::Ptr, pcl::PointCloud<pcl::FPFHSignature33>::Ptr> preprocess_pcd(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const float voxel_size
    );

    // 統計的外れ値除去
    template <typename T>
    void remove_outlier(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const T mean_k,
        const T stddev_mul_thresh,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered
    ) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(mean_k);                       // 何点を見るか
        sor.setStddevMulThresh(stddev_mul_thresh);  // 何σ以上を外れ値とするか
        sor.filter(*cloud_filtered);
    }

    // 法線推定
    template <typename T>
    void estimate_normals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const T search_radius,
        pcl::PointCloud<pcl::Normal>::Ptr &normals
    ) {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(search_radius);
        ne.compute(*normals);
    }

    // 法線による平面検出
    template <typename T>
    void plane_detection_by_normals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals,
        const T distance_threshold,
        pcl::ModelCoefficients::Ptr &coefficients,
        pcl::PointIndices::Ptr &inliers
    ) {
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);              // 平面の係数を最適化
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);   // sacモデルの設定 (法線平面)
        seg.setNormalDistanceWeight(0.1);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(distance_threshold);
        seg.setInputCloud(cloud);
        seg.setInputNormals(normals);
        seg.segment(*inliers, *coefficients);
    }

    // 移動最小二乗法による点群平滑化
    template <typename T>
    void smooth_by_moving_least_squares(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const T search_radius,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_smoothed
    ) {
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        mls.setInputCloud(cloud);
        mls.setSearchRadius(search_radius);
        mls.process(*cloud_smoothed);
    }

    // DBSCAN クラスタリング
    template <typename T>
    void dbscan_clustering(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const T eps,
        const T minPts,
        std::vector<pcl::PointIndices> &cluster_indices
    ) {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;  // ユークリッド距離によるクラスタリング
        ec.setClusterTolerance(eps);                        // クラスタ間の最小距離
        ec.setMinClusterSize(minPts);                       // クラスタの最小点数
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);
    }

    // 法線によるクラスタリング
    template <typename T>
    void normal_clustering(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr normals,
        const int min_cluster_size,                       // クラスタの最小点数
        const int max_cluster_size,                       // クラスタの最大点数
        const int num_neighbours,                         // 探索する近傍点数(k)
        const T smoothness_threshold,                   // residual threshold
        const T curvature_threshold,                    // 曲率の閾値
        std::vector<pcl::PointIndices> &cluster_indices
    ) {
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(min_cluster_size);
        reg.setMaxClusterSize(max_cluster_size);
        reg.setNumberOfNeighbours(num_neighbours);
        reg.setSmoothnessThreshold(smoothness_threshold / 180.0 * M_PI);
        reg.setCurvatureThreshold(curvature_threshold);
        reg.setInputCloud(cloud);
        reg.setInputNormals(normals);
        reg.extract(cluster_indices);
    }

    // robot本体の傾きによって写り込んだ地面を除去
    // imu 情報は queueから取得
    void remove_groud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered
    ) {
        if (!imu_queue_.get()->size() || !is_imu_T_set_) return;
        auto imu = imu_queue_.get()->back();
        Eigen::Quaternionf q(
            imu.get()->orientation.w,
            imu.get()->orientation.x,
            imu.get()->orientation.y,
            imu.get()->orientation.z
        );
        Eigen::Isometry3f imu_T = Eigen::Isometry3f::Identity();
        imu_T.rotate(q);
        
        // lidar -> robot(base_link) -> angle_global(IMUによる角度補正)
        // angle_globalにおいて地面の点群はz座標が0以下になるはず

    }

    // PointCloud2 subscriber callback
    void cb_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    // IMU subscriber callback
    void cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
    std::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> pcd_queue_;
    std::shared_ptr<std::vector<sensor_msgs::msg::Imu::SharedPtr>> imu_queue_;
    std::string lidar_sub_topic_, lidar_pub_topic_;
    std::string imu_sub_topic_, imu_pub_topic_;
    std::string lidar_frame_id_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // imuから算出される"robotの"傾き
    double roll_, pitch_, yaw_;
    // imu_link->base_linkの変換行列 (TF)
    Eigen::Isometry3f imu_T_;
    tf2::Stamped<tf2::Transform> imu_T_tf_;
    bool is_imu_T_set_;
    // base_link->angle_global(IMUによる角度補正)の変換行列 
    Eigen::Isometry3f angle_global_T_;

    // 姿勢推定に使う直前の時刻
    std::shared_ptr<rclcpp::Time> last_time_;

    double voxel_size_;
    double mean_k_;
    double stddev_mul_thresh_;
    double search_radius_;
    double distance_threshold_;
    double leaf_size_;

    bool is_livox_imu_; // livoxのIMUを使う場合は加速度を補正(G -> m/s^2)する必要がある
    int pcd_queue_size_, imu_queue_size_;
};

} // namespace utils
