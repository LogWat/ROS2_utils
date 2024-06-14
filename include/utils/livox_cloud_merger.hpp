#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>
#include <string>

namespace utils {

class LivoxCloudMerger : public rclcpp::Node {
public:
    LivoxCloudMerger(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    LivoxCloudMerger(
        const std::string& name_space,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
    );

private:
    bool checkCompatibility(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl1,
                            const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl2) {
        return (pcl1->is_bigendian == pcl2->is_bigendian) &&
               (pcl1->point_step == pcl2->point_step) &&
               (pcl1->fields.size() == pcl2->fields.size()) &&
               (pcl1->is_dense == pcl2->is_dense);
    }

    void preparePointCloud2Header(sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_out,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl1,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl2) {
        pointcloud_out->header = pcl1->header;
        pointcloud_out->header.stamp = (pcl1->header.stamp.sec > pcl2->header.stamp.sec) ? pcl1->header.stamp : pcl2->header.stamp;
        pointcloud_out->header.frame_id = frame_id_;
        pointcloud_out->is_bigendian = pcl1->is_bigendian;
        pointcloud_out->point_step = pcl1->point_step + sizeof(uint16_t);
        pointcloud_out->fields = pcl1->fields;
        pointcloud_out->is_dense = pcl1->is_dense;
        pointcloud_out->fields.emplace_back();
        auto& ring_field = pointcloud_out->fields.back();
        ring_field.name = "ring";
        ring_field.offset = pcl1->point_step;
        ring_field.datatype = sensor_msgs::msg::PointField::UINT16;
        ring_field.count = 1;
    }

    void addRingField(sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_out,
                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl1) {
        pointcloud_out->height = pcl1->height;
        pointcloud_out->width = pcl1->width * 2;
        pointcloud_out->row_step = pointcloud_out->point_step * pointcloud_out->width;
        pointcloud_out->data.resize(pointcloud_out->row_step * pointcloud_out->height);
    }

    void concatenatePointClouds(sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_out,
                                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl1,
                                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl2) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl2_pcl(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl2_transformed_pcl(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*pcl2, *pcl2_pcl);
        pcl::transformPointCloud(*pcl2_pcl, *pcl2_transformed_pcl, lidar_diff_transform_);
        
        int offset_ring = pcl1->point_step;

        for (size_t i = 0; i < pcl1->data.size(); i += pcl1->point_step) {
            memcpy(&pointcloud_out->data[i], &pcl1->data[i], pcl1->point_step);
            *(uint16_t*)(&pointcloud_out->data[i + offset_ring]) = 0;
        }

        for (size_t i = 0; i < pcl2_transformed_pcl->points.size(); ++i) {
            size_t offset = pcl1->data.size() + i * pointcloud_out->point_step;
            memcpy(&pointcloud_out->data[offset], &pcl2_transformed_pcl->points[i], pcl1->point_step);
            *(uint16_t*)(&pointcloud_out->data[offset + offset_ring]) = 1;
        }
    }

    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl1,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl2) {
        auto pointcloud_out = std::make_shared<sensor_msgs::msg::PointCloud2>();

        if (!checkCompatibility(pcl1, pcl2)) {
            RCLCPP_WARN(this->get_logger(), "PointCloud2 messages are not compatible.");
            return;
        }

        preparePointCloud2Header(pointcloud_out, pcl1, pcl2);
        addRingField(pointcloud_out, pcl1);
        concatenatePointClouds(pointcloud_out, pcl1, pcl2);

        pointcloud_pub_->publish(*pointcloud_out);
    }

    // variables
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud1_sub_, pointcloud2_sub_;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    std::shared_ptr<Sync> sync_;

    std::string pointcloud1_topic_, pointcloud2_topic_, output_topic_, frame_id_;
    unsigned int ring_;
    double lidar_diffx_, lidar_diffy_, lidar_diffz_, lidar_diffroll_, lidar_diffpitch_, lidar_diffyaw_;
    Eigen::Affine3f lidar_diff_transform_;


};

} // namespace utils
