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

    void prepare_header_and_fields(sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_out,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl1,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl2) {
        pointcloud_out->header = pcl1->header;
        pointcloud_out->header.stamp = (pcl1->header.stamp.sec > pcl2->header.stamp.sec) ? pcl1->header.stamp : pcl2->header.stamp;
        pointcloud_out->header.frame_id = frame_id_; // 新しいframe_idに変更
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
        for (auto& field : pointcloud_out->fields) {
            if (field.name == "timestamp") {
                field.name = "time";
                break;
            }
        }
    }

    void resize_ponintcloud_data(sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_out,
                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl1) {
        pointcloud_out->height = pcl1->height;
        pointcloud_out->width = pcl1->width * 2;
        pointcloud_out->row_step = pointcloud_out->point_step * pointcloud_out->width;
        pointcloud_out->data.resize(pointcloud_out->row_step * pointcloud_out->height);
    }

    void concatenate_pointclouds(sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_out,
                                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl1,
                                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl2) {
        // 一旦全部pointcloud_outにコピー
        for (size_t i = 0; i < pcl1->data.size(); i += pcl1->point_step) {
            for (auto &field : pcl1->fields) {
                memcpy(&pointcloud_out->data[i + field.offset], &pcl1->data[i + field.offset], field.datatype * field.count / 8);
            }
        }
        for (size_t i = 0; i < pcl2->data.size(); i += pcl2->point_step) {
            for (auto &field : pcl2->fields) {
                memcpy(&pointcloud_out->data[i + pcl1->point_step + field.offset], &pcl2->data[i + field.offset], field.datatype * field.count / 8);
            }
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl1_pcl(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl2_pcl(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl1_transformed_pcl(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl2_transformed_pcl(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*pcl1, *pcl1_pcl);
        pcl::fromROSMsg(*pcl2, *pcl2_pcl);
        pcl::transformPointCloud(*pcl1_pcl, *pcl1_transformed_pcl, lidar_diff_transform_1_);
        pcl::transformPointCloud(*pcl2_pcl, *pcl2_transformed_pcl, lidar_diff_transform_2_);
        // point stepが変わってるのでx, y, z, intensityのみコピー
        size_t out_x_idx, out_y_idx, out_z_idx, out_intensity_idx;
        for (size_t i = 0; i < pointcloud_out->fields.size(); ++i) {
            if (pointcloud_out->fields[i].name == "x") {
                out_x_idx = i;
            } else if (pointcloud_out->fields[i].name == "y") {
                out_y_idx = i;
            } else if (pointcloud_out->fields[i].name == "z") {
                out_z_idx = i;
            } else if (pointcloud_out->fields[i].name == "intensity") {
                out_intensity_idx = i;
            }
        }
        for (size_t i = 0; i < pointcloud_out->data.size(); i += pointcloud_out->point_step) {
            if (i < pcl1->data.size()) {
                *(float *)(&pointcloud_out->data[i + out_x_idx]) = *(float *)(&pcl1_transformed_pcl->points[i / pcl1->point_step].x);
                *(float *)(&pointcloud_out->data[i + out_y_idx]) = *(float *)(&pcl1_transformed_pcl->points[i / pcl1->point_step].y);
                *(float *)(&pointcloud_out->data[i + out_z_idx]) = *(float *)(&pcl1_transformed_pcl->points[i / pcl1->point_step].z);
                *(float *)(&pointcloud_out->data[i + out_intensity_idx]) = *(float *)(&pcl1_pcl->points[i / pcl1->point_step].intensity);
            } else {
                *(float *)(&pointcloud_out->data[i + out_x_idx]) = *(float *)(&pcl2_transformed_pcl->points[(i - pcl1->data.size()) / pcl2->point_step].x);
                *(float *)(&pointcloud_out->data[i + out_y_idx]) = *(float *)(&pcl2_transformed_pcl->points[(i - pcl1->data.size()) / pcl2->point_step].y);
                *(float *)(&pointcloud_out->data[i + out_z_idx]) = *(float *)(&pcl2_transformed_pcl->points[(i - pcl1->data.size()) / pcl2->point_step].z);
                *(float *)(&pointcloud_out->data[i + out_intensity_idx]) = *(float *)(&pcl2_pcl->points[(i - pcl1->data.size()) / pcl2->point_step].intensity);
            }
        }
    }

    void restore_ring_field(sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_out,
                          const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl1) {
        size_t x_idx, y_idx, z_idx;
        for (size_t i = 0; i < pcl1->fields.size(); ++i) {
            if (pcl1->fields[i].name == "x") {
                x_idx = i;
            } else if (pcl1->fields[i].name == "y") {
                y_idx = i;
            } else if (pcl1->fields[i].name == "z") {
                z_idx = i;
            }
        }
        // ring field 復元
        // 2つのlivoxが同じx, y座標にあると仮定
        // 中心のz座標から見た各点の角度からringを決定
        // 2つのlivoxの垂直視野角は合わせて (52+7) + (52+7) = 118 ≒ 120度
        double deg2rad = M_PI / 180.0;
        double fov_top = 52.0 * deg2rad;
        double fov_bottom = -52.0 * deg2rad;
        double vertical_resolution = (fov_top - fov_bottom) / (ring_num_ - 1);
        double z_center = lidar_diffz_ / 2.0;
        for (size_t i = 0; i < pointcloud_out->data.size(); i += pointcloud_out->point_step) {
            double x = *(float *)(&pointcloud_out->data[i + x_idx]);
            double y = *(float *)(&pointcloud_out->data[i + y_idx]);
            double z = *(float *)(&pointcloud_out->data[i + z_idx]);
            double angle = atan2(z - z_center, sqrt(x * x + y * y));
            int ring = (angle - fov_bottom) / vertical_resolution;
            *(uint16_t *)(&pointcloud_out->data[i + pointcloud_out->fields.back().offset]) = ring;
        }
    }

    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl1,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl2) {
        auto pointcloud_out = std::make_shared<sensor_msgs::msg::PointCloud2>();

        if (!checkCompatibility(pcl1, pcl2)) {
            RCLCPP_WARN(this->get_logger(), "PointCloud2 messages are not compatible.");
            return;
        }

        // headerのcheckとfieldsの編集(add ring, rename timestamp to time)
        prepare_header_and_fields(pointcloud_out, pcl1, pcl2);
        // dataのresize
        resize_ponintcloud_data(pointcloud_out, pcl1);
        // dataのconcatenate
        concatenate_pointclouds(pointcloud_out, pcl1, pcl2);
        // ring fieldの復元
        restore_ring_field(pointcloud_out, pcl1);

        pointcloud_pub_->publish(*pointcloud_out);
    }

    // variables
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud1_sub_, pointcloud2_sub_;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    std::shared_ptr<Sync> sync_;

    std::string pointcloud1_topic_, pointcloud2_topic_, output_topic_, frame_id_;
    int ring_num_;
    double lidar_diffz_; // 2つのLiDARのz座標の差
    Eigen::Affine3f lidar_diff_transform_1_, lidar_diff_transform_2_;


};

} // namespace utils
