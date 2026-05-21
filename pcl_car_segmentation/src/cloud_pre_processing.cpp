#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/features/normal_3d.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;

class GroundSegmentation : public rclcpp::Node {
public:
    GroundSegmentation() : Node("ground_segmentation") {
        voxel_leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.1);
        ransac_distance_threshold_ = this->declare_parameter<double>("ransac_distance_threshold", 0.3);

        input_topic_ = this->declare_parameter<std::string>("input_topic", "/kitti/point_cloud");
        ground_topic_ = this->declare_parameter<std::string>("ground_topic", "/pcl_car_segmentation/ground_cloud");
        filtered_topic_ = this->declare_parameter<std::string>("filtered_topic", "/pcl_car_segmentation/filtered_cloud");

        point_cloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_,
            10,
            std::bind(&GroundSegmentation::point_cloud_cb, this, std::placeholders::_1)
        );

        ground_segmentation_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(ground_topic_, 10);
        filtered_segmented_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(filtered_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "Cloud preprocessing node started");
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Filtered topic: %s", filtered_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Ground topic: %s", ground_topic_.c_str());
    }

private:
    double voxel_leaf_size_;
    double ransac_distance_threshold_;
    std::string input_topic_;
    std::string ground_topic_;
    std::string filtered_topic_;

    void point_cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // Downsample
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_grid.filter(*cloud);

        // Filtering
        pcl::SACSegmentation<pcl::PointXYZ> segmentator;
        segmentator.setOptimizeCoefficients(true);
        segmentator.setModelType(pcl::SACMODEL_PLANE);
        segmentator.setMethodType(pcl::SAC_RANSAC);
        segmentator.setDistanceThreshold(ransac_distance_threshold_);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        segmentator.setInputCloud(cloud);
        segmentator.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "No objects detected!");
            return;
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);


        pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedCloud(new pcl::PointCloud<pcl::PointXYZ>());
        extract.filter(*segmentedCloud);
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*segmentedCloud, output);
        output.header = msg->header;
        filtered_segmented_publisher->publish(output);

        // Ground segmentation
        extract.setNegative(false);
        pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>());
        extract.filter(*groundCloud);
        pcl::toROSMsg(*groundCloud, output);
        output.header = msg->header;
        ground_segmentation_publisher->publish(output);

    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_segmentation_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_segmented_publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundSegmentation>());
    rclcpp::shutdown();
    return 0;
}
