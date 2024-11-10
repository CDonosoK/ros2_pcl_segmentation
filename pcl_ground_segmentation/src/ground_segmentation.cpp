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

class GroundSegmentation: public rclcpp ::Node{
    public:
        GroundSegmentation():  Node("ground_segmentatiom"){
            point_cloud_subscriber = this -> create_subscription<sensor_msgs::msg::PointCloud2>(
                "/kitti/point_cloud", 10, std::bind(&GroundSegmentation::point_cloud_callback, this, std::placeholders::_1)
            );

            ground_segmentation_publisher = this -> create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation", 10);
            filtered_segmented_publisher = this -> create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_point_cloud", 10);

        }

    private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // Configuración del segmentador
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.2);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

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

int main(int argc, char **argv){
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GroundSegmentation>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}