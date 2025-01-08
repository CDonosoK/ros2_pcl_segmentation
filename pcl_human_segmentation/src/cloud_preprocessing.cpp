#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/passthrough.h"
#include "pcl/kdtree/kdtree_flann.h"

typedef pcl::PointXYZ PointT;

class FilteringCloud : public rclcpp::Node {
public:
    FilteringCloud() : Node("ground_segmentation") {
        point_cloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "rslidar_points", 
            10, 
            std::bind(&FilteringCloud::point_cloud_cb, this, std::placeholders::_1)
        );

        filtered_segmented_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/pcl_human_segmentation/filtered_cloud", 
            10
        );

    }

private:
    void point_cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());

        pcl::fromROSMsg(*msg, *cloud);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        double min_angle = -0.5;
        double max_angle = 0.5;

        for (const auto& point : cloud->points) {
            double angle = atan2(point.y, point.x);
            if (angle >= min_angle && angle <= max_angle) {
                cloud_filtered->points.push_back(point);
            }
        }
        cloud_filtered->width = cloud_filtered->points.size();
        cloud_filtered->height = 1;
        cloud_filtered->is_dense = true;

        sensor_msgs::msg::PointCloud2 cloud_filtered_msg;
        pcl::toROSMsg(*cloud_filtered, cloud_filtered_msg);
        cloud_filtered_msg.header = msg->header;
        filtered_segmented_publisher->publish(cloud_filtered_msg);
    }


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_segmented_publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber;

    pcl::PointCloud<PointT>::Ptr previous_cloud{nullptr};
    double movement_threshold;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilteringCloud>());
    rclcpp::shutdown();
    return 0;
}
