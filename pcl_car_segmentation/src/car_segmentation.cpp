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
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>

#include <pcl/features/normal_3d.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;

class CarSegmentation: public rclcpp ::Node{
    public:
        CarSegmentation():  Node("car_segmentation"){
            filtered_cloud_subscriber = this -> create_subscription<sensor_msgs::msg::PointCloud2>(
                "/filtered_point_cloud", 10, std::bind(&CarSegmentation::point_cloud_callback, this, std::placeholders::_1)
            );

            car_segmentation_publisher = this -> create_publisher<sensor_msgs::msg::PointCloud2>("/car_segmentation", 10);

            bounding_box_publisher = this -> create_publisher<visualization_msgs::msg::MarkerArray>("/bounding_boxes", 10);
        }

    private:

    void save_cluster(pcl::PointCloud<PointT> :: Ptr cluster, std::string file_name){
        std::string folder_path = "/home/cdonoso/ros2_ws/src/ros2_pcl_segmentation/pcl_car_segmentation/car_dataset_clouds/";
        file_name = folder_path + file_name;
        pcl::io::savePCDFileASCII(file_name, *cluster);
        std::cerr << "Saved " << cluster -> points.size() << " data points to " << file_name << std::endl;
    }

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud){

        pcl::PointCloud<PointT> :: Ptr pcl_cloud (new pcl:: PointCloud<PointT>) ;
        pcl::fromROSMsg(*input_cloud, *pcl_cloud);

        pcl::PassThrough<PointT> passing_y;
        passing_y.setInputCloud(pcl_cloud);
        passing_y.setFilterFieldName("y");
        passing_y.setFilterLimits(-5.0, 8.0);
        passing_y.filter(*pcl_cloud);

        pcl::PointCloud<PointT> :: Ptr single_segmented_cluster (new pcl:: PointCloud<PointT>) ;
        pcl::PointCloud<PointT> :: Ptr all_clusters (new pcl:: PointCloud<PointT>) ;
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ecludian_cluster_extractor;
        pcl::search::KdTree<PointT>::Ptr tree (new pcl:: search ::KdTree<PointT>());

        tree -> setInputCloud(pcl_cloud);

        int min_cluster_size = 60;
        int max_cluster_size = 180;

        ecludian_cluster_extractor.setClusterTolerance(0.5);
        ecludian_cluster_extractor.setMinClusterSize(min_cluster_size);
        ecludian_cluster_extractor.setMaxClusterSize(max_cluster_size);
        ecludian_cluster_extractor.setSearchMethod(tree);
        ecludian_cluster_extractor.setInputCloud(pcl_cloud);
        ecludian_cluster_extractor.extract(cluster_indices);

        size_t min_cloud_threshold = min_cluster_size;
        size_t max_cloud_threshold = max_cluster_size;

        struct bounding_box{
            float x_min;
            float x_max;
            float y_min;
            float y_max;
            float z_min;
            float z_max;
            double r=1.0;
            double g=0.0;
            double b=0.0;
        };

        std::vector<bounding_box> bounding_boxes;

        for (size_t i = 0; i < cluster_indices.size(); i++){
            if (cluster_indices[i].indices.size() > min_cloud_threshold && cluster_indices[i].indices.size() < max_cloud_threshold){
                pcl::PointCloud<PointT> :: Ptr reasonable_cluster (new pcl:: PointCloud<PointT>) ;
                pcl::ExtractIndices<PointT> extract_indices;
                pcl::IndicesPtr indices (new std::vector<int>(cluster_indices[i].indices.begin(), cluster_indices[i].indices.end()));

                extract_indices.setInputCloud(pcl_cloud);
                extract_indices.setIndices(indices);
                extract_indices.setNegative(false);
                extract_indices.filter(*reasonable_cluster);

                std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

                Eigen::Vector4f min_pt, max_pt;
                pcl::getMinMax3D<PointT>(*reasonable_cluster, min_pt, max_pt);

                pcl::PointXYZ center((min_pt[0] + max_pt[0]) / 2, (min_pt[1] + max_pt[1]) / 2, (min_pt[2] + max_pt[2]) / 2);
                bounding_box box;
                box.x_min = min_pt[0];
                box.x_max = max_pt[0];
                box.y_min = min_pt[1];
                box.y_max = max_pt[1];
                box.z_min = min_pt[2];
                box.z_max = max_pt[2];
                bounding_boxes.push_back(box);

                //save_cluster(reasonable_cluster, "car_cluster_" + std::to_string(now.time_since_epoch().count()) + ".pcd");

                *all_clusters += *reasonable_cluster;

            }
        }

        visualization_msgs::msg::MarkerArray markers_array;
        int id = 0;
        const std_msgs::msg::Header header = input_cloud -> header;
        for (size_t i = 0; i < bounding_boxes.size(); i++){
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "bounding_boxes";
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = (bounding_boxes[i].x_min + bounding_boxes[i].x_max) / 2;
            marker.pose.position.y = (bounding_boxes[i].y_min + bounding_boxes[i].y_max) / 2;
            marker.pose.position.z = (bounding_boxes[i].z_min + bounding_boxes[i].z_max) / 2;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = bounding_boxes[i].x_max - bounding_boxes[i].x_min;
            marker.scale.y = bounding_boxes[i].y_max - bounding_boxes[i].y_min;
            marker.scale.z = bounding_boxes[i].z_max - bounding_boxes[i].z_min;
            marker.color.r = bounding_boxes[i].r;
            marker.color.g = bounding_boxes[i].g;
            marker.color.b = bounding_boxes[i].b;
            marker.color.a = 0.5;
            marker.lifetime.sec = 2.0;
            markers_array.markers.push_back(marker);
            id++;
        }

        sensor_msgs::msg::PointCloud2::SharedPtr car_segmentation_msg (new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*all_clusters, *car_segmentation_msg);
        car_segmentation_msg -> header = input_cloud -> header;
        car_segmentation_publisher -> publish(*car_segmentation_msg);
        bounding_box_publisher -> publish(markers_array);


    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr car_segmentation_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bounding_box_publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>:: SharedPtr filtered_cloud_subscriber;
};

int main(int argc, char **argv){
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CarSegmentation>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}