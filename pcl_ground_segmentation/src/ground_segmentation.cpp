#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/features/normal_3d.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;

class RoadSegmentation: public rclcpp ::Node{
    public:
        RoadSegmentation():  Node("road_segmentatiom"){
            subscriber_ = this -> create_subscription<sensor_msgs::msg::PointCloud2>(
                "/kitti/point_cloud", 10, std::bind(&RoadSegmentation::point_cloud_callback, this, std::placeholders::_1)
            );

            publisher_ = this -> create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation", 10);
        }

    private:

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud){

        pcl::PointCloud<PointT> :: Ptr pcl_cloud (new pcl:: PointCloud<PointT>) ;
        pcl::fromROSMsg(*input_cloud, *pcl_cloud);

        pcl::PointCloud<PointT> :: Ptr voxel_cloud (new pcl:: PointCloud<PointT>) ;
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(pcl_cloud);
        voxel_filter.setLeafSize(0.4, 0.4, 0.4);
        voxel_filter.filter(*voxel_cloud);

        //Road Segmentation
        pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
        pcl::search::KdTree<PointT>::Ptr tree (new pcl:: search ::KdTree<PointT>());
        pcl::PointCloud<pcl::Normal>::Ptr road_normals (new pcl::PointCloud<pcl::Normal>);

        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> road_segmentator_from_normals;
        pcl::PointIndices::Ptr road_inliers (new pcl::PointIndices);
        pcl::SACSegmentation<PointT> plane_segmentor;
        pcl::ExtractIndices<PointT> road_extract_indices;
        pcl:: ModelCoefficients :: Ptr road_coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<PointT> :: Ptr road_cloud (new pcl:: PointCloud<PointT>) ;

        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(voxel_cloud);
        normal_estimator.setKSearch(30);
        normal_estimator.compute(*road_normals);

        road_segmentator_from_normals.setOptimizeCoefficients(true);
        road_segmentator_from_normals.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        road_segmentator_from_normals.setMethodType(pcl::SAC_RANSAC);
        road_segmentator_from_normals.setDistanceThreshold(0.4);
        road_segmentator_from_normals.setNormalDistanceWeight(0.5);
        road_segmentator_from_normals.setMaxIterations(100);
        road_segmentator_from_normals.setInputCloud(voxel_cloud);
        road_segmentator_from_normals.setInputNormals(road_normals); 
        road_segmentator_from_normals.segment(*road_inliers, *road_coefficients);

        road_extract_indices.setInputCloud(voxel_cloud);
        road_extract_indices.setIndices(road_inliers);
        road_extract_indices.setNegative(false);
        road_extract_indices.filter(*road_cloud);

        sensor_msgs::msg::PointCloud2 road_cloud_ros2;
        pcl::toROSMsg(*road_cloud, road_cloud_ros2);
        road_cloud_ros2.header.frame_id = "base_link";
        road_cloud_ros2.header.stamp = this -> now();

        publisher_ -> publish(road_cloud_ros2);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>:: SharedPtr subscriber_;

};


int main(int argc, char **argv){
    
    rclcpp::init(argc, argv); // Se inicializa el nodo

    auto node = std::make_shared<RoadSegmentation>();

    rclcpp::spin(node); // Se hace un spin para mantener vivo el nodo
    
    rclcpp::shutdown(); // Se terminal el nodo

    return 0;
}