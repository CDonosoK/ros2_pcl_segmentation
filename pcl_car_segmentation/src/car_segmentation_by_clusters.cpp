#include <chrono>
#include <memory>
#include <string>
#include <sys/types.h>
#include <pwd.h>

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

#include <Eigen/Dense> 
#include <vector>

using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;
int car_id = 0;
struct passwd *pw = getpwuid(getuid());
const char *home_dir = pw -> pw_dir;


class CarSegmentation: public rclcpp ::Node{
    public:
        std::string folder_path = std::string(home_dir) + "/ros2_ws/src/ros2_pcl_segmentation/pcl_car_segmentation/car_dataset_clouds/";

        CarSegmentation():  Node("car_segmentation"){

            if (!std::filesystem::exists(folder_path)) {
                std::filesystem::create_directories(folder_path);
                std::cerr << "Directory created: " << folder_path << std::endl;
            }

            filtered_cloud_subscriber = this -> create_subscription<sensor_msgs::msg::PointCloud2>(
                "/pcl_car_segmentation/filtered_cloud", 10, std::bind(&CarSegmentation::point_cloud_callback, this, std::placeholders::_1)
            );

            car_segmentation_publisher = this -> create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_car_segmentation/car_segmentation", 10);
        }

    private:

    void save_cluster(pcl::PointCloud<PointT> :: Ptr cluster, std::string file_name){
        file_name = folder_path + file_name;
        pcl::io::savePCDFileASCII(file_name, *cluster);
        std::cerr << "Saved " << cluster -> points.size() << " data points to " << file_name << std::endl;
    }

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud){

        pcl::PointCloud<PointT> :: Ptr pcl_cloud (new pcl:: PointCloud<PointT>) ;
        pcl::fromROSMsg(*input_cloud, *pcl_cloud);

        pcl::PointCloud<PointT> :: Ptr single_segmented_cluster (new pcl:: PointCloud<PointT>) ;
        pcl::PointCloud<PointT> :: Ptr all_clusters (new pcl:: PointCloud<PointT>) ;
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ecludian_cluster_extractor;
        pcl::search::KdTree<PointT>::Ptr tree (new pcl:: search ::KdTree<PointT>());

        // Apply Voxel Grid Filter
        pcl::VoxelGrid<PointT> voxel_grid_filter;
        voxel_grid_filter.setInputCloud(pcl_cloud);
        voxel_grid_filter.setLeafSize(0.2f, 0.2f, 0.2f);
        voxel_grid_filter.filter(*pcl_cloud);

        tree -> setInputCloud(pcl_cloud);

        int min_cluster_size = 100;
        int max_cluster_size = 700;

        ecludian_cluster_extractor.setClusterTolerance(0.6);
        ecludian_cluster_extractor.setMinClusterSize(min_cluster_size);
        ecludian_cluster_extractor.setMaxClusterSize(max_cluster_size);
        ecludian_cluster_extractor.setSearchMethod(tree);
        ecludian_cluster_extractor.setInputCloud(pcl_cloud);
        ecludian_cluster_extractor.extract(cluster_indices);

        size_t min_cloud_threshold = min_cluster_size;
        size_t max_cloud_threshold = max_cluster_size;

        float min_length = 1.0;
        float max_length = 5.0;
        float min_width = 1.0;
        float max_width = 5.0;
        float min_height = 1.0;
        float max_height = 2.0;

        for (size_t i = 0; i < cluster_indices.size(); i++){
            if (cluster_indices[i].indices.size() > min_cloud_threshold + 20 && cluster_indices[i].indices.size() < max_cloud_threshold - 20){
                pcl::PointCloud<PointT> :: Ptr reasonable_cluster (new pcl:: PointCloud<PointT>) ;
                pcl::ExtractIndices<PointT> extract_indices;
                pcl::IndicesPtr indices (new std::vector<int>(cluster_indices[i].indices.begin(), cluster_indices[i].indices.end()));
                

                extract_indices.setInputCloud(pcl_cloud);
                extract_indices.setIndices(indices);
                extract_indices.setNegative(false);
                extract_indices.filter(*reasonable_cluster);

                Eigen::Vector4f min_pt, max_pt;
                pcl::getMinMax3D<PointT>(*reasonable_cluster, min_pt, max_pt);

                float length = max_pt[0] - min_pt[0];
                float width = max_pt[1] - min_pt[1];
                float height = max_pt[2] - min_pt[2];

                if (length >= min_length && length <= max_length &&
                    width >= min_width && width <= max_width &&
                    height >= min_height && height <= max_height) {
                        std::string file_name = "car_cluster_" + std::to_string(cluster_indices[i].indices.size()) + "_" + std::to_string(length) + "_" + std::to_string(width) + "_" + std::to_string(height) + ".pcd";
                        //save_cluster(reasonable_cluster, file_name);
                        *all_clusters += *reasonable_cluster;
                    }

            }
        }

        sensor_msgs::msg::PointCloud2::SharedPtr car_segmentation_msg (new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*all_clusters, *car_segmentation_msg);
        car_segmentation_msg -> header = input_cloud -> header;
        car_segmentation_publisher -> publish(*car_segmentation_msg);


    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr car_segmentation_publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>:: SharedPtr filtered_cloud_subscriber;
};

int main(int argc, char **argv){
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CarSegmentation>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}