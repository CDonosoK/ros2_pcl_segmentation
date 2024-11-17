from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    kitti_publisher = Node(
        package='ros2_kitti_publishers',
        executable='kitti_publishers',
        name='kitti_publishers',
        output='screen'
    )

    pre_processing = Node(
        package='pcl_car_segmentation',
        executable='cloud_pre_processing',
        name='pre_processing',
        output='screen'
    )

    car_segmentation_by_clusters = Node(
        package='pcl_car_segmentation',
        executable='car_segmentation_by_clusters',
        name='pre_processing',
        output='screen'
    )

    return LaunchDescription([kitti_publisher, pre_processing, car_segmentation_by_clusters])