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

    ground_segmentation = Node(
        package='pcl_ground_segmentation',
        executable='ground_segmentation',
        name='ground_segmentation',
        output='screen'
    )

    car_segmentation = Node(
        package='pcl_car_segmentation',
        executable='car_segmentation',
        name='car_segmentation',
        output='screen'
    )

    return LaunchDescription([kitti_publisher, car_segmentation, ground_segmentation])