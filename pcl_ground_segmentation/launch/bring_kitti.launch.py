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

    voxel_grid = Node(
        package='pcl_ground_segmentation',
        executable='pc_road_segmentation',
        name='voxel_grid',
        output='screen'
    )

    return LaunchDescription([kitti_publisher, voxel_grid])