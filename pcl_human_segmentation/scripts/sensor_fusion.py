#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        self.image_sub = self.create_subscription(Image, '/camera/segmented_image', self.image_callback, 10)
        self.pointcloud_sub = self.create_subscription(PointCloud2, '/rslidar_points', self.pointcloud_callback, 10)

    def synchronize_data(self):
        pass

    def image_callback(self, msg):
        pass

    def pointcloud_callback(self, msg):
        pass