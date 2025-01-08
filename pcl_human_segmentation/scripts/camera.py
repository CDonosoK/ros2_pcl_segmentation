#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import yaml
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.image_pub = self.create_publisher(Image, '/pcl_human_segmentation/camera/raw_image', 10)
        self.bridge = CvBridge()

        calibration_file = '/ros2_ws/src/ros2_pcl_segmentation/pcl_human_segmentation/config/camera_calibration.yaml'
        self.camera_matrix, self.dist_coeffs = self.load_calibration(calibration_file)

        try:
            self.camera = cv2.VideoCapture(2)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info('Camera node started successfully')
        except Exception as e:
            self.get_logger().error(f"Failed to connect to camera: {e}")
            rclpy.shutdown()

        self.rate = 30
        self.timer = self.create_timer(1.0 / self.rate, self.camera_callback)

    def load_calibration(self, calibration_file):
        """Carga los parámetros de calibración desde un archivo YAML."""
        try:
            with open(calibration_file, 'r') as file:
                calibration_data = yaml.safe_load(file)

            camera_matrix = np.array(calibration_data['camera_matrix']['data'])
            dist_coeffs = np.array(calibration_data['distortion_coefficients']['data'])

            self.get_logger().info("Camera calibration loaded successfully")
            return camera_matrix, dist_coeffs
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration file: {e}")
            rclpy.shutdown()

    def camera_callback(self):
        """Captura imágenes, aplica la calibración y publica la imagen corregida."""
        ret, frame = self.camera.read()
        if ret:
            if self.camera_matrix is not None and self.dist_coeffs is not None:
                frame = self.undistort_image(frame)

            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_link'
            self.image_pub.publish(image_msg)
        else:
            self.get_logger().error('Failed to read frame from camera')

    def undistort_image(self, frame):
        """Aplica la corrección de distorsión a una imagen."""
        h, w = frame.shape[:2]
        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
        )
        undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)
        return undistorted_frame


def main(args=None):
    rclpy.init(args=args)
    camera = CameraNode()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
