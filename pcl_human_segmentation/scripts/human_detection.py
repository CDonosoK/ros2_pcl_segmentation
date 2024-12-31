#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import warnings
warnings.filterwarnings("ignore", category=FutureWarning)


class HumanSegmentationNode(Node):
    def __init__(self):
        super().__init__('human_segmentation_node')

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f"Using device: {self.device}")

        self.image_sub = self.create_subscription(
            Image, '/camera/raw_image', self.image_callback, 10
        )

        self.segmented_image_pub = self.create_publisher(
            Image, '/camera/segmented_image', 10
        )

        self.bridge = CvBridge()

        self.get_logger().info("Loading YOLOv5 model from local file...")
        model_path = '/ros2_ws/src/ros2_pcl_segmentation/pcl_human_segmentation/model/yolov5s.pt'
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
        self.get_logger().info("Model loaded successfully.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            results = self.model(cv_image)
            detections = results.pandas().xyxy[0]

            human_detections = detections[detections['class'] == 0]

            for _, row in human_detections.iterrows():
                xmin, ymin, xmax, ymax, confidence = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax']), row['confidence']
                cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv2.putText(cv_image, f"Human: {confidence:.2f}", (xmin, ymin - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            segmented_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            segmented_image_msg.header.stamp = self.get_clock().now().to_msg()

            self.segmented_image_pub.publish(segmented_image_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HumanSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
