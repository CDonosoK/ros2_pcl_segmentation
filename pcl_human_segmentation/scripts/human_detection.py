#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from pcl_human_segmentation.msg import HumanPose
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO
import numpy as np
import logging

class HumanSegmentationNode(Node):
    def __init__(self):
        super().__init__('human_segmentation_node')
        logging.getLogger("ultralytics").setLevel(logging.ERROR)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().warn(f"Using device: {self.device}")
        self.image_sub = self.create_subscription(Image, '/pcl_human_segmentation/camera/raw_image', self.image_callback, 10)
        self.segmented_image_publisher = self.create_publisher(Image, '/pcl_human_segmentation/camera/segmented_image', 10)
        self.human_pose_publisher = self.create_publisher(HumanPose, '/pcl_human_segmentation/camera/human_pose', 10)

        self.x_position =[]
        self.y_position =[]

        self.bridge = CvBridge()
        self.confidence_threshold = 0.6
        self.classes_filter = [0]
        self.alpha = 0.2
        model_path = '/ros2_ws/src/ros2_pcl_segmentation/pcl_human_segmentation/model/yolo11s-seg.pt'
        self.model = YOLO(model_path)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            input_image = cv2.resize(cv_image, (320, 320)) if cv_image.shape[:2] > (320, 320) else cv_image
            with torch.no_grad():
                results = self.model(input_image)

            # Obtener bounding boxes, clases y confidencias
            boxes = results[0].boxes.xyxy.cpu().numpy()  # Bounding boxes [x1, y1, x2, y2]
            classes = results[0].boxes.cls.cpu().numpy()  # Clases
            confidences = results[0].boxes.conf.cpu().numpy()  # Confidencias

            # Filtrar por clase y confianza
            x_positions = []
            y_positions = []
            for box, cls, conf in zip(boxes, classes, confidences):
                if cls == self.classes_filter[0] and conf >= self.confidence_threshold:
                    x1, y1, x2, y2 = box
                    center_x = (x1 + x2) / 2.0
                    center_y = (y1 + y2) / 2.0
                    x_positions.append(center_x)
                    y_positions.append(center_y)

            # Publicar las posiciones en el mensaje HumanPose
            human_pose_msg = HumanPose()
            human_pose_msg.x_position = x_positions
            human_pose_msg.y_position = y_positions
            self.human_pose_publisher.publish(human_pose_msg)

            # Procesar máscaras para la imagen segmentada
            masks = results[0].masks.data.cpu().numpy()
            resized_masks = np.array([
                cv2.resize(mask, (cv_image.shape[1], cv_image.shape[0]), interpolation=cv2.INTER_NEAREST)
                for mask in masks
            ])
            binary_masks = np.sum([
                resized_mask.astype(np.uint8) * ((cls == self.classes_filter[0]) and (conf >= self.confidence_threshold))
                for resized_mask, cls, conf in zip(resized_masks, classes, confidences)
            ], axis=0).astype(np.uint8)
            color_mask = np.zeros_like(cv_image)
            color_mask[:, :, 1] = 255
            cv_image[binary_masks == 1] = (
                self.alpha * color_mask[binary_masks == 1] +
                (1 - self.alpha) * cv_image[binary_masks == 1]
            ).astype('uint8')

            segmented_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            segmented_image_msg.header.stamp = self.get_clock().now().to_msg()
            self.segmented_image_publisher.publish(segmented_image_msg)

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
