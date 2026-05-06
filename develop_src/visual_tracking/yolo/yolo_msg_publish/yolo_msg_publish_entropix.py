#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage
from yolo_ros_msgs.msg import BoundingBox, BoundingBoxes

class YoloRosSubscriberNode(Node):
    def __init__(self):
        super().__init__('yolo_ros_subscriber')

        self.declare_parameter('weight_path', '/home/renwang/data_storage/PX4-Autopilot/develop/visual_tracking/yolo/yolo_model/gazebo_target.pt')

        self.declare_parameter('sub_image_topic', '/gazebo_camera/image_raw/compressed')
        self.declare_parameter('pub_boxes_topic', '/yolo/BoundingBoxes')
        self.declare_parameter('conf', 0.5)

        weight_path = self.get_parameter('weight_path').value
        if not weight_path:
            weight_path = "yolov8n.pt"

        self.sub_image_topic = self.get_parameter('sub_image_topic').value
        self.pub_boxes_topic = self.get_parameter('pub_boxes_topic').value
        self.conf = self.get_parameter('conf').value

        self.bridge = CvBridge()

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"YOLO device: {self.device}")

        try:
            self.model = YOLO(weight_path)
            self.get_logger().info("Warming up model...")
            self.model(np.zeros((640, 640, 3), dtype=np.uint8), device=self.device, verbose=False)
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            import sys
            sys.exit(1)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.image_sub = self.create_subscription(
            CompressedImage,
            self.sub_image_topic,
            self.image_cb,
            qos_profile
        )

        self.boxes_pub = self.create_publisher(BoundingBoxes, self.pub_boxes_topic, 10)

        self.is_processing = False

        self.get_logger().info(f"Successfully subscribed to {self.sub_image_topic}, ready for detection!")

    def image_cb(self, msg):
        if self.is_processing:
            return

        self.is_processing = True

        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

            results = self.model(cv_image, device=self.device, verbose=False)
            result = results[0]

            boxes_msg = BoundingBoxes()
            boxes_msg.header = msg.header
            boxes_msg.image_header = msg.header
            boxes_msg.bounding_boxes = []

            filtered_boxes = []
            if result.boxes:
                for box in result.boxes:
                    conf_val = float(box.conf.item())
                    if conf_val < self.conf:
                        continue

                    xyxy = box.xyxy[0].cpu().numpy()
                    cls_id = int(box.cls.item())
                    name = result.names[cls_id]

                    b = BoundingBox()
                    b.xmin = int(xyxy[0])
                    b.ymin = int(xyxy[1])
                    b.xmax = int(xyxy[2])
                    b.ymax = int(xyxy[3])
                    b.probability = conf_val
                    b.class_id = name

                    filtered_boxes.append(b)

            if filtered_boxes:
                boxes_msg.bounding_boxes = filtered_boxes
                self.boxes_pub.publish(boxes_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

        finally:
            self.is_processing = False

def main(args=None):
    rclpy.init(args=args)
    node = YoloRosSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
