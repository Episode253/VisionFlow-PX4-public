#!/usr/bin/env python3

# 可视化yolo检测结果确保话题准确

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time

from yolo_ros_msgs.msg import BoundingBoxes

class RealtimeBoxVisualizer(Node):
    def __init__(self):
        super().__init__('box_visualizer')

        # 订阅 BoundingBoxes 话题
        self.subscription = self.create_subscription(
            BoundingBoxes,
            '/yolo/BoundingBoxes',
            self.listener_callback,
            10
        )

        self.width = 1280
        self.height = 720
        self.window_name = "YOLO Radar View - Box Visualization"

        self.frame_count = 0
        self.start_time = time.time()

        print(f"[{self.window_name}] 节点已启动，等待数据...")

    def listener_callback(self, msg):

        canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        cx, cy = self.width // 2, self.height // 2

        cv2.line(canvas, (cx, 0), (cx, self.height), (50, 50, 50), 1)
        cv2.line(canvas, (0, cy), (self.width, cy), (50, 50, 50), 1)

        if msg.bounding_boxes:
            for box in msg.bounding_boxes:
                xmin, ymin = box.xmin, box.ymin
                xmax, ymax = box.xmax, box.ymax

                label = "Obj"
                if hasattr(box, 'class_id'): label = box.class_id
                elif hasattr(box, 'class_name'): label = box.class_name
                elif hasattr(box, 'Class'): label = box.Class

                conf = box.probability

                cv2.rectangle(canvas, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

                box_cx = (xmin + xmax) // 2
                box_cy = (ymin + ymax) // 2
                cv2.circle(canvas, (box_cx, box_cy), 4, (0, 0, 255), -1)

                text = f"{label} {conf:.2f} ({xmin},{ymin})"
                cv2.putText(canvas, text, (xmin, ymin - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        self.frame_count += 1
        elapsed = time.time() - self.start_time
        if elapsed > 1.0:
            fps = self.frame_count / elapsed
            cv2.putText(canvas, f"Topic FPS: {fps:.1f}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            self.frame_count = 0
            self.start_time = time.time()
        else:
             cv2.putText(canvas, "Topic FPS: calculating...", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        cv2.imshow(self.window_name, canvas)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RealtimeBoxVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
