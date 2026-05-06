#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data

from predict_and_update import KalmanFilter
import cv2
import os
import time
import sys
import numpy as np
from ultralytics import YOLO

class YoloKalmanTrackerNode(Node):

    PROJECT_ROOT = "/home/renwang/data_storage/PX4-Autopilot"
    YOLO_MODEL_DIR = os.path.join(PROJECT_ROOT, "develop/visual_tracking/yolo/yolo_model")

    def __init__(self, model_weights=None):
        super().__init__('yolo_kalman_tracker_node')

        if model_weights is None:
            model_weights = os.path.join(self.YOLO_MODEL_DIR, 'gazebo_target.pt')

        self.model = YOLO(model_weights, verbose=False)
        self.names = self.model.names
        self.get_logger().info(f"已加载YOLO模型: {model_weights}")

        # 卡尔曼滤波参数
        self.dt = 1/30
        self.INIT_POS_STD = 10
        self.INIT_VEL_STD = 10
        self.ACCEL_STD = 40
        self.GPS_POS_STD = 15

        self.kf = KalmanFilter(self.dt, self.INIT_POS_STD, self.INIT_VEL_STD,
                               self.ACCEL_STD, self.GPS_POS_STD)

        self.width = 0
        self.height = 0
        self.frame_count = 0
        self.start_time = time.time()

        self.is_first_frame = True
        self.target_detected = False
        self.track_status = "SEARCHING"
        self.current_pos = (0, 0)

        self.true_circle_color = (0, 255, 0)
        self.predict_circle_color = (255, 0, 0)
        self.update_circle_color = (0, 0, 255)
        self.circle_radius = 8
        self.circle_thickness = 4

        self.subscription = self.create_subscription(
            CompressedImage,
            '/gazebo_camera/image_raw/compressed',
            self.listener_callback,
            qos_profile_sensor_data)

        self.get_logger().info("等待 ROS 2 视频流话题: /gazebo_camera/image_raw/compressed ...")

        print("\n" + "="*50)
        print("\033[92m[INFO] 视觉追踪已启动，开始实时滤波处理\033[0m")
        print("\033[93m[INFO] 提示: 随时按 Ctrl+C 即可安全退出\033[0m")
        print("="*50 + "\n")

    def listener_callback(self, msg):
        """
        ROS 2 订阅回调函数：接收压缩图像并进行处理
        """
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warning("接收到的图像解码失败")
            return

        if self.width == 0 or self.height == 0:
            self.height, self.width = frame.shape[:2]

        processed_frame = self.process_frame(frame.copy())

        cv2.imshow("YOLO Kalman Filter ROS2", processed_frame)
        cv2.waitKey(1)

        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0

        if self.track_status == "TRACKING":
            color_code = "\033[92m"
        elif self.track_status == "PREDICTING":
            color_code = "\033[93m"
        else:
            color_code = "\033[90m"

        status_line = (f"\r{color_code}▶ [{self.track_status}]\033[0m "
                       f"Frame: {self.frame_count:05d} | "
                       f"FPS: {fps:4.1f} | "
                       f"Position: ({self.current_pos[0]:>4d}, {self.current_pos[1]:>4d})     ")
        sys.stdout.write(status_line)
        sys.stdout.flush()

    def _draw_legend(self, frame):
        cv2.circle(frame, (20, 20), 6, self.true_circle_color, 2)
        cv2.circle(frame, (20, 50), 6, self.predict_circle_color, 2)
        cv2.circle(frame, (20, 80), 6, self.update_circle_color, 2)
        cv2.putText(frame, "True (Green)", (40, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.true_circle_color, 2)
        cv2.putText(frame, "Predict (Blue)", (40, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.predict_circle_color, 2)
        cv2.putText(frame, "Update (Red)", (40, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.update_circle_color, 2)

    def _get_bounding_box_center(self, frame, object_class='target'):
        centers = []
        results = self.model(frame, verbose=False)
        person_detected = False

        for result in results:
            for r in result.boxes.data.tolist():
                x1, y1, x2, y2, score, class_id = r
                class_name = self.names.get(int(class_id))

                if class_name == object_class and score > 0.5:
                    if not person_detected:
                        person_detected = True
                        center_x = (int(x1) + int(x2)) // 2
                        center_y = (int(y1) + int(y2)) // 2
                        centers.append((center_x, center_y))

        if not person_detected:
            centers.append("Not detected")

        return centers

    def process_frame(self, frame):
        self.frame_count += 1
        self.target_detected = False
        self._draw_legend(frame)

        centers = self._get_bounding_box_center(frame, object_class='target')

        if len(centers) > 0 and isinstance(centers[0], tuple):
            center = centers[0]
            self.target_detected = True
            self.track_status = "TRACKING"

            true_center = (int(center[0]), int(center[1]))
            cv2.circle(frame, true_center, self.circle_radius, self.true_circle_color, self.circle_thickness)

            raw_x_pred, raw_y_pred = self.kf.predict()
            x_pred = max(0, min(int(self.width) - 1, int(round(raw_x_pred))))
            y_pred = max(0, min(int(self.height) - 1, int(round(raw_y_pred))))
            pred_pos = (x_pred, y_pred)

            if not self.is_first_frame:
                cv2.circle(frame, pred_pos, self.circle_radius, self.predict_circle_color, self.circle_thickness)

            raw_x_updt, raw_y_updt = self.kf.update(true_center)
            x_updt = max(0, min(int(self.width) - 1, int(round(raw_x_updt))))
            y_updt = max(0, min(int(self.height) - 1, int(round(raw_y_updt))))
            updt_pos = (x_updt, y_updt)

            self.current_pos = updt_pos
            cv2.circle(frame, updt_pos, self.circle_radius, self.update_circle_color, self.circle_thickness)
            self.is_first_frame = False

        else:
            if not self.is_first_frame:
                self.track_status = "PREDICTING"
                raw_x_pred, raw_y_pred = self.kf.predict()
                x_pred = max(0, min(int(self.width) - 1, int(round(raw_x_pred))))
                y_pred = max(0, min(int(self.height) - 1, int(round(raw_y_pred))))
                pred_pos = (x_pred, y_pred)

                self.current_pos = pred_pos
                cv2.circle(frame, pred_pos, self.circle_radius, self.predict_circle_color, self.circle_thickness)
            else:
                self.track_status = "SEARCHING"

        return frame

    def cleanup(self):
        cv2.destroyAllWindows()
        print(f"\n\033[92m[INFO] 资源已释放，总计处理 {self.frame_count} 帧。安全退出！\033[0m")


def main(args=None):
    rclpy.init(args=args)

    import argparse
    parser = argparse.ArgumentParser(description='YOLO Kalman Filter (ROS 2 Node)')
    parser.add_argument('--model', type=str, default=None, help='YOLO model weights path')

    parsed_args, ros_args = parser.parse_known_args()

    tracker_node = YoloKalmanTrackerNode(model_weights=parsed_args.model)

    try:
        rclpy.spin(tracker_node)
    except KeyboardInterrupt:
        print("\n\n\033[91m■ [STOPPED] 接收到退出信号 (Ctrl+C)，正在终止进程...\033[0m")
    finally:
        tracker_node.cleanup()
        tracker_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
