#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 优化处理速度，使得yolo检测稳定在30帧左右

import rclpy
from rclpy.node import Node

import os
import sys
import time
import queue
import threading

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import cv2
import numpy as np
import torch
from ultralytics import YOLO

from std_msgs.msg import Header
from sensor_msgs.msg import Image

from yolo_ros_msgs.msg import BoundingBox, BoundingBoxes

Gst.init(None)

class GStreamerCapture:
    def __init__(self, pipeline_str):
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("sink")
        self.appsink.set_property("emit-signals", True)

        self.frame_queue = queue.Queue(maxsize=1)
        self.running = False
        self.thread = None
        self.width = 0
        self.height = 0
        self.lock = threading.Lock()

    def start(self):
        self.running = True
        self.pipeline.set_state(Gst.State.PLAYING)
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        return self

    def _capture_loop(self):
        while self.running:
            try:
                sample = self.appsink.emit("pull-sample")
                if sample:
                    buf = sample.get_buffer()
                    caps = sample.get_caps()
                    structure = caps.get_structure(0)
                    self.width = structure.get_value("width")
                    self.height = structure.get_value("height")

                    arr = np.frombuffer(
                        buf.extract_dup(0, buf.get_size()),
                        dtype=np.uint8
                    )
                    frame = arr.reshape((self.height, self.width, 3))

                    # === 核心优化：强制覆盖旧帧 ===
                    # 如果队列满了，就把旧的拿出来扔掉，把新的放进去
                    if self.frame_queue.full():
                        try:
                            self.frame_queue.get_nowait()
                        except queue.Empty:
                            pass

                    self.frame_queue.put(frame)

            except Exception as e:
                pass

    def read(self):
        """
        非阻塞读取，如果没有新帧返回 None
        如果有积压，只返回最新的一帧
        """
        if self.frame_queue.empty():
            return None

        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        self.pipeline.set_state(Gst.State.NULL)

class YoloStreamDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_stream_detect')

        self.declare_parameter('weight_path', '')
        self.declare_parameter('pub_boxes_topic', '/yolo/BoundingBoxes')
        self.declare_parameter('pub_image_topic', '/yolo/detection_image')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('conf', 0.7)
        self.declare_parameter('visualize', True)
        self.declare_parameter('udp_port', 5600)

        weight_path = self.get_parameter('weight_path').value
        if not weight_path:
            weight_path = os.path.expanduser("/home/renwang/data_storage/PX4-Autopilot/develop/visual_tracking/yolo/yolo_model/gazebo_target.pt")

        self.pub_boxes_topic = self.get_parameter('pub_boxes_topic').value
        self.pub_image_topic = self.get_parameter('pub_image_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        conf = self.get_parameter('conf').value
        self.conf = conf
        self.visualize = self.get_parameter('visualize').value
        udp_port = self.get_parameter('udp_port').value

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"YOLO device: {self.device}")

        try:
            self.model = YOLO(weight_path)
            self.model.conf = conf
            self.get_logger().info("Warming up model...")
            self.model(np.zeros((640, 640, 3), dtype=np.uint8), device=self.device, verbose=False)
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            sys.exit(1)

        self.boxes_pub = self.create_publisher(BoundingBoxes, self.pub_boxes_topic, 10)
        self.image_pub = self.create_publisher(Image, self.pub_image_topic, 5)

        pipeline_str = (
            f'udpsrc port={udp_port} caps="application/x-rtp, media=video, encoding-name=H264, payload=96" ! '
            'rtpjitterbuffer latency=0 ! '
            'rtph264depay ! avdec_h264 ! '
            'videoconvert ! video/x-raw, format=BGR ! '
            'appsink name=sink sync=false max-buffers=1 drop=true'
        )

        self.cap = GStreamerCapture(pipeline_str)
        self.cap.start()

        self.get_logger().info("Waiting for video stream...")
        for _ in range(50):
            if self.cap.width > 0:
                break
            time.sleep(0.1)

        if self.cap.width == 0:
             self.get_logger().warn("Stream not ready yet, but continuing...")

        self.width = self.cap.width
        self.height = self.cap.height

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.frame_count = 0
        self.fps_t0 = time.monotonic()

        self.is_processing = False

    def timer_callback(self):
        if self.is_processing:
            return
        self.is_processing = True

        try:
            frame = self.cap.read()
            if frame is None:
                self.is_processing = False
                return

            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                self.width = frame.shape[1]
                self.height = frame.shape[0]

            stamp = self.get_clock().now().to_msg()
            header = Header()
            header.stamp = stamp
            header.frame_id = self.camera_frame

            results = self.model(frame, device=self.device, verbose=False)
            result = results[0]

            boxes_msg = BoundingBoxes()
            boxes_msg.header = header
            boxes_msg.image_header = header
            boxes_msg.bounding_boxes = []

            filtered_boxes = []
            if result.boxes:
                for box in result.boxes:
                    xyxy = box.xyxy[0].cpu().numpy()
                    cls_id = int(box.cls.item())
                    conf_val = float(box.conf.item())
                    name = result.names[cls_id]

                    if conf_val < self.conf:
                        continue

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

            # 调试阶段始终进行图像可视化/发布，但只绘制满足置信度阈值的框
            if (self.visualize or self.image_pub.get_subscription_count() > 0):
                annotated = frame.copy()

                for b in filtered_boxes:
                    cv2.rectangle(annotated, (b.xmin, b.ymin), (b.xmax, b.ymax), (0, 255, 0), 2)
                    label = f"{b.class_id}:{b.probability:.2f}"
                    y_text = max(b.ymin - 6, 12)
                    cv2.putText(annotated, label, (b.xmin, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if self.image_pub.get_subscription_count() > 0:
                    img_msg = Image()
                    img_msg.header = header
                    img_msg.height = annotated.shape[0]
                    img_msg.width = annotated.shape[1]
                    img_msg.encoding = 'bgr8'
                    img_msg.is_bigendian = 0
                    img_msg.step = annotated.shape[1] * 3
                    img_msg.data = annotated.tobytes()
                    self.image_pub.publish(img_msg)

                if self.visualize:
                    fps_val = 1.0 / (result.speed['inference'] / 1000.0 + 1e-6)
                    cv2.putText(annotated, f'YOLO: {int(fps_val)} FPS', (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                    display_img = cv2.resize(annotated, (0, 0), fx=0.5, fy=0.5)
                    cv2.imshow('YOLO Real-Time', display_img)
                    cv2.waitKey(1)

            self.frame_count += 1
            now = time.monotonic()
            if now - self.fps_t0 >= 2.0:
                fps = self.frame_count / (now - self.fps_t0)
                self.get_logger().info(f"Pipeline FPS: {fps:.1f}")
                self.frame_count = 0
                self.fps_t0 = now

        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}")

        finally:
            self.is_processing = False

    def shutdown(self):
        self.cap.stop()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = YoloStreamDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
