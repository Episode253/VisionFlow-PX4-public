#!/usr/bin/env python3

"""
1. Low-Latency Video Capture:
   Utilizes a GStreamer pipeline to receive and decode H.264 UDP video streams.

2. Real-Time Object Detection:
   Loads a custom YOLO model to detect targets and generate annotated frames bounding boxes.

3. Local Video Recording:
   Provides interactive terminal prompts to optionally record the raw video stream and/or the YOLO-annotated results to local MP4 files for post-flight analysis.

4. ROS 2 Republishing:
   It compresses the raw video frames into JPEG format and continuously publishes them as `sensor_msgs/CompressedImage` to the `/gazebo_camera/image_raw/compressed` topic.

5. GUI Visualization & Monitoring:
   Optionally displays a downscaled real-time preview window of the tracking results and dynamically prints system status in the terminal.
"""

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import cv2
import numpy as np
from ultralytics import YOLO
import torch
import os
import time
import sys
import threading
import queue

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

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

    def start(self):
        self.running = True
        self.pipeline.set_state(Gst.State.PLAYING)
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        print("[INFO] GStreamer capture thread started.")
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

                    if self.frame_queue.full():
                        try:
                            self.frame_queue.get_nowait()
                        except queue.Empty:
                            pass

                    self.frame_queue.put(frame)
            except Exception as e:
                pass

    def read(self):
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        self.pipeline.set_state(Gst.State.NULL)

def main(args=None):
    rclpy.init(args=args)
    ros_node = rclpy.create_node('yolo_video_tracker')

    image_pub = ros_node.create_publisher(CompressedImage, '/gazebo_camera/image_raw/compressed', 10)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[INFO] YOLO device: {device}")

    model_path = os.path.expanduser("/home/renwang/data_storage/PX4-Autopilot/develop/visual_tracking/yolo/yolo_model/gazebo_target.pt")
    model = YOLO(model_path)
    model.to(device)

    pipeline_str = (
        'udpsrc port=5600 caps="application/x-rtp, media=video, encoding-name=H264, payload=96" ! '
        'rtpjitterbuffer latency=0 ! rtph264depay ! avdec_h264 ! videoconvert ! '
        'video/x-raw, format=BGR ! appsink name=sink sync=false max-buffers=1 drop=true'
    )

    save_dir = "/home/renwang"
    raw_path = os.path.join(save_dir, "raw_stream.mp4")
    yolo_path = os.path.join(save_dir, "yolo_result.mp4")
    fps_assumed = 30.0

    cap = GStreamerCapture(pipeline_str)
    cap.start()

    print("[INFO] Waiting for video stream...")
    frame = None
    while frame is None:
        frame = cap.read()
        time.sleep(0.1)

    width, height = cap.width, cap.height

    print("\n" + "="*40)
    print(f"[INFO] Video stream detected: {width}x{height}")
    print("="*40)

    record_raw = input("Record RAW stream? (y/n): ").strip().lower() == 'y'
    record_yolo = input("Record YOLO result? (y/n): ").strip().lower() == 'y'
    enable_visual = input("Enable GUI Visualization? (y/n): ").strip().lower() == 'y'

    print("-" * 40)
    print(f"Configuration:")
    print(f"  > Raw Recording : {'[ON] ' + raw_path if record_raw else '[OFF]'}")
    print(f"  > YOLO Recording: {'[ON] ' + yolo_path if record_yolo else '[OFF]'}")
    print(f"  > GUI Display   : {'[ON]' if enable_visual else '[OFF] (Press Ctrl+C to stop)'}")
    print(f"  > ROS 2 Topic   : /gazebo_camera/image_raw/compressed")
    print("-" * 40 + "\n")

    raw_writer = None
    yolo_writer = None
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')

    if record_raw:
        raw_writer = cv2.VideoWriter(raw_path, fourcc, fps_assumed, (width, height))
    if record_yolo:
        yolo_writer = cv2.VideoWriter(yolo_path, fourcc, fps_assumed, (width, height))

    frame_count = 0
    fps = 0.0
    fps_t0 = time.monotonic()
    last_ui_update = 0.0

    rec_status_str = []
    if record_raw: rec_status_str.append("Raw")
    if record_yolo: rec_status_str.append("YOLO")
    rec_display = "+".join(rec_status_str) if rec_status_str else "None"

    try:
        while rclpy.ok():
            frame = cap.read()
            if frame is None:
                time.sleep(0.005)
                continue

            results = model(frame, device=device, verbose=False)
            annotated_frame = results[0].plot()

            if record_raw: raw_writer.write(frame)
            if record_yolo: yolo_writer.write(annotated_frame)

            msg = CompressedImage()
            msg.header.stamp = ros_node.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            msg.format = "jpeg"

            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            success, encoded_image = cv2.imencode('.jpg', frame, encode_param)

            if success:
                msg.data = np.array(encoded_image).tobytes()
                image_pub.publish(msg)

            rclpy.spin_once(ros_node, timeout_sec=0)

            if enable_visual:
                display_frame = cv2.resize(annotated_frame, (0, 0), fx=0.5, fy=0.5)
                cv2.imshow("YOLO Detection", display_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            frame_count += 1
            now = time.monotonic()
            if now - fps_t0 >= 1.0:
                fps = frame_count / (now - fps_t0)
                frame_count = 0
                fps_t0 = now

            if now - last_ui_update >= 1.0:
                print(f"\r[INFO] FPS: {fps:.1f} | Res: {width}x{height} | [REC: {rec_display}] | ROS2 Pub: Active", end="")
                last_ui_update = now

    except KeyboardInterrupt:
        pass
    finally:
        print("\n\n[INFO] Shutting down...")
        cap.stop()
        if raw_writer: raw_writer.release()
        if yolo_writer: yolo_writer.release()
        if enable_visual: cv2.destroyAllWindows()

        ros_node.destroy_node()
        rclpy.shutdown()

        print("[INFO] Done.")

if __name__ == "__main__":
    main()
