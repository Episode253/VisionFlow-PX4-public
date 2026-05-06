#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import threading
import queue
import ctypes
from ctypes import c_void_p, c_int, c_float, c_uint8, c_char_p, POINTER

import numpy as np
import cv2

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst
Gst.init(None)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

from transforms3d.euler import quat2euler, euler2quat


class GStreamerCapture:
    """
    使用 GStreamer 直接抓取 UDP 视频流，通过 appsink 输出 BGR numpy 帧。
    """

    def __init__(self, pipeline_str: str):
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("sink")

        if self.appsink is None:
            raise RuntimeError("GStreamer pipeline error: Could not find appsink with name='sink'. Please check the pipeline string.")

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
                        dtype=np.uint8,
                    )
                    frame = arr.reshape((self.height, self.width, 3)).copy()

                    if self.frame_queue.full():
                        try:
                            self.frame_queue.get_nowait()
                        except queue.Empty:
                            pass

                    self.frame_queue.put(frame)
            except Exception:
                pass

    def read(self):
        if self.frame_queue.empty():
            return None
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        self.pipeline.set_state(Gst.State.NULL)

ROOT_DIR = os.path.expanduser("/home/renwang/PX4-Autopilot/develop/visual_tracking/light-track-ncnn/LightTrack-ncnn")
BUILD_DIR = os.path.join(ROOT_DIR, "build")
LIB_PATH = os.path.join(BUILD_DIR, "liblighttrack.so")

if not os.path.exists(LIB_PATH):
    raise FileNotFoundError(f"Shared library not found at: {LIB_PATH}. Please compile with 'add_library' in CMakeLists.txt.")

_lighttrack = ctypes.cdll.LoadLibrary(LIB_PATH)

_lighttrack.lighttrack_create.argtypes = [c_char_p, c_char_p]
_lighttrack.lighttrack_create.restype = c_void_p

_lighttrack.lighttrack_destroy.argtypes = [c_void_p]
_lighttrack.lighttrack_destroy.restype = None

_lighttrack.lighttrack_init.argtypes = [
    c_void_p,
    POINTER(c_uint8),
    c_int,
    c_int,
    c_float,
    c_float,
    c_float,
    c_float,
]
_lighttrack.lighttrack_init.restype = None

_lighttrack.lighttrack_track.argtypes = [
    c_void_p,
    POINTER(c_uint8),
    c_int,
    c_int,
    POINTER(c_float),
    POINTER(c_float),
    POINTER(c_float),
    POINTER(c_float),
]
_lighttrack.lighttrack_track.restype = None


class PyLightTrack:
    """Python 封装的 LightTrack"""

    def __init__(self, model_init_base: str, model_update_base: str):
        if not os.path.exists(model_init_base + ".param"):
             raise FileNotFoundError(f"Model file not found: {model_init_base}.param")

        init_b = model_init_base.encode("utf-8")
        update_b = model_update_base.encode("utf-8")
        self._handle = _lighttrack.lighttrack_create(init_b, update_b)
        if not self._handle:
            raise RuntimeError("failed to create LightTrack instance")

    def __del__(self):
        if getattr(self, "_handle", None):
            try:
                _lighttrack.lighttrack_destroy(self._handle)
            except Exception:
                pass
            self._handle = None

    def init(self, frame_bgr: np.ndarray, roi_xywh):
        x, y, w, h = roi_xywh
        x0, y0 = float(x), float(y)
        x1, y1 = float(x + w), float(y + h)

        frame_bgr = np.ascontiguousarray(frame_bgr, dtype=np.uint8)
        h_img, w_img, _ = frame_bgr.shape

        ptr = frame_bgr.ctypes.data_as(POINTER(c_uint8))
        _lighttrack.lighttrack_init(
            self._handle,
            ptr,
            c_int(w_img),
            c_int(h_img),
            c_float(x0),
            c_float(y0),
            c_float(x1),
            c_float(y1),
        )

    def track(self, frame_bgr: np.ndarray):
        frame_bgr = np.ascontiguousarray(frame_bgr, dtype=np.uint8)
        h_img, w_img, _ = frame_bgr.shape

        ptr = frame_bgr.ctypes.data_as(POINTER(c_uint8))

        x0 = c_float(0.0)
        y0 = c_float(0.0)
        x1 = c_float(0.0)
        y1 = c_float(0.0)

        _lighttrack.lighttrack_track(
            self._handle,
            ptr,
            c_int(w_img),
            c_int(h_img),
            ctypes.byref(x0),
            ctypes.byref(y0),
            ctypes.byref(x1),
            ctypes.byref(y1),
        )

        x0_v, y0_v, x1_v, y1_v = x0.value, y0.value, x1.value, y1.value
        w = max(0.0, x1_v - x0_v)
        h = max(0.0, y1_v - y0_v)
        return int(x0_v), int(y0_v), int(w), int(h)

class EmaFilter:
    def __init__(self, alpha=0.4):
        self.alpha = alpha
        self.value = None

    def update(self, new_val):
        if self.value is None:
            self.value = new_val
        else:
            self.value = self.alpha * new_val + (1 - self.alpha) * self.value
        return self.value

    def reset(self):
        self.value = None


class PIDController:
    def __init__(self, kp, kd, out_min, out_max):
        self.kp = kp
        self.kd = kd
        self.min = out_min
        self.max = out_max
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, error, current_time):
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return 0.0

        dt = current_time - self.prev_time
        if dt <= 0.0001:
            return 0.0

        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.kd * derivative)
        output = max(min(output, self.max), self.min)

        self.prev_error = error
        self.prev_time = current_time
        return output

    def reset(self):
        self.prev_time = None
        self.prev_error = 0.0

    def set_limits(self, min_val, max_val):
        self.min = min_val
        self.max = max_val

class LightTrackOffboardTracker(Node):
    def __init__(self):
        super().__init__("lighttrack_offboard_tracker")

        self.data_lock = threading.Lock()

        self.takeoff_height = 1.5
        self.pos_tolerance = 0.15

        self.filter_x = EmaFilter(alpha=0.4)
        self.filter_y = EmaFilter(alpha=0.4)
        self.filter_d = EmaFilter(alpha=0.3)

        self.accel_limit_x = 1.6
        self.accel_limit_z = 0.5
        self.accel_limit_yaw = 1.0

        self.last_vel_x = 0.0
        self.last_vel_z = 0.0
        self.last_vel_yaw = 0.0
        self.last_ctrl_time = time.time()

        self.pid_x = PIDController(kp=2.6, kd=0.8, out_min=-3.0, out_max=3.0)
        self.pid_z = PIDController(kp=1.5, kd=0.1, out_min=-1.0, out_max=1.0)
        self.pid_yaw = PIDController(kp=1.0, kd=0.4, out_min=-1.5, out_max=1.5)

        self.img_width = 1280
        self.img_height = 720
        self.center_x = self.img_width / 2
        self.center_y = self.img_height / 2
        self.target_height_ref = 360.0

        self.current_state = State()
        self.current_odom = None
        self.hover_yaw = 0.0
        self.hover_pos = [0.0, 0.0, 0.0]
        self.flight_phase = "WAIT_FOR_CONNECTION"

        self.target_captured = False
        self.lost_count = 0
        self.target_info = {"x": 0.0, "y": 0.0, "d": 0.0}

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.state_sub = self.create_subscription(State, "/mavros/state", self.state_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/mavros/local_position/odom", self.odom_cb, qos_profile
        )
        self.local_pos_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        self.vel_pub = self.create_publisher(TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        self.timer = self.create_timer(0.05, self.control_loop)
        self.offboard_timer = self.create_timer(1.0, self.offboard_arm_loop)

        self._video_stop = False
        self._video_thread = threading.Thread(target=self._video_loop, daemon=True)
        self._video_thread.start()

        self.get_logger().info(">>> LightTrack Offboard Tracker Initialized <<<")

    def state_cb(self, msg: State):
        self.current_state = msg

    def odom_cb(self, msg: Odometry):
        self.current_odom = msg
        if self.flight_phase == "WAIT_FOR_CONNECTION":
            q = msg.pose.pose.orientation
            _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
            self.hover_yaw = yaw

    def offboard_arm_loop(self):
        if self.flight_phase == "WAIT_FOR_CONNECTION" or self.current_state.mode in [
            "AUTO.LAND", "AUTO.RTL", "LAND"
        ]:
            return
        if self.current_state.connected:
            if self.current_state.mode != "OFFBOARD":
                req = SetMode.Request()
                req.custom_mode = "OFFBOARD"
                self.set_mode_client.call_async(req)
                self.get_logger().info("Requesting OFFBOARD mode...", once=True)
            if not self.current_state.armed:
                req = CommandBool.Request()
                req.value = True
                self.arming_client.call_async(req)
                self.get_logger().info("Requesting ARM...", once=True)

    def control_loop(self):
        if self.current_odom is None:
            return

        if self.flight_phase == "WAIT_FOR_CONNECTION":
            if self.current_state.connected and self.current_odom:
                self.hover_pos[0] = self.current_odom.pose.pose.position.x
                self.hover_pos[1] = self.current_odom.pose.pose.position.y

                current_z = self.current_odom.pose.pose.position.z
                is_armed = self.current_state.armed

                if is_armed and current_z > 0.3:
                    self.hover_pos[2] = current_z
                    self.get_logger().info(f"Airborne detected (z={current_z:.2f}m). Holding altitude.")
                else:
                    self.hover_pos[2] = self.takeoff_height
                    self.get_logger().info(f"On Ground. Takeoff to {self.takeoff_height}m.")

                self.flight_phase = "TAKEOFF"

        elif self.flight_phase == "TAKEOFF":
            self.perform_takeoff()
            curr_z = self.current_odom.pose.pose.position.z
            if abs(curr_z - self.hover_pos[2]) < self.pos_tolerance:
                self.flight_phase = "HOVER"
                self.get_logger().info("Phase: HOVER")

        elif self.flight_phase == "HOVER":
            self.perform_hover()
            with self.data_lock:
                is_captured = self.target_captured

            if is_captured:
                self.pid_x.reset()
                self.pid_z.reset()
                self.pid_yaw.reset()
                self.filter_x.reset()
                self.filter_y.reset()
                self.filter_d.reset()
                self.last_vel_x = 0.0
                self.last_vel_z = 0.0
                self.last_vel_yaw = 0.0
                self.flight_phase = "TRACK"
                self.get_logger().info("Phase: TRACK")

        elif self.flight_phase == "TRACK":
            self.perform_tracking()
            with self.data_lock:
                is_captured = self.target_captured

            if not is_captured:
                self.hover_pos[0] = self.current_odom.pose.pose.position.x
                self.hover_pos[1] = self.current_odom.pose.pose.position.y
                self.hover_pos[2] = self.current_odom.pose.pose.position.z
                if self.current_odom:
                    q = self.current_odom.pose.pose.orientation
                    _, _, current_yaw = quat2euler([q.w, q.x, q.y, q.z])
                    self.hover_yaw = current_yaw
                self.flight_phase = "HOVER"
                self.get_logger().warn("Target Lost -> HOVER")

    def perform_takeoff(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.hover_pos[0]
        pose.pose.position.y = self.hover_pos[1]
        pose.pose.position.z = self.hover_pos[2]
        qw, qx, qy, qz = euler2quat(0.0, 0.0, self.hover_yaw)
        pose.pose.orientation.w, pose.pose.orientation.x = qw, qx
        pose.pose.orientation.y, pose.pose.orientation.z = qy, qz
        self.local_pos_pub.publish(pose)

    def perform_hover(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.hover_pos[0]
        pose.pose.position.y = self.hover_pos[1]
        pose.pose.position.z = self.hover_pos[2]
        qw, qx, qy, qz = euler2quat(0.0, 0.0, self.hover_yaw)
        pose.pose.orientation.w, pose.pose.orientation.x = qw, qx
        pose.pose.orientation.y, pose.pose.orientation.z = qy, qz
        self.local_pos_pub.publish(pose)

    def perform_tracking(self):
        with self.data_lock:
            self.lost_count += 1
            if self.lost_count > 40:
                self.target_captured = False
                return

            tx = self.target_info["x"]
            ty = self.target_info["y"]
            td = self.target_info["d"]

        err_pix_x = tx - self.center_x
        if abs(err_pix_x) < 20: err_pix_x = 0
        err_pix_y = ty - self.center_y
        if abs(err_pix_y) < 20: err_pix_y = 0

        error_yaw = err_pix_x * -0.005
        error_z = err_pix_y * -0.005
        error_x = 1.0 - td / self.target_height_ref
        if abs(error_x) < 0.05: error_x = 0

        dynamic_limit_x = min(0.5 + 2.5 * abs(error_x), 3.0)
        self.pid_x.set_limits(-dynamic_limit_x, dynamic_limit_x)

        now = time.time()
        target_vel_x = self.pid_x.compute(error_x, now)
        target_vel_z = self.pid_z.compute(error_z, now)
        target_vel_yaw = self.pid_yaw.compute(error_yaw, now)

        dt = now - self.last_ctrl_time
        if dt <= 0: dt = 0.05
        self.last_ctrl_time = now

        def apply_ramp(target, current, limit, dt_):
            max_step = limit * dt_
            diff = target - current
            step = max(min(diff, max_step), -max_step)
            return current + step

        smooth_vel_x = apply_ramp(target_vel_x, self.last_vel_x, self.accel_limit_x, dt)
        smooth_vel_z = apply_ramp(target_vel_z, self.last_vel_z, self.accel_limit_z, dt)
        smooth_vel_yaw = apply_ramp(target_vel_yaw, self.last_vel_yaw, self.accel_limit_yaw, dt)

        self.last_vel_x = smooth_vel_x
        self.last_vel_z = smooth_vel_z
        self.last_vel_yaw = smooth_vel_yaw

        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.twist.linear.x = float(smooth_vel_x)
        vel_msg.twist.linear.y = 0.0
        vel_msg.twist.linear.z = float(smooth_vel_z)
        vel_msg.twist.angular.z = float(smooth_vel_yaw)

        self.vel_pub.publish(vel_msg)

    def _video_loop(self):
        gstreamer_pipeline = (
            'udpsrc port=5600 caps="application/x-rtp, media=video, encoding-name=H264, payload=96" ! '
            "rtpjitterbuffer latency=0 ! rtph264depay ! avdec_h264 ! videoconvert ! "
            "video/x-raw, format=BGR ! appsink name=sink sync=false max-buffers=1 drop=true"
        )

        try:
            cap = GStreamerCapture(gstreamer_pipeline).start()
        except Exception as e:
            self.get_logger().error(f"GStreamer Error: {e}")
            return

        model_init_base = os.path.join(ROOT_DIR, "model", "lighttrack_init")
        model_update_base = os.path.join(ROOT_DIR, "model", "lighttrack_update")

        try:
            tracker = PyLightTrack(model_init_base, model_update_base)
        except Exception as e:
            self.get_logger().error(f"Failed to create LightTrack: {e}")
            cap.stop()
            return

        tracker_initialized = False
        cv2.namedWindow("LightTrack Tracking", cv2.WINDOW_NORMAL)

        while not self._video_stop:
            frame = cap.read()
            if frame is None:
                time.sleep(0.01)
                continue

            self.img_height, self.img_width = frame.shape[:2]
            self.center_x = self.img_width / 2.0
            self.center_y = self.img_height / 2.0

            key = cv2.waitKey(1) & 0xFF

            if not tracker_initialized:
                cv2.imshow("LightTrack Tracking", frame)
                if key == ord("s"):
                    roi = cv2.selectROI("LightTrack Tracking", frame, fromCenter=False, showCrosshair=True)
                    x, y, w, h = roi
                    if w > 0 and h > 0:
                        tracker.init(frame, roi)
                        tracker_initialized = True
                        with self.data_lock:
                            self.target_captured = True
                            self.lost_count = 0
                        self.get_logger().info("LightTrack initialized.")
                    else:
                        self.get_logger().warn("ROI cancelled.")
                    continue
                if key == ord("q"):
                    break
                continue

            bbox = tracker.track(frame)
            x, y, w, h = bbox

            with self.data_lock:
                if w <= 0 or h <= 0:
                    self.lost_count += 1
                    if self.lost_count > 40:
                        self.target_captured = False
                else:
                    cx = x + w / 2.0
                    cy = y + h / 2.0
                    self.target_info["x"] = self.filter_x.update(cx)
                    self.target_info["y"] = self.filter_y.update(cy)
                    self.target_info["d"] = self.filter_d.update(float(h))
                    self.target_captured = True
                    self.lost_count = 0

            if w > 0 and h > 0:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cv2.imshow("LightTrack Tracking", frame)

            if key == ord("q"):
                break
            if key == ord("r"):
                tracker_initialized = False
                with self.data_lock:
                    self.target_captured = False
                    self.lost_count = 0

        cap.stop()
        cv2.destroyAllWindows()
        with self.data_lock:
            self.target_captured = False

    def shutdown(self):
        self._video_stop = True
        if self._video_thread and self._video_thread.is_alive():
            try:
                self._video_thread.join(timeout=1.0)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = LightTrackOffboardTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_vel = TwistStamped()
        node.vel_pub.publish(stop_vel)
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
