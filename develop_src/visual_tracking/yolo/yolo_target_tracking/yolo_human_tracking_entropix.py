#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

from yolo_ros_msgs.msg import BoundingBoxes
from transforms3d.euler import quat2euler, euler2quat
from collections import deque
import time

class EmaFilter:
    """
    指数移动平均滤波器 (Exponential Moving Average)
    用于平滑 YOLO 的抖动数据
    """
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
        if dt <= 0.0001: return 0.0

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

class YoloOffboardTracker(Node):
    def __init__(self):
        super().__init__('yolo_human_tracking')

        self.takeoff_height = 1.20

        self.pos_tolerance = 0.065

        self.filter_x = EmaFilter(alpha=0.12)
        self.filter_y = EmaFilter(alpha=0.15)
        self.filter_d = EmaFilter(alpha=0.18)

        self.accel_limit_x = 2.0
        self.accel_limit_z = 1.0
        self.accel_limit_yaw = 1.0

        self.max_vel_x = 2.0
        self.max_vel_z = 0.8
        self.max_vel_yaw = 0.5

        self.last_vel_x = 0.0
        self.last_vel_z = 0.0
        self.last_vel_yaw = 0.0
        self.last_ctrl_time = time.time()

        self.pid_x = PIDController(kp=3.2, kd=0.0, out_min=-4.0, out_max=4.0)
        self.pid_z = PIDController(kp=0.5, kd=0.0, out_min=-1.0, out_max=1.0)
        self.pid_yaw = PIDController(kp=1.8, kd=0.0, out_min=-3.0, out_max=3.0)

        self.img_width = 1280
        self.img_height = 720
        self.center_x = self.img_width / 2
        self.center_y = self.img_height / 2
        self.target_height_ref = 360.0

        self.current_state = State()
        self.current_odom = None
        self.hover_yaw = 0.0
        self.hover_pos = [0.0, 0.0, 0.0]
        self.flight_phase = 'CHECK_STATUS'

        self.target_captured = False
        self.lost_count = 0

        self.target_info = {'x': 0, 'y': 0, 'd': 0}

        self.locked = False
        self.lock_frames = 0
        self.lock_confirm_frames = 40
        self.det_window = deque(maxlen=60)

        self.waiting_for_mavros = False

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/mavros/local_position/odom', self.odom_cb, qos_profile)
        self.yolo_sub = self.create_subscription(BoundingBoxes, '/yolo/BoundingBoxes', self.yolo_cb, qos_profile)

        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.timer = self.create_timer(0.05, self.control_loop)
        self.offboard_timer = self.create_timer(1.0, self.offboard_arm_loop)

        self.get_logger().info("Yolo Human Tracking Node Initialized")

    def state_cb(self, msg):
        self.current_state = msg

    def odom_cb(self, msg):
        self.current_odom = msg
        if self.flight_phase == 'CHECK_STATUS':
            q = msg.pose.pose.orientation
            _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
            self.hover_yaw = yaw

    def offboard_arm_loop(self):
        if self.flight_phase == 'CHECK_STATUS' or self.current_state.mode in ["AUTO.LAND", "AUTO.RTL","AUTO.TAKEOFF"]:
            return
        if self.current_state.connected:
            if self.current_state.mode != 'OFFBOARD':
                req = SetMode.Request()
                req.custom_mode = 'OFFBOARD'
                self.set_mode_client.call_async(req)
                self.get_logger().info("Requesting OFFBOARD mode...", once=True)
            if not self.current_state.armed:
                req = CommandBool.Request()
                req.value = True
                self.arming_client.call_async(req)
                self.get_logger().info("Requesting ARM...", once=True)

    def yolo_cb(self, msg):
        detected = False

        for box in msg.bounding_boxes:
            if box.class_id == "target":
                self.target_captured = True
                self.lost_count = 0
                raw_x = (box.xmin + box.xmax) / 2
                raw_y = (box.ymin + box.ymax) / 2
                raw_d = float(box.ymax - box.ymin)

                self.target_info['x'] = self.filter_x.update(raw_x)
                self.target_info['y'] = self.filter_y.update(raw_y)
                self.target_info['d'] = self.filter_d.update(raw_d)

                self.det_window.append((self.target_info['x'], self.target_info['y']))

                self.lock_frames += 1
                if not self.locked and self.lock_frames >= self.lock_confirm_frames:
                    self.locked = True

                detected = True
                break

        if not detected:
            self.filter_x.reset()
            self.filter_y.reset()
            self.filter_d.reset()

            self.lock_frames = 0

    def control_loop(self):
        if self.current_odom is None or not self.current_state.connected:
            if not self.waiting_for_mavros:
                self.waiting_for_mavros = True
                self.get_logger().info("MAVROS is not running or ODOM not ready. Waiting for connection...")
            return
        else:
            if self.waiting_for_mavros:
                self.waiting_for_mavros = False
                self.get_logger().info("MAVROS connected, starting tracking state machine.")

        if self.flight_phase == 'CHECK_STATUS':
            if self.current_state.connected and self.current_odom:

                self.hover_pos[0] = self.current_odom.pose.pose.position.x
                self.hover_pos[1] = self.current_odom.pose.pose.position.y

                current_z = self.current_odom.pose.pose.position.z
                is_armed = self.current_state.armed

                if is_armed :
                    self.hover_pos[2] = current_z
                    self.get_logger().info(f"Airborne detected (z={current_z:.2f}m). Holding current altitude.")
                else:
                    self.hover_pos[2] = self.takeoff_height
                    self.get_logger().info(f"On Ground. Taking off to default {self.takeoff_height}m.")

                self.flight_phase = 'TAKEOFF'

        elif self.flight_phase == 'TAKEOFF':
            self.perform_takeoff()
            curr_z = self.current_odom.pose.pose.position.z

            if abs(curr_z - self.hover_pos[2]) < self.pos_tolerance:
                self.flight_phase = 'HOVER'
                self.get_logger().info("Phase: HOVER")

        elif self.flight_phase == 'HOVER':
            self.perform_hover()

            if self.target_captured:

                self.pid_x.reset(); self.pid_z.reset(); self.pid_yaw.reset()
                self.filter_x.reset(); self.filter_y.reset(); self.filter_d.reset()
                self.last_vel_x = 0.0; self.last_vel_z = 0.0; self.last_vel_yaw = 0.0

                self.flight_phase = 'TRACK'
                self.get_logger().info("Phase: TRACK")

        elif self.flight_phase == 'TRACK':
            self.perform_tracking()
            if not self.target_captured:
                self.hover_pos[0] = self.current_odom.pose.pose.position.x
                self.hover_pos[1] = self.current_odom.pose.pose.position.y
                self.hover_pos[2] = self.current_odom.pose.pose.position.z
                if self.current_odom:
                    q = self.current_odom.pose.pose.orientation
                    _, _, current_yaw = quat2euler([q.w, q.x, q.y, q.z])
                    self.hover_yaw = current_yaw
                self.flight_phase = 'HOVER'
                self.get_logger().warn("Target Lost -> HOVER")

    def perform_takeoff(self):
        """发布起飞/高度保持指令"""
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
        self.lost_count += 1
        if self.lost_count > 40:
            self.target_captured = False
            return

        tx = self.target_info['x']
        ty = self.target_info['y']
        td = self.target_info['d']

        if self.locked or not self.det_window:
            ref_x = tx
            ref_y = ty
        else:
            sum_x = sum(p[0] for p in self.det_window)
            sum_y = sum(p[1] for p in self.det_window)
            n = len(self.det_window)
            ref_x = sum_x / n
            ref_y = sum_y / n

        DEADBAND_X = 30.0
        DEADBAND_Y = 80.0

        raw_err_x = ref_x - self.center_x
        raw_err_y = ref_y - self.center_y

        if abs(raw_err_x) < DEADBAND_X:
            err_pix_x = 0.0
        else:
            sign_x = 1.0 if raw_err_x > 0 else -1.0
            err_pix_x = (abs(raw_err_x) - DEADBAND_X) * sign_x

        if abs(raw_err_y) < DEADBAND_Y:
            err_pix_y = 0.0
        else:
            sign_y = 1.0 if raw_err_y > 0 else -1.0
            err_pix_y = (abs(raw_err_y) - DEADBAND_Y) * sign_y

        if not self.locked:
            error_yaw = err_pix_x * -0.005
            error_z = 0.0
            error_x = 0.0
        else:
            error_yaw = err_pix_x * -0.005
            error_z = err_pix_y * -0.005
            error_x = (1.0 - td / self.target_height_ref)
            if abs(error_x) < 0.05:
                error_x = 0

        dynamic_limit_x = min(0.5 + 2.5 * abs(error_x), 3.0)
        self.pid_x.set_limits(-dynamic_limit_x, dynamic_limit_x)

        now = time.time()
        target_vel_x = self.pid_x.compute(error_x, now)
        target_vel_z = self.pid_z.compute(error_z, now)
        target_vel_yaw = self.pid_yaw.compute(error_yaw, now)

        dt = now - self.last_ctrl_time
        if dt <= 0: dt = 0.05
        self.last_ctrl_time = now

        def apply_ramp(target, current, limit, dt):
            max_step = limit * dt
            diff = target - current
            step = max(min(diff, max_step), -max_step)
            return current + step

        smooth_vel_x = apply_ramp(target_vel_x, self.last_vel_x, self.accel_limit_x, dt)
        smooth_vel_z = apply_ramp(target_vel_z, self.last_vel_z, self.accel_limit_z, dt)
        smooth_vel_yaw = apply_ramp(target_vel_yaw, self.last_vel_yaw, self.accel_limit_yaw, dt)

        smooth_vel_x = max(-self.max_vel_x, min(self.max_vel_x, smooth_vel_x))
        smooth_vel_z = max(-self.max_vel_z, min(self.max_vel_z, smooth_vel_z))
        smooth_vel_yaw = max(-self.max_vel_yaw, min(self.max_vel_yaw, smooth_vel_yaw))

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

def main(args=None):
    rclpy.init(args=args)
    node = YoloOffboardTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_vel = TwistStamped()
        node.vel_pub.publish(stop_vel)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
