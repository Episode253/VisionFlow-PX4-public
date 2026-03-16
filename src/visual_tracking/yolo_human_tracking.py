#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# https://dl.djicdn.com/downloads/neo/20240905/DJI_Neo_User_Manual_v1.0_zh-cn.pdf

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

# class LowPassFilter:
#     def __init__(self, init_val=None):
#         self.s = init_val

#     def reset(self):
#         self.s = None

#     def filter(self, x, alpha):
#         if self.s is None:
#             self.s = x
#         else:
#             self.s = alpha * x + (1.0 - alpha) * self.s
#         return self.s

# class OneEuroFilter:
#     """
#     一阶欧式滤波器 (One Euro Filter)
#     继指数移动平均滤波器 (Exponential Moving Average)的进一步尝试
#     """
#     def __init__(self, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
#         self.min_cutoff = min_cutoff
#         self.beta = beta
#         self.d_cutoff = d_cutoff

#         self.x_prev = None
#         self.last_time = None
#         self.x_lp = LowPassFilter()
#         self.dx_lp = LowPassFilter()

#     def reset(self):
#         self.x_prev = None
#         self.last_time = None
#         self.x_lp.reset()
#         self.dx_lp.reset()

#     @staticmethod
#     def _alpha(cutoff, dt):
#         if dt <= 0.0:
#             return 1.0
#         tau = 1.0 / (2.0 * math.pi * cutoff)
#         return 1.0 / (1.0 + tau / dt)

#     def update(self, x, timestamp=None):
#         if timestamp is None:
#             timestamp = time.time()
#         if self.last_time is None:
#             dt = 1.0 / 30.0
#         else:
#             dt = max(1e-6, timestamp - self.last_time)
#         self.last_time = timestamp

#         if self.x_prev is None:
#             dx = 0.0
#         else:
#             dx = (x - self.x_prev) / dt
#         self.x_prev = x

#         alpha_d = self._alpha(self.d_cutoff, dt)
#         dx_hat = self.dx_lp.filter(dx, alpha_d)

#         cutoff = self.min_cutoff + self.beta * abs(dx_hat)
#         alpha = self._alpha(cutoff, dt)

#         x_hat = self.x_lp.filter(x, alpha)
#         return x_hat

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

        # DJI Neo 起飞高度是1.20m，提醒用户无人机上方1.20m内若有障碍物，请手动调整起飞高度
        self.takeoff_height = 1.20

        # PX4-Autopilot/develop/data_plotting/local_position/odom.py 有评估策略
        self.pos_tolerance = 0.065

        # 使用EMA滤波器进行平滑
        # 平滑因子，单参数调节，相比于One Euro效率更高，而且运行很稳健
        # self.filter_x = EmaFilter(alpha=0.42)
        # self.filter_y = EmaFilter(alpha=0.45)
        # self.filter_d = EmaFilter(alpha=0.50)

        # 参数整定文件位于PX4-Autopilot/develop/visual_tracking/yolo/yolo_msg_publish/EMA_debug.py
        # 参数越小，降噪比越大但是延迟越明显

        # 超高降噪比的激进版本，对画面稳定性有略微的改善，但是速度控制输出的噪声没有明显降低
        self.filter_x = EmaFilter(alpha=0.12)
        self.filter_y = EmaFilter(alpha=0.15)
        self.filter_d = EmaFilter(alpha=0.18)

        # 使用One Euro滤波器进行平滑，实际测试发现效果相比于EMA并没有明显提升
        # self.filter_x = OneEuroFilter(min_cutoff=0.1, beta=0.01, d_cutoff=1.0)
        # self.filter_y = OneEuroFilter(min_cutoff=0.1, beta=0.01, d_cutoff=1.0)
        # self.filter_d = OneEuroFilter(min_cutoff=0.1, beta=0.02, d_cutoff=1.0)

        # 加速度限制
        self.accel_limit_x = 2.0
        self.accel_limit_z = 1.0
        self.accel_limit_yaw = 1.0

        # 速度上限
        self.max_vel_x = 2.0
        self.max_vel_z = 0.8
        # 这个速度太大容易导致失控
        self.max_vel_yaw = 0.5

        self.last_vel_x = 0.0
        self.last_vel_z = 0.0
        self.last_vel_yaw = 0.0
        self.last_ctrl_time = time.time()

        # PID 控制器参数，不允许引入D分量，避免高频噪声放大
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

        # 当前目标信息（平滑后的像素位置与框高）
        self.target_info = {'x': 0, 'y': 0, 'd': 0}

        # 锁定逻辑相关
        self.locked = False
        self.lock_frames = 0
        self.lock_confirm_frames = 40  # 连续约 2s (0.05*40) 检测到目标后认为锁定
        self.det_window = deque(maxlen=60)

        self.waiting_for_mavros = False

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        # 订阅话题：状态、位置、目标框
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/mavros/local_position/odom', self.odom_cb, qos_profile)
        self.yolo_sub = self.create_subscription(BoundingBoxes, '/yolo/BoundingBoxes', self.yolo_cb, qos_profile)

        # 发布话题：位置、速度
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        # 创建服务客户端：解锁、模式切换
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
        # 处于POSITION控制模式或者READY准备状态下，才允许进入OFFBOARD/ARM
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

        # 这里直接选择第一个检测到的目标，没有设置置信判断
        for box in msg.bounding_boxes:
            # class_id是根据YOLO设定的类别名称定义的
            if box.class_id == "target":
                self.target_captured = True
                self.lost_count = 0
                # 坐标系定义是图像左上角为(0,0)，右下角为(width,height)
                raw_x = (box.xmin + box.xmax) / 2
                raw_y = (box.ymin + box.ymax) / 2
                # 目标框高度作为距离参考，结合内参可以估算出较精准的实际距离
                raw_d = float(box.ymax - box.ymin)

                # 使用EMA滤波器进行平滑
                self.target_info['x'] = self.filter_x.update(raw_x)
                self.target_info['y'] = self.filter_y.update(raw_y)
                self.target_info['d'] = self.filter_d.update(raw_d)

                # 使用One Euro滤波器进行平滑
                # ts = self.get_clock().now().nanoseconds / 1e9
                # self.target_info['x'] = self.filter_x.update(raw_x, timestamp=ts)
                # self.target_info['y'] = self.filter_y.update(raw_y, timestamp=ts)
                # self.target_info['d'] = self.filter_d.update(raw_d, timestamp=ts)

                # 记录到最近检测窗口中，用于未锁定前的视角调整/搜索
                self.det_window.append((self.target_info['x'], self.target_info['y']))

                # 连续检测计数，用于“目标锁定”判定
                self.lock_frames += 1
                if not self.locked and self.lock_frames >= self.lock_confirm_frames:
                    self.locked = True

                detected = True
                break

        # 检测不到目标时，重置滤波器状态，防止沿用旧的平滑值
        if not detected:
            self.filter_x.reset()
            self.filter_y.reset()
            self.filter_d.reset()

            self.lock_frames = 0

    # 状态机控制的主循环
    def control_loop(self):
        # 等待 MAVROS 检测机制
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

                # 如果处于Armed状态，就一定是处于Position模式
                if is_armed :
                    self.hover_pos[2] = current_z
                    self.get_logger().info(f"Airborne detected (z={current_z:.2f}m). Holding current altitude.")
                else:
                    # 默认的起飞高度是1.5米，可按照应用场景调节
                    self.hover_pos[2] = self.takeoff_height
                    self.get_logger().info(f"On Ground. Taking off to default {self.takeoff_height}m.")

                self.flight_phase = 'TAKEOFF'

        elif self.flight_phase == 'TAKEOFF':
            self.perform_takeoff()
            curr_z = self.current_odom.pose.pose.position.z

            # 判断是否达到预定高度，有一个误差容忍范围
            if abs(curr_z - self.hover_pos[2]) < self.pos_tolerance:
                self.flight_phase = 'HOVER'
                self.get_logger().info("Phase: HOVER")

        elif self.flight_phase == 'HOVER':
            self.perform_hover()

            # 检测到第一帧目标后，进入TRACK状态
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

        # 这里的高度可能是预设起飞高度或者当前高度
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
        # 这里的计数逻辑和yolo_cb配合使用
        self.lost_count += 1
        if self.lost_count > 40:
            self.target_captured = False
            return

        tx = self.target_info['x']
        ty = self.target_info['y']
        td = self.target_info['d']

        # 加入锁定判定，防止瞬时识别造成的无人机过冲失控
        if self.locked or not self.det_window:
            ref_x = tx
            ref_y = ty
        else:
            sum_x = sum(p[0] for p in self.det_window)
            sum_y = sum(p[1] for p in self.det_window)
            n = len(self.det_window)
            ref_x = sum_x / n
            ref_y = sum_y / n

        # 计算像素误差，过滤掉小幅度抖动

        # err_pix_x = ref_x - self.center_x
        # if abs(err_pix_x) < 20:
        #     err_pix_x = 0
        # err_pix_y = ref_y - self.center_y
        # if abs(err_pix_y) < 20:
        #     err_pix_y = 0

        DEADBAND_X = 30.0
        DEADBAND_Y = 80.0

        raw_err_x = ref_x - self.center_x
        raw_err_y = ref_y - self.center_y

        if abs(raw_err_x) < DEADBAND_X:
            err_pix_x = 0.0
        else:
            sign_x = 1.0 if raw_err_x > 0 else -1.0
            # 这里做了一个差值处理，防止超出死区后反应过猛
            err_pix_x = (abs(raw_err_x) - DEADBAND_X) * sign_x

        if abs(raw_err_y) < DEADBAND_Y:
            err_pix_y = 0.0  # 只要人在画面垂直中间 1/5 的区域内，高度完全不动
        else:
            sign_y = 1.0 if raw_err_y > 0 else -1.0
            err_pix_y = (abs(raw_err_y) - DEADBAND_Y) * sign_y

        if not self.locked:
            # 未锁定阶段尝试用 YAW 调整视角
            error_yaw = err_pix_x * -0.005
            error_z = 0.0
            error_x = 0.0
        else:
            # 已锁定阶段：开启全向跟踪
            error_yaw = err_pix_x * -0.005

            # 这里直接使用经过软死区处理后的 err_pix_y
            error_z = err_pix_y * -0.005

            error_x = (1.0 - td / self.target_height_ref)
            if abs(error_x) < 0.05:
                error_x = 0

        if not self.locked:
            # 未锁定阶段尝试用 YAW 调整视角
            error_yaw = err_pix_x * -0.005
            error_z = 0.0
            error_x = 0.0
        else:
            # 已锁定阶段：开启全向跟踪
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

        # 速度硬限幅，保证不会超过安全速度
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
