#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File: circular_tracking.py
ROS 2 Humble OFFBOARD takeoff & circular tracking example

offboard 模式一键起飞圆形轨迹跟随测试
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from math import atan2, cos, sin, pi, sqrt
from transforms3d.euler import quat2euler, euler2quat


class OffboardTakeoffCircle(Node):

    def __init__(self):
        super().__init__('circular_tracking')

        # =============================
        # 参数
        # =============================
        self.takeoff_height = 2.0
        self.circle_radius = 3.0
        self.circle_omega = 0.4
        self.control_rate = 20.0

        self.hover_time = 1.5
        self.transition_time = 3.0     # 关键：后退过渡时间（秒）
        self.pos_tolerance = 0.01

        # =============================
        # 状态
        # =============================
        self.current_state = State()
        self.odom = None

        self.initial_yaw = None

        self.circle_center = None
        self.circle_start = None
        self.theta = 0.0

        self.flight_phase = 'WAIT'
        self.hover_start_time = None

        # 过渡用
        self.transition_start_time = None
        self.transition_start_pos = None

        # =============================
        # QoS
        # =============================
        qos_odom = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # =============================
        # Subscriber
        # =============================
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_cb,
            qos_odom
        )

        # =============================
        # Publisher
        # =============================
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # =============================
        # Services
        # =============================
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # =============================
        # 定时器
        # =============================
        self.setpoint_timer = self.create_timer(
            1.0 / self.control_rate,
            self.setpoint_loop
        )

        self.offboard_timer = self.create_timer(
            5.0,
            self.offboard_arm_loop
        )

        self.get_logger().info('Offboard takeoff + smooth transition + circle started')

    # =====================================================
    # 回调
    # =====================================================
    def state_cb(self, msg):
        self.current_state = msg

    def odom_cb(self, msg):
        self.odom = msg

        if self.initial_yaw is None:
            q = msg.pose.pose.orientation
            _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
            self.initial_yaw = yaw
            self.get_logger().info(f'Initial yaw: {yaw:.3f} rad')

    # =====================================================
    # OFFBOARD / ARM 管理
    # =====================================================
    def offboard_arm_loop(self):
        if not self.current_state.connected:
            return

        if self.current_state.mode != 'OFFBOARD':
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.set_mode_client.call_async(req)

        if not self.current_state.armed:
            req = CommandBool.Request()
            req.value = True
            self.arming_client.call_async(req)

    # =====================================================
    # 主控制逻辑
    # =====================================================
    def setpoint_loop(self):
        if self.odom is None or self.initial_yaw is None:
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        px = self.odom.pose.pose.position.x
        py = self.odom.pose.pose.position.y
        pz = self.odom.pose.pose.position.z

        # 默认保持初始航向
        qw, qx, qy, qz = euler2quat(0.0, 0.0, self.initial_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        # =============================
        # WAIT
        # =============================
        if self.flight_phase == 'WAIT':
            self.flight_phase = 'TAKEOFF'
            self.get_logger().info('Flight phase: TAKEOFF')

        # =============================
        # TAKEOFF
        # =============================
        if self.flight_phase == 'TAKEOFF':
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = self.takeoff_height

            if abs(pz - self.takeoff_height) < self.pos_tolerance:
                self.circle_center = (px, py, self.takeoff_height)
                self.hover_start_time = self.get_clock().now()
                self.flight_phase = 'HOVER_AFTER_TAKEOFF'
                self.get_logger().info('Flight phase: HOVER_AFTER_TAKEOFF')

            self.setpoint_pub.publish(pose)
            return

        # =============================
        # HOVER
        # =============================
        if self.flight_phase == 'HOVER_AFTER_TAKEOFF':
            cx, cy, cz = self.circle_center
            pose.pose.position.x = cx
            pose.pose.position.y = cy
            pose.pose.position.z = cz

            elapsed = (self.get_clock().now() - self.hover_start_time).nanoseconds * 1e-9
            if elapsed > self.hover_time:
                yaw = self.initial_yaw
                self.circle_start = (
                    cx - self.circle_radius * cos(yaw),
                    cy - self.circle_radius * sin(yaw),
                    cz
                )

                self.transition_start_pos = (cx, cy, cz)
                self.transition_start_time = self.get_clock().now()
                self.flight_phase = 'TRANSITION_TO_CIRCLE_START'

                self.get_logger().info('Flight phase: TRANSITION_TO_CIRCLE_START')

            self.setpoint_pub.publish(pose)
            return

        # =============================
        # 平滑后退过渡
        # =============================
        if self.flight_phase == 'TRANSITION_TO_CIRCLE_START':
            t = (self.get_clock().now() - self.transition_start_time).nanoseconds * 1e-9
            s = min(max(t / self.transition_time, 0.0), 1.0)

            x0, y0, z0 = self.transition_start_pos
            x1, y1, z1 = self.circle_start

            pose.pose.position.x = x0 + s * (x1 - x0)
            pose.pose.position.y = y0 + s * (y1 - y0)
            pose.pose.position.z = z0 + s * (z1 - z0)

            if s >= 1.0:
                self.theta = pi
                self.flight_phase = 'CIRCLE'
                self.get_logger().info('Flight phase: CIRCLE')

            self.setpoint_pub.publish(pose)
            return

        # =============================
        # CIRCLE
        # =============================
        if self.flight_phase == 'CIRCLE':
            cx, cy, cz = self.circle_center

            self.theta += self.circle_omega / self.control_rate
            self.theta %= 2.0 * pi

            tx = cx + self.circle_radius * cos(self.theta)
            ty = cy + self.circle_radius * sin(self.theta)

            yaw_to_center = atan2(cy - ty, cx - tx)
            qw, qx, qy, qz = euler2quat(0.0, 0.0, yaw_to_center)

            pose.pose.position.x = tx
            pose.pose.position.y = ty
            pose.pose.position.z = cz
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            self.setpoint_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoffCircle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
