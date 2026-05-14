#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File: official_offboard.py
ROS 2 Humble OFFBOARD takeoff & hover example

offboard 模式一键起飞定高悬停测试
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy
)


class OffboardNode(Node):

    def __init__(self):
        super().__init__('official_offboard')

        # ------------------------------------
        # Parameters
        # ------------------------------------
        self.takeoff_height = 1.5  # meters (relative takeoff)

        # ------------------------------------
        # Internal states
        # ------------------------------------
        self.current_state = State()
        self.odom_received = False
        self.initial_pose = None

        # ------------------------------------
        # MAVROS QoS profiles
        # ------------------------------------
        mavros_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ------------------------------------
        # Subscribers
        # ------------------------------------
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            mavros_qos
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_cb,
            mavros_qos
        )

        # ------------------------------------
        # Publisher
        # ------------------------------------
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            mavros_qos
        )

        # ------------------------------------
        # Service clients
        # ------------------------------------
        self.arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')

        self.set_mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')

        # ------------------------------------
        # Target pose (setpoint)
        # ------------------------------------
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'

        # ------------------------------------
        # Timers
        # ------------------------------------
        # 20 Hz setpoint publishing
        self.setpoint_timer = self.create_timer(
            0.05,
            self.publish_setpoint
        )

        # 5 s OFFBOARD / ARM check
        self.control_timer = self.create_timer(
            5.0,
            self.offboard_arm_check
        )

        self.get_logger().info('OFFBOARD node initialized')

    # ------------------------------------
    # Callbacks
    # ------------------------------------
    def state_cb(self, msg: State):
        self.current_state = msg

    def odom_cb(self, msg: Odometry):
        # Only capture initial pose once
        if self.odom_received:
            return

        self.initial_pose = msg.pose.pose
        self.odom_received = True

        # Initialize target pose based on current pose
        self.pose.pose.position.x = self.initial_pose.position.x
        self.pose.pose.position.y = self.initial_pose.position.y
        self.pose.pose.position.z = (
            self.initial_pose.position.z + self.takeoff_height
        )

        self.pose.pose.orientation = self.initial_pose.orientation

        self.get_logger().info(
            f'Initial pose received: '
            f'x={self.initial_pose.position.x:.2f}, '
            f'y={self.initial_pose.position.y:.2f}, '
            f'z={self.initial_pose.position.z:.2f}'
        )

        self.get_logger().info(
            f'Takeoff target Z: {self.pose.pose.position.z:.2f} m'
        )

    # ------------------------------------
    # Timer: publish position setpoints
    # ------------------------------------
    def publish_setpoint(self):
        if not self.current_state.connected:
            return

        if not self.odom_received:
            return

        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.local_pos_pub.publish(self.pose)

    # ------------------------------------
    # Timer: OFFBOARD & ARM logic
    # ------------------------------------
    def offboard_arm_check(self):
        if not self.odom_received:
            return

        if self.current_state.mode != 'OFFBOARD':
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            future = self.set_mode_client.call_async(req)
            future.add_done_callback(self.offboard_done_cb)

        if not self.current_state.armed:
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            future.add_done_callback(self.arm_done_cb)

    # ------------------------------------
    # Service callbacks
    # ------------------------------------
    def offboard_done_cb(self, future):
        try:
            result = future.result()
            if result and result.mode_sent:
                self.get_logger().info('OFFBOARD mode enabled')
        except Exception as e:
            self.get_logger().error(f'Failed to set OFFBOARD: {e}')

    def arm_done_cb(self, future):
        try:
            result = future.result()
            if result and result.success:
                self.get_logger().info('Vehicle armed')
        except Exception as e:
            self.get_logger().error(f'Failed to arm: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('OFFBOARD node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
