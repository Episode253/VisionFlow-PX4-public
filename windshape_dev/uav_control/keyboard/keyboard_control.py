#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 Humble keyboard control for PX4 drone via MAVROS.

Keybindings:
  i/,  : forward/backward
  j/l  : turn left/right
  r/f  : ascend/descend
  5    : switch to OFFBOARD mode
  6    : arm vehicle
  7    : auto takeoff
  space: auto land
  k    : force stop
  q/z  : increase/decrease both speeds
  w/x  : increase/decrease linear speed
  e/c  : increase/decrease angular speed
  CTRL-C: quit
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

import math
import sys
import select
import termios
import tty


HELP_MSG = """
Control Your Drone!
---------------------------
Moving around:
        i (forward)
   j    k    l  (turn left / stop / turn right)
        , (backward)

r/f : ascend / descend
q/z : increase / decrease max speeds by 10%
w/x : increase / decrease only linear speed by 10%
e/c : increase / decrease only angular speed by 10%
space : auto land
5     : enable OFFBOARD mode
6     : arm vehicle
7     : auto takeoff
k     : force stop
CTRL-C to quit
"""


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control')

        # ---- MAVROS QoS profile ----
        mavros_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- Parameters ----
        self.speed = 0.2   # default linear speed (m/s)
        self.turn = 1.0    # default angular speed (rad/s)

        # ---- Orientation state ----
        self.orientation_z = 0.0
        self.orientation_w = 0.0
        self.orientation_zf = 1   # sign for Y-component projection
        self.yaw_deg = 0.0

        # ---- Drone state ----
        self.current_state = State()

        # ---- Control variables ----
        self.cmd_x = 0    # forward/backward direction
        self.cmd_z = 0    # vertical direction
        self.cmd_th = 0   # turn direction

        self.control_speed = 0.0   # actual linear velocity
        self.control_z_speed = 0.0  # actual vertical velocity
        self.control_turn = 0.0     # actual angular velocity

        # ---- Subscribers ----
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, mavros_qos
        )

        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_cb, mavros_qos
        )

        # ---- Publisher ----
        self.vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', mavros_qos
        )

        # ---- Service clients ----
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # ---- Key polling timer (50 Hz) ----
        self.settings = termios.tcgetattr(sys.stdin)
        self.poll_timer = self.create_timer(0.02, self.main_loop)

        self.get_logger().info('Keyboard control node started')

    # ---------- Callbacks ----------
    def state_cb(self, msg: State):
        self.current_state = msg

    def pose_cb(self, msg: PoseStamped):
        """Extract yaw angle and orientation sign from pose."""
        self.orientation_z = msg.pose.orientation.z
        self.orientation_w = msg.pose.orientation.w

        # Sign for Y-axis projection
        if self.orientation_z * self.orientation_w > 0:
            self.orientation_zf = 1
        else:
            self.orientation_zf = -1

        self.yaw_deg = 2 * math.acos(
            max(-1.0, min(1.0, self.orientation_w))
        ) * 180.0 / math.pi

    # ---------- Async service helpers ----------
    def call_set_mode(self, custom_mode: str):
        """Non-blocking set_mode call with spin-until-done."""
        if not self.set_mode_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('set_mode service unavailable')
            return

        req = SetMode.Request()
        req.custom_mode = custom_mode
        future = self.set_mode_client.call_async(req)

        # Spin until the future completes
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.01)

        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f'Mode set to {custom_mode}')

    def call_arm(self, value: bool):
        """Non-blocking arm call with spin-until-done."""
        if not self.arm_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('arm service unavailable')
            return

        req = CommandBool.Request()
        req.value = value
        future = self.arm_client.call_async(req)

        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.01)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Arming {"success" if value else "disarmed"}')

    # ---------- Keyboard input ----------
    def get_key(self):
        """Non-blocking key read from terminal."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    # ---------- Smoothing helpers ----------
    @staticmethod
    def smooth_step(target, current, step):
        """Ramp current toward target by at most `step`."""
        if target > current:
            return min(target, current + step)
        elif target < current:
            return max(target, current - step)
        return target

    # ---------- Main loop (timer callback) ----------
    def main_loop(self):
        key = self.get_key()

        # ---- Movement keys ----
        if key == 'i':        # forward
            self.cmd_x = 1
            self.cmd_z = 0
        elif key == ',':      # backward
            self.cmd_x = -1
            self.cmd_z = 0
        elif key == 'j':      # turn left (yaw)
            self.cmd_th = 1
            self.cmd_z = 0
        elif key == 'l':      # turn right (yaw)
            self.cmd_th = -1
            self.cmd_z = 0
        elif key == 'r':      # ascend
            self.cmd_z = 1
        elif key == 'f':      # descend
            self.cmd_z = -1

        # ---- Speed adjustment keys ----
        elif key == 'q':
            self.speed *= 1.1
            self.turn *= 1.1
            self.get_logger().info(f'speed={self.speed:.2f} turn={self.turn:.2f}')
        elif key == 'z':
            self.speed *= 0.9
            self.turn *= 0.9
            self.get_logger().info(f'speed={self.speed:.2f} turn={self.turn:.2f}')
        elif key == 'w':
            self.speed *= 1.1
            self.get_logger().info(f'speed={self.speed:.2f}')
        elif key == 'x':
            self.speed *= 0.9
            self.get_logger().info(f'speed={self.speed:.2f}')
        elif key == 'e':
            self.turn *= 1.1
            self.get_logger().info(f'turn={self.turn:.2f}')
        elif key == 'c':
            self.turn *= 0.9
            self.get_logger().info(f'turn={self.turn:.2f}')

        # ---- Action keys ----
        elif key == 'k':      # force stop
            self.cmd_x = 0
            self.cmd_z = 0
            self.cmd_th = 0
            self.control_speed = 0.0
            self.control_z_speed = 0.0
            self.control_turn = 0.0

        elif key == ' ':      # auto land
            self.get_logger().info('Vehicle Land')
            self.call_set_mode('AUTO.LAND')

        elif key == '5':      # enable OFFBOARD
            if self.current_state.mode != 'OFFBOARD':
                self.call_set_mode('OFFBOARD')
                self.get_logger().info('Offboard enabled')

        elif key == '6':      # arm
            self.call_arm(True)
            self.get_logger().info('Vehicle armed')

        elif key == '7':      # auto takeoff
            self.get_logger().info('Vehicle Takeoff')
            self.call_set_mode('AUTO.TAKEOFF')

        elif key == '\x03':   # CTRL-C
            raise KeyboardInterrupt

        # ---- Compute target velocities ----
        target_speed = self.speed * self.cmd_x
        target_z_speed = self.speed * self.cmd_z
        target_turn = self.turn * self.cmd_th

        # ---- Smooth control ramping ----
        self.control_speed = self.smooth_step(target_speed, self.control_speed, 0.02 * self.speed)
        self.control_z_speed = self.smooth_step(target_z_speed, self.control_z_speed, 0.02 * self.speed)
        self.control_turn = self.smooth_step(target_turn, self.control_turn, 0.02 * self.turn)

        # ---- Y-axis projection factor ----
        sin_yaw = abs(math.sin(self.yaw_deg / 180.0 * math.pi))
        y_factor = sin_yaw * self.orientation_zf

        # ---- Build and publish Twist ----
        twist = Twist()
        twist.linear.x = self.control_speed * math.cos(self.yaw_deg / 180.0 * math.pi)
        twist.linear.y = self.control_speed * y_factor
        twist.linear.z = self.control_z_speed
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.control_turn

        self.vel_pub.publish(twist)

    # ---------- Cleanup ----------
    def shutdown(self):
        """Publish zero velocity and restore terminal."""
        twist = Twist()
        self.vel_pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        self.get_logger().info('Node shut down')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()

    print(HELP_MSG)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
