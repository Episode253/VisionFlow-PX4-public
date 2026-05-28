#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File: eight_tracking_s_curve_yaw.py
ROS 2 Humble + MAVROS OFFBOARD takeoff & smooth figure-eight tracking example
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from math import atan2, cos, sin, pi
from transforms3d.euler import quat2euler, euler2quat


class OffboardTakeoffFigureEightSmooth(Node):

    def __init__(self):
        super().__init__('figure_eight_tracking_smooth')

        self.takeoff_height = 2.0
        self.forward_distance = 4.0

        self.control_rate = 80.0

        self.eight_forward_amplitude = 2.0
        self.eight_lateral_amplitude = 1.0

        self.eight_omega = 0.30

        self.trajectory_ramp_time = 5.0

        self.hover_time = 1.5
        self.center_hover_time = 1.0
        self.forward_transition_time = 5.0

        self.pos_tolerance = 0.15

        self.yaw_mode = 'SMOOTH_TANGENT'

        self.max_yaw_rate = 0.85
        self.max_yaw_accel = 2.20
        self.yaw_blend_time = 1.4
        self.yaw_lookahead_time = 0.28
        self.yaw_natural_freq = 2.4
        self.yaw_damping_ratio = 1.05
        self.yaw_target_filter_tau = 0.18

        self.command_yaw = None
        self.command_yaw_rate = 0.0
        self.filtered_desired_yaw = None
        self.last_yaw_update_time = None

        self.current_state = State()
        self.odom = None
        self.initial_yaw = None

        self.flight_phase = 'WAIT'

        self.takeoff_xy = None
        self.takeoff_hover_pos = None
        self.eight_center = None

        self.hover_start_time = None
        self.center_hover_start_time = None
        self.forward_start_time = None
        self.forward_start_pos = None
        self.trajectory_start_time = None

        qos_odom = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

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

        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        self.arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )
        self.set_mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )

        self.setpoint_timer = self.create_timer(
            1.0 / self.control_rate,
            self.setpoint_loop
        )

        self.offboard_timer = self.create_timer(
            5.0,
            self.offboard_arm_loop
        )

    def state_cb(self, msg):
        self.current_state = msg

    def odom_cb(self, msg):
        self.odom = msg

        if self.initial_yaw is None:
            q = msg.pose.pose.orientation
            _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
            self.initial_yaw = yaw
            self.command_yaw = yaw
            self.get_logger().info(f'Initial yaw: {yaw:.3f} rad')

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

    def get_now(self):
        return self.get_clock().now()

    @staticmethod
    def smoothstep(s):
        s = min(max(s, 0.0), 1.0)
        return s * s * (3.0 - 2.0 * s)

    @staticmethod
    def smoothstep_derivative(s):
        s = min(max(s, 0.0), 1.0)
        return 6.0 * s * (1.0 - s)

    @staticmethod
    def wrap_pi(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update_yaw_smooth(self, desired_yaw, now):
        if self.command_yaw is None:
            self.command_yaw = desired_yaw
            self.command_yaw_rate = 0.0
            self.last_yaw_update_time = now
            return desired_yaw

        if self.last_yaw_update_time is None:
            dt = 1.0 / self.control_rate
        else:
            dt = (now - self.last_yaw_update_time).nanoseconds * 1e-9

        if dt <= 0.0 or dt > 0.2:
            dt = 1.0 / self.control_rate

        yaw_error = self.wrap_pi(desired_yaw - self.command_yaw)

        wn = self.yaw_natural_freq
        zeta = self.yaw_damping_ratio

        yaw_accel = wn * wn * yaw_error - 2.0 * zeta * wn * self.command_yaw_rate

        if yaw_accel > self.max_yaw_accel:
            yaw_accel = self.max_yaw_accel
        elif yaw_accel < -self.max_yaw_accel:
            yaw_accel = -self.max_yaw_accel

        self.command_yaw_rate += yaw_accel * dt

        if self.command_yaw_rate > self.max_yaw_rate:
            self.command_yaw_rate = self.max_yaw_rate
        elif self.command_yaw_rate < -self.max_yaw_rate:
            self.command_yaw_rate = -self.max_yaw_rate

        if abs(yaw_error) < 0.002 and abs(self.command_yaw_rate) < 0.02:
            self.command_yaw = desired_yaw
            self.command_yaw_rate = 0.0
        else:
            self.command_yaw = self.wrap_pi(
                self.command_yaw + self.command_yaw_rate * dt
            )

        self.last_yaw_update_time = now
        return self.command_yaw

    def filter_desired_yaw(self, raw_yaw):
        if self.filtered_desired_yaw is None:
            self.filtered_desired_yaw = raw_yaw
            return raw_yaw

        dt = 1.0 / self.control_rate
        alpha = dt / (self.yaw_target_filter_tau + dt)
        alpha = min(max(alpha, 0.0), 1.0)

        yaw_error = self.wrap_pi(raw_yaw - self.filtered_desired_yaw)
        self.filtered_desired_yaw = self.wrap_pi(
            self.filtered_desired_yaw + alpha * yaw_error
        )

        return self.filtered_desired_yaw

    def blend_yaw(self, yaw_from, yaw_to, alpha):
        alpha = min(max(alpha, 0.0), 1.0)
        return self.wrap_pi(yaw_from + alpha * self.wrap_pi(yaw_to - yaw_from))

    def set_pose_yaw(self, pose, yaw):
        qw, qx, qy, qz = euler2quat(0.0, 0.0, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

    def local_to_world(self, center, forward_offset, right_offset):
        cx, cy, cz = center
        yaw = self.initial_yaw

        x = cx + forward_offset * cos(yaw) - right_offset * sin(yaw)
        y = cy + forward_offset * sin(yaw) + right_offset * cos(yaw)
        z = cz

        return x, y, z

    def local_velocity_to_world(self, forward_velocity, right_velocity):
        yaw = self.initial_yaw

        vx = forward_velocity * cos(yaw) - right_velocity * sin(yaw)
        vy = forward_velocity * sin(yaw) + right_velocity * cos(yaw)

        return vx, vy

    def make_pose(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_now().to_msg()
        pose.header.frame_id = 'map'
        return pose

    def publish_pose(self, pose):
        self.setpoint_pub.publish(pose)

    def get_elapsed(self, now, start_time):
        return (now - start_time).nanoseconds * 1e-9

    def setpoint_loop(self):
        if self.odom is None or self.initial_yaw is None:
            return

        now = self.get_now()

        px = self.odom.pose.pose.position.x
        py = self.odom.pose.pose.position.y
        pz = self.odom.pose.pose.position.z

        pose = self.make_pose()

        self.set_pose_yaw(pose, self.initial_yaw)

        if self.flight_phase == 'WAIT':
            self.takeoff_xy = (px, py)
            self.command_yaw = self.initial_yaw
            self.command_yaw_rate = 0.0
            self.filtered_desired_yaw = self.initial_yaw
            self.last_yaw_update_time = now
            self.flight_phase = 'TAKEOFF'

            self.get_logger().info(
                f'Flight phase: TAKEOFF, lock takeoff XY=({px:.2f}, {py:.2f})'
            )

        if self.flight_phase == 'TAKEOFF':
            tx, ty = self.takeoff_xy

            pose.pose.position.x = tx
            pose.pose.position.y = ty
            pose.pose.position.z = self.takeoff_height
            self.set_pose_yaw(pose, self.initial_yaw)

            if abs(pz - self.takeoff_height) < self.pos_tolerance:
                self.takeoff_hover_pos = (tx, ty, self.takeoff_height)
                self.hover_start_time = now
                self.flight_phase = 'HOVER_AFTER_TAKEOFF'
                self.get_logger().info('Flight phase: HOVER_AFTER_TAKEOFF')

            self.publish_pose(pose)
            return

        if self.flight_phase == 'HOVER_AFTER_TAKEOFF':
            tx, ty, tz = self.takeoff_hover_pos

            pose.pose.position.x = tx
            pose.pose.position.y = ty
            pose.pose.position.z = tz
            self.set_pose_yaw(pose, self.initial_yaw)

            elapsed = self.get_elapsed(now, self.hover_start_time)

            if elapsed >= self.hover_time:
                cx = tx + self.forward_distance * cos(self.initial_yaw)
                cy = ty + self.forward_distance * sin(self.initial_yaw)
                cz = tz

                self.eight_center = (cx, cy, cz)
                self.forward_start_pos = self.takeoff_hover_pos
                self.forward_start_time = now
                self.flight_phase = 'FORWARD_TO_EIGHT_CENTER'

                self.get_logger().info(
                    'Flight phase: FORWARD_TO_EIGHT_CENTER, '
                    f'eight center=({cx:.2f}, {cy:.2f}, {cz:.2f})'
                )

            self.publish_pose(pose)
            return

        if self.flight_phase == 'FORWARD_TO_EIGHT_CENTER':
            t = self.get_elapsed(now, self.forward_start_time)
            s = self.smoothstep(t / self.forward_transition_time)

            x0, y0, z0 = self.forward_start_pos
            x1, y1, z1 = self.eight_center

            pose.pose.position.x = x0 + s * (x1 - x0)
            pose.pose.position.y = y0 + s * (y1 - y0)
            pose.pose.position.z = z0 + s * (z1 - z0)
            self.set_pose_yaw(pose, self.initial_yaw)

            dist_to_center = (
                (px - x1) ** 2 +
                (py - y1) ** 2 +
                (pz - z1) ** 2
            ) ** 0.5

            if s >= 1.0 and dist_to_center < max(0.30, self.pos_tolerance):
                self.center_hover_start_time = now
                self.flight_phase = 'HOVER_AT_EIGHT_CENTER'
                self.get_logger().info('Flight phase: HOVER_AT_EIGHT_CENTER')

            self.publish_pose(pose)
            return

        if self.flight_phase == 'HOVER_AT_EIGHT_CENTER':
            cx, cy, cz = self.eight_center

            pose.pose.position.x = cx
            pose.pose.position.y = cy
            pose.pose.position.z = cz
            self.set_pose_yaw(pose, self.initial_yaw)

            elapsed = self.get_elapsed(now, self.center_hover_start_time)

            if elapsed >= self.center_hover_time:
                self.trajectory_start_time = now
                self.command_yaw = self.initial_yaw
                self.command_yaw_rate = 0.0
                self.filtered_desired_yaw = self.initial_yaw
                self.last_yaw_update_time = now
                self.flight_phase = 'FIGURE_EIGHT'

                self.get_logger().info(
                    'Flight phase: FIGURE_EIGHT, soft-start trajectory + S-curve tangent yaw'
                )

            self.publish_pose(pose)
            return

        if self.flight_phase == 'FIGURE_EIGHT':
            elapsed = self.get_elapsed(now, self.trajectory_start_time)

            a = self.eight_forward_amplitude
            b = self.eight_lateral_amplitude
            omega = self.eight_omega

            ramp_s = elapsed / self.trajectory_ramp_time
            ramp = self.smoothstep(ramp_s)

            if 0.0 < ramp_s < 1.0:
                ramp_dot = self.smoothstep_derivative(ramp_s) / self.trajectory_ramp_time
            else:
                ramp_dot = 0.0

            theta = omega * elapsed

            sin_theta = sin(theta)
            cos_theta = cos(theta)
            sin_2theta = sin(2.0 * theta)
            cos_2theta = cos(2.0 * theta)

            forward_offset = ramp * a * sin_theta
            right_offset = ramp * b * sin_2theta

            tx, ty, tz = self.local_to_world(
                self.eight_center,
                forward_offset,
                right_offset
            )

            pose.pose.position.x = tx
            pose.pose.position.y = ty
            pose.pose.position.z = tz

            forward_velocity = (
                ramp_dot * a * sin_theta +
                ramp * a * omega * cos_theta
            )
            right_velocity = (
                ramp_dot * b * sin_2theta +
                ramp * 2.0 * b * omega * cos_2theta
            )

            vx, vy = self.local_velocity_to_world(
                forward_velocity,
                right_velocity
            )

            speed_xy = (vx ** 2 + vy ** 2) ** 0.5

            if self.yaw_mode == 'SMOOTH_TANGENT' and speed_xy > 0.03:
                yaw_elapsed = elapsed + self.yaw_lookahead_time

                yaw_ramp_s = yaw_elapsed / self.trajectory_ramp_time
                yaw_ramp = self.smoothstep(yaw_ramp_s)

                if 0.0 < yaw_ramp_s < 1.0:
                    yaw_ramp_dot = self.smoothstep_derivative(yaw_ramp_s) / self.trajectory_ramp_time
                else:
                    yaw_ramp_dot = 0.0

                yaw_theta = omega * yaw_elapsed

                yaw_sin_theta = sin(yaw_theta)
                yaw_cos_theta = cos(yaw_theta)
                yaw_sin_2theta = sin(2.0 * yaw_theta)
                yaw_cos_2theta = cos(2.0 * yaw_theta)

                yaw_forward_velocity = (
                    yaw_ramp_dot * a * yaw_sin_theta +
                    yaw_ramp * a * omega * yaw_cos_theta
                )
                yaw_right_velocity = (
                    yaw_ramp_dot * b * yaw_sin_2theta +
                    yaw_ramp * 2.0 * b * omega * yaw_cos_2theta
                )

                yaw_vx, yaw_vy = self.local_velocity_to_world(
                    yaw_forward_velocity,
                    yaw_right_velocity
                )

                tangent_yaw = atan2(yaw_vy, yaw_vx)

                yaw_blend_s = self.smoothstep(elapsed / self.yaw_blend_time)
                desired_yaw = self.blend_yaw(
                    self.initial_yaw,
                    tangent_yaw,
                    yaw_blend_s
                )

                desired_yaw = self.filter_desired_yaw(desired_yaw)
                yaw_cmd = self.update_yaw_smooth(desired_yaw, now)
            else:
                desired_yaw = self.filter_desired_yaw(self.initial_yaw)
                yaw_cmd = self.update_yaw_smooth(desired_yaw, now)

            self.set_pose_yaw(pose, yaw_cmd)

            self.publish_pose(pose)
            return


def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoffFigureEightSmooth()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
