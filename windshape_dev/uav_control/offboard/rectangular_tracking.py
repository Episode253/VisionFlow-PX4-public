#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File: rectangular_tracking.py
ROS 2 Humble OFFBOARD rectangular trajectory tracking (smoothstep 版本)

飞行轨迹：
  1. TAKEOFF    — 原地起飞到目标高度
  2. HOVER      — 悬停片刻
  3. LEG_0~3    — 逐段飞行 A→B, B→C, C→D, D→A
                 使用三次多项式 smoothstep 插值生成 S 型速度曲线轨迹
                 同时发送位置+速度前馈到 PX4（PositionTarget type_mask=2048）
                 每段之间在角点悬停 corner_pause 秒
  4. RETURN     — 飞回起飞点上空
  5. LAND       — 降落

特点：
  - 机头保持初始朝向不变
  - 飞行段使用三次多项式 smoothstep 插值（Hermite 型，S 型速度曲线，起止速度为零）
  - PositionTarget 同时发送位置+速度（type_mask=2048）
  - 角点悬停，避免 overshoot
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from math import atan2, cos, sin, sqrt
from transforms3d.euler import quat2euler, euler2quat


class RectangularTracking(Node):

    def __init__(self):
        super().__init__('rectangular_tracking')

        # =============================================
        # 可调参数
        # =============================================
        self.takeoff_height = 2.0      # 起飞目标高度 (m)

        # 矩形尺寸：以起飞点为矩形中心
        self.rect_half_x = 3.0         # 矩形半长 (前后方向, m)
        self.rect_half_y = 2.0         # 矩形半宽 (左右方向, m)

        self.hover_time = 1.5          # 起飞后悬停时间 (s)
        self.corner_pause = 2.0        # 角点悬停时间 (s)
        self.leg_speed = 0.5           # 飞行速度 (m/s) — 用于计算段时长
        self.control_rate = 20.0       # 控制频率 (Hz)
        self.pos_tolerance = 0.15      # 到达判断阈值 (m)

        # =============================================
        # 内部状态
        # =============================================
        self.current_state = State()
        self.odom = None
        self.initial_yaw = None
        self.takeoff_pos = None        # 起飞点 (x, y)
        self.flight_phase = 'WAIT'
        self.phase_start_time = None

        # smoothstep 轨迹用 — 记录当前飞行段的起点/终点/开始时间
        self.leg_start_point = None    # (x, y, z)
        self.leg_end_point = None      # (x, y, z)
        self.leg_start_time = None     # ros2 Time
        self.leg_duration = 0.0        # 秒

        # 矩形四角 (A,B,C,D) 和当前段索引
        self.corners = []
        self.current_leg = 0           # 0=A→B, 1=B→C, 2=C→D, 3=D→A

        # =============================================
        # QoS
        # =============================================
        qos_odom = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # =============================================
        # Subscribers
        # =============================================
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/mavros/local_position/odom', self.odom_cb, qos_odom
        )

        # =============================================
        # Publishers
        #  - setpoint_pub: PoseStamped 用于起飞/悬停/降落
        #  - raw_pub:      PositionTarget 用于飞行段 (smoothstep + 速度馈送)
        # =============================================
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )
        self.raw_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10
        )

        # =============================================
        # Service clients
        # =============================================
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # =============================================
        # Timers
        # =============================================
        self.setpoint_timer = self.create_timer(
            1.0 / self.control_rate, self.setpoint_loop
        )
        self.offboard_timer = self.create_timer(5.0, self.offboard_arm_loop)

        self.get_logger().info('Rectangular tracking node started (smoothstep + PositionTarget)')
        self.get_logger().info(
            f'Rectangle: {self.rect_half_x*2:.0f}m x {self.rect_half_y*2:.0f}m, '
            f'speed={self.leg_speed}m/s, height={self.takeoff_height}m, '
            f'corner_pause={self.corner_pause}s'
        )

    # =================================================
    # 回调
    # =================================================
    def state_cb(self, msg):
        self.current_state = msg

    def odom_cb(self, msg):
        self.odom = msg
        if self.initial_yaw is None:
            q = msg.pose.pose.orientation
            _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
            self.initial_yaw = yaw
            self.get_logger().info(f'Initial yaw: {yaw:.3f} rad')

    # =================================================
    # OFFBOARD / ARM 管理
    # =================================================
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

    # =================================================
    # 辅助：生成矩形四角点
    # =================================================
    def compute_corners(self, cx, cy):
        """以 (cx,cy) 为中心，initial_yaw 为前进方向"""
        yaw = self.initial_yaw
        c = cos(yaw)
        s = sin(yaw)
        hx = self.rect_half_x
        hy = self.rect_half_y

        A = (cx + hx * c - hy * s, cy + hx * s + hy * c)  # 前左
        B = (cx + hx * c + hy * s, cy + hx * s - hy * c)  # 前右
        C = (cx - hx * c + hy * s, cy - hx * s - hy * c)  # 后右
        D = (cx - hx * c - hy * s, cy - hx * s + hy * c)  # 后左

        return [A, B, C, D]

    # =================================================
    # 核心：三次多项式 smoothstep 轨迹插值
    #
    # 方法：使用三次 Hermite 多项式 s(τ) = 3τ² - 2τ³ 对两点间位置进行插值，
    #       其一阶导数 ds/dτ = 6τ(1-τ) 给出 S 型速度曲线（起止为零，中点最大）。
    #       这是一种开环轨迹生成方法，不依赖在线优化或 MPC。
    # =================================================
    def smoothstep_traj(self, t, T, start, end):
        """
        三次多项式 smoothstep 轨迹插值（Hermite 型，S 型速度曲线）

        参数:
            t:     当前段内时间 (s)
            T:     段总时长 (s)
            start: 起点 (x, y, z)
            end:   终点 (x, y, z)

        返回:
            (px, py, pz, vx, vy, vz)
              位置和速度均为 ROS ENU 坐标系
        """
        if T <= 0:
            return end[0], end[1], end[2], 0.0, 0.0, 0.0

        tau = max(0.0, min(t / T, 1.0))
        s = tau * tau * (3.0 - 2.0 * tau)          # 位置比例 [0,1]
        ds = 6.0 * tau * (1.0 - tau) / T           # 速度比例 [0, v_max, 0]

        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dz = end[2] - start[2]

        px = start[0] + s * dx
        py = start[1] + s * dy
        pz = start[2] + s * dz
        vx = ds * dx
        vy = ds * dy
        vz = ds * dz

        return px, py, pz, vx, vy, vz

    # =================================================
    # 发送 PositionTarget (setpoint_raw)
    # =================================================
    def publish_raw_setpoint(self, px, py, pz, vx, vy, vz):
        """
        发布 PositionTarget 到 /mavros/setpoint_raw/local

        MAVROS 会将 ROS ENU 坐标转为 MAVLink NED 坐标。
        这里 px/py/pz/vx/vy/vz 均为 ROS ENU 坐标系。
        """
        cmd = PositionTarget()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED  # = 1
        # type_mask = 2048: 位置+速度有效, yaw 有效, yaw_rate 忽略
        cmd.type_mask = 2048

        # ROS ENU → MAVROS 会翻成 NED 给 PX4
        cmd.position.x = float(px)
        cmd.position.y = float(py)
        cmd.position.z = float(pz)
        cmd.velocity.x = float(vx)
        cmd.velocity.y = float(vy)
        cmd.velocity.z = float(vz)

        cmd.yaw = float(self.initial_yaw) if self.initial_yaw is not None else 0.0
        cmd.yaw_rate = 0.0

        self.raw_pub.publish(cmd)

    # =================================================
    # 辅助：开始新的一段飞行（smoothstep 版）
    # =================================================
    def _start_new_leg_smooth(self, start_corner_idx):
        """
        初始化新飞行段：
          start = corners[start_corner_idx]
          end   = corners[(start_corner_idx+1) % 4]
        根据距离和 leg_speed 计算段时长 T
        """
        sx, sy = self.corners[start_corner_idx]
        ex, ey = self.corners[(start_corner_idx + 1) % 4]
        sz = self.takeoff_height

        self.leg_start_point = (sx, sy, sz)
        self.leg_end_point = (ex, ey, sz)

        dist = sqrt((ex - sx) ** 2 + (ey - sy) ** 2)
        # T = dist / v_avg, smoothstep 的 v_max = 1.5 * v_avg
        self.leg_duration = max(dist / self.leg_speed, 1.0)
        self.leg_start_time = self.get_clock().now()

        label = ['A', 'B', 'C', 'D'][start_corner_idx]
        next_label = ['B', 'C', 'D', 'A'][start_corner_idx]
        self.get_logger().info(
            f'LEG {start_corner_idx}: {label}→{next_label}, '
            f'dist={dist:.2f}m, T={self.leg_duration:.2f}s, '
            f'v_avg={self.leg_speed:.2f}m/s, v_max={1.5*self.leg_speed:.2f}m/s'
        )

    # =================================================
    # 主控制循环
    # =================================================
    def setpoint_loop(self):
        if self.odom is None or self.initial_yaw is None:
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        px = self.odom.pose.pose.position.x
        py = self.odom.pose.pose.position.y
        pz = self.odom.pose.pose.position.z

        # 获取当前速度（用于调试）
        vx = self.odom.twist.twist.linear.x
        vy = self.odom.twist.twist.linear.y
        speed = sqrt(vx**2 + vy**2)

        # 每 2 秒打印一次状态
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if not hasattr(self, '_last_speed_log'):
            self._last_speed_log = 0.0
        if now_s - self._last_speed_log > 2.0:
            self._last_speed_log = now_s
            self.get_logger().info(
                f'Speed: {speed:.2f} m/s  '
                f'(vx={vx:.2f}, vy={vy:.2f})  '
                f'phase={self.flight_phase}  '
                f'pos=({px:.2f}, {py:.2f}, {pz:.2f})'
            )

        # 默认保持初始航向
        qw, qx, qy, qz = euler2quat(0.0, 0.0, self.initial_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        # =============================
        # WAIT → TAKEOFF
        # =============================
        if self.flight_phase == 'WAIT':
            self.flight_phase = 'TAKEOFF'
            self.takeoff_pos = (px, py)
            self.get_logger().info(
                f'Takeoff position: ({px:.2f}, {py:.2f}), phase: TAKEOFF'
            )

        # =============================
        # TAKEOFF — 用 PoseStamped 让 PX4 处理起飞
        # =============================
        if self.flight_phase == 'TAKEOFF':
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = self.takeoff_height

            if abs(pz - self.takeoff_height) < self.pos_tolerance:
                self.phase_start_time = self.get_clock().now()
                self.flight_phase = 'HOVER'
                self.get_logger().info('Phase: HOVER')

            self.setpoint_pub.publish(pose)
            return

        # =============================
        # HOVER — PoseStamped 定点悬停
        # =============================
        if self.flight_phase == 'HOVER':
            tx, ty = self.takeoff_pos
            pose.pose.position.x = tx
            pose.pose.position.y = ty
            pose.pose.position.z = self.takeoff_height

            elapsed = (self.get_clock().now() - self.phase_start_time).nanoseconds * 1e-9
            if elapsed > self.hover_time:
                # 计算矩形四角
                self.corners = self.compute_corners(tx, ty)
                self.current_leg = 0
                # 先从起飞点飞到第一个角点 A（过渡段）
                self.flight_phase = 'TO_CORNER_A'
                ax, ay = self.corners[0]
                self.leg_start_point = (tx, ty, self.takeoff_height)
                self.leg_end_point = (ax, ay, self.takeoff_height)
                dist = sqrt((ax - tx) ** 2 + (ay - ty) ** 2)
                self.leg_duration = max(dist / self.leg_speed, 1.0)
                self.leg_start_time = self.get_clock().now()
                self.get_logger().info(
                    f'Phase: TO_CORNER_A, dist={dist:.2f}m, T={self.leg_duration:.2f}s'
                )
            else:
                self.setpoint_pub.publish(pose)
            return

        # =============================
        # TO_CORNER_A — smoothstep 从起飞点飞到 A 角
        # 仅在无人机已在 A 角附近时才进入 LEG
        # =============================
        if self.flight_phase == 'TO_CORNER_A':
            t_elapsed = (self.get_clock().now() - self.leg_start_time).nanoseconds * 1e-9
            T = self.leg_duration

            if t_elapsed >= T:
                # smoothstep 计时到，发终点但等待无人机实际到达
                ax, ay = self.corners[0]
                self.publish_raw_setpoint(ax, ay, self.takeoff_height, 0.0, 0.0, 0.0)

                real_dist = sqrt((ax - px) ** 2 + (ay - py) ** 2)
                if real_dist < self.pos_tolerance:
                    self.flight_phase = 'LEG'
                    self.current_leg = 0
                    self._start_new_leg_smooth(self.current_leg)
                return

            gx, gy, gz, gvx, gvy, gvz = self.smoothstep_traj(
                t_elapsed, T, self.leg_start_point, self.leg_end_point
            )
            self.publish_raw_setpoint(gx, gy, gz, gvx, gvy, gvz)
            return

        # =============================
        # LEG — smoothstep 轨迹（clik_main 风格）
        # 时间到后，如果无人机未到位则悬停在终点等待到位后才切换
        # =============================
        if self.flight_phase == 'LEG':
            t_elapsed = (self.get_clock().now() - self.leg_start_time).nanoseconds * 1e-9
            T = self.leg_duration

            if t_elapsed >= T:
                # 段计时结束：持续发送终点（速度=0），等待无人机到位
                ex, ey, ez = self.leg_end_point
                self.publish_raw_setpoint(ex, ey, ez, 0.0, 0.0, 0.0)

                # 目标角点
                target_idx = (self.current_leg + 1) % 4
                tx, ty = self.corners[target_idx]
                real_dist = sqrt((tx - px) ** 2 + (ty - py) ** 2)

                if real_dist < self.pos_tolerance:
                    if self.current_leg < 3:
                        self.flight_phase = 'CORNER_PAUSE'
                        self.phase_start_time = self.get_clock().now()
                        self.get_logger().info(
                            f'Phase: CORNER_PAUSE at corner {self.current_leg+1}'
                        )
                    else:
                        # 四段飞完，进入 RETURN
                        self.flight_phase = 'RETURN'
                        sx, sy = self.corners[0]
                        sz = self.takeoff_height
                        tx_ret, ty_ret = self.takeoff_pos
                        self.leg_start_point = (sx, sy, sz)
                        self.leg_end_point = (tx_ret, ty_ret, sz)
                        dist = sqrt((tx_ret - sx) ** 2 + (ty_ret - sy) ** 2)
                        self.leg_duration = max(dist / self.leg_speed, 1.0)
                        self.leg_start_time = self.get_clock().now()
                        self.get_logger().info(
                            f'Phase: RETURN, dist={dist:.2f}m, T={self.leg_duration:.2f}s'
                        )
                return

            # smoothstep 计算并发布
            gx, gy, gz, gvx, gvy, gvz = self.smoothstep_traj(
                t_elapsed, T, self.leg_start_point, self.leg_end_point
            )
            self.publish_raw_setpoint(gx, gy, gz, gvx, gvy, gvz)
            return

        # =============================
        # CORNER_PAUSE — 角点悬停
        # 悬停期间持续发送角点位置，确保无人机稳定后再进入下一段
        # =============================
        if self.flight_phase == 'CORNER_PAUSE':
            next_corner_idx = (self.current_leg + 1) % 4
            cx, cy = self.corners[next_corner_idx]
            pose.pose.position.x = cx
            pose.pose.position.y = cy
            pose.pose.position.z = self.takeoff_height

            elapsed = (self.get_clock().now() - self.phase_start_time).nanoseconds * 1e-9
            if elapsed > self.corner_pause:
                # 检查无人机是否确实在角点附近
                real_dist = sqrt((cx - px) ** 2 + (cy - py) ** 2)
                if real_dist < self.pos_tolerance * 2.0:  # 放宽一点阈值
                    self.current_leg += 1
                    self.flight_phase = 'LEG'
                    self._start_new_leg_smooth(self.current_leg)
                # 如果没到位，继续悬停等待（不发日志避免刷屏）
            else:
                self.setpoint_pub.publish(pose)
            return

        # =============================
        # RETURN — smoothstep 飞回起飞点
        # =============================
        if self.flight_phase == 'RETURN':
            t_elapsed = (self.get_clock().now() - self.leg_start_time).nanoseconds * 1e-9
            T = self.leg_duration

            if t_elapsed >= T:
                ex, ey, _ = self.leg_end_point
                self.publish_raw_setpoint(ex, ey, self.takeoff_height, 0.0, 0.0, 0.0)

                tx, ty = self.takeoff_pos
                real_dist = sqrt((tx - px) ** 2 + (ty - py) ** 2)
                if real_dist < self.pos_tolerance:
                    self.flight_phase = 'LAND'
                    self.get_logger().info('Phase: LAND')
                return

            gx, gy, gz, gvx, gvy, gvz = self.smoothstep_traj(
                t_elapsed, T, self.leg_start_point, self.leg_end_point
            )
            self.publish_raw_setpoint(gx, gy, gz, gvx, gvy, gvz)
            return

        # =============================
        # LAND — PoseStamped 原地降落
        # =============================
        if self.flight_phase == 'LAND':
            tx, ty = self.takeoff_pos
            pose.pose.position.x = tx
            pose.pose.position.y = ty
            pose.pose.position.z = 0.0

            qw, qx, qy, qz = euler2quat(0.0, 0.0, self.initial_yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            if pz < 0.1:
                self.get_logger().info('Landed! Disarming...')
                req = CommandBool.Request()
                req.value = False
                self.arming_client.call_async(req)
                self.flight_phase = 'DONE'

            self.setpoint_pub.publish(pose)
            return

        # =============================
        # DONE
        # =============================
        if self.flight_phase == 'DONE':
            self.setpoint_timer.cancel()
            self.offboard_timer.cancel()
            self.get_logger().info('Mission complete.')


def main(args=None):
    rclpy.init(args=args)
    node = RectangularTracking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
