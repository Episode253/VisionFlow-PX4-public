#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped
from yolo_ros_msgs.msg import BoundingBoxes

import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np
from collections import deque
import time
import signal
import sys

# ================= 配置区域 =================
DEADBAND_X = 30.0
DEADBAND_Y = 80.0
TARGET_HEIGHT = 360.0
IMG_W, IMG_H = 1280, 720

# 绘图降频：每收到 N 帧数据才重绘一次图表
# 增大此值可以显著降低 CPU 占用，提高流畅度
DRAW_SKIP_FRAMES = 3

# 速度放大倍率 (为了在单轴图上能看清速度曲线)
# 1.0 m/s 将被画成 100.0 的高度
VEL_SCALE = 100.0

plt.style.use('dark_background')
COLORS = {
    'input': '#00E5FF',  # 青色 (误差)
    'output': '#FFAB40', # 橙色 (速度)
    'deadband': '#222222',
    'text': '#FFFFFF',
    'grid': '#444444',
    'status_good': '#69F0AE',
    'status_warn': '#FFD740',
    'status_bad': '#FF5252',
    'table_head': '#333333',
    'table_row': '#1e1e1e'
}

shutdown_flag = False
def signal_handler(sig, frame):
    global shutdown_flag
    shutdown_flag = True
    print("\nShutdown signal received...")

signal.signal(signal.SIGINT, signal_handler)

class ControlAnalyzerLite(Node):
    def __init__(self):
        super().__init__('p_control_analyzer')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', self.vel_cb, qos)
        self.create_subscription(BoundingBoxes, '/yolo/BoundingBoxes', self.yolo_cb, qos)

        self.maxlen = 500
        self.start_t = time.time()
        self.update_count = 0

        self.times = deque(maxlen=self.maxlen)
        self.dist_err = deque(maxlen=self.maxlen)
        self.vel_x    = deque(maxlen=self.maxlen)
        self.pix_y    = deque(maxlen=self.maxlen)
        self.vel_z    = deque(maxlen=self.maxlen)
        self.pix_x    = deque(maxlen=self.maxlen)
        self.vel_yaw  = deque(maxlen=self.maxlen)

        self.curr_vel = {'x':0, 'z':0, 'yaw':0}
        self.curr_yolo = {'pix_x':0, 'pix_y':0, 'h':0}

        self.get_logger().info("Analyzer Lite Started (Single Axis Mode)...")

    def vel_cb(self, msg):
        self.curr_vel['x'] = msg.twist.linear.x
        self.curr_vel['z'] = msg.twist.linear.z
        self.curr_vel['yaw'] = msg.twist.angular.z

    def yolo_cb(self, msg):
        for box in msg.bounding_boxes:
            if box.class_id in ["target", "person"]:
                cx = (box.xmin + box.xmax) / 2 - (IMG_W / 2)
                cy = (box.ymin + box.ymax) / 2 - (IMG_H / 2)
                h = box.ymax - box.ymin

                self.curr_yolo['pix_x'] = cx
                self.curr_yolo['pix_y'] = cy
                self.curr_yolo['h'] = h

                self.record_data()
                break

    def record_data(self):
        self.update_count += 1
        now = time.time() - self.start_t
        self.times.append(now)

        self.dist_err.append(TARGET_HEIGHT - self.curr_yolo['h'])
        self.vel_x.append(self.curr_vel['x'])

        self.pix_y.append(self.curr_yolo['pix_y'])
        self.vel_z.append(self.curr_vel['z'])

        self.pix_x.append(self.curr_yolo['pix_x'])
        self.vel_yaw.append(self.curr_vel['yaw'])

def calculate_stats(vel_data):
    if len(vel_data) < 10: return 0.0, "Wait...", "white"
    diffs = np.diff(vel_data)
    noise = np.std(diffs)
    if noise < 0.03: return noise, "SILK", COLORS['status_good']
    elif noise < 0.08: return noise, "GOOD", COLORS['status_good']
    elif noise < 0.15: return noise, "OK", COLORS['status_warn']
    else: return noise, "BAD", COLORS['status_bad']

def main(args=None):
    global shutdown_flag
    rclpy.init(args=args)
    node = ControlAnalyzerLite()

    plt.ion()
    fig = plt.figure(figsize=(14, 8))
    fig.canvas.manager.set_window_title('P-Control Lite (Fast Mode)')

    gs = GridSpec(3, 4, figure=fig, width_ratios=[1, 1, 1, 0.6], wspace=0.1, hspace=0.3)

    ax_fwd = fig.add_subplot(gs[0, 0:3])
    ax_hgt = fig.add_subplot(gs[1, 0:3], sharex=ax_fwd)
    ax_yaw = fig.add_subplot(gs[2, 0:3], sharex=ax_fwd)
    ax_panel = fig.add_subplot(gs[:, 3])
    ax_panel.axis('off')

    axes = [ax_fwd, ax_hgt, ax_yaw]
    titles = ["1. Forward (Dist)", "2. Height (Z)", "3. Yaw (Rot)"]
    deadbands = [None, DEADBAND_Y, DEADBAND_X]

    lines_input = []
    lines_output = []

    for i, ax in enumerate(axes):
        ax.set_title(titles[i], fontsize=10, fontweight='bold', color='white', loc='left')
        ax.grid(True, linestyle=':', alpha=0.3, color=COLORS['grid'])
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

        if deadbands[i]:
            ax.axhspan(-deadbands[i], deadbands[i], color=COLORS['deadband'], alpha=0.6)

        l_in, = ax.plot([], [], '--', color=COLORS['input'], alpha=0.6, linewidth=1, label='Error [px]')

        l_out, = ax.plot([], [], '-', color=COLORS['output'], alpha=0.9, linewidth=1.5, label=f'Vel x{int(VEL_SCALE)}')

        lines_input.append(l_in)
        lines_output.append(l_out)

        if i == 0:
            ax.legend(loc='upper right', fontsize=8, framealpha=0.2)

    col_labels = ['Jitter', 'Stat']
    row_labels = ['Fwd', 'Hgt', 'Yaw']
    table_vals = [["0.00", "-"], ["0.00", "-"], ["0.00", "-"]]

    ax_panel.text(0.5, 0.9, "METRICS", ha='center', fontsize=12, fontweight='bold', color='white')
    the_table = ax_panel.table(cellText=table_vals, colLabels=col_labels, rowLabels=row_labels,
                               loc='center', bbox=[0.1, 0.6, 0.8, 0.25])

    the_table.auto_set_font_size(False)
    the_table.set_fontsize(9)
    cells = the_table.get_celld()
    for (row, col), cell in cells.items():
        cell.set_edgecolor(COLORS['grid'])
        if row == 0: cell.set_facecolor(COLORS['table_head'])
        elif col == -1: cell.set_facecolor(COLORS['table_head'])
        else: cell.set_facecolor(COLORS['table_row'])
        cell.set_text_props(color='white')

    try:
        loop_counter = 0
        while not shutdown_flag and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)

            loop_counter += 1
            if loop_counter % DRAW_SKIP_FRAMES == 0 and len(node.times) > 10:

                t = list(node.times)
                if t[-1] > 8:
                    x_min = t[-1] - 8
                    x_max = t[-1] + 0.2
                else:
                    x_min = t[0]
                    x_max = max(t[-1], 8)

                data_in = [list(node.dist_err), list(node.pix_y), list(node.pix_x)]
                data_out_raw = [list(node.vel_x), list(node.vel_z), list(node.vel_yaw)]

                data_out_scaled = [[v * VEL_SCALE for v in arr] for arr in data_out_raw]

                for i in range(3):
                    ax = axes[i]
                    lines_input[i].set_data(t, data_in[i])
                    lines_output[i].set_data(t, data_out_scaled[i])

                    ax.set_xlim(x_min, x_max)

                    if len(data_in[i]) > 0:
                        y_lim = max(150, np.max(np.abs(data_in[i])) * 1.2)
                        ax.set_ylim(-y_lim, y_lim)

                    noise, status, col = calculate_stats(data_out_raw[i])
                    cells[(i+1, 0)].get_text().set_text(f"{noise:.3f}")
                    cells[(i+1, 1)].get_text().set_text(status)
                    cells[(i+1, 1)].set_text_props(color=col, weight='bold')

                fig.canvas.draw_idle()
                fig.canvas.flush_events()

            time.sleep(0.02)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()
