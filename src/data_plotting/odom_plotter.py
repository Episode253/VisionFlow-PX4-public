#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from collections import deque
import time
import numpy as np
import signal
import sys

plt.style.use('dark_background')
COLORS = {
    'x': '#FF5252',
    'y': '#69F0AE',
    'z': '#448AFF',
    'text': '#E0E0E0',
    'table_head': '#333333',
    'table_row': '#1e1e1e'
}
SCALE = 1000.0
MARGIN = 200

# PLOT_LEN: 可视化保留的历史长度 (例如 2000 个点，约 40-60秒)
PLOT_LEN = 2000

# CALC_LEN: 仅使用最近的 N 个点进行统计计算 (例如 300 个点，约 5-6秒)
# 这样可以排除旧数据的干扰，只评估当前的稳态精度
CALC_LEN = 300

shutdown_flag = False

def signal_handler(sig, frame):
    global shutdown_flag
    shutdown_flag = True
    print("\nShutdown signal received...")

signal.signal(signal.SIGINT, signal_handler)

class OdomPlotter(Node):
    def __init__(self):
        super().__init__('odom_plotter')
        self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )
        self.x = deque(maxlen=PLOT_LEN)
        self.y = deque(maxlen=PLOT_LEN)
        self.z = deque(maxlen=PLOT_LEN)
        self.t = deque(maxlen=PLOT_LEN)
        self.start_time = time.time()
        self.get_logger().info(f'Odom plotter started. History: {PLOT_LEN}, Calc Window: {CALC_LEN}')

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        now = time.time() - self.start_time
        self.x.append(p.x)
        self.y.append(p.y)
        self.z.append(p.z)
        self.t.append(now)

def calculate_stats(data):
    """
    计算统计数据
    """
    if len(data) < 2: return None
    arr = np.array(data)
    mean = np.mean(arr)
    std = np.std(arr)

    return {
        'mean': mean,
        'std': std,
        'max': np.max(arr),
        'min': np.min(arr),
        'max_dev': np.max(np.abs(arr - mean)),
        'sigma_3': 3 * std
    }

def main():
    global shutdown_flag
    rclpy.init()
    node = OdomPlotter()

    plt.ion()
    fig = plt.figure(figsize=(16, 9))
    fig.canvas.manager.set_window_title('PX4 Odometry Analysis Dashboard')

    gs = GridSpec(2, 4, figure=fig, height_ratios=[1, 1.2], wspace=0.25, hspace=0.3)

    gs_top = GridSpec(2, 3, figure=fig, height_ratios=[1, 1.2], hspace=0.3, wspace=0.2)
    ax_x = fig.add_subplot(gs_top[0, 0])
    ax_y = fig.add_subplot(gs_top[0, 1])
    ax_z = fig.add_subplot(gs_top[0, 2])

    lines = {}
    axes_map = {'x': ax_x, 'y': ax_y, 'z': ax_z}

    for axis_key, ax in axes_map.items():
        c = COLORS[axis_key]
        lines[axis_key], = ax.plot([], [], color=c, linewidth=1.5, label=f'{axis_key.upper()}')
        ax.set_title(f'{axis_key.upper()}-Axis Deviation', fontsize=11, color=c, fontweight='bold')
        ax.grid(True, linestyle=':', alpha=0.3, color='#555555')
        ax.set_ylabel('mm', fontsize=9)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

    ax_3d = fig.add_subplot(gs[1, 0:2], projection='3d')
    line_3d, = ax_3d.plot([], [], [], color='white', lw=1, alpha=0.7)
    ax_3d.set_title('Spatial Trajectory (3D)', fontsize=11, pad=-5, color='white')

    ax_3d.set_xlabel('X [mm]', color=COLORS['x'], labelpad=5)
    ax_3d.set_ylabel('Y [mm]', color=COLORS['y'], labelpad=5)
    ax_3d.set_zlabel('Z [mm]', color=COLORS['z'], labelpad=5)

    ax_3d.tick_params(axis='x', colors='gray')
    ax_3d.tick_params(axis='y', colors='gray')
    ax_3d.tick_params(axis='z', colors='gray')

    pane_color = (0.15, 0.15, 0.15, 1.0)
    ax_3d.xaxis.set_pane_color(pane_color)
    ax_3d.yaxis.set_pane_color(pane_color)
    ax_3d.zaxis.set_pane_color(pane_color)
    ax_3d.grid(False)

    ax_stats = fig.add_subplot(gs[1, 2:])
    ax_stats.axis('off')

    ax_stats.text(0.0, 1.0, "REAL-TIME STATISTICS", fontsize=14,
                  fontweight='bold', color='white', transform=ax_stats.transAxes)

    text_samples = ax_stats.text(0.0, 0.92, "Initializing...", fontsize=10,
                                 color='#AAAAAA', family='monospace', transform=ax_stats.transAxes)

    col_labels = ['Mean (mm)', 'Std Dev', '3-Sigma (99.7%)']
    row_labels = ['X Axis', 'Y Axis', 'Z Axis']
    table_vals = [[0.0]*3 for _ in range(3)]

    the_table = ax_stats.table(cellText=table_vals, colLabels=col_labels, rowLabels=row_labels,
                               loc='center', bbox=[0.0, 0.45, 1.0, 0.4])

    the_table.auto_set_font_size(False)
    the_table.set_fontsize(10)

    cells = the_table.get_celld()
    for (row, col), cell in cells.items():
        cell.set_edgecolor('#444444')
        cell.set_linewidth(0.5)
        cell.set_height(0.12)
        if row == 0:
            cell.set_facecolor(COLORS['table_head'])
            cell.set_text_props(weight='bold', color='white')
        elif col == -1:
            cell.set_facecolor(COLORS['table_head'])
            color_key = ['x', 'y', 'z'][row-1]
            cell.set_text_props(weight='bold', color=COLORS[color_key])
        else:
            cell.set_facecolor(COLORS['table_row'])
            cell.set_text_props(color='white', family='monospace')

    ax_stats.text(0.0, 0.35, "RECOMMENDED POS_TOLERANCE", fontsize=10,
                  color='#AAAAAA', transform=ax_stats.transAxes)

    text_tolerance = ax_stats.text(0.0, 0.15, "--- m", fontsize=40,
                                   fontweight='bold', color=COLORS['y'], transform=ax_stats.transAxes)

    ax_stats.text(0.0, 0.05, f"Based on 3σ of last {CALC_LEN} samples.\nDynamic noise analysis.",
                  fontsize=9, color='#888888', style='italic', transform=ax_stats.transAxes)

    hlines = {'x': [], 'y': [], 'z': []}

    try:
        while not shutdown_flag:
            rclpy.spin_once(node, timeout_sec=0.01)

            if len(node.x) > 1:
                t = list(node.t)
                full_data_map = {
                    'x': [v * SCALE for v in node.x],
                    'y': [v * SCALE for v in node.y],
                    'z': [v * SCALE for v in node.z]
                }

                for axis in ['x', 'y', 'z']:
                    lines[axis].set_data(t, full_data_map[axis])
                    axes_map[axis].relim()
                    axes_map[axis].autoscale_view()

                    for line in hlines[axis]: line.remove()
                    hlines[axis].clear()

                line_3d.set_data(full_data_map['x'], full_data_map['y'])
                line_3d.set_3d_properties(full_data_map['z'])

                ax_3d.set_xlim(min(full_data_map['x'])-MARGIN, max(full_data_map['x'])+MARGIN)
                ax_3d.set_ylim(min(full_data_map['y'])-MARGIN, max(full_data_map['y'])+MARGIN)
                ax_3d.set_zlim(min(full_data_map['z'])-MARGIN, max(full_data_map['z'])+MARGIN)

                stats = {}
                for i, axis in enumerate(['x', 'y', 'z']):

                    calc_data = full_data_map[axis][-CALC_LEN:]

                    stats[axis] = calculate_stats(calc_data)
                    s = stats[axis]

                    hlines[axis].append(axes_map[axis].axhline(s['mean'], color='white', linestyle='--', alpha=0.3, lw=1))

                    cells[(i+1, 0)].get_text().set_text(f"{s['mean']:.2f}")
                    cells[(i+1, 1)].get_text().set_text(f"{s['std']:.2f}")
                    cells[(i+1, 2)].get_text().set_text(f"{s['sigma_3']:.2f}")

                duration = t[-1] - t[-min(len(t), CALC_LEN)]
                text_samples.set_text(f"Total History: {len(t)} | Analysis Window: Last {min(len(t), CALC_LEN)} samples (~{duration:.1f}s)")

                xy_sigma3 = max(stats['x']['sigma_3'], stats['y']['sigma_3'])
                z_sigma3 = stats['z']['sigma_3']
                rec_val_m = max(xy_sigma3, z_sigma3) / 1000.0

                text_tolerance.set_text(f"{rec_val_m:.3f} m")

                fig.canvas.draw_idle()
                fig.canvas.flush_events()

            time.sleep(0.05)

    except Exception as e:
        print(f"Main Loop Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass
        plt.close('all')
        print("Clean exit.")

if __name__ == '__main__':
    main()
