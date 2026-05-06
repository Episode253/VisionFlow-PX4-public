#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from yolo_ros_msgs.msg import BoundingBoxes

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np
from collections import deque
import threading
import time

# CURRENT_ALPHA_X = 0.42
# CURRENT_ALPHA_Y = 0.45
# CURRENT_ALPHA_D = 0.50

CURRENT_ALPHA_X = 0.12
CURRENT_ALPHA_Y = 0.15
CURRENT_ALPHA_D = 0.18

TUNING_MODE = 'sport'

class EmaFilterLocal:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.value = None

    def update(self, new_val):
        if new_val is None: return None
        if self.value is None:
            self.value = new_val
        else:
            self.value = self.alpha * new_val + (1 - self.alpha) * self.value
        return self.value

class StableRecommender:
    """
    防止推荐参数跳变，计算最近 30 次推荐值的平均数
    """
    def __init__(self):
        self.history = deque(maxlen=30)

    def update(self, new_rec):
        self.history.append(new_rec)
        if len(self.history) < 1: return new_rec
        return np.mean(self.history)

class AxisData:
    def __init__(self, name, current_alpha):
        self.name = name
        self.current_alpha = current_alpha
        self.raw = []
        self.sim_curr = []
        self.sim_best = []

        self.rec_engine = StableRecommender()
        self.rec_alpha_stable = 0.4

        self.lag = 0
        self.nrr = 0
        self.score = 0
        self.status = ""

class AdvancedTuningDashboard(Node):
    def __init__(self):
        super().__init__('three_axis_tuning')
        self.maxlen = 100

        self.raw_x = deque(maxlen=self.maxlen)
        self.raw_y = deque(maxlen=self.maxlen)
        self.raw_d = deque(maxlen=self.maxlen)
        self.times = deque(maxlen=self.maxlen)

        self.start_t = time.time()
        self.connected = False

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(BoundingBoxes, '/yolo/BoundingBoxes', self.cb, qos)

    def cb(self, msg):
        if not self.connected:
            self.connected = True

        now = time.time() - self.start_t
        for box in msg.bounding_boxes:
            cid = box.class_id.strip()
            if cid == "target" or cid == "person":
                cx = (box.xmin + box.xmax) / 2.0
                cy = (box.ymin + box.ymax) / 2.0
                cd = float(box.ymax - box.ymin)

                self.raw_x.append(cx)
                self.raw_y.append(cy)
                self.raw_d.append(cd)
                self.times.append(now)
                break

def simulate_stream(data_list, alpha):
    f = EmaFilterLocal(alpha)
    return [f.update(v) for v in data_list]

def evaluate_performance(raw, filt, dt):
    raw = np.array(raw)
    filt = np.array(filt)
    if len(raw) < 5: return 0,0,0

    # 计算延迟
    rmse = np.sqrt(np.mean((raw - filt)**2))
    velocity = np.mean(np.abs(np.diff(filt))) / dt
    if velocity < 2.0: lag = 0
    else:
        lag = (rmse / velocity) * 1000
        lag = min(lag, 600)

    # 计算 NRR (Noise Reduction Rate)
    raw_noise = np.std(np.diff(raw)) + 1e-9
    filt_noise = np.std(np.diff(filt))
    nrr = (1 - (filt_noise / raw_noise)) * 100
    nrr = max(0, min(100, nrr))

    score_lag = max(0, 100 - max(0, (lag - 60) * 1.5))
    score_nrr = nrr

    # 分数评判标准，属于均衡类型
    final_score = 0.5 * score_lag + 0.5 * score_nrr

    return lag, nrr, final_score

def find_best_alpha(raw_data, dt, current_alpha):
    best_a = current_alpha
    best_score = -1
    best_curve = []

    for a in np.arange(0.10, 0.95, 0.05):
        sim = simulate_stream(raw_data, a)
        _, _, score = evaluate_performance(raw_data, sim, dt)
        if score > best_score:
            best_score = score
            best_a = a
            best_curve = sim

    return best_a, best_curve

def analyze_axis(raw_data, dt, axis_obj):
    axis_obj.raw = list(raw_data)
    axis_obj.sim_curr = simulate_stream(raw_data, axis_obj.current_alpha)

    instant_best, axis_obj.sim_best = find_best_alpha(raw_data, dt, axis_obj.current_alpha)

    axis_obj.rec_alpha_stable = axis_obj.rec_engine.update(instant_best)

    axis_obj.lag, axis_obj.nrr, axis_obj.score = evaluate_performance(raw_data, axis_obj.sim_curr, dt)

    # 评估状态
    if axis_obj.lag > 120: axis_obj.status = "Too Slow (Lag)"
    elif axis_obj.nrr < 40: axis_obj.status = "Too Shaky"
    elif axis_obj.score > 85: axis_obj.status = "Perfect"
    else: axis_obj.status = "Good"

def plot_thread(node):
    plt.style.use('default')

    fig = plt.figure(figsize=(16, 10))
    fig.canvas.manager.set_window_title('3-Axis Filter Tuning (White Theme)')

    gs = GridSpec(3, 5, figure=fig)
    ax_x = fig.add_subplot(gs[0, 0:4])
    ax_y = fig.add_subplot(gs[1, 0:4], sharex=ax_x)
    ax_d = fig.add_subplot(gs[2, 0:4], sharex=ax_x)
    ax_panel = fig.add_subplot(gs[:, 4])
    ax_panel.axis('off')

    axis_x = AxisData('X-AXIS', CURRENT_ALPHA_X)
    axis_y = AxisData('Y-AXIS', CURRENT_ALPHA_Y)
    axis_d = AxisData('DEPTH', CURRENT_ALPHA_D)

    axes_map = {
        'x': (ax_x, axis_x), 'y': (ax_y, axis_y), 'd': (ax_d, axis_d)
    }

    plot_lines = {}
    for key, (ax, obj) in axes_map.items():
        lr, = ax.plot([], [], '.', color='tab:red', alpha=0.25, markersize=4, label='Raw Input')
        lc, = ax.plot([], [], '-', color='tab:blue', linewidth=2, alpha=0.8, label=f'Current ({obj.current_alpha})')
        lb, = ax.plot([], [], '--', color='tab:orange', linewidth=2.5, label='Auto-Rec')

        plot_lines[key] = (lr, lc, lb)
        ax.set_ylabel(obj.name, fontsize=10, fontweight='bold')
        ax.grid(True, linestyle=':', alpha=0.6)

        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

        if key == 'x': ax.legend(loc='upper right', frameon=True)

    def draw_panel():
        ax_panel.clear()
        ax_panel.axis('off')

        ax_panel.text(0.5, 0.98, "DIAGNOSTICS", ha='center', color='black', fontsize=14, weight='bold', transform=ax_panel.transAxes)

        def draw_block(y_start, obj):
            ax_panel.axhline(y_start, color='#DDDDDD', linewidth=1)
            base_y = y_start - 0.03

            ax_panel.text(0.5, base_y, obj.name, ha='center', color='black', fontsize=12, weight='bold', transform=ax_panel.transAxes)

            ax_panel.text(0.5, base_y - 0.06, "Rec. Alpha", ha='center', color='#666666', fontsize=9, transform=ax_panel.transAxes)

            ax_panel.text(0.5, base_y - 0.11, f"{obj.rec_alpha_stable:.2f}", ha='center', color='tab:orange', fontsize=24, weight='bold', transform=ax_panel.transAxes)

            s_color = 'tab:green' if "Perfect" in obj.status or "Good" in obj.status else 'tab:red'
            ax_panel.text(0.5, base_y - 0.16, obj.status, ha='center', color=s_color, fontsize=11, weight='bold', transform=ax_panel.transAxes)

            info_text = f"Lag: {int(obj.lag)}ms\nNRR: {int(obj.nrr)}%"
            ax_panel.text(0.5, base_y - 0.22, info_text, ha='center', color='#555555', fontsize=9, transform=ax_panel.transAxes)

        draw_block(0.95, axis_x)
        draw_block(0.63, axis_y)
        draw_block(0.31, axis_d)

    while rclpy.ok():
        if len(node.raw_x) > 20:
            ts = list(node.times)
            dt = (ts[-1] - ts[0]) / len(ts) if len(ts) > 1 else 0.033

            analyze_axis(node.raw_x, dt, axis_x)
            analyze_axis(node.raw_y, dt, axis_y)
            analyze_axis(node.raw_d, dt, axis_d)

            for key, (ax, obj) in axes_map.items():
                lr, lc, lb = plot_lines[key]
                lr.set_data(ts, obj.raw)
                lc.set_data(ts, obj.sim_curr)
                lb.set_data(ts, obj.sim_best)

                if obj.raw:
                    center = np.mean(obj.raw)
                    span = max(30, (max(obj.raw) - min(obj.raw)) * 0.6)
                    ax.set_ylim(center - span, center + span)
                ax.set_xlim(min(ts), max(ts)+0.1)

            draw_panel()
            fig.canvas.draw()
            fig.canvas.flush_events()

        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedTuningDashboard()
    t = threading.Thread(target=rclpy.spin, args=(node,))
    t.start()
    try:
        plot_thread(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        t.join()

if __name__ == '__main__':
    main()
