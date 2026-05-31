#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import os
import struct
import sys
import time
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


JS_EVENT_FORMAT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

DEFAULT_DEVICE = "/dev/input/by-id/usb-EdgeTX_WFLY_ET16S_www.wflysz.com_ET16S-joystick"

DEFAULT_CONFIG = {
    "roll": {
        "axis": 3,
        "raw_min": -24500,
        "raw_center": 0,
        "raw_max": 24500,
        "invert": False
    },
    "pitch": {
        "axis": 2,
        "raw_min": -24500,
        "raw_center": 0,
        "raw_max": 24500,
        "invert": False
    },
    "throttle": {
        "axis": 1,
        "raw_min": -24500,
        "raw_center": 0,
        "raw_max": 24500,
        "invert": True
    },
    "yaw": {
        "axis": 0,
        "raw_min": -24500,
        "raw_center": 0,
        "raw_max": 24500,
        "invert": False
    }
}


class CenteringAxisFilter:
    def __init__(
        self,
        raw_min: int,
        raw_center: int,
        raw_max: int,
        deadband: float = 0.04,
        endpoint_zone: float = 0.04,
        alpha: float = 0.35,
        invert: bool = False,
    ):
        self.raw_min = int(raw_min)
        self.raw_center = int(raw_center)
        self.raw_max = int(raw_max)
        self.deadband = float(deadband)
        self.endpoint_zone = float(endpoint_zone)
        self.alpha = float(alpha)
        self.invert = bool(invert)
        self.filtered = 0.0

        if self.raw_min >= self.raw_center:
            raise ValueError(f"raw_min must be smaller than raw_center: {self.raw_min} >= {self.raw_center}")

        if self.raw_max <= self.raw_center:
            raise ValueError(f"raw_max must be larger than raw_center: {self.raw_max} <= {self.raw_center}")

    def normalize(self, raw: int) -> float:
        raw = int(raw)

        if raw >= self.raw_center:
            denom = max(self.raw_max - self.raw_center, 1)
            x = (raw - self.raw_center) / denom
        else:
            denom = max(self.raw_center - self.raw_min, 1)
            x = (raw - self.raw_center) / denom

        x = max(-1.0, min(1.0, x))

        if self.invert:
            x = -x

        return x

    def update(self, raw: int) -> int:
        x = self.normalize(raw)

        if abs(x) < self.deadband:
            x = 0.0

        if x > 1.0 - self.endpoint_zone:
            x = 1.0
        elif x < -1.0 + self.endpoint_zone:
            x = -1.0

        self.filtered = self.alpha * x + (1.0 - self.alpha) * self.filtered
        return int(max(-1000, min(1000, round(self.filtered * 1000))))


class NonCenteringThrottleFilter:
    def __init__(
        self,
        raw_min: int,
        raw_max: int,
        low_deadband: float = 0.01,
        endpoint_zone: float = 0.04,
        alpha: float = 0.5,
        invert: bool = False,
    ):
        self.raw_min = int(raw_min)
        self.raw_max = int(raw_max)
        self.low_deadband = float(low_deadband)
        self.endpoint_zone = float(endpoint_zone)
        self.alpha = float(alpha)
        self.invert = bool(invert)
        self.filtered = -1.0

        if self.raw_max <= self.raw_min:
            raise ValueError(f"raw_max must be larger than raw_min: {self.raw_max} <= {self.raw_min}")

    def normalize_01(self, raw: int) -> float:
        raw = int(raw)
        t = (raw - self.raw_min) / max(self.raw_max - self.raw_min, 1)
        t = max(0.0, min(1.0, t))

        if self.invert:
            t = 1.0 - t

        return t

    def update(self, raw: int) -> int:
        t = self.normalize_01(raw)

        if t < self.low_deadband:
            t = 0.0

        if t > 1.0 - self.endpoint_zone:
            t = 1.0

        x = 2.0 * t - 1.0
        self.filtered = self.alpha * x + (1.0 - self.alpha) * self.filtered
        return int(max(-1000, min(1000, round(self.filtered * 1000))))


def load_config(config_path: str | None) -> dict:
    if not config_path:
        return json.loads(json.dumps(DEFAULT_CONFIG))

    path = Path(config_path)

    if not path.exists():
        path.write_text(json.dumps(DEFAULT_CONFIG, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"[INFO] Config file did not exist, created default config: {path}")
        return json.loads(json.dumps(DEFAULT_CONFIG))

    return json.loads(path.read_text(encoding="utf-8"))


def apply_axis_overrides(config: dict, args: argparse.Namespace) -> dict:
    mapping = {
        "roll": args.axis_roll,
        "pitch": args.axis_pitch,
        "throttle": args.axis_throttle,
        "yaw": args.axis_yaw,
    }

    for name, axis in mapping.items():
        if axis is not None:
            config[name]["axis"] = int(axis)

    if args.invert_roll:
        config["roll"]["invert"] = not config["roll"].get("invert", False)

    if args.invert_pitch:
        config["pitch"]["invert"] = not config["pitch"].get("invert", False)

    if args.invert_throttle:
        config["throttle"]["invert"] = not config["throttle"].get("invert", False)

    if args.invert_yaw:
        config["yaw"]["invert"] = not config["yaw"].get("invert", False)

    return config


def open_joystick(device: str) -> int:
    if not os.path.exists(device):
        raise FileNotFoundError(
            f"Joystick device not found: {device}\n"
            f"Try: ls -l /dev/input/by-id/ | grep WFLY"
        )

    return os.open(device, os.O_RDONLY | os.O_NONBLOCK)


def read_joystick_events(fd: int, axes: dict[int, int], buttons_state: int) -> int:
    while True:
        try:
            data = os.read(fd, JS_EVENT_SIZE)
        except BlockingIOError:
            break

        if not data or len(data) < JS_EVENT_SIZE:
            break

        _, value, event_type, number = struct.unpack(JS_EVENT_FORMAT, data)
        event_type_clean = event_type & ~JS_EVENT_INIT

        if event_type_clean == JS_EVENT_AXIS:
            axes[int(number)] = int(value)

        elif event_type_clean == JS_EVENT_BUTTON:
            if value:
                buttons_state |= (1 << int(number))
            else:
                buttons_state &= ~(1 << int(number))

    return buttons_state


def build_filters(config: dict, args: argparse.Namespace):
    filters = {}

    for name in ("roll", "pitch", "yaw"):
        item = config[name]
        filters[name] = CenteringAxisFilter(
            raw_min=item["raw_min"],
            raw_center=item["raw_center"],
            raw_max=item["raw_max"],
            deadband=args.stick_deadband,
            endpoint_zone=args.stick_endpoint_zone,
            alpha=args.stick_alpha,
            invert=item.get("invert", False),
        )

    item = config["throttle"]
    filters["throttle"] = NonCenteringThrottleFilter(
        raw_min=item["raw_min"],
        raw_max=item["raw_max"],
        low_deadband=args.throttle_low_deadband,
        endpoint_zone=args.throttle_endpoint_zone,
        alpha=args.throttle_alpha,
        invert=item.get("invert", False),
    )

    return filters


def get_axis(config: dict, name: str) -> int:
    return int(config[name]["axis"])


def throttle_to_mavlink(throttle_filtered: int, mode: str) -> int:
    if mode == "centered":
        return throttle_filtered

    if mode == "positive":
        return int(max(0, min(1000, round((throttle_filtered + 1000) * 0.5))))

    raise ValueError(f"Unknown throttle mode: {mode}")



def color(text: str, code: str, enabled: bool = True) -> str:
    if not enabled:
        return text
    return f"\033[{code}m{text}\033[0m"


def bar(value: int, width: int = 20) -> str:
    """
    Build a compact ASCII bar for values in [-1000, 1000].
    Example:
        left negative, right positive, center marker.
    """
    value = max(-1000, min(1000, int(value)))
    center = width // 2
    pos = int(round((value + 1000) / 2000 * (width - 1)))
    chars = [" "] * width
    chars[center] = "|"
    chars[pos] = "●"
    return "[" + "".join(chars) + "]"


def throttle_bar(value: int, width: int = 20) -> str:
    """
    Build a compact throttle bar for MAVLink positive throttle [0, 1000].
    """
    value = max(0, min(1000, int(value)))
    filled = int(round(value / 1000 * width))
    return "[" + "█" * filled + " " * (width - filled) + "]"


def clear_line() -> str:
    return "\r\033[K"


def print_banner(title: str) -> None:
    line = "=" * 72
    print(line)
    print(title)
    print(line)

def print_mapping(config: dict) -> None:
    print("Axis mapping:")
    print(f"  Roll      A{config['roll']['axis']}   invert={config['roll'].get('invert', False)}")
    print(f"  Pitch     A{config['pitch']['axis']}   invert={config['pitch'].get('invert', False)}")
    print(f"  Throttle  A{config['throttle']['axis']}   invert={config['throttle'].get('invert', False)}")
    print(f"  Yaw       A{config['yaw']['axis']}   invert={config['yaw'].get('invert', False)}")


def print_monitor_line(raw_axes: dict[int, int], filtered: dict[str, int], buttons: int) -> None:
    raw_part = " ".join(f"A{i}:{raw_axes.get(i, 0):6d}" for i in range(8))
    line = (
        clear_line()
        + f"RAW {raw_part}  |  "
        + f"R {filtered['roll']:5d}{bar(filtered['roll'])}  "
        + f"P {filtered['pitch']:5d}{bar(filtered['pitch'])}  "
        + f"T {filtered['throttle']:5d}{bar(filtered['throttle'])}  "
        + f"Y {filtered['yaw']:5d}{bar(filtered['yaw'])}  "
        + f"BTN 0x{buttons:04x}"
    )
    print(line, end="", flush=True)


def run_monitor(args: argparse.Namespace) -> None:
    config = apply_axis_overrides(load_config(args.config), args)
    filters = build_filters(config, args)

    fd = open_joystick(args.device)
    axes = {i: 0 for i in range(16)}
    buttons = 0

    print_banner("WFLY / EdgeTX → PX4 Manual Control Bridge | MONITOR")
    print("Mode: monitor only. Press Ctrl+C to stop.")
    print("Layout: Mode 2 / American hand. Pitch and throttle directions are fixed.")
    print_mapping(config)
    print("Move one stick at a time. Use --axis-* or --invert-* only if something is still wrong.")
    time.sleep(0.5)

    last_print = 0.0

    try:
        while True:
            buttons = read_joystick_events(fd, axes, buttons)

            filtered = {
                "roll": filters["roll"].update(axes.get(get_axis(config, "roll"), 0)),
                "pitch": filters["pitch"].update(axes.get(get_axis(config, "pitch"), 0)),
                "throttle": filters["throttle"].update(axes.get(get_axis(config, "throttle"), 0)),
                "yaw": filters["yaw"].update(axes.get(get_axis(config, "yaw"), 0)),
            }

            now = time.time()

            if now - last_print >= 1.0 / args.rate:
                print_monitor_line(axes, filtered, buttons)
                last_print = now

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\n[INFO] Monitor stopped.")

    finally:
        os.close(fd)


def run_calibrate(args: argparse.Namespace) -> None:
    config = apply_axis_overrides(load_config(args.config), args)

    fd = open_joystick(args.device)
    axes = {i: 0 for i in range(16)}
    buttons = 0

    print("[INFO] Calibration mode.")
    print_mapping(config)
    print("[INFO] Keep roll/pitch/yaw centered. Put throttle at low or current desired reference.")
    print("[INFO] Capturing current values for 2 seconds...")
    t0 = time.time()

    while time.time() - t0 < 2.0:
        buttons = read_joystick_events(fd, axes, buttons)
        time.sleep(0.01)

    centers = {i: axes.get(i, 0) for i in range(8)}
    mins = dict(centers)
    maxs = dict(centers)

    print(f"[INFO] Current values captured: {centers}")
    print(f"[INFO] Move all sticks/knobs through full range for {args.seconds:.1f} seconds.")

    start = time.time()

    try:
        while time.time() - start < args.seconds:
            buttons = read_joystick_events(fd, axes, buttons)

            for i in range(8):
                v = axes.get(i, 0)
                mins[i] = min(mins.get(i, v), v)
                maxs[i] = max(maxs.get(i, v), v)

            remain = max(0.0, args.seconds - (time.time() - start))
            print(f"\r[INFO] Remaining: {remain:4.1f}s  mins={mins}  maxs={maxs}", end="", flush=True)
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass

    finally:
        os.close(fd)

    print("\n\n[INFO] Calibration result by axis:")
    for i in range(8):
        print(f"Axis {i}: raw_min={mins[i]}, raw_center={centers[i]}, raw_max={maxs[i]}")

    for name in ("roll", "pitch", "yaw"):
        axis_index = config[name]["axis"]
        config[name]["raw_min"] = mins[axis_index]
        config[name]["raw_center"] = centers[axis_index]
        config[name]["raw_max"] = maxs[axis_index]

    throttle_axis = config["throttle"]["axis"]
    config["throttle"]["raw_min"] = mins[throttle_axis]
    config["throttle"]["raw_max"] = maxs[throttle_axis]
    config["throttle"]["raw_center"] = centers[throttle_axis]

    print("\n[INFO] Suggested config JSON:")
    print(json.dumps(config, indent=2, ensure_ascii=False))


def run_send(args: argparse.Namespace) -> None:
    if mavutil is None:
        raise ImportError("pymavlink is not installed. Install it with: pip3 install --user pymavlink")

    config = apply_axis_overrides(load_config(args.config), args)
    filters = build_filters(config, args)

    fd = open_joystick(args.device)
    axes = {i: 0 for i in range(16)}
    buttons = 0

    print_banner("WFLY / EdgeTX → PX4 Manual Control Bridge | SEND")
    print("Default command is now enough:")
    print("  python3 wfly_to_px4_manual.py")
    print()
    print(f"MAVLink endpoint : {args.endpoint}")
    print(f"Throttle mode    : {args.throttle_mode}")
    print(f"Send rate        : {args.rate:.1f} Hz")
    print(f"Status output    : {'on' if args.print_output else 'off'}")
    print_mapping(config)
    print()
    print("Notes:")
    print("  - PX4 shell check: listener manual_control_setpoint")
    print("  - Do NOT enable QGC virtual joystick at the same time.")
    print("  - Stop with Ctrl+C.")
    print()

    mav = mavutil.mavlink_connection(
        args.endpoint,
        source_system=args.source_system,
        source_component=args.source_component,
        autoreconnect=True,
    )

    target_system = args.target_system

    if args.wait_heartbeat:
        print("Waiting for PX4 heartbeat ...")
        hb = mav.wait_heartbeat(timeout=10)

        if hb is None:
            print("WARN: no heartbeat received within 10 seconds.")
            print("      Check PX4 shell: mavlink status")
        else:
            target_system = mav.target_system or args.target_system
            print(f"Heartbeat received: target_system={target_system}, target_component={mav.target_component}")
            print()

    send_interval = 1.0 / args.rate
    heartbeat_interval = 1.0 / max(args.heartbeat_rate, 0.1)

    last_send = 0.0
    last_heartbeat = 0.0
    last_print = 0.0

    try:
        while True:
            buttons = read_joystick_events(fd, axes, buttons)

            roll = filters["roll"].update(axes.get(get_axis(config, "roll"), 0))
            pitch = filters["pitch"].update(axes.get(get_axis(config, "pitch"), 0))
            throttle_raw = filters["throttle"].update(axes.get(get_axis(config, "throttle"), 0))
            yaw = filters["yaw"].update(axes.get(get_axis(config, "yaw"), 0))
            throttle = throttle_to_mavlink(throttle_raw, args.throttle_mode)

            now = time.time()

            if now - last_heartbeat >= heartbeat_interval:
                mav.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                last_heartbeat = now

            if now - last_send >= send_interval:
                mav.mav.manual_control_send(
                    target_system,
                    pitch,
                    roll,
                    throttle,
                    yaw,
                    buttons,
                )
                last_send = now

            # Print less frequently than sending, otherwise the terminal becomes soup.
            if args.print_output and (now - last_print >= 1.0 / max(args.print_rate, 1.0)):
                if args.throttle_mode == "positive":
                    thr_visual = throttle_bar(throttle)
                else:
                    thr_visual = bar(throttle)

                line = (
                    clear_line()
                    + f"PX4#{target_system}  "
                    + f"PITCH {pitch:5d}{bar(pitch)}  "
                    + f"ROLL {roll:5d}{bar(roll)}  "
                    + f"THR {throttle:5d}{thr_visual}  "
                    + f"YAW {yaw:5d}{bar(yaw)}  "
                    + f"BTN 0x{buttons:04x}"
                )
                print(line, end="", flush=True)
                last_print = now

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        os.close(fd)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Use WFLY/EdgeTX USB joystick as PX4 MAVLink MANUAL_CONTROL input, Mode 2 friendly."
    )

    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--monitor", action="store_true", help="Only monitor raw and filtered joystick values.")
    mode.add_argument("--calibrate", action="store_true", help="Collect min/current/max values for axes.")
    mode.add_argument("--send", action="store_true", help="Send MAVLink MANUAL_CONTROL to PX4. Default mode.")

    parser.add_argument("--device", default=DEFAULT_DEVICE, help="Linux joystick device path.")
    parser.add_argument("--config", default=None, help="Optional JSON config path for axis mapping and calibration.")

    parser.add_argument("--endpoint", default="udpin:0.0.0.0:14540", help="MAVLink endpoint. For PX4 SITL companion link, use udpin:0.0.0.0:14540.")
    parser.add_argument("--wait-heartbeat", action="store_true", default=True, help="Wait for PX4 heartbeat before sending. Enabled by default.")
    parser.add_argument("--no-wait-heartbeat", dest="wait_heartbeat", action="store_false", help="Do not wait for PX4 heartbeat.")
    parser.add_argument("--heartbeat-rate", type=float, default=1.0, help="GCS heartbeat send rate in Hz.")
    parser.add_argument("--target-system", type=int, default=1, help="MAVLink target system id.")
    parser.add_argument("--source-system", type=int, default=255, help="MAVLink source system id.")
    parser.add_argument("--source-component", type=int, default=0, help="MAVLink source component id.")

    parser.add_argument("--rate", type=float, default=30.0, help="Loop/send rate in Hz.")

    parser.add_argument("--axis-roll", type=int, default=None, help="Override roll axis number.")
    parser.add_argument("--axis-pitch", type=int, default=None, help="Override pitch axis number.")
    parser.add_argument("--axis-throttle", type=int, default=None, help="Override throttle axis number.")
    parser.add_argument("--axis-yaw", type=int, default=None, help="Override yaw axis number.")

    parser.add_argument("--invert-roll", action="store_true", help="Invert roll direction.")
    parser.add_argument("--invert-pitch", action="store_true", help="Invert pitch direction.")
    parser.add_argument("--invert-throttle", action="store_true", help="Invert throttle direction.")
    parser.add_argument("--invert-yaw", action="store_true", help="Invert yaw direction.")

    parser.add_argument("--stick-deadband", type=float, default=0.04, help="Middle deadband for roll/pitch/yaw.")
    parser.add_argument("--stick-endpoint-zone", type=float, default=0.04, help="Endpoint snap zone for roll/pitch/yaw.")
    parser.add_argument("--stick-alpha", type=float, default=0.35, help="Low-pass alpha for roll/pitch/yaw.")

    parser.add_argument("--throttle-low-deadband", type=float, default=0.01, help="Low-end deadband for non-centering throttle.")
    parser.add_argument("--throttle-endpoint-zone", type=float, default=0.04, help="High-end snap zone for throttle.")
    parser.add_argument("--throttle-alpha", type=float, default=0.5, help="Low-pass alpha for throttle.")

    parser.add_argument("--seconds", type=float, default=10.0, help="Calibration duration in seconds.")
    parser.add_argument("--throttle-mode", choices=["centered", "positive"], default="positive")
    parser.add_argument("--print-output", action="store_true", default=True, help="Print status output in send mode. Enabled by default.")
    parser.add_argument("--quiet", dest="print_output", action="store_false", help="Disable live status output in send mode.")
    parser.add_argument("--print-rate", type=float, default=10.0, help="Status output refresh rate in Hz.")

    args = parser.parse_args()

    if not args.monitor and not args.calibrate and not args.send:
        args.send = True

    try:
        if args.monitor:
            run_monitor(args)
        elif args.calibrate:
            run_calibrate(args)
        else:
            run_send(args)

    except Exception as exc:
        print(f"\n[ERROR] {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
