# UAV Control Scripts

This page covers the VisionFlow-PX4 UAV control scripts, including keyboard and joystick manual control (`keyboard/`) and Offboard automated trajectory tracking (`offboard/`).

## Keyboard Control

Manual flight control via keyboard, communicating with PX4 through MAVROS in OFFBOARD mode.

### Launch

```bash
# Inside the Docker container
python3 windshape_dev/uav_control/keyboard/keyboard_control.py
```

### Keybindings

| Key | Action |
|-----|--------|
| `i` / `,` | Forward / Backward |
| `j` / `l` | Turn left / Turn right |
| `r` / `f` | Ascend / Descend |
| `5` | Switch to OFFBOARD mode |
| `6` | Arm vehicle |
| `7` | Auto takeoff |
| `Space` | Auto land |
| `k` | Emergency stop |
| `q` / `z` | Increase / decrease both speeds |
| `w` / `x` | Increase / decrease linear speed |
| `e` / `c` | Increase / decrease angular speed |
| `Ctrl+C` | Quit |

---

## Joystick Control (WFLY)

Flight control via WFLY ET16S joystick, mapping joystick axes to PX4 RC input channels.

### Launch

```bash
python3 windshape_dev/uav_control/keyboard/wfly_joystick_control.py
```

### Default Device

```
/dev/input/by-id/usb-EdgeTX_WFLY_ET16S_www.wflysz.com_ET16S-joystick
```

Adjust in the script's `DEFAULT_DEVICE` variable or via the config dictionary for custom axis mappings (roll, pitch, throttle, yaw).

---

## Offboard Trajectory Scripts

Automated flight trajectories using MAVROS OFFBOARD mode with PositionTarget messages.

### Available Scripts

| Script | Trajectory | Description |
|--------|-----------|------|
| `official_offboard.py` | Hover | Takeoff → OFFBOARD → hover at target altitude |
| `rectangular_tracking.py` | Rectangle | Smoothstep-interpolated rectangular path with corner pauses |
| `circular_tracking.py` | Circle | Takeoff + circular trajectory tracking |
| `figure-eight_tracking.py` | Figure-8 | Smooth figure-eight with heading control |

### Launch Example

```bash
# Inside the container
ros2 run <package> rectangular_tracking.py
# or directly:
python3 windshape_dev/uav_control/offboard/rectangular_tracking.py
```

### Common Parameters (adjust in script)

| Parameter | Default | Description |
|-----------|---------|------|
| `takeoff_height` | `2.0` m | Target takeoff altitude |
| `hover_time` | `1.5` s | Hover duration after takeoff |
| `control_rate` | `20` Hz | Control loop frequency |
| `pos_tolerance` | `0.15` m | Position threshold for "arrived" detection |

### Rectangular Tracking Details

The `rectangular_tracking.py` script uses cubic polynomial (smoothstep / Hermite) interpolation to generate S-curve velocity profiles:

1. **TAKEOFF** — ascend to target height
2. **HOVER** — hold position briefly
3. **LEG_0~3** — fly A→B, B→C, C→D, D→A with smoothstep interpolation
   - Position + velocity feedforward sent via `PositionTarget` (type_mask=2048)
   - Nose keeps initial heading throughout
   - Corner pause (`corner_pause`) between segments to avoid overshoot
4. **RETURN** — fly back above takeoff point
5. **LAND** — descend and land

### Related Pages

- [Tools Overview](index.md)
- [Arm Control GUI](arm-control-gui.md)
- [Quick Start](../getting-started/quick-start.md)
