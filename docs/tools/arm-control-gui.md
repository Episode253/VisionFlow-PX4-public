# Arm Control GUI

This page introduces the Gamma arm Web control panel (`gamma_arm_web_control`), which provides a browser-based interface for joint position control, end-effector pose manipulation, gripper operation, and real-time status feedback.

## Overview

The arm control GUI is a QtWebEngine-based embedded web application that runs inside the Docker container. It connects to the simulation via rosbridge WebSocket and provides:

- **Joint position sliders** — control each of the 6 arm joints individually
- **End-effector pose display** — real-time position and orientation of the gripper
- **Gripper control** — open/close the gripper with position or velocity commands
- **Status feedback** — current joint angles, velocities, and system state

## Quick Start

```bash
# Launch from host (auto-starts inside running container)
bash docker/into_gz_sitl.sh
```

After entering the container, the Web control starts automatically. Access it at:

**http://127.0.0.1:9000/index.html**

rosbridge WebSocket: **ws://127.0.0.1:9090**

## Manual Launch

```bash
bash windshape_dev/arm_control/gamma_arm/gamma_arm_web_control.sh
```

### Environment Variables

| Variable | Default | Description |
|------|--------|------|
| `GUI_ENABLE` | `1` | Launch embedded QtWebEngine GUI (0 = backend only) |
| `WEB_PORT` | `9000` | Web UI port |
| `ROSBRIDGE_PORT` | `9090` | rosbridge WebSocket port |
| `WEB_HOST` | `127.0.0.1` | Bind address |
| `AUTO_RESTART` | `1` | Auto-restart on process crash |
| `GZ_KEEPALIVE` | `0` | Keep Gazebo command topics subscribed |
| `LOG_ENABLE` | `0` | Write subprocess logs to `./log/` |

## Helper Commands (inside container)

| Command | Description |
|------|------|
| `webstart` | Start Web control with current settings |
| `webstart_gui` | Start with embedded GUI enabled |
| `webstart_headless` | Start backend only (no GUI window) |
| `webstop` | Stop Web control |
| `weblog` | Show recent logs |
| `webattach` | Attach to tmux Web control session |
| `webps` | Show related processes |
| `webcheck` | Check processes, HTTP endpoint, and ROS topics |

## Related Pages

- [Tools Overview](index.md)
- [UAV Control Scripts](uav-control-scripts.md)
- [Gamma Arm Integration](../modules/gamma-arm-integration/index.md)
