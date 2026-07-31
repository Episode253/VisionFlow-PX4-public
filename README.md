# VisionFlow-PX4

## Overview

> A customized PX4 Autopilot fork developed by **WindyLab**, integrating UAVs with robotic arms (Gamma series) for manipulation tasks in Gazebo simulation, featuring Prescribed Performance Guidance and Management Estimator (PreGME) control and ROS2 integration.

- **PreGME Controllers** — Full replacement of standard `mc_att_control` and `mc_pos_control` with sliding-mode Prescribed Performance Control (PPC) algorithms, including centroid compensation and composite error state observer (CESO).
- **Gamma Arm Integration** — Tight coupling between PX4 flight control and Gamma-series robotic arm dynamics via the `gamma_arm_dynamics` library.
- **Rich Gazebo Simulation** — Custom worlds, models, and plugins for indoor laboratory manipulation scenarios (landing boxes, VLA tasks, hardware-in-the-loop).
- **ROS2 Ecosystem** — Zenoh middleware, uXRCE-DDS client, and a complete ROS2 Humble Docker environment.

## Prerequisites

> 本地部署需要以下环境与组件支持

| 环境 / 组件 | 版本或信息 |
|-------------|-----------|
| 操作系统 | Ubuntu 22.04 |
| ROS 2 版本 | Humble |
| Gazebo Sim 版本 | Harmonic V8.11.0 |
| Ros-GZ Bridge 版本 | `ros-humble-ros-gz-harmonic` |
| PX4 版本 | V1.17.0 |
| QGC 版本 / 下载地址 | <https://github.com/Renwang-Huang/VisionFlow-PX4/releases/tag/V1.17.0> |

> For a step-by-step local installation guide covering the PX4 toolchain,
> ROS 2 Humble, repository setup, and WSL2 GPU configuration,
> see [Local Installation Guide](docs/en/getting-started/local-installation.md).

## Quick Start

### Check Available Build Targets

> `ninja -t targets` 列出所有可构建目标，用 `grep` 过滤特定关键词即可查找，以 `gz_q940_ti` 为例：

```bash
# 查看所有 gz_ 开头的仿真目标
ninja -C build/px4_sitl_default -t targets | grep "^gz_"

# 按关键词筛选，例如查找 q940_ti 相关目标
ninja -C build/px4_sitl_default -t targets | grep gz_q940_ti

# 查找所有 swan_gamma 相关目标
ninja -C build/px4_sitl_default -t targets | grep gz_swan_gamma
```

### Native Launch (Local Deployment)

> 以下命令直接在本地宿主机上运行 PX4 SITL + Gazebo

| Profile | Description | Command |
|---------|-------------|---------|
| Entity 1 | PreGME q940_ti with landing box (季梦玉) | `PX4_GZ_WORLD=laboratory_landingbox make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 2 | PreGME q940_ti with VLA task | `PX4_GZ_WORLD=laboratory_landingbox_vla_task0 make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox_vla_task0 EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 3 | Swan gamma v1 (company legacy) | `PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_swan_gamma_v1_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 4 | Swan gamma v2 (company new) | `PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 5 | Swan gamma v2 with VLA task | `PX4_GZ_WORLD=laboratory_no_landingbox_vla_task0 make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox_vla_task0 EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 6 | X500 with gimbal | `PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_x500_gimbal_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 7 | Differential drive rover | `PX4_GZ_MODEL_POSE="0,0,0.5,0,0,0" make px4_sitl gz_differential_rover_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 8 | PreGME q940_ti in yungu world | `PX4_GZ_WORLD=yungu make px4_sitl gz_q940_ti_gripper4_yungu EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |

### Docker-Based Launch (Recommended)

> Docker 方式封装了完整的 ROS2 Humble + Gazebo 环境，推荐首次使用或需要隔离开发环境的场景

```bash
# List available profiles
bash docker/run_gz_sitl.sh --list

# Launch with default profile (Entity 1)
bash docker/run_gz_sitl.sh --profile "Entity 1"

# Rebuild Docker image + launch
bash docker/run_gz_sitl.sh --build --profile "Entity 4"
```

### Additional Nodes

```bash
# Data bridge (Gazebo ↔ ROS2)
bash windshape_dev/data_stream/gz_bridge/bridge_gz_ros.sh

# Camera stream visualization
bash windshape_dev/data_stream/image_stream/camera_stream.sh

# Arm web control
bash docker/into_gz_sitl.sh

# MAVROS node
source thirdparty/install/setup.bash && ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557

# HITL simulation
gz sim -r Tools/simulation/gz/worlds/laboratory_landingbox_hitl.sdf
```

## Repository Structure

```
VisionFlow-PX4/
├── boards/                    # Board configs (HKUST nxt-dual, nxt-v1 + 10 vendors)
├── build/                     # Build artifacts (px4_sitl_default, px4_fmu-v6x_default)
├── cmake/                     # Custom CMake helpers
├── docs/references/           # PreGME research papers (Parameter reference, PPC theory)
├── docker/                    # Docker SITL workflow (ROS2 Humble + Gazebo)
│   ├── run_gz_sitl.sh         # Profile-based launcher with 7 entities
│   ├── compose.yaml           # Docker Compose setup
│   └── gz_sitl_profiles.conf  # SITL profile definitions
├── msg/                       # uORB messages (~180 files, incl. custom arm/collision/NN msgs)
├── platforms/                 # NuttX/POSIX platform support
├── posix-configs/             # POSIX/SITL configurations
├── ROMFS/                     # Root filesystem (init scripts, airframes, rcS)
├── src/                       # PX4 source
│   ├── modules/               # Flight control modules
│   │   ├── pregme_att_control/    # PreGME attitude controller (sliding-mode PPC)
│   │   ├── pregme_pos_control/    # PreGME position controller
│   │   ├── mc_nn_control/         # Neural network control (TensorFlow Lite Micro)
│   │   ├── camera_feedback/       # Camera trigger processing
│   │   ├── gimbal/                # Gimbal manager
│   │   ├── local_position_estimator/  # Block-based LPE
│   │   ├── rover_differential/    # Differential rover controller
│   │   ├── zenoh/                 # DDS alternative middleware
│   │   ├── muorb/                 # micro-ORB aggregator
│   │   ├── temperature_compensation/  # Per-sensor temp calibration
│   │   └── simulation/            # Gazebo bridge, plugins, sensor sims
│   ├── lib/
│   │   ├── gamma_arm_dynamics/  # Gamma robotic arm dynamics library
│   │   └── controllib/          # Extended control library (PID, blocks)
│   └── drivers/               # Sensor drivers
├── thirdparty/                # External deps (MAVROS, ROS2 msgs)
├── Tools/
│   └── simulation/
│       ├── gz/worlds/         # Gazebo worlds (lab, dining, coast)
│       └── gz/models/         # Gazebo models (q940_ti, swan_gamma, rover, ...)
├── validation/                # Module schema validation
└── windshape_dev/             # Project-specific code
    ├── arm_control/           # Gamma arm control GUI & web scripts
    ├── uav_control/           # Keyboard/joystick control, offboard flight
    ├── plugins/               # Gazebo C++ plugins (gamma_arm_control, px4_gzsim_bridge)
    ├── code_reference/        # Reference ROS2/catkin packages (moveit2, acados, pinocchio)
    ├── flight_review/         # Web-based flight log review (PID analysis, 3D plots)
    ├── data_plotting/         # Local position odometer plotting
    ├── image_stream/          # Camera stream & Gazebo-ROS bridge scripts
    └── parameter/             # Parameter management tools
```

## Custom Modules

### PreGME Controllers (Core Research Contribution)

| Module | Purpose | Key Features |
|--------|---------|--------------|
| `pregme_att_control` | Attitude control | Sliding-mode PPC, CESO, inertia matrix, rate limits, trajectory presets |
| `pregme_pos_control` | Position control | Sliding-mode PPC, lambda_p/Kp gains per axis, takeoff, collision constraints |

### Simulation Stack

| Module | Purpose |
|--------|---------|
| `gz_bridge` | Gazebo-PX4 actuator mixing (ESC, servo, wheel, gimbal) |
| `gz_plugins` | Custom Gazebo plugins (generic_motor, gstreamer, motor_failure, moving_platform, buoyancy, airspeed) |
| `simulator_mavlink` | MAVLink simulator bridge |
| `simulator_sih` | Software-in-the-loop simulator |
| `sensor_*_sim` | GPS, mag, baro, airspeed, AGP sensor simulators |

## Simulation Assets

### Worlds (`Tools/simulation/gz/worlds/`)

| World | Description |
|-------|-------------|
| `laboratory_landingbox.sdf` | Main lab with landing box |
| `laboratory_landingbox_vla_task0.sdf` | Lab with Vision-Language-Action task |
| `laboratory_no_landingbox.sdf` | Lab without landing box |
| `laboratory_no_landingbox_vla_task0.sdf` | VLA task without landing box |
| `laboratory_landingbox_hitl.sdf` | Hardware-in-the-loop version |
| `indoor_dining.sdf` | Indoor dining environment |
| `baylands_coast.sdf` | Baylands coastal environment |
| `yungu.sdf` | Yungu lab environment (yungu.glb visual + yungu_collider.stl collision) |

### Models (`Tools/simulation/gz/models/`)

| Model | Description |
|-------|-------------|
| `q940_ti_gripper3/`, `q940_ti_gripper4/` | Q940TI drone with 3/4-finger gripper |
| `swan_gamma_v1/`, `swan_gamma_v2/` | Swan UAV with Gamma arm (legacy/new) |
| `x500_gimbal/`, `x500_base/` | X500 quadcopter variants |
| `ti5_arm/` | TI5 robotic arm |
| `differential_rover/` | Differential drive rover |
| `Intel_realsense_d435/` | Intel RealSense D435 camera |
| Household objects | landing_box, red_coke_can, cracker_box, bookshelf, drawer, depot, etc. |

## Airframe Configurations

Custom posix airframes in `ROMFS/px4fmu_common/init.d-posix/airframes/`:

| Airframe ID | Description |
|-------------|-------------|
| `4001_gz_x500` | Standard X500 quad |
| `4002_gz_differential_rover` | Differential drive rover |
| `4003_gz_x500_gimbal` | X500 with gimbal |
| `4004_gz_q940_ti_gripper3` | Q940TI with gripper3 |
| `4005_gz_swan_gamma_v1` | Swan UAV with gamma arm v1 |
| `4006_gz_q940_ti_gripper4` | Q940TI with gripper4 |
| `4007_gz_swan_gamma_v2` | Swan UAV with gamma arm v2 |

## Custom uORB Messages

| Message | Purpose |
|---------|---------|
| `ArmJointState.msg` | Robotic arm joint states |
| `CollisionConstraints.msg` | Collision avoidance constraints |
| `NeuralControl.msg` | Neural control status |
| `FigureEightStatus.msg` | Figure-eight trajectory tracking |
| `PositionControllerLandingStatus.msg` | Landing status |
| `TrajectorySetpoint6dof.msg` | 6-DOF trajectory setpoints |
| `Rover*` series | Rover-specific control messages |
| `pos_helper.msg` | Position helper |

## Board Support

| Board | MCU | Description |
|-------|-----|-------------|
| `hkust/nxt-dual` | STM32 | Dual-IMU custom board |
| `hkust/nxt-v1` | STM32 | Single board variant |
| *(+ 10 other vendors)* | — | Stock PX4 boards |

## Firmware Builds

| Build Target | Platform | Description |
|-------------|----------|-------------|
| `px4_sitl_default` | POSIX | Gazebo SITL (primary development target) |
| `px4_fmu-v6x_default` | NuttX | STM32H7 firmware (FMUv6X) |

## Key Differences from Stock PX4

1. **PreGME Controllers** — Sliding-mode PPC replaces standard MC controllers (core research contribution)
2. **Robotic Arm Integration** — `gamma_arm_dynamics` bridges PX4 flight control with Gamma-series arm dynamics
3. **Heavy Gazebo Simulation** — Custom worlds, models, plugins for indoor lab manipulation
4. **ROS2 Integration** — Zenoh middleware, uXRCE-DDS, Gazebo-ROS bridge, complete ROS2 Humble Docker
5. **Camera Feedback Pipeline** — OAK-D and Intel RealSense support with geotagging
6. **Differential Rover Support** — Full rover control stack alongside quadcopter

## ROS2 Usage

This project uses **ROS2 Humble** as the middleware layer for simulation, offboard control, arm manipulation, visualization, and data streaming. Below are common workflows.

### Prerequisites

Ensure ROS2 Humble is sourced:

```bash
source /opt/ros/humble/setup.bash
```

Or using the Docker environment (recommended):

```bash
bash docker/run_gz_sitl.sh --profile "Entity 1"
```

### Basic ROS2 Commands

```bash
# List active topics
ros2 topic list

# Echo a specific topic (e.g., drone odometry)
ros2 topic echo /model/q940_ti_0/odometry

# List active nodes
ros2 node list

# Get node info
ros2 node info <node_name>

# Call a service (e.g., arm the drone via MAVROS)
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Set OFFBOARD mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"

# List all services
ros2 service list

# Get parameter from a node
ros2 param get <node_name> <param_name>
```

### Offboard Control

Fly the drone via MAVROS:

```bash
python3 windshape_dev/uav_control/offboard/official_offboard.py
```

Other offboard scripts:

| Script | Description |
|--------|-------------|
| `windshape_dev/uav_control/offboard/official_offboard.py` | Takeoff & hover |
| `windshape_dev/uav_control/offboard/circular_tracking.py` | Circular trajectory |
| `windshape_dev/uav_control/offboard/figure-eight_tracking.py` | Figure-8 path with yaw blending |

### Keyboard Control

```bash
python3 windshape_dev/uav_control/keyboard/keyboard_control.py
```

### Gazebo ↔ ROS2 Bridge

Bridge simulation data to ROS2 topics:

```bash
bash windshape_dev/image_stream/bridge_gz_ros.sh
```

This publishes arm joint states, gripper states, drone odometry, and more to ROS2 topics (see [Topics & Services](#topics--services) below).

### Camera Stream

```bash
# Start bridge then camera stream
bash windshape_dev/image_stream/bridge_gz_ros.sh
bash windshape_dev/image_stream/camera_stream.sh
```

View at: `http://localhost:8080/`

### Arm Control

Launch the Gamma arm web control interface:

```bash
bash windshape_dev/arm_control/gamma_arm/gamma_arm_web_control.sh
```

### Odometry Plotting

Real-time position dashboard:

```bash
python3 windshape_dev/data_plotting/local_position/odom_plotter.py
```

### Topics & Services

Key ROS2 topics used in the project:

| Topic | Type | Description |
|-------|------|-------------|
| `/mavros/state` | `mavros_msgs/msg/State` | MAVLink connection state |
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | UAV local odometry |
| `/mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped` | UAV local pose |
| `/mavros/setpoint_position/local` | `geometry_msgs/msg/PoseStamped` | OFFBOARD position setpoint |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | OFFBOARD velocity setpoint |
| `/mavros/rc/override` | `mavros_msgs/msg/OverrideRCIn` | RC override commands |
| `/model/q940_ti_0/odometry` | `nav_msgs/msg/Odometry` | Gazebo drone ground truth |
| `/gamma_arm/joint_states` | `sensor_msgs/msg/JointState` | Gamma arm joint states |
| `/gripper3/joint_state` | `sensor_msgs/msg/JointState` | Gripper joint state |

Key services:

| Service | Type | Purpose |
|---------|------|---------|
| `/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` | Arm / disarm |
| `/mavros/set_mode` | `mavros_msgs/srv/SetMode` | Flight mode (e.g., OFFBOARD) |

### Visualizing with RViz2

```bash
# Load a preconfigured RViz2 setup from the SUPER planner
rviz2 -d windshape_dev/yungu/src/SUPER/super_planner/rviz/super_planner.rviz
```

## Citation

If you use this codebase in your research, please cite the associated paper:

```bibtex
@article{ji2025pregme,
  title={PreGME: Prescribed Performance Control of Aerial Manipulators based on Variable-Gain ESO},
  author={Ji, Mengyu and Guo, Shiliang and Li, Zhengzhen and Shen, Jiahao and Cao, Huazi and Zhao, Shiyu},
  journal={arXiv preprint arXiv:2512.22957},
  year={2025}
}
```

## Documentation

- [`docs/references/PreGME: Prescribed Performance Control of Aerial Manipulators based on Variable-Gain ESO.pdf`](docs/references/PreGME:%20Prescribed%20Performance%20Control%20of%20Aerial%20Manipulators%20based%20on%20Variable-Gain%20ESO.pdf) — PreGME theoretical foundation
- [`docs/references/PreGME:Parameter Reference.pdf`](docs/references/PreGME:Parameter%20Reference.pdf) — Parameter reference guide
