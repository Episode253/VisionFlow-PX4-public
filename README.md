<div align="center">

# VisionFlow-PX4

## Overview (THIS BRANCH SWITCHED TO ORIGINAL PX4 CONTROLLERS)

> A customized PX4 Autopilot fork that integrates UAVs with Gamma-series robotic arms for manipulation tasks in Gazebo simulation. It includes Prescribed Performance Guidance and Management Estimator (PreGME) control and ROS 2 integration, with the following features:

- **PreGME Controllers** — Completely replace the standard `mc_rate_control`, `mc_att_control`, and `mc_pos_control` modules with sliding-mode Prescribed Performance Control (PPC), including center-of-mass compensation and a composite error state observer (CESO).
- **Gamma Arm Integration** — Tightly couples PX4 flight control with Gamma-series robotic-arm dynamics through the `gamma_arm_dynamics` library.
- **Rich Gazebo Resources** — Provides custom worlds, models, and plugins for indoor laboratory manipulation scenarios, including VLA tasks, software-in-the-loop simulation, and hardware-in-the-loop simulation.
- **ROS 2 Ecosystem** — Includes the uXRCE-DDS client, MAVROS, and a complete ROS 2 Humble Docker environment.

## Prerequisites

| Environment / Component | Version or Information |
|-------------------------|------------------------|
| Operating System | Ubuntu 22.04 |
| ROS 2 Distribution | Humble |
| Gazebo Sim | Harmonic V8.11.0 |
| ros-gz bridge | `ros-humble-ros-gzharmonic` |
| PX4-Autopilot | V1.17.0 |
| QGroundControl Download | <https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html> |

> For a step-by-step local installation guide covering the PX4 toolchain,
> ROS 2 Humble, repository setup, and WSL2 GPU configuration,
> see [Local Installation Guide](docs/en/getting-started/local-installation.md).

## Quick Start

### Check Available Build Targets

Use `ninja -t targets` to list all available build targets, then filter them with `grep`. For example, to locate `gz_q940_ti` targets:

```bash
# List all simulation targets beginning with gz_
ninja -C build/px4_sitl_default -t targets | grep "^gz_"

# Filter by keyword, for example q940_ti
ninja -C build/px4_sitl_default -t targets | grep gz_q940_ti

# Find all swan_gamma targets
ninja -C build/px4_sitl_default -t targets | grep gz_swan_gamma
```

### Native Launch (Non-Docker Environment)

The following command is recommended for a one-step installation of `ROS 2 Humble`:

| Profile | Description | Command |
|---------|-------------|---------|
| Entity 1 | PreGME q940_ti with landing box (季梦玉) | `PX4_GZ_WORLD=laboratory_landingbox make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 2 | PreGME q940_ti with VLA task | `PX4_GZ_WORLD=laboratory_landingbox_vla_task0 make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox_vla_task0 EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 3 | Swan gamma v1 (company legacy) | `PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_swan_gamma_v1_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 4 | Swan gamma v2 (company new) | `PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 5 | Swan gamma v2 with VLA task | `PX4_GZ_WORLD=laboratory_no_landingbox_vla_task0 make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox_vla_task0 EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 6 | X500 with gimbal | `PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_x500_gimbal_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 6b | X500 with 3D LiDAR | `PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_x500_lidar_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 7 | Differential drive rover | `PX4_GZ_MODEL_POSE="0,0,0.5,0,0,0" make px4_sitl gz_differential_rover_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 8 | PreGME q940_ti in yungu world | `PX4_GZ_WORLD=yungu make px4_sitl gz_q940_ti_gripper4_yungu EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |

### Docker-Based Launch (Recommended)

The Docker workflow packages a complete ROS 2 Humble + Gazebo environment. It is recommended for first-time users and for development that requires environment isolation:

```bash
# Interactive selection — choose the target profile from the menu
bash docker/run_gz_sitl.sh

# List available profiles
bash docker/run_gz_sitl.sh --list

# Launch the default profile (Entity 1)
bash docker/run_gz_sitl.sh --profile "Entity 1"

# Rebuild the image and launch
bash docker/run_gz_sitl.sh --build --profile "Entity 4"
```

#### ⚠️ When to Use the `--build` Flag

The `--build` flag rebuilds the Docker image. This is **slow**, but it is **required** in specific situations:

| Scenario | Need `--build`? | Action |
|----------|-----------------|--------|
| **First simulation launch** | ✅ **Yes** | `bash docker/run_gz_sitl.sh --build --profile "Entity 4"` (30–60 minutes) |
| **Dockerfile was modified** | ✅ **Yes** | `bash docker/run_gz_sitl.sh --build --profile "Entity 4"` |
| **Dockerfile and dependencies are unchanged** | ❌ **No** | `bash docker/run_gz_sitl.sh --profile "Entity 4"` (10–30 seconds) |
| **Re-running the simulation** | ❌ **No** | `bash docker/run_gz_sitl.sh --profile "Entity 4"` (fastest) |

#### Quick Reference

```bash
# First run — image build is required
bash docker/run_gz_sitl.sh --build --profile "Entity 4"

# Subsequent runs — skip the build step for a much faster launch
bash docker/run_gz_sitl.sh --profile "Entity 4"

# After editing the Dockerfile or Tools/setup/requirements.txt
bash docker/run_gz_sitl.sh --build --profile "Entity 4"
```

**Note**: Build time depends partly on network speed. Users in mainland China can benefit from the built-in Alibaba Cloud mirror acceleration.

### Additional Nodes

## TODO

- [x] Implement the robotic-arm web control panel
- [ ] Implement visual-data-stream acquisition and frontend visualization
- [ ] Build a one-shot automated task execution script, such as a single pick-and-place task

```bash
# Robotic-arm web control: open the control panel and enter the Docker container terminal
bash docker/into_gz_sitl.sh
```

## Maintenance and Troubleshooting

For help with project issues, refer to the maintenance guides: [English](docs/development/maintenance.md) | [中文](docs/zh/development/maintenance.md)

| Topic | Description |
|-------|-------------|
| Contact the Maintainer | Channels for reporting issues or requesting support |
| Issue and PR Workflow | Complete workflow from reporting a bug to submitting a fix |
| Troubleshooting | Build / Docker / Gazebo / PX4 runtime / ROS 2 communication |
| Debugging and Recovery | Log analysis, debugging techniques, cache cleanup, and Git rollback |

## Repository Structure

> The following directory trees show tracked source and configuration directories together with the project's main maintained components. Local build outputs, caches, logs, IDE files, and runtime artifacts are not shown.

### Main Directory

```text
VisionFlow-PX4/
├── .github/                  # CI workflows, repository configuration, and templates
├── boards/                   # 10 vendor namespaces; 45 board targets
├── cmake/                    # PX4 CMake helpers
├── docker/                   # ROS 2 Humble + Gazebo workflow
├── docs/                     # Documentation source files
├── msg/                      # uORB messages and ROS 2 message tooling
├── platforms/                # Common, NuttX, POSIX, QURT, and ROS 2 support
├── posix-configs/            # POSIX/SITL configuration
├── ROMFS/                    # PX4 runtime files, initialization scripts, and airframe configuration
├── site/                     # Static site generated by MkDocs
├── src/                      # PX4 source code
├── thirdparty/               # MAVROS Humble workspace
├── Tools/                    # Build, analysis, messaging, simulation, and utility tools
├── validation/               # Module configuration schema
└── windshape_dev/            # WindyLab tools and integrations
```

### PX4 Source Tree

In addition to upstream PX4 areas, the `src/` directory contains the following project-specific modules and libraries:

```text
src/
├── drivers/                 # Sensor and peripheral drivers
├── examples/                # Example applications
├── include/                 # Shared PX4 headers
├── lib/                     # PX4 libraries; project additions:
│   ├── gamma_arm_dynamics/  # Gamma robotic-arm dynamics library
│   └── controllib/          # Extended control library (PID and blocks)
├── modules/                 # Control, estimation, middleware, system, and simulation modules:
│   ├── pregme_att_control/  # PreGME attitude controller (sliding-mode PPC)
│   ├── pregme_pos_control/  # PreGME position controller
│   ├── mc_nn_control/       # Neural-network control (TensorFlow Lite Micro)
│   ├── camera_feedback/     # Camera trigger processing
│   ├── gimbal/              # Gimbal management
│   ├── local_position_estimator/ # Block-based LPE
│   ├── rover_differential/  # Differential-drive rover controller
│   ├── zenoh/               # DDS alternative middleware
│   ├── muorb/               # micro-ORB aggregator
│   ├── temperature_compensation/ # Per-sensor temperature calibration
│   └── simulation/          # Gazebo bridge, plugins, and sensor simulators
├── systemcmds/              # System command modules
└── templates/               # Module templates
```

### Messages and Interfaces

```text
msg/
├── *.msg                    # 212 current top-level message definitions
├── versioned/               # 37 versioned message definitions
├── px4_msgs_old/msg/        # 18 legacy message definitions
└── translation_node/        # ROS 2 message translation package
```

There are currently approximately 267 `.msg` files across the current, versioned, and legacy message trees.

### Tools and Simulation

```text
Tools/
├── ci/                       # CI helpers
├── ecl_ekf/                  # EKF analysis tools
├── HIL/                      # Hardware-in-the-loop tools
├── kconfig/                  # Kconfig tooling
├── module_config/            # Module configuration tooling
├── msg/                      # Message tooling
├── px4airframes/             # Airframe tooling
├── python_scripts/           # PX4 Python utilities
├── setup/                    # Installation helpers
└── simulation/
    ├── gz/
    │   ├── worlds/           # Gazebo worlds
    │   ├── models/           # Gazebo models
    │   ├── sdf_parsing/      # SDF parsing tools
    │   └── server.config     # Gazebo server configuration
    └── iscca_model/          # ISCCA URDF, meshes, and RViz resources
```

### Documentation and Containers

```text
docs/
├── architecture/             # System architecture
├── development/              # Development guides
├── getting-started/          # Installation and launch guides
├── hardware/                 # Board-level and hardware documentation
├── messages/                 # Message documentation
├── modules/                  # Module documentation
├── references/               # Papers and reference pages
├── simulation/               # Simulation documentation
├── tools/                    # Tool documentation
└── en/                       # English documentation tree

docker/
├── Dockerfile.humble-gz      # ROS 2 Humble + Gazebo image
├── compose.yaml              # Docker Compose configuration
├── entrypoint.sh             # Container entrypoint
├── gz_sitl_profiles.conf     # SITL profile definitions
├── into_gz_sitl.sh           # Enter the SITL container
├── run_flight_review.sh      # Launch Flight Review
└── run_gz_sitl.sh            # Profile-based SITL launcher
```

### WindyLab Project

```text
windshape_dev/
├── arm_control/
│   └── gamma_arm/            # Gamma robotic-arm GUI and web control
├── code_reference/           # ROS 2/ament reference projects and packages
│   ├── pregme_v1_13/         # PreGME reference project snapshot
│   └── windylab_gamma_arm_01_v2/ # Gamma robotic-arm reference project snapshot
├── data_plotting/
│   └── local_position/       # Local-position and odometry plotting
├── flight_review/
│   ├── app/                  # Web log-review server, plotting, and 3D processing
│   └── data/                 # Flight Review data, logs, and cache
├── image_stream/             # Gazebo-ROS bridge and camera streams
├── parameter/                # Static Gazebo, PX4 parameter, and RViz configuration
├── plugins/                  # Custom Gazebo plugins
│   ├── gamma_arm_control/
│   ├── joint_position_controller/
│   └── px4_gzsim_bridge/
├── px4_original_tools/       # PX4 MAVLink, log, and calibration utilities
└── uav_control/              # Keyboard, gamepad, and Offboard control
    ├── keyboard/
    └── offboard/
```

## Custom Modules

### PreGME Controllers (Core Research Contribution)

| Module | Purpose | Key Features |
|--------|---------|--------------|
| `pregme_att_control` | Attitude control | Sliding-mode PPC, CESO, inertia matrix, angular-rate limits, trajectory presets |
| `pregme_pos_control` | Position control | Sliding-mode PPC, per-axis lambda_p/Kp gains, takeoff, collision constraints |

### Simulation Stack

| Module | Purpose |
|--------|---------|
| `gz_bridge` | Gazebo-PX4 actuator mixing for ESCs, servos, wheels, and gimbals |
| `gz_plugins` | Custom Gazebo plugins: generic_motor, gstreamer, motor_failure, moving_platform, buoyancy, and airspeed |
| `simulator_mavlink` | MAVLink simulation bridge |
| `simulator_sih` | Software-in-the-loop simulator |
| `sensor_*_sim` | GPS, magnetometer, barometer, airspeed, and AGP sensor simulators |

## Simulation Assets

### Worlds (`Tools/simulation/gz/worlds/`)

| World | Description |
|-------|-------------|
| `laboratory_landingbox.sdf` | Main laboratory with a landing box |
| `laboratory_landingbox_vla_task0.sdf` | Laboratory with a VLA task |
| `laboratory_no_landingbox.sdf` | Laboratory without a landing box |
| `laboratory_no_landingbox_vla_task0.sdf` | VLA task scenario without a landing box |
| `laboratory_landingbox_hitl.sdf` | Hardware-in-the-loop version |
| `indoor_dining.sdf` | Indoor dining environment |
| `baylands_coast.sdf` | Baylands coastal environment |
| `yungu.sdf` | Yungu lab environment (yungu.glb visual + yungu_collider.stl collision) |

### Models (`Tools/simulation/gz/models/`)

| Model | Description |
|-------|-------------|
| `q940_ti_gripper3/`, `q940_ti_gripper4/` | Q940TI UAV with a three- or four-finger gripper |
| `swan_gamma_v1/`, `swan_gamma_v2/` | Swan UAV with a Gamma arm (legacy/current) |
| `x500_gimbal/`, `x500_base/` | X500 quadcopter variants |
| `ti5_arm/` | TI5 robotic arm |
| `differential_rover/` | Differential-drive rover |
| `Intel_realsense_d435/` | Intel RealSense D435 camera |
| Household objects | landing_box, red_coke_can, cracker_box, bookshelf, drawer, depot, and others |

## Airframe Configurations

Custom POSIX airframes are located in `ROMFS/px4fmu_common/init.d-posix/airframes/`:

| Airframe ID | Description |
|-------------|-------------|
| `4001_gz_x500` | Standard X500 quadcopter |
| `4002_gz_differential_rover` | Differential-drive rover |
| `4003_gz_x500_gimbal` | X500 with gimbal |
| `4004_gz_q940_ti_gripper3` | Q940TI with gripper3 |
| `4005_gz_swan_gamma_v1` | Swan UAV with Gamma arm v1 |
| `4006_gz_q940_ti_gripper4` | Q940TI with gripper4 |
| `4007_gz_swan_gamma_v2` | Swan UAV with Gamma arm v2 |

## Custom uORB Messages

| Message | Purpose |
|---------|---------|
| `ArmJointState.msg` | Robotic-arm joint states |
| `CollisionConstraints.msg` | Collision-avoidance constraints |
| `NeuralControl.msg` | Neural-control status |
| `FigureEightStatus.msg` | Figure-eight trajectory tracking |
| `PositionControllerLandingStatus.msg` | Landing status |
| `TrajectorySetpoint6dof.msg` | 6-DOF trajectory setpoints |
| `Rover*` series | Rover-specific control messages |
| `pos_helper.msg` | Position helper |

## Board Support

| Board | MCU | Description |
|-------|-----|-------------|
| `hkust/nxt-dual` | STM32 | Custom dual-IMU board |
| `hkust/nxt-v1` | STM32 | Single-board variant |
| 10 vendor namespaces, including HKUST | — | 45 board targets in total |

## Firmware Builds

| Build Target | Platform | Description |
|--------------|----------|-------------|
| `px4_sitl_default` | POSIX | Primary Gazebo SITL build target |
| `px4_fmu-v6x_default` | NuttX | Supported STM32H7 FMUv6X target, generated on demand |

## Key Differences from Stock PX4

1. **PreGME Controllers** — Sliding-mode PPC replaces the standard multicopter controllers.
2. **Robotic Arm Integration** — `gamma_arm_dynamics` bridges PX4 flight control with Gamma-series robotic-arm dynamics.
3. **Extensive Gazebo Simulation** — Custom worlds, models, and plugins target indoor laboratory manipulation.
4. **ROS 2 Integration** — uXRCE-DDS, MAVROS, the Gazebo-ROS bridge, and a complete ROS 2 Humble Docker environment.
5. **Camera Feedback Pipeline** — Support for OAK-D and Intel RealSense cameras.
6. **Differential Rover Support** — A complete rover-control stack alongside the quadcopter stack.

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

- [`Prescribed Performance Control of Aerial Manipulators based on Variable-Gain ESO.pdf`](docs/references/PreGME:%20Prescribed%20Performance%20Control%20of%20Aerial%20Manipulators%20based%20on%20Variable-Gain%20ESO.pdf) — Theoretical foundation of PreGME
- [`Parameter Reference.pdf`](docs/references/PreGME:Parameter%20Reference.pdf) — Parameter reference manual
