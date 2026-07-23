# VisionFlow-PX4

> **🔄 ACTIVE DEVELOPMENT** — This repository is **actively maintained with daily updates**. Please check this page regularly and pull the latest changes to stay synchronized with the project.
>
> - **Last Updated**: See [Commits](https://github.com/Renwang-Huang/VisionFlow-PX4/commits)
> - **Falling behind?** Run `git pull origin main` to sync with the latest version

> **📖 Documentation: [English](README.md) |  [中文](README_zh.md)**

## Overview

> A customized PX4 Autopilot fork developed by **WindyLab**, integrating UAVs with robotic arms (Gamma series) for manipulation tasks in Gazebo simulation, featuring Prescribed Performance Guidance and Management Estimator (PreGME) control and ROS2 integration.

- **PreGME Controllers** — Full replacement of standard `mc_att_control` and `mc_pos_control` with sliding-mode Prescribed Performance Control (PPC) algorithms, including centroid compensation and composite error state observer (CESO).
- **Gamma Arm Integration** — Tight coupling between PX4 flight control and Gamma-series robotic arm dynamics via the `gamma_arm_dynamics` library.
- **Rich Gazebo Simulation** — Custom worlds, models, and plugins for indoor laboratory manipulation scenarios (landing boxes, VLA tasks, hardware-in-the-loop).
- **ROS2 Ecosystem** — Zenoh middleware, uXRCE-DDS client, and a complete ROS2 Humble Docker environment.

## Prerequisites

| Component | Version / Info |
|-----------|---------------|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble |
| Gazebo Sim | Harmonic V8.11.0 |
| Ros-GZ Bridge | `ros-humble-ros-gz-harmonic` |
| PX4 | V1.17.0 |
| QGC Download | <https://github.com/Renwang-Huang/VisionFlow-PX4/releases/tag/V1.17.0> |

## Quick Start

### Check Available Build Targets

Use `ninja -t targets` to list all available build targets, and filter with `grep`:

```bash
# List all gz_ simulation targets
ninja -C build/px4_sitl_default -t targets | grep "^gz_"

# Filter by keyword, e.g. q940_ti
ninja -C build/px4_sitl_default -t targets | grep gz_q940_ti

# Find all swan_gamma targets
ninja -C build/px4_sitl_default -t targets | grep gz_swan_gamma
```

### Native Launch (Local Deployment)

The following commands run PX4 SITL + Gazebo directly on the host machine:

| Profile | Description | Command |
|---------|-------------|---------|
| Entity 1 | PreGME q940_ti with landing box (季梦玉) | `PX4_GZ_WORLD=laboratory_landingbox make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 2 | PreGME q940_ti with VLA task | `PX4_GZ_WORLD=laboratory_landingbox_vla_task0 make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox_vla_task0 EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 3 | Swan gamma v1 (company legacy) | `PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_swan_gamma_v1_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 4 | Swan gamma v2 (company new) | `PX4_GZ_MODEL_POSE="0,0,1.15392,0,0,0" PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 5 | Swan gamma v2 with VLA task | `PX4_GZ_WORLD=laboratory_no_landingbox_vla_task0 make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox_vla_task0 EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 6 | X500 with gimbal | `PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_x500_gimbal_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 7 | Differential drive rover | `PX4_GZ_MODEL_POSE="0,0,0.5,0,0,0" make px4_sitl gz_differential_rover_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |

### Docker-Based Launch (Recommended)

Docker wraps a complete ROS2 Humble + Gazebo environment. Recommended for first-time use or when an isolated dev environment is needed:

```bash
# Interactive mode — select a profile from the menu
bash docker/run_gz_sitl.sh

# List available profiles
bash docker/run_gz_sitl.sh --list

# Launch with default profile (Entity 1)
bash docker/run_gz_sitl.sh --profile "Entity 1"

# Rebuild Docker image + launch
bash docker/run_gz_sitl.sh --build --profile "Entity 4"
```

#### ⚠️ When to Use `--build` Flag

The `--build` flag reconstructs the Docker image, which is **slow** but necessary in specific cases:

| Scenario | Need `--build`? | Action |
|----------|-----------------|--------|
| **First time launching** | ✅ **YES** | `bash docker/run_gz_sitl.sh --build --profile "Entity 4"` (30-60 min) |
| **After modifying Dockerfile** | ✅ **YES** | `bash docker/run_gz_sitl.sh --build --profile "Entity 4"` |
| **Dockerfile or dependencies unchanged** | ❌ **NO** | `bash docker/run_gz_sitl.sh --profile "Entity 4"` (10-30 sec) |
| **Re-running simulation** | ❌ **NO** | `bash docker/run_gz_sitl.sh --profile "Entity 4"` (fastest) |

#### Quick Reference

```bash
# First run — must build
bash docker/run_gz_sitl.sh --build --profile "Entity 4"

# Subsequent runs — skip build (⚡ much faster)
bash docker/run_gz_sitl.sh --profile "Entity 4"

# After editing Dockerfile or Tools/setup/requirements.txt
bash docker/run_gz_sitl.sh --build --profile "Entity 4"
```

**Note**: Build time depends on your internet connection. China-based users benefit from built-in Aliyun mirror acceleration.

### Additional Nodes

```bash
# Data bridge (Gazebo ↔ ROS2)
bash windshape_dev/image_stream/bridge_gz_ros.sh

# Camera stream visualization
bash windshape_dev/image_stream/camera_stream.sh

# Arm web control
bash docker/into_gz_sitl.sh

# MAVROS node
source thirdparty/install/setup.bash && ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557

# HITL simulation
gz sim -r Tools/simulation/gz/worlds/laboratory_landingbox_hitl.sdf
```

## Maintenance & Troubleshooting

Having issues? Check the maintenance guide: [中文](docs/development/maintenance.md) | [English](docs/en/development/maintenance.md)

| Topic | Description |
|-------|-------------|
| Contact Maintainer | Channels for reporting issues or seeking support |
| Issue & PR Workflow | Full workflow from bug report to merged fix |
| Troubleshooting | Build / Docker / Gazebo / PX4 runtime / ROS2 communication |
| Debug & Recovery | Log analysis, debugging tips, cache cleanup, git rollback |

## Repository Structure

The trees below show tracked source/configuration directories and the main
maintained project components. Local build, cache, log, IDE, and runtime
artifacts are not expanded as source trees; generated content is identified
where it is relevant.

### Main Directory Tree

```
VisionFlow-PX4/
├── .github/                  # CI workflows, repository instructions, and templates
├── boards/                   # 10 vendor namespaces; 45 board targets
├── cmake/                    # PX4 CMake helpers
├── docker/                   # ROS 2 Humble + Gazebo workflow
├── docs/                     # Documentation source; see documentation tree below
├── msg/                      # uORB messages and ROS 2 message tooling
├── platforms/                # Common, NuttX, POSIX, QURT, and ROS 2 support
├── posix-configs/            # POSIX/SITL configurations
├── ROMFS/                    # PX4 runtime files, init scripts, and airframes
├── site/                     # Tracked generated MkDocs static site
├── src/                      # PX4 source; see PX4 source tree below
├── thirdparty/               # Vendored MAVROS Humble workspace
├── Tools/                    # Build, analysis, messaging, simulation, and utility tools
├── validation/               # Module configuration schema
└── windshape_dev/            # WindyLab tools and integrations; see project tree below
```

`build/`, `docker/cache/`, `thirdparty/build/`, `thirdparty/install/`,
`thirdparty/log/`, and plugin `build/` directories are local generated or
runtime content. The current local PX4 artifacts are
`build/px4_sitl_default/` and `build/docker/px4_sitl_default/`.

### PX4 Source Tree

The `src/` tree includes upstream PX4 areas in addition to the selected
project-specific modules and libraries shown here:

```
src/
├── drivers/                 # Sensor and peripheral drivers
├── examples/                # Example applications
├── include/                 # Shared PX4 headers
├── lib/                     # PX4 libraries; selected project additions:
│   ├── gamma_arm_dynamics/  # Gamma robotic arm dynamics library
│   └── controllib/           # Extended control library (PID, blocks)
├── modules/                 # Control, estimation, middleware, system, and simulation modules:
│   ├── pregme_att_control/  # PreGME attitude controller (sliding-mode PPC)
│   ├── pregme_pos_control/  # PreGME position controller
│   ├── mc_nn_control/       # Neural network control (TensorFlow Lite Micro)
│   ├── camera_feedback/     # Camera trigger processing
│   ├── gimbal/              # Gimbal manager
│   ├── local_position_estimator/ # Block-based LPE
│   ├── rover_differential/  # Differential rover controller
│   ├── zenoh/               # DDS alternative middleware
│   ├── muorb/               # micro-ORB aggregator
│   ├── temperature_compensation/ # Per-sensor temperature calibration
│   └── simulation/          # Gazebo bridge, plugins, and sensor simulators
├── systemcmds/              # System command modules
└── templates/               # Module templates
```

### Message and Interface Tree

```
msg/
├── *.msg                    # 212 current top-level message definitions
├── versioned/               # 37 versioned message definitions
├── px4_msgs_old/msg/        # 18 legacy message definitions
└── translation_node/        # ROS 2 message translation package
```

There are currently about 267 `.msg` files across the current, versioned,
and legacy message trees. `translation_node/` is a ROS 2 package and does not
belong to the `.msg` file count.

### Tools and Simulation Tree

`Tools/` contains more than simulation assets. The following is a focused
navigation tree for its main utility and simulation areas:

```
Tools/
├── ci/                       # CI helpers
├── ecl_ekf/                  # EKF analysis tools
├── filepaths/                # File path utilities
├── HIL/                      # Hardware-in-the-loop tools
├── kconfig/                  # Kconfig tooling
├── log_encryption/           # Log encryption tools
├── module_config/            # Module configuration tooling
├── msg/                      # Message tooling
├── px4airframes/             # Airframe tooling
├── px4events/                # Event tooling
├── python_scripts/           # General PX4 Python utilities
├── serial/                   # Serial utilities
├── setup/                    # Setup helpers
└── simulation/
    ├── gz/
    │   ├── worlds/           # Gazebo worlds
    │   ├── models/            # Gazebo models
    │   ├── sdf_parsing/       # SDF parsing utilities
    │   └── server.config      # Gazebo server configuration
    └── iscca_model/           # ISCCA URDF, meshes, and RViz assets
```

### Documentation and Container Trees

```
docs/
├── architecture/             # System architecture
├── development/              # Development guides
├── getting-started/          # Setup and launch guides
├── hardware/                 # Board and hardware documentation
├── messages/                 # Message documentation
├── modules/                  # Module documentation
├── references/               # Papers and reference pages
├── simulation/               # Simulation documentation
├── tools/                    # Tool documentation
└── en/                       # English documentation tree

docker/
├── Dockerfile.humble-gz      # ROS 2 Humble + Gazebo image
├── compose.yaml              # Docker Compose setup
├── entrypoint.sh             # Container entrypoint
├── gz_sitl_profiles.conf     # SITL profile definitions
├── into_gz_sitl.sh           # Enter the SITL container
├── run_flight_review.sh      # Launch flight review
└── run_gz_sitl.sh            # Profile-based SITL launcher
```

### WindyLab Project Tree

```
windshape_dev/
├── arm_control/
│   └── gamma_arm/            # Gamma arm GUI and web control
├── code_reference/            # ROS 2/ament reference projects and packages
│   ├── pregme_v1_13/          # PreGME reference project snapshot
│   └── windylab_gamma_arm_01_v2/ # Gamma arm reference project snapshot
├── data_plotting/
│   └── local_position/        # Local-position and odometry plotting
├── flight_review/
│   ├── app/                   # Web log-review server, plotting, and 3D handlers
│   └── data/                  # Flight-review data, logs, and cache
├── image_stream/              # Gazebo-ROS bridge and camera streaming
│   ├── bridge_gz_ros.sh       # Gazebo to ROS 2 topic bridge
│   └── camera_stream.sh       # Camera stream visualization
├── parameter/                 # Static Gazebo, PX4 parameter, and RViz configs
├── plugins/                   # Custom Gazebo plugins
│   ├── gamma_arm_control/
│   ├── joint_position_controller/
│   └── px4_gzsim_bridge/
├── px4_original_tools/        # PX4 utility scripts for MAVLink, logs, and calibration
└── uav_control/               # Keyboard, joystick, and offboard control
    ├── keyboard/
    └── offboard/
```

`code_reference/` contains ROS 2/ament packages and robotics reference code;
MoveIt-related launch files, acados-generated MPC code, and Pinocchio-based
components are inside those snapshots rather than being direct child
directories. `validation/module_schema.yaml` is the Cerberus schema for
module configuration files.

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
| 10 vendor namespaces (including HKUST) | — | 45 board targets across stock and custom families |

## Firmware Builds

The table below lists supported build targets; it does not imply that every
target already has a generated directory in the current workspace.

| Build Target | Platform | Description |
|-------------|----------|-------------|
| `px4_sitl_default` | POSIX | Current primary Gazebo SITL build artifact/target |
| `px4_fmu-v6x_default` | NuttX | Supported STM32H7 FMUv6X target, generated on demand |

## Key Differences from Stock PX4

1. **PreGME Controllers** — Sliding-mode PPC replaces standard MC controllers (core research contribution)
2. **Robotic Arm Integration** — `gamma_arm_dynamics` bridges PX4 flight control with Gamma-series arm dynamics
3. **Heavy Gazebo Simulation** — Custom worlds, models, plugins for indoor lab manipulation
4. **ROS2 Integration** — Zenoh middleware, uXRCE-DDS, Gazebo-ROS bridge, complete ROS2 Humble Docker
5. **Camera Feedback Pipeline** — OAK-D and Intel RealSense support with geotagging
6. **Differential Rover Support** — Full rover control stack alongside quadcopter

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
