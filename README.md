# VisionFlow-PX4

## Overview

> A customized PX4 Autopilot fork developed by **WindyLab**, integrating UAVs with robotic arms (Gamma series) for manipulation tasks in Gazebo simulation, featuring Prescribed Performance Guidance and Management Estimator (PreGME) control and ROS2 integration.

- **PreGME Controllers** — Full replacement of standard `mc_att_control` and `mc_pos_control` with sliding-mode Prescribed Performance Control (PPC) algorithms, including centroid compensation and composite error state observer (CESO).
- **Gamma Arm Integration** — Tight coupling between PX4 flight control and Gamma-series robotic arm dynamics via the `gamma_arm_dynamics` library.
- **Neural Network Control** — TensorFlow Lite Micro integration for learned control policies alongside traditional PPC.
- **Rich Gazebo Simulation** — Custom worlds, models, and plugins for indoor laboratory manipulation scenarios (landing boxes, VLA tasks, hardware-in-the-loop).
- **ROS2 Ecosystem** — Zenoh middleware, uXRCE-DDS client, and a complete ROS2 Humble Docker environment.

## Quick Start

### Check Available Build Targets

```bash
ninja -C build/px4_sitl_default -t targets | grep gz_q940_ti
```

### Launch Simulation Environments

| Profile | Description | Command |
|---------|-------------|---------|
| Entity 1 | PreGME q940_ti with landing box (季梦玉) | `PX4_GZ_WORLD=laboratory_landingbox make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 2 | PreGME q940_ti with VLA task | `PX4_GZ_WORLD=laboratory_landingbox_vla_task0 make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox_vla_task0 EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 3 | Swan gamma v1 (company legacy) | `PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_swan_gamma_v1_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 4 | Swan gamma v2 (company new) | `PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 5 | Swan gamma v2 with VLA task | `PX4_GZ_WORLD=laboratory_no_landingbox_vla_task0 make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox_vla_task0 EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 6 | X500 with gimbal | `PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_x500_gimbal_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 7 | Differential drive rover | `PX4_GZ_MODEL_POSE="0,0,0.5,0,0,0" make px4_sitl gz_differential_rover_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |

### Recommended: Docker-Based Launch

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

# Ti5 arm web control
bash windshape_dev/arm_control/ti5_arm_web_control.sh

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
├── doc/                       # PreGME research papers (Parameter reference, PPC theory)
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

Both modules use Chinese-language parameter files (`*_params_zh.yaml`) and were consolidated to a unified V2 version in recent commits.

### Neural Network Control

| Module | Purpose | Key Features |
|--------|---------|--------------|
| `mc_nn_control` | Learned control | TensorFlow Lite Micro integration, motor RPM normalization, thrust coefficient control |

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
| *(+ 10 other vendors)* | — | Stock PX4 boards |

## Firmware Builds

| Build Target | Platform | Description |
|-------------|----------|-------------|
| `px4_sitl_default` | POSIX | Gazebo SITL (primary development target) |
| `px4_fmu-v6x_default` | NuttX | STM32H7 firmware (FMUv6X) |

## Recent Development History

| Commit | Description |
|--------|-------------|
| `a59add95` | Merged Centroid Compensation Algorithm |
| `1b8c2e59` | Standardize PreGME naming (removed version suffixes) |
| `23f53cbd` | Consolidate PreGME into V2 version |
| `d38c2237` | Remove V1 PreGME version |
| `378cf788` | Control law triggered by pose updates instead of angular velocity |
| `442aea22` | Modify world environment |
| `566a0361` | Add XY lateral CESO protection |
| `0f9268dc` | Fix uORB process hanging during compilation |
| `7ee4dea1` | Add Intel RealSense wrist-mounted camera |
| `35e0e3eb` | Add flight review tool and Docker support |

## Key Differences from Stock PX4

1. **PreGME Controllers** — Sliding-mode PPC replaces standard MC controllers (core research contribution)
2. **Robotic Arm Integration** — `gamma_arm_dynamics` bridges PX4 flight control with Gamma-series arm dynamics
3. **Neural Network Control** — TFLite Micro integration alongside traditional PPC
4. **Heavy Gazebo Simulation** — Custom worlds, models, plugins for indoor lab manipulation
5. **ROS2 Integration** — Zenoh middleware, uXRCE-DDS, Gazebo-ROS bridge, complete ROS2 Humble Docker
6. **HKUST Custom Hardware** — nxt-dual and nxt-v1 board configs
7. **Camera Feedback Pipeline** — OAK-D and Intel RealSense support with geotagging
8. **Local Position Estimator** — Block-based LPE as EKF2 alternative
9. **Differential Rover Support** — Full rover control stack alongside quadcopter
10. **Flight Review System** — Web-based log review with PID analysis and 3D visualization

## Documentation

- [`doc/PreGME: Prescribed Performance Control of.pdf`](doc/PreGME:%20Prescribed%20Performance%20Control%20of.pdf) — PreGME theoretical foundation
- [`doc/PreGME:Parameter reference.pdf`](doc/PreGME:Parameter%20reference.pdf) — Parameter reference guide
