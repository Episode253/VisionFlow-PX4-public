# Native Launch Guide

The native launch method runs PX4 SITL + Gazebo directly on the host machine, suitable for scenarios requiring direct access to system resources.

## View Available Build Targets

```bash
# View all simulation targets starting with gz_
ninja -C build/px4_sitl_default -t targets | grep "^gz_"

# Filter by keyword
ninja -C build/px4_sitl_default -t targets | grep gz_q940_ti
ninja -C build/px4_sitl_default -t targets | grep gz_swan_gamma
```

## Launch Commands

### Entity 1 — PreGME q940_ti with Landing Box

```bash
PX4_GZ_WORLD=laboratory_landingbox \
  make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### Entity 2 — PreGME q940_ti VLA Task

```bash
PX4_GZ_WORLD=laboratory_landingbox_vla_task0 \
  make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox_vla_task0 \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### Entity 3 — Swan Gamma v1 (Company Legacy)

```bash
PX4_GZ_WORLD=laboratory_no_landingbox \
  make px4_sitl gz_swan_gamma_v1_laboratory_no_landingbox \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### Entity 4 — Swan Gamma v2 (Company New)

```bash
PX4_GZ_WORLD=laboratory_no_landingbox \
  make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### Entity 5 — Swan Gamma v2 VLA Task

```bash
PX4_GZ_WORLD=laboratory_no_landingbox_vla_task0 \
  make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox_vla_task0 \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### Entity 6 — X500 Gimbal

```bash
PX4_GZ_WORLD=laboratory_no_landingbox \
  make px4_sitl gz_x500_gimbal_laboratory_no_landingbox \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### Entity 7 — Differential Drive Rover

```bash
PX4_GZ_MODEL_POSE="0,0,0.5,0,0,0" \
  make px4_sitl gz_differential_rover_laboratory_no_landingbox \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

## Launch Additional Nodes

### Data Bridge (Gazebo ↔ ROS2)

```bash
bash windshape_dev/data_stream/gz_bridge/bridge_gz_ros.sh
```

### Camera Stream Visualization

```bash
bash windshape_dev/data_stream/image_stream/camera_stream.sh
```

### MAVROS Node

```bash
source thirdparty/install/setup.bash && \
  ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557
```

### Hardware-in-the-Loop Simulation

```bash
gz sim -r Tools/simulation/gz/worlds/laboratory_landingbox_hitl.sdf
```

## Parameter Description

| Parameter | Description |
|------|------|
| `PX4_GZ_WORLD` | Specify the Gazebo simulation scene |
| `PX4_GZ_MODEL_POSE` | Model initial pose (x,y,z,roll,pitch,yaw) |
| `EXTRA_CMAKE_ARGS` | Extra arguments passed to CMake |
| `ENABLE_LOCKSTEP_SCHEDULER` | Enable lockstep scheduler to ensure simulation timing synchronization |

## FAQ

### Slow Simulation Startup

The first launch requires compiling Gazebo plugins. Using ccache can speed up subsequent compilations:

```bash
export CCACHE_MAX_SIZE=20G
export CCACHE_DIR=~/.ccache
```

### Missing Gazebo Models

Ensure GAZEBO_MODEL_PATH includes the project model directory:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/Tools/simulation/gz/models
```
