# 本地启动指南

本地启动方式直接在宿主机上运行 PX4 SITL + Gazebo，适合需要直接访问系统资源的场景。

## 查看可用构建目标

```bash
# 查看所有 gz_ 开头的仿真目标
ninja -C build/px4_sitl_default -t targets | grep "^gz_"

# 按关键词筛选
ninja -C build/px4_sitl_default -t targets | grep gz_q940_ti
ninja -C build/px4_sitl_default -t targets | grep gz_swan_gamma
```

## 启动命令

### Entity 1 — PreGME q940_ti 带降落箱

```bash
PX4_GZ_WORLD=laboratory_landingbox \
  make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### Entity 2 — PreGME q940_ti VLA 任务

```bash
PX4_GZ_WORLD=laboratory_landingbox_vla_task0 \
  make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox_vla_task0 \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### Entity 3 — Swan Gamma v1（公司旧版）

```bash
PX4_GZ_WORLD=laboratory_no_landingbox \
  make px4_sitl gz_swan_gamma_v1_laboratory_no_landingbox \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### Entity 4 — Swan Gamma v2（公司新版）

```bash
PX4_GZ_WORLD=laboratory_no_landingbox \
  make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### Entity 5 — Swan Gamma v2 VLA 任务

```bash
PX4_GZ_WORLD=laboratory_no_landingbox_vla_task0 \
  make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox_vla_task0 \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### Entity 6 — X500 云台

```bash
PX4_GZ_WORLD=laboratory_no_landingbox \
  make px4_sitl gz_x500_gimbal_laboratory_no_landingbox \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### Entity 7 — 差动驱动小车

```bash
PX4_GZ_MODEL_POSE="0,0,0.5,0,0,0" \
  make px4_sitl gz_differential_rover_laboratory_no_landingbox \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

## 启动额外节点

### 数据桥接（Gazebo ↔ ROS2）

```bash
bash windshape_dev/data_stream/gz_bridge/bridge_gz_ros.sh
```

### 摄像头流可视化

```bash
bash windshape_dev/data_stream/image_stream/camera_stream.sh
```

### MAVROS 节点

```bash
source thirdparty/install/setup.bash && \
  ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557
```

### 硬件在环仿真

```bash
gz sim -r Tools/simulation/gz/worlds/laboratory_landingbox_hitl.sdf
```

## 参数说明

| 参数 | 说明 |
|------|------|
| `PX4_GZ_WORLD` | 指定 Gazebo 仿真场景 |
| `PX4_GZ_MODEL_POSE` | 模型初始位姿（x,y,z,roll,pitch,yaw） |
| `EXTRA_CMAKE_ARGS` | 传递给 CMake 的额外参数 |
| `ENABLE_LOCKSTEP_SCHEDULER` | 启用锁步调度器，确保仿真时序同步 |

## 常见问题

### 仿真启动慢

首次启动需要编译 Gazebo 插件。使用 ccache 可以加速后续编译：

```bash
export CCACHE_MAX_SIZE=20G
export CCACHE_DIR=~/.ccache
```

### 缺少 Gazebo 模型

确保 GAZEBO_MODEL_PATH 包含项目模型目录：

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/Tools/simulation/gz/models
```
