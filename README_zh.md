# VisionFlow-PX4

> **🔄 活跃开发中** — 本仓库**每天持续更新**，请定期关注此页面并拉取最新更改以保持项目同步。
>
> - **最后更新**：查看 [提交记录](https://github.com/Renwang-Huang/VisionFlow-PX4/commits)
> - **版本落后？** 运行 `git pull origin main` 同步到最新版本

> **📖 Documentation: [English](README.md) ·  [中文](README_zh.md)**

## 概述

> 由 **WindyLab** 开发的 PX4 Autopilot 定制版，将无人机与机械臂（Gamma 系列）集成，用于 Gazebo 仿真中的操控任务，具备预设性能制导与管理估计器（PreGME）控制及 ROS2 集成。

- **PreGME 控制器** — 以滑模预设性能控制（PPC）算法全面替代标准 `mc_att_control` 和 `mc_pos_control`，包含质心补偿和组合误差状态观测器（CESO）。
- **Gamma 机械臂集成** — 通过 `gamma_arm_dynamics` 库实现 PX4 飞控与 Gamma 系列机械臂动力学的紧耦合。
- **丰富 Gazebo 仿真** — 自定义世界、模型和插件，用于室内实验室操控场景（降落箱、VLA 任务、硬件在环）。
- **ROS2 生态** — Zenoh 中间件、uXRCE-DDS 客户端以及完整的 ROS2 Humble Docker 环境。

## 环境要求

| 环境 / 组件 | 版本或信息 |
|-------------|-----------|
| 操作系统 | Ubuntu 22.04 |
| ROS 2 版本 | Humble |
| Gazebo Sim 版本 | Harmonic V8.11.0 |
| Ros-GZ Bridge 版本 | `ros-humble-ros-gz-harmonic` |
| PX4 版本 | V1.17.0 |
| QGC 版本 / 下载地址 | <https://github.com/Renwang-Huang/VisionFlow-PX4/releases/tag/V1.17.0> |

## 快速开始

### 查看可用构建目标

`ninja -t targets` 列出所有可构建目标，用 `grep` 过滤特定关键词即可查找，以 `gz_q940_ti` 为例：

```bash
# 查看所有 gz_ 开头的仿真目标
ninja -C build/px4_sitl_default -t targets | grep "^gz_"

# 按关键词筛选，例如查找 q940_ti 相关目标
ninja -C build/px4_sitl_default -t targets | grep gz_q940_ti

# 查找所有 swan_gamma 相关目标
ninja -C build/px4_sitl_default -t targets | grep gz_swan_gamma
```

### 本地启动

以下命令直接在本地宿主机上运行 PX4 SITL + Gazebo：

| Profile | 描述 | 命令 |
|---------|------|------|
| Entity 1 | PreGME q940_ti 带降落箱（季梦玉） | `PX4_GZ_WORLD=laboratory_landingbox make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 2 | PreGME q940_ti VLA 任务 | `PX4_GZ_WORLD=laboratory_landingbox_vla_task0 make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox_vla_task0 EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 3 | Swan gamma v1（公司旧版） | `PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_swan_gamma_v1_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 4 | Swan gamma v2（公司新版） | `PX4_GZ_MODEL_POSE="0,0,1.15392,0,0,0" PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 5 | Swan gamma v2 VLA 任务 | `PX4_GZ_WORLD=laboratory_no_landingbox_vla_task0 make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox_vla_task0 EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 6 | X500 带云台 | `PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_x500_gimbal_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |
| Entity 7 | 差速小车 | `PX4_GZ_MODEL_POSE="0,0,0.5,0,0,0" make px4_sitl gz_differential_rover_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"` |

### Docker 启动（推荐）

Docker 方式封装了完整的 ROS2 Humble + Gazebo 环境，推荐首次使用或需要隔离开发环境的场景：

```bash
# 交互式选择 — 从菜单中选取 Profile
bash docker/run_gz_sitl.sh

# 列出可用 Profile
bash docker/run_gz_sitl.sh --list

# 使用默认 Profile（Entity 1）启动
bash docker/run_gz_sitl.sh --profile "Entity 1"

# 重新构建镜像 + 启动
bash docker/run_gz_sitl.sh --build --profile "Entity 4"
```

#### ⚠️ 何时使用 `--build` 标志

`--build` 标志会重新构建 Docker 镜像，速度**较慢**但在特定情况下是**必需的**：

| 场景 | 需要 `--build` | 操作 |
|------|----------------|------|
| **首次启动仿真** | ✅ **需要** | `bash docker/run_gz_sitl.sh --build --profile "Entity 4"` (30-60 分钟) |
| **修改了 Dockerfile** | ✅ **需要** | `bash docker/run_gz_sitl.sh --build --profile "Entity 4"` |
| **Dockerfile 和依赖未改变** | ❌ **不需要** | `bash docker/run_gz_sitl.sh --profile "Entity 4"` (10-30 秒) |
| **重新运行仿真** | ❌ **不需要** | `bash docker/run_gz_sitl.sh --profile "Entity 4"` (最快) |

#### 快速参考

```bash
# 首次运行 — 必须构建
bash docker/run_gz_sitl.sh --build --profile "Entity 4"

# 后续运行 — 跳过构建（⚡ 快得多）
bash docker/run_gz_sitl.sh --profile "Entity 4"

# 编辑 Dockerfile 或 Tools/setup/requirements.txt 之后
bash docker/run_gz_sitl.sh --build --profile "Entity 4"
```

**说明**：构建时间取决于网络连接速度。中国用户受益于内置的阿里云镜像加速。

### 附加节点

```bash
# 数据桥接（Gazebo ↔ ROS2）
bash windshape_dev/image_stream/bridge_gz_ros.sh

# 相机流可视化
bash windshape_dev/image_stream/camera_stream.sh

# 机械臂 Web 控制
bash docker/into_gz_sitl.sh

# MAVROS 节点
source thirdparty/install/setup.bash && ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557

# HITL 仿真
gz sim -r Tools/simulation/gz/worlds/laboratory_landingbox_hitl.sdf
```

## 维护与故障排查

遇到问题？查阅维护指南获取帮助：[中文](docs/development/maintenance.md) ｜ [English](docs/en/development/maintenance.md)

| 内容 | 说明 |
|------|------|
| 联系维护者 | 报告问题或寻求支持的渠道 |
| Issue & PR 流程 | 从报告 BUG 到提交修复的完整工作流 |
| 故障排查 | 编译 / Docker / Gazebo / PX4 运行时 / ROS2 通信 |
| 调试与恢复 | 日志分析、调试技巧、缓存清理、Git 回退 |

## 仓库结构

以下目录树展示已跟踪的源码/配置目录及主要维护的项目组件。本地构建、缓存、日志、IDE 和运行时产物不在此展示。

### 主目录

```
VisionFlow-PX4/
├── .github/                  # CI 工作流、仓库配置和模板
├── boards/                   # 10 个厂商命名空间；45 个板级目标
├── cmake/                    # PX4 CMake 辅助工具
├── docker/                   # ROS 2 Humble + Gazebo 工作流
├── docs/                     # 文档源文件
├── msg/                      # uORB 消息和 ROS 2 消息工具
├── platforms/                # Common、NuttX、POSIX、QURT 和 ROS 2 支持
├── posix-configs/            # POSIX/SITL 配置
├── ROMFS/                    # PX4 运行时文件、初始化脚本和机架配置
├── site/                     # MkDocs 生成的静态站点
├── src/                      # PX4 源码
├── thirdparty/               # MAVROS Humble 工作空间
├── Tools/                    # 构建、分析、消息、仿真和实用工具
├── validation/               # 模块配置 Schema
└── windshape_dev/            # WindyLab 工具和集成
```
### PX4 源码树

`src/` 目录除上游 PX4 区域外，还包含以下项目特定模块和库：

```
src/
├── drivers/                 # 传感器和外设驱动
├── examples/                # 示例应用
├── include/                 # PX4 公共头文件
├── lib/                     # PX4 库；项目新增：
│   ├── gamma_arm_dynamics/  # Gamma 机械臂动力学库
│   └── controllib/           # 扩展控制库（PID、blocks）
├── modules/                 # 控制、估计、中间件、系统和仿真模块：
│   ├── pregme_att_control/  # PreGME 姿态控制器（滑模 PPC）
│   ├── pregme_pos_control/  # PreGME 位置控制器
│   ├── mc_nn_control/       # 神经网络控制（TensorFlow Lite Micro）
│   ├── camera_feedback/     # 相机触发处理
│   ├── gimbal/              # 云台管理
│   ├── local_position_estimator/ # 基于块的 LPE
│   ├── rover_differential/  # 差速小车控制器
│   ├── zenoh/               # DDS 替代中间件
│   ├── muorb/               # micro-ORB 聚合器
│   ├── temperature_compensation/ # 逐传感器温度标定
│   └── simulation/          # Gazebo 桥接、插件和传感器模拟器
├── systemcmds/              # 系统命令模块
└── templates/               # 模块模板
```

### 消息与接口

```
msg/
├── *.msg                    # 212 个当前顶层消息定义
├── versioned/               # 37 个版本化消息定义
├── px4_msgs_old/msg/        # 18 个历史消息定义
└── translation_node/        # ROS 2 消息翻译包
```

当前共约 267 个 `.msg` 文件，分布于 current、versioned 和 legacy 消息树中。

### 工具与仿真

```
Tools/
├── ci/                       # CI 辅助
├── ecl_ekf/                  # EKF 分析工具
├── HIL/                      # 硬件在环工具
├── kconfig/                  # Kconfig 工具
├── module_config/            # 模块配置工具
├── msg/                      # 消息工具
├── px4airframes/             # 机架工具
├── python_scripts/           # PX4 Python 实用脚本
├── setup/                    # 安装辅助
└── simulation/
    ├── gz/
    │   ├── worlds/           # Gazebo 世界
    │   ├── models/           # Gazebo 模型
    │   ├── sdf_parsing/      # SDF 解析工具
    │   └── server.config     # Gazebo 服务器配置
    └── iscca_model/          # ISCCA URDF、网格和 RViz 资源
```

### 文档与容器

```
docs/
├── architecture/             # 系统架构
├── development/              # 开发指南
├── getting-started/          # 安装与启动指南
├── hardware/                 # 板级与硬件文档
├── messages/                 # 消息文档
├── modules/                  # 模块文档
├── references/               # 论文与参考页面
├── simulation/               # 仿真文档
├── tools/                    # 工具文档
└── en/                       # 英文文档树

docker/
├── Dockerfile.humble-gz      # ROS 2 Humble + Gazebo 镜像
├── compose.yaml              # Docker Compose 配置
├── entrypoint.sh             # 容器入口
├── gz_sitl_profiles.conf     # SITL Profile 定义
├── into_gz_sitl.sh           # 进入 SITL 容器
├── run_flight_review.sh      # 启动 Flight Review
└── run_gz_sitl.sh            # 基于 Profile 的 SITL 启动器
```

### WindyLab 项目

```
windshape_dev/
├── arm_control/
│   └── gamma_arm/            # Gamma 机械臂 GUI 和 Web 控制
├── code_reference/           # ROS 2/ament 参考项目和包
│   ├── pregme_v1_13/         # PreGME 参考项目快照
│   └── windylab_gamma_arm_01_v2/ # Gamma 机械臂参考项目快照
├── data_plotting/
│   └── local_position/       # 本地位置和里程计绘图
├── flight_review/
│   ├── app/                  # Web 日志回放服务器、绘图和 3D 处理
│   └── data/                 # Flight Review 数据、日志和缓存
├── image_stream/             # Gazebo-ROS 桥接和相机流
├── parameter/                # Gazebo、PX4 参数和 RViz 静态配置
├── plugins/                  # 自定义 Gazebo 插件
│   ├── gamma_arm_control/
│   ├── joint_position_controller/
│   └── px4_gzsim_bridge/
├── px4_original_tools/       # PX4 MAVLink、日志和标定实用脚本
└── uav_control/              # 键盘、手柄和 Offboard 控制
    ├── keyboard/
    └── offboard/
```

## 自定义模块

### PreGME 控制器（核心研究贡献）

| 模块 | 用途 | 关键特性 |
|------|------|----------|
| `pregme_att_control` | 姿态控制 | 滑模 PPC、CESO、惯性矩阵、角速度限制、轨迹预设 |
| `pregme_pos_control` | 位置控制 | 滑模 PPC、lambda_p/Kp 逐轴增益、起飞、碰撞约束 |

### 仿真栈

| 模块 | 用途 |
|------|------|
| `gz_bridge` | Gazebo-PX4 执行器混合（ESC、舵机、轮组、云台） |
| `gz_plugins` | 自定义 Gazebo 插件（generic_motor、gstreamer、motor_failure、moving_platform、buoyancy、airspeed） |
| `simulator_mavlink` | MAVLink 仿真桥接 |
| `simulator_sih` | 软件在环仿真器 |
| `sensor_*_sim` | GPS、磁力计、气压计、空速、AGP 传感器模拟器 |

## 仿真资源

### 世界（`Tools/simulation/gz/worlds/`）

| 世界 | 描述 |
|------|------|
| `laboratory_landingbox.sdf` | 主实验室带降落箱 |
| `laboratory_landingbox_vla_task0.sdf` | 带 VLA 任务的实验室 |
| `laboratory_no_landingbox.sdf` | 无降落箱的实验室 |
| `laboratory_no_landingbox_vla_task0.sdf` | 无降落箱 VLA 任务 |
| `laboratory_landingbox_hitl.sdf` | 硬件在环版本 |
| `indoor_dining.sdf` | 室内餐厅环境 |
| `baylands_coast.sdf` | 海岸环境 |

### 模型（`Tools/simulation/gz/models/`）

| 模型 | 描述 |
|------|------|
| `q940_ti_gripper3/`、`q940_ti_gripper4/` | Q940TI 无人机带 3/4 指夹爪 |
| `swan_gamma_v1/`、`swan_gamma_v2/` | Swan 无人机带 Gamma 臂（旧版/新版） |
| `x500_gimbal/`、`x500_base/` | X500 四旋翼变体 |
| `ti5_arm/` | TI5 机械臂 |
| `differential_rover/` | 差速小车 |
| `Intel_realsense_d435/` | Intel RealSense D435 相机 |
| 家居物品 | landing_box、red_coke_can、cracker_box、bookshelf、drawer、depot 等 |

## 机架配置

自定义 POSIX 机架位于 `ROMFS/px4fmu_common/init.d-posix/airframes/`：

| 机架 ID | 描述 |
|---------|------|
| `4001_gz_x500` | 标准 X500 四旋翼 |
| `4002_gz_differential_rover` | 差速小车 |
| `4003_gz_x500_gimbal` | X500 带云台 |
| `4004_gz_q940_ti_gripper3` | Q940TI 带 gripper3 |
| `4005_gz_swan_gamma_v1` | Swan 带 Gamma 臂 v1 |
| `4006_gz_q940_ti_gripper4` | Q940TI 带 gripper4 |
| `4007_gz_swan_gamma_v2` | Swan 带 Gamma 臂 v2 |

## 自定义 uORB 消息

| 消息 | 用途 |
|------|------|
| `ArmJointState.msg` | 机械臂关节状态 |
| `CollisionConstraints.msg` | 碰撞避障约束 |
| `NeuralControl.msg` | 神经网络控制状态 |
| `FigureEightStatus.msg` | 八字形轨迹跟踪 |
| `PositionControllerLandingStatus.msg` | 降落状态 |
| `TrajectorySetpoint6dof.msg` | 6-DOF 轨迹设定点 |
| `Rover*` 系列 | 小车专用控制消息 |
| `pos_helper.msg` | 位置辅助 |

## 板级支持

| 板卡 | MCU | 描述 |
|------|-----|------|
| `hkust/nxt-dual` | STM32 | 双 IMU 定制板 |
| `hkust/nxt-v1` | STM32 | 单板变体 |
| 10 个厂商命名空间（含 HKUST） | — | 共 45 个板级目标 |

## 固件构建

| 构建目标 | 平台 | 描述 |
|---------|------|------|
| `px4_sitl_default` | POSIX | 当前主要 Gazebo SITL 构建目标 |
| `px4_fmu-v6x_default` | NuttX | 支持的 STM32H7 FMUv6X 目标，按需生成 |

## 与原版 PX4 的主要差异

1. **PreGME 控制器** — 滑模 PPC 替代标准多旋翼控制器（核心研究贡献）
2. **机械臂集成** — `gamma_arm_dynamics` 桥接 PX4 飞控与 Gamma 系列机械臂动力学
3. **重度 Gazebo 仿真** — 自定义世界、模型、插件，面向室内实验室操控
4. **ROS2 集成** — Zenoh 中间件、uXRCE-DDS、Gazebo-ROS 桥接、完整 ROS2 Humble Docker
5. **相机反馈管线** — OAK-D 和 Intel RealSense 支持，带地理标记
6. **差速小车支持** — 与四旋翼并行的完整小车控制栈

## 引用

如果你在研究中使用了本代码库，请引用相关论文：

```bibtex
@article{ji2025pregme,
  title={PreGME: Prescribed Performance Control of Aerial Manipulators based on Variable-Gain ESO},
  author={Ji, Mengyu and Guo, Shiliang and Li, Zhengzhen and Shen, Jiahao and Cao, Huazi and Zhao, Shiyu},
  journal={arXiv preprint arXiv:2512.22957},
  year={2025}
}
```

## 文档

- [`docs/references/PreGME: Prescribed Performance Control of Aerial Manipulators based on Variable-Gain ESO.pdf`](docs/references/PreGME:%20Prescribed%20Performance%20Control%20of%20Aerial%20Manipulators%20based%20on%20Variable-Gain%20ESO.pdf) — PreGME 理论基础
- [`docs/references/PreGME:Parameter Reference.pdf`](docs/references/PreGME:Parameter%20Reference.pdf) — 参数参考手册
