# 快速开始

欢迎来到 VisionFlow-PX4 开发手册的快速开始指南。本章节将帮助您从零开始搭建开发环境并运行第一个仿真。

## 选择启动方式

| 方式 | 优点 | 适用场景 |
|------|------|---------|
| **Docker（推荐）** | 环境隔离、一键部署、依赖完整 | 首次使用、团队协作 |
| **本地部署** | 直接访问系统资源、调试方便 | 高级用户、GPU 直通需求 |

## 快速上手

### 第一步：确认环境

参见 [环境要求](prerequisites.md)，确保满足操作系统、ROS2、Gazebo 等要求。

### 第二步：启动仿真

**Docker 方式：**
```bash
bash docker/run_gz_sitl.sh --profile "Entity 1"
```

**本地方式：**
```bash
PX4_GZ_WORLD=laboratory_landingbox \
  make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### 第三步：验证

参见 [快速验证](quick-start.md)，确认环境正常工作。

## 仿真配置一览

| Profile | 无人机 | 场景 | 说明 |
|---------|--------|------|------|
| Entity 1 | q940_ti_gripper4 | laboratory_landingbox | 主要测试平台 |
| Entity 2 | q940_ti_gripper4 | vla_task0 | VLA 任务 |
| Entity 3 | swan_gamma_v1 | laboratory_no_landingbox | 公司旧版 |
| Entity 4 | swan_gamma_v2 | laboratory_no_landingbox | 公司新版 |
| Entity 5 | swan_gamma_v2 | vla_task0 | VLA 任务 |
| Entity 6 | x500_gimbal | laboratory_no_landingbox | 云台测试 |
| Entity 7 | differential_rover | laboratory_no_landingbox | 小车测试 |

## 下一步

- [Docker 启动详解](docker-launch.md)
- [本地启动详解](native-launch.md)
- [系统架构](../architecture/overview.md)
- [核心模块](../modules/index.md)
