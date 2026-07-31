# X500 变体

本页详细介绍 X500 标准四旋翼平台的 Gazebo 模型及其变体配置。X500 作为基准测试平台，用于对比评估 PreGME 算法与标准 PX4 控制器的性能差异。

> **TODO**: 本页正在建设中，内容将逐步完善。

## 变体列表

| 变体 | 说明 | Build Target |
|------|------|--------------|
| `x500` | 标准 X500 四旋翼（基础模型） | `make px4_sitl gz_x500_<world>` |
| `x500_gimbal` | X500 + 相机云台 | `make px4_sitl gz_x500_gimbal_<world>` |
| `x500_lidar` | X500 + 机顶固态 3D LiDAR（发布 `/x500_lidar/scan`） | `make px4_sitl gz_x500_lidar_<world>` |

## 相关页面

- [仿真模型总览](index.md)
- [Q940TI 平台](q940-ti.md)
- [仿真概述](../index.md)
