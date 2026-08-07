# 模块索引

VisionFlow-PX4 包含 51 个 PX4 模块，分为标准模块和自定义模块两大类。

## 模块分类

| 类别 | 模块数 | 说明 |
|------|--------|------|
| 控制 | 12 | 姿态、位置、速率、自整定等 |
| 状态估计 | 6 | EKF2、LPE、着陆点估计等 |
| 仿真 | 8 | Gazebo 桥接、传感器模拟等 |
| 通信 | 4 | MAVLink、Zenoh、uXRCE-DDS、muORB |
| 传感器 | 3 | 传感器初始化、校准、温度补偿 |
| 系统管理 | 6 | 指挥官、日志、负载监控等 |
| 其他 | 12 | 电池、事件、数据管理等 |

## 自定义模块

以下模块是本项目的核心贡献：

| 模块 | 说明 |
|------|------|
| `pregme_att_control` | PreGME 滑模预设性能姿态控制 |
| `pregme_pos_control` | PreGME 滑模预设性能位置控制 |
| `uxrce_dds_client` | uXRCE-DDS 客户端 |
| `muorb` | micro-ORB 聚合器 |
| `camera_feedback` | 相机反馈处理 |

## 标准 PX4 模块（部分）

| 模块 | 说明 |
|------|------|
| `ekf2` | 扩展卡尔曼滤波器 v2 |
| `commander` | 飞行模式和管理器 |
| `navigator` | 任务导航 |
| `control_allocator` | 执行器分配 |
| `gimbal` | 云台管理器 |
| `logger` | ULog 数据记录 |
| `mavlink` | MAVLink 协议处理 |

---

*注：PreGME 控制器和 Gamma 机械臂集成文档已移至主侧边栏，可直接从导航进入。
