# VisionFlow-PX4 开发手册

> 定制化 PX4 Autopilot 分支。将无人机与机械臂（Gamma 系列）在 Gazebo 仿真中集成用于操作任务，具备 Prescribed Performance Guidance and Management Estimator (PreGME) 控制和 ROS2 集成能力。

## 核心特性

- **PreGME 控制器** — 以滑模预设性能控制 (PPC) 算法完全替代标准 `mc_att_control` 和 `mc_pos_control`，包含质心补偿和复合误差状态观测器 (CESO)
- **Gamma 机械臂集成** — 通过 `gamma_arm_dynamics` 库实现 PX4 飞行控制与 Gamma 系列机械臂动力学的深度耦合
- **丰富的 Gazebo 仿真** — 定制世界、模型和插件，用于室内实验室操作场景（降落箱、VLA 任务、硬件在环）
- **ROS2 生态系统** — Zenoh 中间件、uXRCE-DDS 客户端以及完整的 ROS2 Humble Docker 环境

## 与标准 PX4 的关键区别

1. **PreGME 控制器** — 滑模 PPC 取代标准 MC 控制器（核心研究贡献）
2. **机械臂集成** — `gamma_arm_dynamics` 桥接飞行控制与机械臂动力学
3. **Gazebo 仿真** — 定制世界、模型、插件，专为室内实验室操作设计
4. **ROS2 集成** — Zenoh 中间件、uXRCE-DDS、Gazebo-ROS 桥接、完整 ROS2 Humble Docker
5. **相机反馈管道** — OAK-D 和 Intel RealSense 支持，具备地理标记功能
6. **差动小车支持** — 完整的差动小车控制栈

## 快速导航

| 章节 | 内容 |
|------|------|
| [快速部署工具链](getting-started/index.md) | 环境搭建、Docker 启动、本地启动 |
| [系统架构](architecture/overview.md) | 控制栈、仿真栈、通信栈 |
| [仿真资产](simulation/index.md) | 世界场景、模型、资产使用指南 |
| [硬件支持](hardware/index.md) | HKUST 定制板、支持板卡列表 |
| [通信协议](messages/index.md) | 自定义 uORB 消息、消息参考 |
| [开发指南](development/index.md) | 编译流程、模块开发、贡献指南 |
| [参考资料](references/index.md) | 机架配置、参数参考、引用文献 |

## 引用

如果您在研究中使用此代码库，请引用相关论文：

```bibtex
@article{ji2025pregme,
  title={PreGME: Prescribed Performance Control of Aerial Manipulators based on Variable-Gain ESO},
  author={Ji, Mengyu and Guo, Shiliang and Li, Zhengzhen and Shen, Jiahao and Cao, Huazi and Zhao, Shiyu},
  journal={arXiv preprint arXiv:2512.22957},
  year={2025}
}
```
