---
# https://vitepress.dev/reference/default-theme-home-page
layout: home

title: VisionFlow-PX4
titleTemplate: 无人机-机械臂协同操作仿真与控制

hero:
  name: "VisionFlow-PX4"
  text: "无人机-机械臂协同操作平台"
  tagline: 定制化 PX4 Autopilot 分支，集成 PreGME 控制器、Gamma 机械臂动力学与 ROS2 生态。由 WindyLab 开发。
  image:
    src: /logo.svg
    alt: VisionFlow-PX4
  actions:
    - theme: brand
      text: 阅读文档
      link: /zh/
    - theme: alt
      text: English Docs
      link: /en/
    - theme: alt
      text: GitHub 源码
      link: https://github.com/Renwang-Huang/VisionFlow-PX4

features:
  - title: PreGME 控制器
    details: 以滑模预设性能控制 (PPC) 完全替代标准 mc_att_control 与 mc_pos_control，包含变增益扩张状态观测器 (CESO) 与质心耦合补偿。核心研究贡献。
    link: /zh/modules/pregme-controllers/
    linkText: 了解控制器
  - title: Gamma 机械臂集成
    details: gamma_arm_dynamics 库根据 6-DOF 关节角计算 UAV+机械臂组合质心，并将质心耦合补偿注入姿态与位置控制器。
    link: /zh/modules/gamma-arm-integration/
    linkText: 机械臂动力学
  - title: 丰富的 Gazebo 仿真
    details: 定制实验室世界、swan_gamma / q940 模型与 C++ Gazebo 插件，覆盖室内操作、降落箱、VLA 任务与硬件在环场景。
    link: /zh/simulation/
    linkText: 仿真资产
  - title: ROS2 生态系统
    details: Zenoh 中间件、uXRCE-DDS 客户端、完整 ROS2 Humble Docker 环境与 MAVROS，开箱即用的 SITL 工作流。
    link: /zh/architecture/communication-stack
    linkText: 通信栈
  - title: 神经 / RL 控制器
    details: 实验性 mc_raptor (RL 策略) 与 mc_nn_control (神经网络) 控制器，由 rl_tools 与 tensorflow_lite_micro 库支撑。
    link: /zh/modules/neural-network-control/
    linkText: 神经网络控制
  - title: Docker SITL 工作流
    details: 一条命令启动 7 种实体配置的 Gazebo SITL 仿真，内置机械臂插件构建、uORB 头文件构建停滞看门狗与自动重试。
    link: /zh/getting-started/docker-launch
    linkText: Docker 启动

search: false
---
