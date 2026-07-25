---
# https://vitepress.dev/reference/default-theme-home-page
layout: home

title: VisionFlow-PX4
titleTemplate: UAV–Robotic Arm Collaborative Manipulation Platform · Simulation & Development Toolchain

hero:
  name: "VisionFlow-PX4"
  text: "Simulation & Development Toolchain"
  tagline: A customized PX4 Autopilot fork integrating PreGME controllers, Gamma arm dynamics, and the ROS2 ecosystem.
  image:
    src: /logo.svg
    alt: VisionFlow-PX4
  actions:
    - theme: brand
      text: Get Started
      link: /getting-started/
    - theme: alt
      text: GitHub Source
      link: https://github.com/Renwang-Huang/VisionFlow-PX4

features:
  - title: PreGME Controllers
    details: Full replacement of standard mc_att_control and mc_pos_control with sliding-mode Prescribed Performance Control (PPC), including variable-gain Extended State Observer (CESO) and CoM coupling compensation. Core research contribution.
    link: /modules/pregme-controllers/
    linkText: Learn controllers
  - title: Gamma Arm Integration
    details: gamma_arm_dynamics library computes UAV+arm combined center of mass from 6-DOF joint angles, injecting CoM coupling compensation into both attitude and position controllers.
    link: /modules/gamma-arm-integration/
    linkText: Arm dynamics
  - title: Rich Gazebo Simulation
    details: Custom laboratory worlds, swan_gamma / q940 models, and C++ Gazebo plugins covering indoor manipulation, landing boxes, VLA tasks, and hardware-in-the-loop scenarios.
    link: /simulation/
    linkText: Simulation assets
  - title: ROS2 Ecosystem
    details: Zenoh middleware, uXRCE-DDS client, complete ROS2 Humble Docker environment and MAVROS — an out-of-the-box SITL workflow.
    link: /architecture/communication-stack
    linkText: Communication stack
  - title: Camera / Visual Feedback
    details: OAK-D and Intel RealSense vision sensor integration supporting real-time object detection and geotagging. Visual stream feeds into the control loop via PX4 VisionFlow pipeline.
    link: /architecture/overview/
    linkText: Vision pipeline
  - title: Docker SITL Workflow
    details: One-command launch of 7 entity-configured Gazebo SITL simulations, with built-in arm plugin build, uORB header generation watchdog, and auto-retry.
    link: /getting-started/docker-launch
    linkText: Docker launch

search: false
---
