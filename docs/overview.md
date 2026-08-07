# VisionFlow-PX4 Developer Manual

> A customized PX4 Autopilot fork integrating UAVs with robotic arms (Gamma series) for manipulation tasks in Gazebo simulation, featuring Prescribed Performance Guidance and Management Estimator (PreGME) control and ROS2 integration.

## Core Features

- **PreGME Controllers** — Full replacement of standard `mc_att_control` and `mc_pos_control` with sliding-mode Prescribed Performance Control (PPC) algorithms, including centroid compensation and composite error state observer (CESO)
- **Gamma Arm Integration** — Tight coupling between PX4 flight control and Gamma-series robotic arm dynamics via the `gamma_arm_dynamics` library
- **Rich Gazebo Simulation** — Custom worlds, models, and plugins for indoor laboratory manipulation scenarios (landing boxes, VLA tasks, hardware-in-the-loop)
- **ROS2 Ecosystem** — Zenoh middleware, uXRCE-DDS client, and a complete ROS2 Humble Docker environment

## Key Differences from Stock PX4

1. **PreGME Controllers** — Sliding-mode PPC replaces standard MC controllers (core research contribution)
2. **Robotic Arm Integration** — `gamma_arm_dynamics` bridges PX4 flight control with Gamma-series arm dynamics
3. **Heavy Gazebo Simulation** — Custom worlds, models, plugins for indoor lab manipulation
4. **ROS2 Integration** — Zenoh middleware, uXRCE-DDS, Gazebo-ROS bridge, complete ROS2 Humble Docker
5. **Camera Feedback Pipeline** — OAK-D and Intel RealSense support with geotagging

## Quick Navigation

| Section | Content |
|------|------|
| [Quick Deployment Toolchain](getting-started/index.md) | Environment setup, Docker launch, native launch |
| [System Architecture](architecture/overview.md) | Control stack, simulation stack, communication stack |
| [Simulation Assets](simulation/index.md) | World scenes, models, asset usage guide |
| [Hardware Support](hardware/index.md) | HKUST boards, supported boards list |
| [Communication Protocols](messages/index.md) | Custom uORB messages, message reference |
| [Development Guide](development/index.md) | Build workflow, module development, contributing |
| [References](references/index.md) | Airframes, parameters, citations |

## Citation

If you use this codebase in your research, please cite the associated paper:

```bibtex
@article{ji2025pregme,
  title={PreGME: Prescribed Performance Control of Aerial Manipulators based on Variable-Gain ESO},
  author={Ji, Mengyu and Guo, Shiliang and Li, Zhengzhen and Shen, Jiahao and Cao, Huazi and Zhao, Shiyu},
  journal={arXiv preprint arXiv:2512.22957},
  year={2025}
}
```
