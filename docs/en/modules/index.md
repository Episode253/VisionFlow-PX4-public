# Module Index

VisionFlow-PX4 contains 51 PX4 modules, divided into standard modules and custom modules.

## Module Categories

| Category | Module Count | Description |
|------|--------|------|
| Control | 12 | Attitude, position, rate, auto-tuning, etc. |
| State Estimation | 6 | EKF2, LPE, landing point estimation, etc. |
| Simulation | 8 | Gazebo bridges, sensor simulation, etc. |
| Communication | 4 | MAVLink, Zenoh, uXRCE-DDS, muORB |
| Sensors | 3 | Sensor initialization, calibration, temperature compensation |
| System Management | 6 | Commander, logger, load monitoring, etc. |
| Other | 12 | Battery, events, data management, etc. |

## Custom Modules

The following modules are the core contributions of this project:

| Module | Description |
|------|------|
| `pregme_att_control` | PreGME sliding mode prescribed performance attitude control |
| `pregme_pos_control` | PreGME sliding mode prescribed performance position control |
| `mc_nn_control` | Neural network control based on TensorFlow Lite Micro |
| `rover_differential` | Differential drive rover controller |
| `zenoh` | Zenoh DDS alternative middleware |
| `uxrce_dds_client` | uXRCE-DDS client |
| `muorb` | micro-ORB aggregator |
| `camera_feedback` | Camera feedback processing |

## Standard PX4 Modules (Partial)

| Module | Description |
|------|------|
| `ekf2` | Extended Kalman Filter v2 |
| `commander` | Flight mode and manager |
| `navigator` | Mission navigation |
| `control_allocator` | Actuator allocation |
| `gimbal` | Gimbal manager |
| `logger` | ULog data logging |
| `mavlink` | MAVLink protocol processing |

## Detailed Documentation

- [PreGME Controllers](pregme-controllers/index.md) — Prescribed performance control details
- [Gamma Arm Integration](gamma-arm-integration/index.md) — Arm dynamics and plugins
- [Neural Network Control](neural-network-control/index.md) — TFLite integration
- [Rover Controller](rover-controller/index.md) — Ground robot control
- [Zenoh Middleware](zenoh-middleware/index.md) — Lightweight DDS alternative
