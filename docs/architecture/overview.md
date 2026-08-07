# System Architecture Overview

VisionFlow-PX4 is a customized fork based on PX4 Autopilot V1.17.0, designed specifically for UAV-arm collaborative operation simulation.

## Overall Architecture

```mermaid
graph TB
    subgraph "Perception Layer"
        S1[IMU / Accelerometer]
        S2[Gyroscope]
        S3[Magnetometer]
        S4[GPS]
        S5[Barometer]
        S6[Depth Camera OAK-D]
        S7[RealSense D435]
    end

    subgraph "State Estimation Layer"
        E1[EKF2 Extended Kalman Filter]
        E2[LPE Local Position Estimation]
        E3[Landing Point Estimation]
        E4[Declination Estimation]
        E5[Temperature Compensation]
    end

    subgraph "Control Layer"
        C1[PreGME Position Control]
        C2[PreGME Attitude Control]
        C3[Standard MC Position Control]
        C4[Standard MC Attitude Control]
        C5[Standard MC Attitude Control]
    end

    subgraph "Actuator Allocation"
        A1[Control Allocator]
    end

    subgraph "Execution Layer"
        M1[Motor / ESC]
        M2[Servo]
        M3[Gamma Arm]
        M4[Gimbal]
    end

    subgraph "Communication Layer"
        T1[uORB Messages]
        T2[MAVLink]
        T3[Zenoh DDS]
        T4[uXRCE-DDS]
    end

    subgraph "Simulation Layer"
        G1[Gazebo Simulator]
        G2[Gazebo Plugin]
    end

    S1 --> E1
    S2 --> E1
    S3 --> E1
    S4 --> E1
    S5 --> E1
    S6 --> E2
    S7 --> E2
    S6 --> E3

    E1 --> C1
    E2 --> C1
    E3 --> C1

    C1 --> C2
    C2 --> A1

    C3 --> C4
    C4 --> A1

    A1 --> M1
    A1 --> M2
    A1 --> M3
    A1 --> M4

    G1 --> G2
    G2 --> A1
    M3 --> G2

    T1 <--> E1
    T1 <--> C1
    T1 <--> C2
    T2 <--> T1
    T3 <--> T1
    T4 <--> T1
```

## Key Design Decisions

### Dual Controller Coexistence

Standard PX4 controllers (`mc_att_control` / `mc_pos_control`) and PreGME controllers (`pregme_att_control` / `pregme_pos_control`) coexist simultaneously. The active controller is selected via the airframe configuration.

### Modular Architecture

Each control function is implemented as an independent PX4 module, communicating via uORB messages. This design enables:
- New controllers to be developed in parallel without affecting existing functionality
- Sensor simulators to operate independently from control logic
- Arm integration to be implemented through independent Gazebo plugins

### Multi-Level Simulation Support

| Simulation Level | Description | Use Case |
|---------|------|---------|
| SITL | Software-in-the-loop, PX4 runs on host | Controller development, parameter tuning |
| Gazebo | Full physics simulation | System integration testing |
| HITL | Hardware-in-the-loop, real flight controller connected | Firmware verification, hardware testing |
| SIH | Simulation-in-hardware | Algorithm prototype validation |

## Differences from Standard PX4

1. **PreGME Controllers** — Sliding-mode PPC replacing standard MPC
2. **Gamma Arm Integration** — `gamma_arm_dynamics` bridges flight controller and arm
3. **Enhanced Simulation Stack** — Custom worlds, models, plugins
4. **Native ROS2 Integration** — Zenoh, uXRCE-DDS, Gazebo-ROS Bridge
5. **Camera Feedback Pipeline** — OAK-D and Intel RealSense with geotagging
