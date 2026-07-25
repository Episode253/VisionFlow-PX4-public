# Simulation Overview

VisionFlow-PX4 provides rich Gazebo simulation assets, supporting full-process simulation from controller development to system integration testing.

## Simulation Assets Overview

```mermaid
graph LR
    subgraph "Scene Worlds"
        W1[laboratory_landingbox]
        W2[laboratory_no_landingbox]
        W3[laboratory_landingbox_vla_task0]
        W4[indoor_dining]
        W5[baylands_coast]
        W6[laboratory_landingbox_hitl]
    end

    subgraph "Drone Platforms"
        M1[q940_ti]
        M2[swan_gamma]
        M3[x500]
    end

    subgraph "Manipulator Arms/End Effectors"
        M4[gamma_arm]
        M5[ti5_arm]
        M6[gripper1-4]
    end

    subgraph "Sensors"
        S1[RealSense D435]
        S2[OAK-D-lite]
        S3[mono_cam]
    end

    subgraph "Environmental Objects"
        O1[Furniture]
        O2[Manipulation Objects]
        O3[Building Structures]
    end

    W1 --> M1
    W1 --> M2
    W2 --> M3
    M1 --> M4
    M1 --> M6
    M2 --> M4
    M2 --> M6
    M1 --> S1
    M1 --> S2
```

## Scene Worlds

| Scene | Description | Supported Platforms |
|------|------|---------|
| `laboratory_landingbox` | Main laboratory with landing box | q940_ti, swan_gamma |
| `laboratory_no_landingbox` | Laboratory without landing box | swan_gamma, x500 |
| `laboratory_landingbox_vla_task0` | Laboratory with VLA task | q940_ti |
| `laboratory_no_landingbox_vla_task0` | VLA scenario without landing box | swan_gamma |
| `indoor_dining` | Indoor dining environment | General |
| `baylands_coast` | Bay Area coastal outdoor environment | General |
| `laboratory_landingbox_hitl` | Hardware-in-the-loop version | q940_ti_hitl |

For detailed scene descriptions, please refer to [Scene Worlds](worlds/index.md).

## Drone Platforms

| Platform | Model | Description |
|------|------|------|
| q940_ti | Q940TI + 3-finger/4-finger gripper | Primary test platform |
| swan_gamma_v1/v2 | Swan + Gamma arm | Company version (old/new) |
| x500 | Standard X500 quadcopter | Baseline test platform |
| differential_rover | Differential drive rover | Ground robot platform |

## Simulation Configuration

All simulations enable the lockstep scheduler to ensure timing accuracy:

```bash
EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

## Next Steps

- [Scene Worlds Details](worlds/index.md)
- [Model Details](models/index.md)
- [Asset Usage Guide](assets-guide.md)
