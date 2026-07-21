# Quick Start

Welcome to the Quick Start guide of the VisionFlow-PX4 development manual. This section will help you set up the development environment from scratch and run your first simulation.

## Choose a Launch Method

| Method | Advantages | Use Case |
|------|------|---------|
| **Docker (Recommended)** | Environment isolation, one-click deployment, complete dependencies | First-time use, team collaboration |
| **Native Deployment** | Direct access to system resources, easy debugging | Advanced users, GPU passthrough needs |

## Getting Started

### Step 1: Verify Environment

See [Prerequisites](prerequisites.md) to ensure your system meets the OS, ROS2, Gazebo, and other requirements.

### Step 2: Launch Simulation

**Docker method:**
```bash
bash docker/run_gz_sitl.sh --profile "Entity 1"
```

**Native method:**
```bash
PX4_GZ_WORLD=laboratory_landingbox \
  make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### Step 3: Verify

See [Quick Verification](quick-start.md) to confirm the environment is working properly.

## Simulation Configuration Overview

| Profile | Drone | Scene | Description |
|---------|--------|------|------|
| Entity 1 | q940_ti_gripper4 | laboratory_landingbox | Main test platform |
| Entity 2 | q940_ti_gripper4 | vla_task0 | VLA task |
| Entity 3 | swan_gamma_v1 | laboratory_no_landingbox | Company legacy version |
| Entity 4 | swan_gamma_v2 | laboratory_no_landingbox | Company new version |
| Entity 5 | swan_gamma_v2 | vla_task0 | VLA task |
| Entity 6 | x500_gimbal | laboratory_no_landingbox | Gimbal test |
| Entity 7 | differential_rover | laboratory_no_landingbox | Rover test |

## Next Steps

- [Docker Launch Details](docker-launch.md)
- [Native Launch Details](native-launch.md)
- [System Architecture](../architecture/overview.md)
- [Core Modules](../modules/index.md)
