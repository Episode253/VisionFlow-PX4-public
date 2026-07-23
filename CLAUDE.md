# VisionFlow-PX4

> A customized PX4 Autopilot fork integrating UAVs with robotic arms (Gamma series) for manipulation tasks in Gazebo simulation.

## Project Overview

- **PreGME Controllers** — Full replacement of `mc_att_control` and `mc_pos_control` with sliding-mode Prescribed Performance Control (PPC) algorithms, including centroid compensation and CESO.
- **Gamma Arm Integration** — Tight coupling between PX4 flight control and Gamma-series robotic arm dynamics via `gamma_arm_dynamics` library.
- **Gazebo Simulation** — Custom worlds, models, and plugins for indoor laboratory manipulation scenarios.
- **ROS2 Ecosystem** — Zenoh middleware, uXRCE-DDS client, and ROS2 Humble Docker environment.

## Key Directories

| Path | Purpose |
|------|---------|
| `src/modules/pregme_att_control/` | PreGME attitude controller (sliding-mode PPC) |
| `src/modules/pregme_pos_control/` | PreGME position controller |
| `src/lib/gamma_arm_dynamics/` | Gamma robotic arm dynamics library |
| `Tools/simulation/gz/` | Gazebo worlds, models, plugins |
| `docker/` | ROS2 Humble + Gazebo Docker workflow |
| `ROMFS/px4fmu_common/init.d-posix/airframes/` | Airframe configs (4004-4007) |
| `windshape_dev/` | WindyLab tools, plugins, data streaming |
| `docs/` | MkDocs documentation (zh/en) |

## Build Commands

### Native SITL
```bash
PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### Docker SITL
```bash
bash docker/run_gz_sitl.sh --profile "Entity 4"
```

### List available targets
```bash
ninja -C build/px4_sitl_default -t targets | grep "^gz_"
```

## Code Conventions

- **Language**: C++17 (PX4 style), Python (scripts), Bash (Docker/CI)
- **Naming**: `snake_case` for files and functions, `PascalCase` for classes
- **Parameters**: Defined in `.yaml` under module dirs, registered via `pregme_pos_control_params_*.yaml`
- **Messages**: uORB `.msg` files in `msg/` directory
- **Commits**: Conventional commits (feat/fix/docs/refactor)

## Testing

- SITL simulation is the primary test method
- CI workflows in `.github/workflows/` cover build checks, SITL tests, ROS integration
- Flight logs can be analyzed with PX4 Flight Review

## Common Patterns

- **Adding a new airframe**: Add `.px4board` in `boards/`, airframe script in `ROMFS/.../airframes/`, model SDF in `Tools/simulation/gz/models/`
- **Adding a new world**: Create `.sdf` in `Tools/simulation/gz/worlds/`, add profile in `docker/gz_sitl_profiles.conf`
- **Modifying controller params**: Edit the `.yaml` param file, regenerate with `make parameters`
- **Docker cache issues**: Run `docker compose -f docker/compose.yaml build --no-cache` or clear `docker/cache/`

## Maintainer

- **Renwang Huang** — <RenwangHuangX@gmail.com>
- Repository: <https://github.com/Renwang-Huang/VisionFlow-PX4>
