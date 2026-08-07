# VisionFlow-PX4

> A customized PX4 Autopilot fork (by **WindyLab**) integrating UAVs with robotic arms (Gamma series) for aerial-manipulation tasks in Gazebo simulation. Base: PX4 v1.17.0, ROS2 Humble, Gazebo Harmonic. Primary language of in-code comments and docs is Chinese (bilingual zh/en).

## Project Overview

- **PreGME Controllers** — Custom replacements for `mc_att_control` and `mc_pos_control`. PreGME = *Prescribed Performance Control of Aerial Manipulators based on Variable-Gain ESO* (paper: Ji et al., 2025, arXiv:2512.22957). Uses sliding-mode Prescribed Performance Control (PPC), a variable-gain Extended State Observer (CESO), prescribed-performance preset trajectories, and arm center-of-mass (CoM) coupling compensation.
- **Gamma Arm Integration** — `gamma_arm_dynamics` library computes the combined UAV+arm center of mass from 6-DOF joint angles and feeds CoM coupling compensation into both controllers.
- **Neural / RL Controllers** — Additional experimental controllers: `mc_raptor` (RL policy, `policy.tar`) and `mc_nn_control` (neural network), backed by `rl_tools` and `tensorflow_lite_micro` libs.
- **Gazebo Simulation** — Custom laboratory worlds, swan_gamma / q940 models, and C++ Gazebo plugins for indoor manipulation scenarios.
- **ROS2 Ecosystem** — Zenoh middleware, uXRCE-DDS client, ROS2 Humble Docker environment, MAVROS (thirdparty/).

## Key Directories

| Path | Purpose |
|------|---------|
| `src/modules/pregme_att_control/` | PreGME attitude controller. Main class `UserAttitudeControl`; inner law in `Att_control/Att_Control`. Params prefix `USR_*`. |
| `src/modules/pregme_pos_control/` | PreGME position controller. Main class `PregmePositionControl`; inner law in `PosControl`. Params prefix `PREGME_*`. |
| `src/lib/gamma_arm_dynamics/` | Arm+UAV CoM/dynamics lib. `ArmJointSubscriber` (singleton) reads joint angles and caches system CoM. |
| `src/modules/mc_raptor/` | RL flight controller (RAPTOR). Has own README.md/CHECKLIST.md/module.yaml. |
| `src/modules/mc_nn_control/` | Neural-network controller; publishes `NeuralControl` msg. |
| `src/lib/rl_tools/`, `src/lib/tensorflow_lite_micro/` | ML/RL support libs for NN controllers. |
| `Tools/simulation/gz/` | Gazebo `worlds/`, `models/`, `sdf_parsing/`, `server.config`. |
| `docker/` | ROS2 Humble + Gazebo Docker SITL workflow. |
| `ROMFS/px4fmu_common/init.d-posix/airframes/` | Airframe configs 4004-4007 (see below). |
| `windshape_dev/` | WindyLab tools: arm control GUI, Gazebo plugins, image streaming, offboard scripts, flight review. |
| `docs/` | MkDocs (Material) documentation, bilingual zh (default) / en. |
| `thirdparty/mavros-humble/` | Vendored MAVROS build for ROS2 Humble. |

## Airframes (custom)

| ID | Airframe |
|----|----------|
| 4004 | `4004_gz_q940_ti_gripper3` |
| 4005 | `4005_gz_swan_gamma_v1` |
| 4006 | `4006_gz_q940_ti_gripper4` |
| 4007 | `4007_gz_swan_gamma_v2` |

## Simulation Assets

- **Worlds** (`Tools/simulation/gz/worlds/`): `laboratory_landingbox`, `laboratory_landingbox_hitl`, `laboratory_landingbox_vla_task0`, `laboratory_no_landingbox`, `laboratory_no_landingbox_vla_task0`, plus `baylands_coast`, `indoor_dining`.
- **Key models**: `swan_gamma_v1`, `swan_gamma_v2`, `swan_uav_v1/v2`, `q940*`, `q940_ti_gripper3/4`, `gamma_arm`, `gripper1-4`, `x500_gimbal`, `differential_rover`.
- **Custom Gazebo plugins** live in `windshape_dev/plugins/` (C++): `gamma_arm_control/`, `joint_position_controller/`, `px4_gzsim_bridge/` (MAVLink interface bridge).

## Build Commands

### Native SITL
```bash
PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### Docker SITL (recommended — arm plugin build/install is handled correctly)
```bash
bash docker/run_gz_sitl.sh --profile "Entity 4"   # select profile by id
bash docker/run_gz_sitl.sh --build --profile "Entity 4"   # rebuild image (after Dockerfile change)
bash docker/run_gz_sitl.sh --list                 # list profiles
# No --profile → interactive selection; default profile is "Entity 1"
```
`run_gz_sitl.sh` includes a uORB ucdr header build-stall watchdog with automatic retry and stale-cache detection.

### Docker profiles (`docker/gz_sitl_profiles.conf`, 7 entities)
| ID | Model / Target |
|----|----------------|
| Entity 1 | q940_ti_gripper4, laboratory_landingbox |
| Entity 2 | q940_ti_gripper4, laboratory_landingbox_vla_task0 |
| Entity 3 | swan_gamma_v1 (old), laboratory_no_landingbox |
| Entity 4 | swan_gamma_v2 (new), laboratory_no_landingbox |
| Entity 5 | swan_gamma_v2 (new), laboratory_no_landingbox_vla_task0 |
| Entity 6 | x500_gimbal, laboratory_no_landingbox |
| Entity 7 | differential_rover (小车) |

### List available targets
```bash
ninja -C build/px4_sitl_default -t targets | grep "^gz_"
```

### Docs (VitePress, i18n zh/en — mirrors the official PX4 guide stack)
```bash
cd docs && npm install   # first time only
npm run dev              # local preview at http://localhost:5173/VisionFlow-PX4/
npm run build            # output to ../site/ (outDir); base = /VisionFlow-PX4/
npm run preview          # serve the built site/ locally
```
Sidebar is generated from `docs/<lang>/SUMMARY.md` (GitBook-style nested list) via
`docs/.vitepress/get_sidebar.js`. Config: `docs/.vitepress/config.mjs`. Theme (indigo
brand + hero gradient, medium-zoom): `docs/.vitepress/theme/`. Mermaid renders
client-side via `vitepress-plugin-mermaid`. Content lives in `docs/zh/` (default) and
`docs/en/`; static assets (logo, favicon, PDFs) in `docs/public/`. Set `BASE=/` env var
to build for a root-hosted deploy.

## Code Conventions

- **Language**: C++17 (PX4 style), Python (scripts/tools), Bash (Docker/CI). Custom control code carries Chinese inline comments.
- **Naming**: `snake_case` for files and functions, `PascalCase` for classes.
- **Parameters**: Defined in `.yaml` under module dirs, in **bilingual pairs** (`*_params_en.yaml` / `*_params_zh.yaml`). Attitude params use `USR_*`, position params use `PREGME_*`. CoM compensation toggles: `USR_COM_COMP_EN`, `PREGME_COMCP_EN`.
- **Messages**: uORB `.msg` files in `msg/`. Custom ones: `mavros_gs.msg` (CoM offset), `pos_helper.msg` (estimated disturbance), `NeuralControl.msg`, `versioned/RaptorInput.msg`, `versioned/RaptorStatus.msg`. Register in `msg/CMakeLists.txt`.
- **Commits**: Conventional commits (feat/fix/docs/refactor).

## Important Runtime Detail

Arm joint angles are **not** sent via the `ArmJointState.msg` uORB topic (that file is orphaned/unregistered). At runtime `ArmJointSubscriber` reads 6 joint angles from the MAVLink `DEBUG_FLOAT_ARRAY` named `arm_joint` (via uORB `debug_array`), computes the combined CoM in `gamma_arm_dynamics`, and caches it (`getSystemCom()`) for lock-free reads by both controllers. Arm model constants (masses, inertias, DH table) are hard-coded in `gamma_arm_dynamics_params.hpp::makeDefaultParam()`.

## Testing

- SITL simulation is the primary test method (prefer Docker to avoid arm-plugin init issues seen in native builds — see `TODO.md`).
- CI workflows in `.github/workflows/` (largely upstream PX4): build matrices (`build_all_targets`, `compile_ubuntu`), `sitl_tests`, `ros_integration_tests`, `mavros_tests`, `failsafe_sim`, docs deploy/crowdin, SBOM audits.
- Flight logs analyzed with PX4 Flight Review (`windshape_dev/flight_review/`, `docker/run_flight_review.sh`).

## Common Patterns

- **Adding a new airframe**: Add `.px4board` in `boards/`, airframe script in `ROMFS/.../airframes/`, model SDF in `Tools/simulation/gz/models/`. If parameters cause body oscillation, isolate the airframe with its own tuned params (see `TODO.md` items 4-5).
- **Adding a new world**: Create `.sdf` in `Tools/simulation/gz/worlds/`, add a profile in `docker/gz_sitl_profiles.conf` via `add_sitl_profile`.
- **Modifying controller params**: Edit **both** the `_en` and `_zh` `.yaml` files to keep them in sync, then `make parameters`.
- **Docker cache / mirror issues**: `docker compose -f docker/compose.yaml build --no-cache` or clear `docker/cache/`. Network/registry pull failures may require a domestic mirror (see `TODO.md` item 8).

## Repo Notes

- `TODO.md` — active issue/roadmap list (Chinese). Check it for known problems and their status.
- `README.md` / `README_zh.md` — bilingual project README.
- `site/` — pre-built VitePress output (generated by `cd docs && npm run build`); `build/` — PX4 build output (both generated).

## Maintainer

- **Renwang Huang** — <RenwangHuangX@gmail.com>
- Repository: <https://github.com/Renwang-Huang/VisionFlow-PX4>
