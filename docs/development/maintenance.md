# Maintenance Guide

This page provides the maintenance guide for the VisionFlow-PX4 project, including project maintainer information, issue reporting procedures, bug triage and fix workflows, and common troubleshooting methods.

## Project Maintainers

| Role | Name | Contact |
|------|------|---------|
| Lead Maintainer | Renwang Huang | RenwangHuangX@gmail.com |
| Repository | <https://github.com/Renwang-Huang/VisionFlow-PX4> | |

| Role | Name | Contact |
|------|------|---------|
| Lead Maintainer | Hangan Xu | 1509797189@qq.com |
| Repository | <https://github.com/Renwang-Huang/VisionFlow-PX4> | |

---

To report issues or suggest improvements:

- **GitHub Issues**: Submit to [VisionFlow-PX4 Issues](https://github.com/Renwang-Huang/VisionFlow-PX4/issues)
- **Email**: Contact the maintainer directly

## Issue Reporting Guide

### When to Create an Issue

- You found a bug in code or documentation
- You need a new feature or improvement
- Documentation is missing or incorrect
- You encountered a problem during simulation or runtime that you cannot resolve

### Issue Template Selection

The repository provides the following issue templates. Choose based on the type of problem:

| Template | Use Case | File |
|----------|---------|------|
| 🐛 Bug Report | Abnormal code or simulation behavior | `.github/issue_template/bug_report.yml` |
| 📑 Documentation Bug | Documentation errors or omissions | `.github/issue_template/docs_bug_report.yml` |
| 🚀 Feature Request | New feature suggestions | `.github/issue_template/feature_request.yml` |

### High-Quality Report Checklist

- [ ] Use the correct issue template
- [ ] Title is concise and includes the problem keyword
- [ ] Describe reproduction steps (environment, commands, sequence)
- [ ] Attach relevant logs or screenshots
- [ ] Include PX4 version, Gazebo version, ROS2 version
- [ ] For flight-related issues, upload flight log to [PX4 Flight Review](http://logs.px4.io/)

## Bug Triage and Fix Workflow

### 1. Reproduce the Bug

- Confirm the reproduction environment (Docker / native)
- Record reproduction steps and the Entity Profile used
- Determine if the bug is consistently reproducible

### 2. Locate the Root Cause

**Log Analysis:**

- PX4 runtime logs: Check terminal output, look for `[error]`, `[warn]` markers
- Gazebo logs: Check Gazebo server output
- Docker logs: `docker logs visionflow-px4-sitl`

**Flight Log Analysis:**

- Upload `.ulog` files to [PX4 Flight Review](http://logs.px4.io/)
- Check EKF status, controller output, sensor data

**Code Localization:**

- Add temporary debug output with `printf` / `PX4_ERR` / `PX4_WARN`
- Use `gzdbg` / `gzerr` in Gazebo plugins
- Check uORB topic publication: `uorb top <topic_name>`

### 3. Write the Fix

- Follow project code conventions (see [CLAUDE.md](../../CLAUDE.md))
- Ensure the fix does not break existing functionality
- Update related parameter documentation (if parameters are changed)

### 4. Submit a PR

- Reference [PULL_REQUEST_TEMPLATE.md](https://github.com/Renwang-Huang/VisionFlow-PX4/blob/main/.github/PULL_REQUEST_TEMPLATE.md)
- Use Conventional Commits format for PR titles: `fix:` / `feat:` / `docs:` / `refactor:`
- Link the issue in the PR description (`Fixes #issue_number`)
- Describe the changes and testing methodology

### 5. Verification and Merge

- Ensure CI checks pass (build, static analysis, SITL tests)
- Verify in both Docker and native environments
- Maintainer reviews and merges

## Common Troubleshooting

### Build Issues

#### CMake Cache Conflicts (Docker vs Native)

**Symptom:** Strange build errors after switching between Docker and native builds.

**Cause:** Docker containers and the host use different build directories, but CMake caches may conflict.

**Solution:**
```bash
# Clean Docker build cache
rm -rf build/docker

# Clean local build cache
rm -rf build/px4_sitl_default
```

#### uORB ucdr Header Generation Stalls

**Symptom:** Build hangs at `Generating uORB topic ucdr headers` step.

**Cause:** The header generation script does not exit promptly after output stabilizes.

**Solution:** A built-in watchdog mechanism (`PX4_UCDR_HEADER_STALL_TIMEOUT`, default 5s) automatically detects and retries. If it persists:

```bash
# Increase timeout
export PX4_UCDR_HEADER_STALL_TIMEOUT=10
export PX4_UCDR_HEADER_WATCH_INTERVAL=5
```

#### Missing Dependencies

**Symptom:** Build errors about missing headers or libraries.

**Solution:** Refer to [Prerequisites](../getting-started/prerequisites.md) to install dependencies. Docker handles dependencies automatically.

### Docker Issues

#### Container Fails to Start

**Symptom:** `docker compose run` exits with an error.

**Checklist:**

1. Confirm Docker is running: `docker info`
2. Check for port conflicts: `lsof -i :14540` (MAVROS port)
3. Check X11 permissions: `xhost +local:docker`
4. View container logs: `docker logs visionflow-px4-sitl`

#### X11 Display Issues

**Symptom:** Gazebo GUI cannot display or reports `cannot connect to X server`.

**Solution:**
```bash
# Allow Docker to access X11
xhost +local:docker

# Check DISPLAY environment variable
echo $DISPLAY  # Should output :0 or :1
```

#### Permission / User ID Mismatch

**Symptom:** Files created inside the container are owned by root and cannot be modified on the host.

**Cause:** User ID inside the container differs from the host.

**Solution:** The script automatically exports `USER_UID` and `USER_GID` to the container. If issues persist, set them manually:

```bash
export USER_UID=$(id -u)
export USER_GID=$(id -g)
```

### Gazebo Simulation Issues

#### Model Loading Fails / Missing Models

**Symptom:** Models appear pink or are missing after Gazebo starts.

**Solution:**
```bash
# Ensure GAZEBO_MODEL_PATH includes the project model directory
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/Tools/simulation/gz/models

# First launch requires model downloads; ensure network connectivity
# When using Docker, model cache is at docker/cache/gz/
```

#### Incorrect Drone Initial Position

**Symptom:** The drone appears at the wrong location.

**Cause:** The `PX4_GZ_MODEL_POSE` environment variable is not set correctly.

**Solution:**

- Check the default pose in the airframe config file: `ROMFS/px4fmu_common/init.d-posix/airframes/`
- Check the `--pose` parameter in Docker profiles: `docker/gz_sitl_profiles.conf`
- Pose format: `x,y,z,roll,pitch,yaw` (6 comma-separated values)

#### Arm Plugin Not Loading

**Symptom:** The robotic arm cannot be controlled, or Gazebo reports plugin loading errors.

**Solution:**
```bash
# Manually build and install the Gamma arm control plugin
cd windshape_dev/plugins/gamma_arm_control
cmake -S . -B build
cmake --build build
sudo cmake --install build
```

#### Simulation Timing Desynchronized

**Symptom:** Simulation runs at abnormal speed (too fast or too slow).

**Solution:** Ensure the lockstep scheduler is enabled:

```bash
EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### PX4 Runtime Issues

#### Sensor Calibration Fails

**Symptom:** PX4 reports sensor errors on startup.

**Solution:** In simulation, sensors are simulated by Gazebo. Check:

- Whether the Gazebo sensor plugin is loaded correctly
- Whether `EKF2_SENS_EN` includes the required sensors
- Skip calibration in simulation: `param set CAL_GYRO0_EN 0`

#### EKF Not Converging

**Symptom:** Flight logs show abnormal EKF status or position estimate drift.

**Solution:**

- Check if GPS simulation is working properly
- Check if IMU noise parameters match the model
- Adjust EKF2 parameters: `EKF2_*`

#### Abnormal Controller Output

**Symptom:** The drone exhibits abnormal flight behavior or cannot hover.

**Solution:**

- Check control allocation parameters: `CA_AIRFRAME`, `CA_ROTOR*`
- Check PreGME controller parameters: `USR_LAMBDA_Q_*`, `USR_I_YY`, etc.
- Verify that the motor mapping in the airframe config matches the SDF model

### ROS2 / Communication Issues

#### MAVROS Connection Failure

**Symptom:** `ros2 launch mavros px4.launch` cannot connect.

**Solution:**
```bash
# Confirm PX4 is running and listening on UDP
# PX4 listens on port 14540 by default
ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557
```

#### Zenoh Bridge Not Working

**Symptom:** Zenoh nodes cannot communicate with each other.

**Solution:** Check the Zenoh configuration file and confirm network ports are open.

#### uXRCE-DDS Agent Disconnected

**Symptom:** ROS2 topics cannot receive PX4 data.

**Solution:** Restart the uXRCE-DDS agent:

```bash
# Inside the container
MicroXRCEAgent udp4 -p 8888
```

## Debugging Tips

### Enable Verbose Logging

```bash
# PX4 verbose logging
export PX4_DEBUG=1

# Gazebo verbose logging
export GZ_VERBOSE=3
```

### Using PX4 Flight Review for Log Analysis

1. After running a simulation, logs are saved in `~/PX4/`
2. Upload `.ulog` files to <http://logs.px4.io/>
3. Analyze EKF status, controller performance, sensor data

### Gazebo Debug Tools

```bash
# List Gazebo topics
gz topic -l

# View specific topic data
gz topic -e -t /world/<world_name>/model/<model_name>/pose

# View model pose
gz topic -e -t /gazebo/default/model/<model_name>/pose
```

## Rollback and Recovery

### Docker Cache Cleanup

```bash
# Clean all Docker build caches
docker compose -f docker/compose.yaml build --no-cache

# Clean local cache directories
rm -rf docker/cache/ccache/*
rm -rf docker/cache/gz/*
```

### Build Directory Cleanup

```bash
# Clean PX4 build artifacts
rm -rf build/px4_sitl_default
rm -rf build/docker

# Clean Gamma plugin build artifacts
rm -rf windshape_dev/plugins/gamma_arm_control/build
```

### Git Rollback Strategies

```bash
# View recent commits
git log --oneline -10

# Revert to a specific commit (local)
git reset --hard <commit_hash>

# Discard uncommitted changes
git checkout -- <file>
git restore <file>
```

---

> This guide is continuously updated. If you encounter issues not covered here, please submit an Issue or contact the maintainer.
