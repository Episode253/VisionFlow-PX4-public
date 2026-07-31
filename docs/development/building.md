# Build Guide

This page details the build process for VisionFlow-PX4, including Docker and native build methods, CMake options, SITL/real hardware build commands, and troubleshooting for common build issues.

## Docker Method (Recommended)

The Docker method handles all dependencies automatically and is the recommended approach for building and running the project. The complete build-and-launch workflow is encapsulated in the `docker/run_gz_sitl.sh` script.

### Quick Build

```bash
# Build the Docker image (run once, or when the Dockerfile changes)
bash docker/run_gz_sitl.sh --build

# Launch simulation (select an Entity Profile)
bash docker/run_gz_sitl.sh --profile "Entity 4"
```

### List Available Profiles

```bash
bash docker/run_gz_sitl.sh --list
```

Example output:

```
Available SITL profiles:

  1) Entity 1
     name   : PreGME Jimengyu model (laboratory_landingbox)
     world  : laboratory_landingbox
     target : gz_q940_ti_gripper4_laboratory_landingbox
     extra  : -DENABLE_LOCKSTEP_SCHEDULER=ON
     pose   : <airframe default>

  2) Entity 2
     ...
```

### Build Directories

Docker builds place artifacts under `build/docker/` (inside the container: `/workspace/VisionFlow-PX4/build/docker/`), completely isolated from native builds in `build/px4_sitl_default/`, preventing cache conflicts.

```bash
# Clean Docker build artifacts
rm -rf build/docker/
```

### CCache Compiler Cache

The Docker image uses ccache to accelerate repeated compilations. The cache is mounted from the host directory `docker/cache/ccache/` and persists across container rebuilds.

```bash
# Clear ccache (increases next build time)
rm -rf docker/cache/ccache/*

# Check ccache status (inside the container)
ccache -s
```

### Manual Build Inside the Container

For finer-grained build control:

```bash
# Enter the container
bash docker/into_gz_sitl.sh

# Build PX4 SITL inside the container
cd /workspace/VisionFlow-PX4
PX4_GZ_WORLD=laboratory_no_landingbox \
  make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox \
  BUILD_BASE_DIR=build/docker \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

---

## Native Build Method

> **Note**: Native builds require manual installation of all dependencies (see [Prerequisites](../getting-started/prerequisites.md)). The arm Gazebo plugin may also have initialization issues in some environments. The Docker method is the recommended approach.

### Build PX4 SITL

```bash
# Basic SITL build
make px4_sitl

# Build with specific world and airframe
PX4_GZ_WORLD=laboratory_no_landingbox \
  make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### List Available Targets

```bash
# List all Gazebo SITL targets
ninja -C build/px4_sitl_default -t targets | grep "^gz_"
```

### CMake Options

| Option | Default | Description |
|------|--------|------|
| `ENABLE_LOCKSTEP_SCHEDULER` | `ON` | Enable lockstep scheduler for precise simulation timing (recommended) |
| `ENABLE_LOCKSTEP_SCHEDULER` | `OFF` | Disable lockstep for faster execution, may reduce simulation stability |

Pass options via `EXTRA_CMAKE_ARGS`:

```bash
make px4_sitl gz_swan_gamma_v2 \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON -DMICROAIRDOGS_SUPPORT=ON"
```

---

## Real Hardware Compilation

### Pixhawk Series

```bash
# Compile firmware
make px4_fmu-v5_default        # Pixhawk 4
make px4_fmu-v6x_default       # Pixhawk 6X
make px4_fmu-v6c_default       # Pixhawk 6C

# Flash (after connecting via USB)
make px4_fmu-v5_default upload
```

### Other Boards

See [Supported Boards](../hardware/supported-boards.md).

---

## Troubleshooting

### CMake Cache Conflicts (Docker vs Native)

**Symptom**: Strange build errors after switching between Docker and native builds.

**Cause**: Different build directories are used between the Docker container and the host, but stale CMake cache may contain conflicting configurations.

**Solution**:
```bash
# Clean Docker build cache
rm -rf build/docker

# Clean native build cache
rm -rf build/px4_sitl_default
```

### uORB ucdr Header Generation Stalls

**Symptom**: Build hangs at `Generating uORB topic ucdr headers`.

**Cause**: The header generation script does not exit promptly after output stabilizes.

**Solution**: The script has a built-in watchdog that auto-retries. If it persists, increase the timeout:

```bash
export PX4_UCDR_HEADER_STALL_TIMEOUT=10
export PX4_UCDR_HEADER_WATCH_INTERVAL=5
```

Or clear stale cache and retry:

```bash
rm -rf build/docker/px4_sitl_default
bash docker/run_gz_sitl.sh --build
```

### Missing Dependencies

**Symptom**: Build errors about missing headers or libraries.

**Solution**: Refer to [Prerequisites](../getting-started/prerequisites.md) to install dependencies. The Docker method handles all dependencies automatically.

### Gamma Arm Plugin Build Failure

**Symptom**: `gamma_arm_control` plugin fails to compile.

**Solution**:
```bash
cd windshape_dev/plugins/gamma_arm_control
cmake -S . -B build
cmake --build build -j$(nproc)
sudo cmake --install build
```

---

## Related Pages

- [Development Guide Overview](index.md)
- [Adding Modules](adding-modules.md)
- [Quick Start](../getting-started/quick-start.md)
