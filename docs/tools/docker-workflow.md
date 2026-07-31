# Docker Workflow

This page provides a detailed guide to the VisionFlow-PX4 Docker workflow, including image build process, container management, the `run_gz_sitl.sh` startup script, Entity profile configuration, and troubleshooting.

## Directory Structure

```
docker/
├── Dockerfile.humble-gz          # PX4 + Gazebo + ROS2 base image
├── compose.yaml                  # Docker Compose service definition
├── gz_sitl_profiles.conf         # Simulation Entity Profile configuration
├── run_gz_sitl.sh               # Main SITL launch script
├── into_gz_sitl.sh              # Helper script to enter a running container
├── run_flight_review.sh         # Flight Review launch script
└── entrypoint.sh                # Container entrypoint script
```

Cache directories:

```
docker/cache/
├── ccache/   # ccache compiler cache (accelerates repeated builds)
└── gz/       # Gazebo model / material cache
```

---

## Docker Image Build

### Basic Information

| Property | Value |
|------|---|
| Base image | `osrf/ros:humble-desktop-full` |
| Target image | `visionflow-px4:humble-gz` |
| Container name | `visionflow-px4-sitl` |
| Default user inside container | `px4` (UID/GID synced with host) |

### What's Included in the Image

1. **PX4 Build Toolchain** — CMake, Ninja, Python dependencies, NuttX cross-compiler
2. **Gazebo Harmonic** — Simulation engine plus `ros-humble-ros-gzharmonic` bridge package
3. **ROS 2 Humble** — Full Desktop-Full installation
4. **MAVROS** — Built from local source in `thirdparty/mavros-humble/`
5. **Flight Review dependencies** — Bokeh, PyULog, Tornado, etc. (`/opt/flight_review_venv/`)
6. **Gamma Arm Web GUI** — System libraries for the arm control panel (QtWebEngine, GTK3, etc.)
7. **Chinese fonts** — `fonts-noto-cjk` (for CJK UI display)

### Build Arguments

| Argument | Default | Description |
|------|--------|------|
| `USER_UID` | `1000` | UID of the `px4` user inside the container; should match the host to avoid file permission issues |
| `USER_GID` | `1000` | GID of the `px4` user |
| `INSTALL_XTENSA_ESP` | `0` | Whether to download the Xtensa ESP compiler (set to `1` only if needed on x86_64) |

Build command:

```bash
# Normal build (uses host proxy settings)
bash docker/run_gz_sitl.sh --build

# Force clean rebuild
docker compose -f docker/compose.yaml build --no-cache px4-humble-gz
```

### APT Mirror Configuration

The Dockerfile automatically replaces Ubuntu and ROS 2 APT sources with Aliyun mirrors to accelerate builds in mainland China:

- `archive.ubuntu.com` → `mirrors.aliyun.com/ubuntu`
- `packages.ros.org/ros2/ubuntu` → `mirrors.aliyun.com/ros2/ubuntu`

The build process verifies the replacement; if any official `packages.ros.org` entries remain, the build fails with an error.

### PyPI Mirror

Uses Tsinghua TUNA mirror (`pypi.tuna.tsinghua.edu.cn`) to avoid HTML deprecation warnings from the Aliyun pyside6-addons index.

---

## Docker Compose Configuration

`docker/compose.yaml` defines a single service `px4-humble-gz`:

```yaml
services:
  px4-humble-gz:
    build:
      context: ..                        # Project root
      dockerfile: docker/Dockerfile.humble-gz
      network: host                      # Host network during build (access local proxy)
      args:
        USER_UID: ${USER_UID:-1000}
        USER_GID: ${USER_GID:-1000}
        # Proxy variables forwarded as build ARGs
        http_proxy: ${http_proxy:-}
        https_proxy: ${https_proxy:-}
        ...
    image: visionflow-px4:humble-gz
    container_name: visionflow-px4-sitl
    volumes:
      - ..:/workspace/VisionFlow-PX4:rw          # Full project directory (rw)
      - ./cache/ccache:/home/px4/.ccache:rw      # ccache compiler cache
      - ./cache/gz:/home/px4/.gz:rw              # Gazebo resource cache
      - /tmp/.X11-unix:/tmp/.X11-unix:rw         # X11 display socket
      - /dev/dri:/dev/dri                         # GPU device passthrough
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - QTWEBENGINE_DISABLE_SANDBOX=1
      - NVIDIA_VISIBLE_DEVICES=all
      - CCACHE_DIR=/home/px4/.ccache
      ...
    gpus: all          # Enable NVIDIA GPU
    privileged: true   # Privileged mode (required by Gazebo)
    network_mode: host # Host network (MAVLink/ROS2 ports exposed directly)
    shm_size: "2gb"    # Shared memory size
```

### Mount Volume Reference

| Mount Path | Type | Purpose |
|---------|------|------|
| `/workspace/VisionFlow-PX4` (from host `..`) | Host directory (rw, bidirectional) | Full codebase access; changes inside the container reflect on the host immediately |
| `./cache/ccache` → `/home/px4/.ccache` | Host directory | ccache compiler cache for faster repeated builds |
| `./cache/gz` → `/home/px4/.gz` | Host directory | Gazebo model/material cache to avoid re-downloading |
| `/tmp/.X11-unix` | Host socket | X11 display forwarding |
| `/dev/dri` | Device | OpenGL / GPU passthrough |

---

## Simulation Launch Script

### `run_gz_sitl.sh` — Main Launch Script

**Usage:**

```bash
bash docker/run_gz_sitl.sh --profile "Entity 4"        # Launch specified profile
bash docker/run_gz_sitl.sh --build --profile "Entity 1" # Rebuild image then launch
bash docker/run_gz_sitl.sh --list                       # List all available profiles
bash docker/run_gz_sitl.sh                              # Interactive selection (default: Entity 1)
```

**Execution flow:**

```
[1/6] Create cache dirs   docker/cache/ccache, docker/cache/gz
[2/6] Set script perms    chmod +x entrypoint.sh
[3/6] Export user ID      USER_UID, USER_GID (synced with host)
[4/6] Allow X11 access    xhost +local:docker
[5/6] Build image         docker compose build (only with --build)
[6/6] Run container       docker compose run → PX4 SITL + Gazebo
```

**UORB ucdr Header Watchdog:**

The script includes a built-in watchdog that detects stalls during uORB ucdr header generation:

- Monitors the `Generating uORB topic ucdr headers` phase
- If log file size hasn't changed for `PX4_UCDR_HEADER_STALL_TIMEOUT` (default 5s), the build is terminated and automatically retried
- Maximum retries: `PX4_UCDR_HEADER_RETRIES` (default 1)
- On detecting stale cache (CMakeCache conflict), automatically cleans `build/docker/px4_sitl_default` and retries

Adjustable via environment variables:

```bash
export PX4_UCDR_HEADER_STALL_TIMEOUT=10  # Stall detection timeout (seconds)
export PX4_UCDR_HEADER_WATCH_INTERVAL=5  # Check interval (seconds)
export PX4_UCDR_HEADER_RETRIES=2         # Max retry count
```

### `gz_sitl_profiles.conf` — Profile Configuration

Each Entity Profile is defined by the following fields (pipe-delimited):

| Field | Description | Example |
|------|------|------|
| `id` | Unique profile identifier | `Entity 4` |
| `name` | Display name | `PreGME (Company) New Version` |
| `world` | Gazebo world filename | `laboratory_no_landingbox` |
| `target` | PX4 build target | `gz_swan_gamma_v2_laboratory_no_landingbox` |
| `extra` | Extra CMake arguments | `-DENABLE_LOCKSTEP_SCHEDULER=ON` |
| `pose` | Initial drone pose (optional) | `0,0,1.15392,0,0,0` (x,y,z,roll,pitch,yaw) |

**Adding a new profile:**

Append to the end of `gz_sitl_profiles.conf`:

```bash
add_sitl_profile \
    --id "Entity 8" \
    --name "My Custom Model" \
    --world "my_world" \
    --target "gz_my_drone_my_world" \
    --pose "0,0,0.5,0,0,0"    # optional
```

---

## Entering the Container

### `into_gz_sitl.sh` — Interactive Shell Entry

```bash
bash docker/into_gz_sitl.sh                  # Default: start Web control + interactive shell
bash docker/into_gz_sitl.sh --no-web         # Skip Web control startup
bash docker/into_gz_sitl.sh --no-gui         # Headless mode (no Gazebo GUI)
bash docker/into_gz_sitl.sh --gui=0          # Same as above
```

**Available environment variables:**

| Variable | Default | Description |
|------|--------|------|
| `GUI_ENABLE` | `1` | Launch Gazebo GUI |
| `LOG_ENABLE` | `0` | Enable verbose logging |
| `AUTO_RESTART` | `1` | Auto-restart Web control on crash |
| `GZ_KEEPALIVE` | `0` | Auto-restart Gazebo on crash |
| `WEB_PORT` | `9000` | Web control UI port |
| `ROSBRIDGE_PORT` | `9090` | rosbridge WebSocket port |

**Helper commands after entering the container:**

| Command | Description |
|------|------|
| `webstart` | Start Gamma arm Web control |
| `webstart_gui` | Start with embedded GUI |
| `webstart_headless` | Start Web backend only (no GUI) |
| `webstop` | Stop Web control |
| `weblog` | Show Web control logs |
| `webattach` | Attach to tmux Web control session |
| `webps` | Show related processes |
| `webcheck` | Comprehensive check (processes, HTTP, ROS topics) |
| `croot` | Change to project root directory |
| `gzlist` | List Gazebo topics |
| `gzps` | Show simulation-related processes |

---

## Flight Log Review

### `run_flight_review.sh`

Launches the Flight Review Web application inside a Docker container:

```bash
bash docker/run_flight_review.sh              # Launch (using existing image)
bash docker/run_flight_review.sh --build      # Rebuild image then launch
```

Access at: `http://127.0.0.1:5006/upload`

**Environment variables:**

| Variable | Default | Description |
|------|--------|------|
| `FR_PORT` | `5006` | Flight Review Web port |
| `CONTAINER_NAME` | `visionflow-flight-review` | Container name |
| `SERVICE_NAME` | `px4-humble-gz` | Compose service to use |

---

## Troubleshooting

### Container Fails to Start

```bash
# Check Docker service status
docker info

# View container logs
docker logs visionflow-px4-sitl

# Check for port conflicts (common MAVLink/MAVROS ports)
lsof -i :14540
lsof -i :14550
lsof -i :9000
```

### X11 Display Issues

```bash
# Allow container to access X11
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY   # Should output :0 or :1
```

### GPU Not Recognized

```bash
# Confirm NVIDIA Container Toolkit is installed
nvidia-smi

# Test GPU container
docker run --rm --gpus all nvidia/cuda:12.2-base nvidia-smi
```

### uORB ucdr Header Stall

The built-in watchdog handles this automatically. If it persists:

```bash
# Clear PX4 build cache
rm -rf build/docker/px4_sitl_default

# Or clear all caches and rebuild
bash docker/run_gz_sitl.sh --build
```

### Arm Plugin Not Loading

```bash
# Manually build and install inside the container
cd /workspace/VisionFlow-PX4/windshape_dev/plugins/gamma_arm_control
cmake -S . -B build
cmake --build build
sudo cmake --install build
```

### Cache Cleanup

```bash
# Clear ccache (saves disk space, next build will be slower)
rm -rf docker/cache/ccache/*

# Clear Gazebo cache (models will be re-downloaded)
rm -rf docker/cache/gz/*

# Clear PX4 build artifacts
rm -rf build/docker/
```

---

## Related Pages

- [Quick Deployment Toolchain](../getting-started/index.md)
- [Docker Launch Guide](../getting-started/docker-launch.md)
- [Prerequisites](../getting-started/prerequisites.md)
- [Flight Log Review](flight-review.md)
- [Maintenance Guide](../development/maintenance.md)
