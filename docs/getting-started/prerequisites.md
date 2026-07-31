# Prerequisites

Before you begin, please ensure your development environment meets the following requirements.

## System Requirements

| Environment / Component | Version or Info |
|-------------|-----------|
| Operating System | Ubuntu 22.04 LTS |
| ROS 2 Version | Humble Hawksbill |
| Gazebo Sim Version | Harmonic V8.11.0 |
| Ros-GZ Bridge Version | `ros-humble-ros-gz-harmonic` |
| PX4 Version | V1.17.0 |
| QGC Version | [Download Link](https://github.com/Renwang-Huang/VisionFlow-PX4/releases/tag/V1.17.0) |

## Recommended Hardware

| Component | Minimum | Recommended |
|------|---------|---------|
| CPU | 4 cores | 8 cores+ |
| Memory | 8 GB | 16 GB+ |
| GPU | Integrated GPU | NVIDIA GPU (CUDA support) |
| Disk | 50 GB available space | 100 GB SSD |

## Software Dependencies

### System Packages

```bash
sudo apt update
sudo apt install -y \
  cmake \
  build-essential \
  git \
  wget \
  python3 \
  python3-pip \
  ninja-build \
  ccache
```

### ROS 2 Humble

```bash
# Install ROS 2 Humble Desktop Full
sudo apt install ros-humble-desktop-full
source /opt/ros/humble/setup.bash
```

### Gazebo Harmonic

```bash
# Install Gazebo Harmonic and ros-gz bridge
sudo apt install -y \
  gazebo11 \
  ros-humble-ros-gz-harmonic
```

### Python Dependencies

```bash
pip install \
  mkdocs-material \
  mkdocs-awesome-pages-plugin \
  mkdocs-mermaid2-plugin
```

## Docker Method (Recommended)

The Docker method is recommended for most users. It automatically handles all dependencies, eliminating the need to manually install ROS 2, Gazebo, or PX4 build tools.

### Host Requirements

| Component | Requirement |
|------|------|
| Docker | ≥ 20.10 |
| Docker Compose | ≥ 2.0 (v2 plugin mode) |
| NVIDIA Container Toolkit | Required for GPU acceleration (recommended) |

### Install Docker

```bash
# Official installation script for Ubuntu 22.04
curl -fsSL https://get.docker.com | sudo sh

# Add current user to docker group (avoid sudo each time)
sudo usermod -aG docker $USER
newgrp docker
```

Verify installation:

```bash
docker info
docker compose version
```

### Install NVIDIA Container Toolkit (GPU Acceleration)

If your host has an NVIDIA GPU and you need GPU-accelerated Gazebo rendering, this component is required:

```bash
# Add NVIDIA Docker repository
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.repo \
    | sudo tee /etc/yum.repos.d/nvidia-docker.repo

# Install
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# Verify
docker run --rm --gpus all nvidia/cuda:12.2-base nvidia-smi
```

### X11 Display Permissions

The Docker container needs access to the host X server to display the Gazebo GUI:

```bash
# Allow container access to X11 (temporary, lost after reboot)
xhost +local:docker

# Verify the setting
xhost | grep docker
# Should output: local:docker
```

> **Tip**: Add the above command to `~/.bashrc` for automatic execution on login.

### Disk Space Estimation

| Item | Size |
|------|------|
| Docker image | ~8 GB |
| ccache compiler cache | Dynamic, default limit 20 GB |
| Gazebo model cache | ~1-2 GB |
| Build artifacts (`build/docker/`) | ~2-3 GB |
| **Recommended free space** | **≥ 50 GB** |

### First-Time Setup Flow

```bash
# 1. Build the Docker image (run once, or when Dockerfile changes)
bash docker/run_gz_sitl.sh --build

# 2. Launch simulation
bash docker/run_gz_sitl.sh --profile "Entity 1"

# 3. Enter the container for development and debugging
bash docker/into_gz_sitl.sh
```

The build process automatically syncs the host's `USER_UID` / `USER_GID` into the container, ensuring files created inside the container have the correct ownership on the host — avoiding permission issues entirely.

The Docker image comes with all necessary dependencies pre-installed, including ROS 2 Humble, Gazebo Harmonic, PX4 build tools, and custom plugins.
