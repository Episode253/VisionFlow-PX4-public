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

For most users, the Docker method is recommended to get a complete environment:

```bash
# Build Docker image
bash docker/run_gz_sitl.sh --build

# Launch
bash docker/run_gz_sitl.sh --profile "Entity 1"
```

The Docker image comes with all necessary dependencies pre-installed, including ROS 2 Humble, Gazebo Harmonic, PX4 build tools, and custom plugins.
