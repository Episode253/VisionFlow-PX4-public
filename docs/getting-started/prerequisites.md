# 环境要求

在开始之前，请确保您的开发环境满足以下要求。

## 系统要求

| 环境 / 组件 | 版本或信息 |
|-------------|-----------|
| 操作系统 | Ubuntu 22.04 LTS |
| ROS 2 版本 | Humble Hawksbill |
| Gazebo Sim 版本 | Harmonic V8.11.0 |
| Ros-GZ Bridge 版本 | `ros-humble-ros-gz-harmonic` |
| PX4 版本 | V1.17.0 |
| QGC 版本 | [下载链接](https://github.com/Renwang-Huang/VisionFlow-PX4/releases/tag/V1.17.0) |

## 推荐硬件

| 组件 | 最低配置 | 推荐配置 |
|------|---------|---------|
| CPU | 4 核 | 8 核+ |
| 内存 | 8 GB | 16 GB+ |
| GPU | 集成显卡 | NVIDIA GPU（CUDA 支持） |
| 磁盘 | 50 GB 可用空间 | 100 GB SSD |

## 依赖软件

### 系统包

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
# 安装 ROS 2 Humble Desktop Full
sudo apt install ros-humble-desktop-full
source /opt/ros/humble/setup.bash
```

### Gazebo Harmonic

```bash
# 安装 Gazebo Harmonic 和 ros-gz bridge
sudo apt install -y \
  gazebo11 \
  ros-humble-ros-gz-harmonic
```

### Python 依赖

```bash
pip install \
  mkdocs-material \
  mkdocs-awesome-pages-plugin \
  mkdocs-mermaid2-plugin
```

## Docker 方式（推荐）

对于大多数用户，推荐使用 Docker 方式获取完整环境：

```bash
# 构建 Docker 镜像
bash docker/run_gz_sitl.sh --build

# 启动
bash docker/run_gz_sitl.sh --profile "Entity 1"
```

Docker 镜像已预装所有必要依赖，包括 ROS 2 Humble、Gazebo Harmonic、PX4 构建工具和自定义插件。
