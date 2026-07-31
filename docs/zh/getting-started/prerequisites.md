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

对于大多数用户，推荐使用 Docker 方式获取完整环境。此方式自动处理所有依赖，无需手动安装 ROS 2、Gazebo 或 PX4 构建工具。

### 宿主机要求

| 组件 | 要求 |
|------|------|
| Docker | ≥ 20.10 |
| Docker Compose | ≥ 2.0（v2 插件模式） |
| NVIDIA Container Toolkit | 如需 GPU 加速（推荐） |

### 安装 Docker

```bash
# Ubuntu 22.04 官方安装脚本
curl -fsSL https://get.docker.com | sudo sh

# 将当前用户加入 docker 组（避免每次sudo）
sudo usermod -aG docker $USER
newgrp docker
```

验证安装：

```bash
docker info
docker compose version
```

### 安装 NVIDIA Container Toolkit（GPU 加速）

如果宿主机有 NVIDIA 显卡且需要 GPU 渲染 Gazebo，必须安装此组件：

```bash
# 添加 NVIDIA Docker 仓库
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.repo \
    | sudo tee /etc/yum.repos.d/nvidia-docker.repo

# 安装
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# 验证
docker run --rm --gpus all nvidia/cuda:12.2-base nvidia-smi
```

### X11 显示权限

Docker 容器需要访问宿主机 X server 才能显示 Gazebo GUI：

```bash
# 允许容器访问 X11（临时，重启后失效）
xhost +local:docker

# 确认设置成功
xhost | grep docker
# 应输出：local:docker
```

> **提示**：可将上述命令加入 `~/.bashrc` 以实现开机自动生效。

### 磁盘空间预估

| 项目 | 占用 |
|------|------|
| Docker 镜像 | ~8 GB |
| ccache 编译器缓存 | 动态增长，默认上限 20 GB |
| Gazebo 模型缓存 | ~1-2 GB |
| 构建产物 (`build/docker/`) | ~2-3 GB |
| **建议可用空间** | **≥ 50 GB** |

### 首次使用流程

```bash
# 1. 构建 Docker 镜像（首次或 Dockerfile 更新后执行）
bash docker/run_gz_sitl.sh --build

# 2. 启动仿真
bash docker/run_gz_sitl.sh --profile "Entity 1"

# 3. 进入容器进行开发调试
bash docker/into_gz_sitl.sh
```

构建镜像时会自动同步宿主机的 `USER_UID` / `USER_GID`，确保容器内创建的文件属主与宿主机一致，不会出现在文件权限问题上。

Docker 镜像已预装所有必要依赖，包括 ROS 2 Humble、Gazebo Harmonic、PX4 构建工具和自定义插件。
