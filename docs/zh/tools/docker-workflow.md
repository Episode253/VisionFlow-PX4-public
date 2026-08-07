# Docker 工作流详解

本页详细介绍 VisionFlow-PX4 的 Docker 工作流，包括 Docker 镜像构建、容器管理、仿真启动脚本（`run_gz_sitl.sh`）的使用方法和预定义仿真配置（Entity profiles）的说明。

## 目录结构

```
docker/
├── Dockerfile.humble-gz          # PX4 + Gazebo + ROS2 基础镜像
├── compose.yaml                  # Docker Compose 服务定义
├── gz_sitl_profiles.conf         # 仿真 Entity Profile 配置
├── run_gz_sitl.sh               # SITL 启动主脚本
├── into_gz_sitl.sh              # 进入运行中容器的辅助脚本
├── run_flight_review.sh         # 飞行日志审查启动脚本
└── entrypoint.sh                # 容器启动入口脚本
```

缓存目录：

```
docker/cache/
├── ccache/   # ccache 编译器缓存（加速重复构建）
└── gz/       # Gazebo 模型/材质缓存
```

---

## Docker 镜像构建

### 基础信息

| 属性 | 值 |
|------|---|
| 基础镜像 | `osrf/ros:humble-desktop-full` |
| 目标镜像名 | `visionflow-px4:humble-gz` |
| 容器名称 | `visionflow-px4-sitl` |
| 容器内默认用户 | `px4`（UID/GID 与宿主机一致） |

### 镜像包含的内容

1. **PX4 构建工具链** — CMake、Ninja、Python 依赖、NuttX 交叉编译工具
2. **Gazebo Harmonic** — 仿真引擎及 `ros-humble-ros-gzharmonic` 桥接包
3. **ROS 2 Humble** — 完整的 Desktop-Full 安装
4. **MAVROS** — 通过 `thirdparty/mavros-humble/` 本地源码编译
5. **Flight Review 依赖** — Bokeh、PyULog、Tornado 等（`/opt/flight_review_venv/`）
6. **Gamma Arm Web GUI** — 机械臂控制面板所需系统库（QtWebEngine、GTK3 等）
7. **中文字体** — `fonts-noto-cjk`（用于中文界面显示）

### 镜像构建参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `USER_UID` | `1000` | 容器内 `px4` 用户的 UID，应与宿主机一致以避免文件权限问题 |
| `USER_GID` | `1000` | 容器内 `px4` 用户的 GID |
| `INSTALL_XTENSA_ESP` | `0` | 是否下载 Xtensa ESP 编译器（仅 x86_64 架构需要时设为 `1`） |

构建命令：

```bash
# 正常构建（使用宿主代理设置）
bash docker/run_gz_sitl.sh --build

# 强制无缓存重建
docker compose -f docker/compose.yaml build --no-cache px4-humble-gz
```

### APT 源镜像

Dockerfile 自动将 Ubuntu 和 ROS 2 的 APT 源替换为阿里云镜像，以加速中国大陆地区的构建：

- `archive.ubuntu.com` → `mirrors.aliyun.com/ubuntu`
- `packages.ros.org/ros2/ubuntu` → `mirrors.aliyun.com/ros2/ubuntu`

构建过程中会验证替换结果，若仍残留官方源地址则报错退出。

### PyPI 镜像

使用清华大学 TUNA 镜像站（`pypi.tuna.tsinghua.edu.cn`），避免阿里云 pyside6-addons 索引的 HTML 格式警告。

---

## Docker Compose 配置详解

`docker/compose.yaml` 定义了单个服务 `px4-humble-gz`：

```yaml
services:
  px4-humble-gz:
    build:
      context: ..                        # 项目根目录
      dockerfile: docker/Dockerfile.humble-gz
      network: host                      # 构建时使用 host 网络（访问本地代理）
      args:
        USER_UID: ${USER_UID:-1000}
        USER_GID: ${USER_GID:-1000}
        # 代理变量透传到 build ARG
        http_proxy: ${http_proxy:-}
        https_proxy: ${https_proxy:-}
        ...
    image: visionflow-px4:humble-gz
    container_name: visionflow-px4-sitl
    volumes:
      - ..:/workspace/VisionFlow-PX4:rw          # 整个项目目录双向挂载
      - ./cache/ccache:/home/px4/.ccache:rw      # ccache 编译器缓存
      - ./cache/gz:/home/px4/.gz:rw              # Gazebo 资源缓存
      - /tmp/.X11-unix:/tmp/.X11-unix:rw         # X11 显示套接字
      - /dev/dri:/dev/dri                         # GPU 设备直通
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - QTWEBENGINE_DISABLE_SANDBOX=1
      - NVIDIA_VISIBLE_DEVICES=all
      - CCACHE_DIR=/home/px4/.ccache
      ...
    gpus: all          # 启用 NVIDIA GPU
    privileged: true   # 特权模式（Gazebo 需要）
    network_mode: host # host 网络（MAVLink/ROS2 端口直接暴露）
    shm_size: "2gb"    # 共享内存大小
```

### 挂载卷说明

| 挂载路径 | 类型 | 用途 |
|---------|------|------|
| `/workspace/VisionFlow-PX4`（映射自 `..`） | 宿主机目录（双向读写） | 完整代码库访问，容器内修改立即反映到宿主机 |
| `./cache/ccache` → `/home/px4/.ccache` | 宿主机目录 | ccache 编译器缓存，跨次构建加速 |
| `./cache/gz` → `/home/px4/.gz` | 宿主机目录 | Gazebo 模型/材质缓存，避免重复下载 |
| `/tmp/.X11-unix` | 宿主机套接字 | X11 图形显示转发 |
| `/dev/dri` | 设备 | OpenGL/GPU 直通 |

---

## 仿真启动脚本

### `run_gz_sitl.sh` — 主启动脚本

**用法：**

```bash
bash docker/run_gz_sitl.sh --profile "Entity 4"        # 启动指定配置
bash docker/run_gz_sitl.sh --build --profile "Entity 1" # 重建镜像后启动
bash docker/run_gz_sitl.sh --list                       # 列出所有可用配置
bash docker/run_gz_sitl.sh                              # 交互式选择（默认 Entity 1）
```

**执行流程：**

```
[1/6] 创建缓存目录  docker/cache/ccache, docker/cache/gz
[2/6] 设置脚本权限  chmod +x entrypoint.sh
[3/6] 导出用户 ID   USER_UID, USER_GID（与宿主机保持一致）
[4/6] 允许 X11 访问 xhost +local:docker
[5/6] 构建镜像      docker compose build（仅 --build 时）
[6/6] 运行容器      docker compose run → PX4 SITL + Gazebo
```

**UORB ucdr Header Watchdog：**

脚本内置了 uORB ucdr 头文件生成停滞检测机制，防止编译卡死：

- 监控 `Generating uORB topic ucdr headers` 阶段
- 若日志文件大小在 `PX4_UCDR_HEADER_STALL_TIMEOUT`（默认 5 秒）内不变，则终止并自动重试
- 最大重试次数：`PX4_UCDR_HEADER_RETRIES`（默认 1 次）
- 检测到 stale cache（CMakeCache 冲突）时自动清理 `build/docker/px4_sitl_default` 并重试

可通过环境变量调整：

```bash
export PX4_UCDR_HEADER_STALL_TIMEOUT=10  # 停滞检测超时（秒）
export PX4_UCDR_HEADER_WATCH_INTERVAL=5  # 检测间隔（秒）
export PX4_UCDR_HEADER_RETRIES=2         # 最大重试次数
```

### `gz_sitl_profiles.conf` — Profile 配置文件

每个 Entity Profile 由以下字段定义（管道符分隔）：

| 字段 | 说明 | 示例 |
|------|------|------|
| `id` | Profile 唯一标识 | `Entity 4` |
| `name` | 中文名称 | `PreGME(公司)模型(新版本)` |
| `world` | Gazebo 世界文件名 | `laboratory_no_landingbox` |
| `target` | PX4 构建目标 | `gz_swan_gamma_v2_laboratory_no_landingbox` |
| `extra` | 额外 CMake 参数 | `-DENABLE_LOCKSTEP_SCHEDULER=ON` |
| `pose` | 无人机初始位姿（可选） | `0,0,1.15392,0,0,0`（x,y,z,roll,pitch,yaw） |

**添加新 Profile 的方法：**

在 `gz_sitl_profiles.conf` 末尾追加：

```bash
add_sitl_profile \
    --id "Entity 8" \
    --name "我的自定义模型" \
    --world "my_world" \
    --target "gz_my_drone_my_world" \
    --pose "0,0,0.5,0,0,0"    # 可选
```

---

## 进入容器

### `into_gz_sitl.sh` — 交互 shell 入口

```bash
bash docker/into_gz_sitl.sh                  # 默认方式（启动 Web 控制 + 交互 shell）
bash docker/into_gz_sitl.sh --no-web         # 不启动 Web 控制
bash docker/into_gz_sitl.sh --no-gui         # 无 GUI 模式（headless）
bash docker/into_gz_sitl.sh --gui=0          # 同上
```

**可用环境变量：**

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `GUI_ENABLE` | `1` | 是否启动 Gazebo GUI |
| `LOG_ENABLE` | `0` | 是否启用详细日志 |
| `AUTO_RESTART` | `1` | Web 控制崩溃后是否自动重启 |
| `GZ_KEEPALIVE` | `0` | Gazebo 崩溃后是否自动重启 |
| `WEB_PORT` | `9000` | Web 控制 UI 端口 |
| `ROSBRIDGE_PORT` | `9090` | rosbridge WebSocket 端口 |

**进入容器后的 Helper 命令：**

| 命令 | 说明 |
|------|------|
| `webstart` | 启动 Gamma 机械臂 Web 控制 |
| `webstart_gui` | 启动带嵌入式 GUI 的 Web 控制 |
| `webstart_headless` | 仅启动 Web 后端（无 GUI） |
| `webstop` | 停止 Web 控制 |
| `weblog` | 查看 Web 控制日志 |
| `webattach` | 附加到 tmux Web 控制会话 |
| `webps` | 查看相关进程 |
| `webcheck` | 综合检查（进程、HTTP、ROS 话题） |
| `croot` | 切换到项目根目录 |
| `gzlist` | 列出 Gazebo 话题 |
| `gzps` | 查看仿真相关进程 |

---

## 飞行日志审查

### `run_flight_review.sh`

在 Docker 容器内启动 Flight Review Web 应用：

```bash
bash docker/run_flight_review.sh              # 启动（使用已有镜像）
bash docker/run_flight_review.sh --build      # 重建镜像后启动
```

访问地址：`http://127.0.0.1:5006/upload`

**环境变量：**

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `FR_PORT` | `5006` | Flight Review Web 端口 |
| `CONTAINER_NAME` | `visionflow-flight-review` | 容器名称 |
| `SERVICE_NAME` | `px4-humble-gz` | 使用的 compose 服务名 |

---

## 故障排查

### 容器无法启动

```bash
# 检查 Docker 服务状态
docker info

# 查看容器日志
docker logs visionflow-px4-sitl

# 检查端口冲突（MAVLink/MAVROS 常用端口）
lsof -i :14540
lsof -i :14550
lsof -i :9000
```

### X11 显示问题

```bash
# 允许容器访问 X11
xhost +local:docker

# 检查 DISPLAY 变量
echo $DISPLAY   # 应输出 :0 或 :1
```

### GPU 不被识别

```bash
# 确认 NVIDIA Container Toolkit 已安装
nvidia-smi

# 测试 GPU 容器
docker run --rm --gpus all nvidia/cuda:12.2-base nvidia-smi
```

### uORB ucdr Header 停滞

系统已内置 watchdog 自动处理。若持续失败：

```bash
# 清除 PX4 构建缓存
rm -rf build/docker/px4_sitl_default

# 或清除全部缓存后重建
bash docker/run_gz_sitl.sh --build
```

### 机械臂插件未加载

```bash
# 在容器内手动编译安装
cd /workspace/VisionFlow-PX4/windshape_dev/plugins/gamma_arm_control
cmake -S . -B build
cmake --build build
sudo cmake --install build
```

### 缓存清理

```bash
# 清理 ccache（节省磁盘空间，但下次构建会变慢）
rm -rf docker/cache/ccache/*

# 清理 Gazebo 缓存（模型会被重新下载）
rm -rf docker/cache/gz/*

# 清理 PX4 构建产物
rm -rf build/docker/
```

---

## 相关页面

- [快速部署工具链](../getting-started/index.md)
- [Docker 启动指南](../getting-started/docker-launch.md)
- [环境要求](../getting-started/prerequisites.md)
- [飞行日志审查](flight-review.md)
- [维护指南](../development/maintenance.md)
