# 编译指南

本页详细介绍 VisionFlow-PX4 的编译流程，包括 Docker 方式和本地方式的构建命令、CMake 选项、SITL/真实硬件构建命令以及常见编译问题的排错方法。

## Docker 方式（推荐）

Docker 方式自动处理所有依赖，是项目推荐的构建和运行方式。完整的构建启动流程由 `docker/run_gz_sitl.sh` 脚本封装。

### 快速构建

```bash
# 构建 Docker 镜像（首次或 Dockerfile 有变更时执行）
bash docker/run_gz_sitl.sh --build

# 启动仿真（选择 Entity Profile）
bash docker/run_gz_sitl.sh --profile "Entity 4"
```

### 列出可用 Profile

```bash
bash docker/run_gz_sitl.sh --list
```

输出示例：

```
Available SITL profiles:

  1) Entity 1
     name   : PreGME(季梦玉)模型（laboratory_landingbox）
     world  : laboratory_landingbox
     target : gz_q940_ti_gripper4_laboratory_landingbox
     extra  : -DENABLE_LOCKSTEP_SCHEDULER=ON
     pose   : <airframe default>

  2) Entity 2
     ...
```

### 构建目录

Docker 方式的构建产物存放在 `build/docker/` 目录下（容器内路径为 `/workspace/VisionFlow-PX4/build/docker/`），与本地构建的 `build/px4_sitl_default/` 完全隔离，避免缓存冲突。

```bash
# 清理 Docker 构建产物
rm -rf build/docker/
```

### CCache 编译器缓存

Docker 镜像使用 ccache 加速重复编译。缓存目录映射到宿主机的 `docker/cache/ccache/`，跨容器重建保留编译缓存。

```bash
# 清除 ccache 缓存（增大下次构建时间）
rm -rf docker/cache/ccache/*

# 查看 ccache 状态（进入容器后）
ccache -s
```

### 手动在容器内构建

如需在容器内进行 finer-grained 的构建操作：

```bash
# 进入容器
bash docker/into_gz_sitl.sh

# 在容器内执行 PX4 构建
cd /workspace/VisionFlow-PX4
PX4_GZ_WORLD=laboratory_no_landingbox \
  make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox \
  BUILD_BASE_DIR=build/docker \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

---

## 本地构建方式

> **注意**：本地构建需要手动安装所有依赖（参见 [环境要求](../getting-started/prerequisites.md)），且机械臂 Gazebo 插件在部分环境下可能出现初始化问题。Docker 方式是推荐方案。

### 构建 PX4 SITL

```bash
# 基础 SITL 构建
make px4_sitl

# 指定仿真世界和机型
PX4_GZ_WORLD=laboratory_no_landingbox \
  make px4_sitl gz_swan_gamma_v2_laboratory_no_landingbox \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### 可用构建目标

```bash
# 列出所有 Gazebo SITL 目标
ninja -C build/px4_sitl_default -t targets | grep "^gz_"
```

### CMake 选项

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `ENABLE_LOCKSTEP_SCHEDULER` | `ON` | 启用锁步调度器，确保仿真时序精确（推荐开启） |
| `ENABLE_LOCKSTEP_SCHEDULER` | `OFF` | 关闭锁步调度，提升运行速度但可能影响仿真稳定性 |

通过 `EXTRA_CMAKE_ARGS` 传递：

```bash
make px4_sitl gz_swan_gamma_v2 \
  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON -DMICROAIRDOGS_SUPPORT=ON"
```

---

## 真实硬件编译

### Pixhawk 系列

```bash
# 编译固件
make px4_fmu-v5_default        # Pixhawk 4
make px4_fmu-v6x_default       # Pixhawk 6X
make px4_fmu-v6c_default       # Pixhawk 6C

# 烧录（连接 USB 后）
make px4_fmu-v5_default upload
```

### 其他板卡

参见 [支持板卡列表](../hardware/supported-boards.md)。

---

## 常见问题排查

### CMake 缓存冲突（Docker vs 本地）

**现象**：切换 Docker 和本地构建后出现奇怪的编译错误。

**原因**：Docker 容器内和宿主机使用不同的构建目录，但 CMake 缓存可能残留冲突配置。

**解决**：
```bash
# 清理 Docker 构建缓存
rm -rf build/docker

# 清理本地构建缓存
rm -rf build/px4_sitl_default
```

### uORB ucdr 头文件生成卡死

**现象**：编译过程中卡在 `Generating uORB topic ucdr headers` 步骤。

**原因**：头文件生成脚本在输出稳定后未及时退出。

**解决**：脚本已内置 watchdog 自动重试。若持续失败，可增加超时时间：

```bash
export PX4_UCDR_HEADER_STALL_TIMEOUT=10
export PX4_UCDR_HEADER_WATCH_INTERVAL=5
```

或清除 stale 缓存后重试：

```bash
rm -rf build/docker/px4_sitl_default
bash docker/run_gz_sitl.sh --build
```

### 缺少依赖

**现象**：编译报错提示找不到头文件或库。

**解决**：参考 [环境要求](../getting-started/prerequisites.md) 安装依赖。使用 Docker 方式会自动处理所有依赖。

### 机械臂插件编译失败

**现象**：`gamma_arm_control` 插件编译报错。

**解决**：
```bash
cd windshape_dev/plugins/gamma_arm_control
cmake -S . -B build
cmake --build build -j$(nproc)
sudo cmake --install build
```

---

## 相关页面

- [开发指南总览](index.md)
- [添加模块](adding-modules.md)
- [Docker 工作流详解](../tools/docker-workflow.md)
- [快速开始](../getting-started/quick-start.md)
