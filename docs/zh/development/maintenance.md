# 维护指南

本页提供 VisionFlow-PX4 项目的维护指南，包括项目维护者信息、问题报告流程、BUG 定位与修复流程以及常见故障排查方法。

## 项目维护者

| 角色 | 姓名 | 联系方式 |
|------|------|---------|
| 主要维护人 | Renwang Huang | RenwangHuangX@gmail.com |
| 仓库地址 | <https://github.com/Renwang-Huang/VisionFlow-PX4> | |

如有问题或建议，欢迎通过以下方式联系：

- **GitHub Issues**：提交到 [VisionFlow-PX4 Issues](https://github.com/Renwang-Huang/VisionFlow-PX4/issues)
- **邮件**：直接联系维护人

## 问题报告指南

### 何时创建 Issue

- 发现代码或文档中的 BUG
- 需要新功能或改进
- 文档缺失或错误
- 仿真或运行时遇到无法解决的问题

### Issue 模板选择

仓库提供以下 Issue 模板，请根据问题类型选择：

| 模板 | 适用场景 | 文件 |
|------|---------|------|
| 🐛 Bug Report | 代码或仿真行为异常 | `.github/ISSUE_TEMPLATE/bug_report.yml` |
| 📑 Documentation Bug | 文档错误或缺失 | `.github/ISSUE_TEMPLATE/docs_bug_report.yml` |
| 🚀 Feature Request | 新功能建议 | `.github/ISSUE_TEMPLATE/feature_request.yml` |

### 高质量报告 Checklist

- [ ] 使用正确的 Issue 模板
- [ ] 标题简洁明了，包含问题关键词
- [ ] 描述复现步骤（环境、命令、操作顺序）
- [ ] 附上相关日志或截图
- [ ] 标注 PX4 版本、Gazebo 版本、ROS2 版本
- [ ] 如果是飞行相关问题，上传飞行日志到 [PX4 Flight Review](http://logs.px4.io/)

## BUG 定位与修复流程

### 1. 复现 BUG

- 确认复现环境（Docker / 本地部署）
- 记录复现步骤和使用的 Entity Profile
- 确认 BUG 是否稳定复现

### 2. 定位根因

**日志分析：**

- PX4 运行日志：查看终端输出，关注 `[error]`、`[warn]` 标记
- Gazebo 日志：检查 Gazebo 服务端输出
- Docker 日志：`docker logs visionflow-px4-sitl`

**飞行日志分析：**

- 使用 [PX4 Flight Review](http://logs.px4.io/) 上传 `.ulog` 文件
- 检查 EKF 状态、控制器输出、传感器数据

**代码定位：**

- 使用 `printf` / `PX4_ERR` / `PX4_WARN` 添加临时调试输出
- 在 Gazebo 插件中使用 `gzdbg` / `gzerr` 输出
- 检查 uORB 主题发布情况：`uorb top <topic_name>`

### 3. 编写修复

- 遵循项目代码规范（参见 [CLAUDE.md](../../CLAUDE.md)）
- 确保修复不破坏现有功能
- 更新相关参数文档（如果涉及参数变更）

### 4. 提交 PR

- 参考 [PULL_REQUEST_TEMPLATE.md](https://github.com/Renwang-Huang/VisionFlow-PX4/blob/main/.github/PULL_REQUEST_TEMPLATE.md)
- PR 标题使用 Conventional Commits 格式：`fix:` / `feat:` / `docs:` / `refactor:`
- 在 PR 描述中关联 Issue（`Fixes #issue_number`）
- 描述修改内容和测试方法

### 5. 验证与合入

- 确保 CI 检查通过（编译、静态分析、SITL 测试）
- 在 Docker 和本地环境分别验证
- 维护人审核后合入

## 常见故障排查

### 编译问题

#### CMake 缓存冲突（Docker vs 本地）

**现象：** 切换 Docker 和本地构建后出现奇怪的编译错误。

**原因：** Docker 容器内和宿主机使用不同的构建目录，但 CMake 缓存可能冲突。

**解决：**
```bash
# 清理 Docker 构建缓存
rm -rf build/docker

# 清理本地构建缓存
rm -rf build/px4_sitl_default
```

#### uORB ucdr 头文件生成卡死

**现象：** 编译过程中卡在 `Generating uORB topic ucdr headers` 步骤。

**原因：** 头文件生成脚本在输出稳定后未及时退出。

**解决：** 系统已内置 watchdog 机制（`PX4_UCDR_HEADER_STALL_TIMEOUT`，默认 5 秒），会自动检测并重试。如果持续失败：

```bash
# 增加超时时间
export PX4_UCDR_HEADER_STALL_TIMEOUT=10
export PX4_UCDR_HEADER_WATCH_INTERVAL=5
```

#### 缺少依赖

**现象：** 编译报错提示找不到头文件或库。

**解决：** 参考 [环境要求](../getting-started/prerequisites.md) 安装依赖。Docker 方式会自动处理依赖。

### Docker 问题

#### 容器启动失败

**现象：** `docker compose run` 报错退出。

**检查步骤：**

1. 确认 Docker 服务运行中：`docker info`
2. 检查端口冲突：`lsof -i :14540`（MAVROS 端口）
3. 检查 X11 权限：`xhost +local:docker`
4. 查看容器日志：`docker logs visionflow-px4-sitl`

#### X11 显示问题

**现象：** Gazebo GUI 无法显示或报 `cannot connect to X server`。

**解决：**
```bash
# 允许 Docker 访问 X11
xhost +local:docker

# 检查 DISPLAY 环境变量
echo $DISPLAY  # 应输出 :0 或 :1
```

#### 权限 / 用户 ID 不匹配

**现象：** 容器内创建的文件属主为 root，无法在宿主机修改。

**原因：** 容器内用户 ID 与宿主机不一致。

**解决：** 脚本会自动导出 `USER_UID` 和 `USER_GID` 到容器。如果仍有问题，手动指定：

```bash
export USER_UID=$(id -u)
export USER_GID=$(id -g)
```

### Gazebo 仿真问题

#### 模型加载失败 / 缺少模型

**现象：** Gazebo 启动后模型显示为粉色或缺失。

**解决：**
```bash
# 确保 GAZEBO_MODEL_PATH 包含项目模型目录
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/Tools/simulation/gz/models

# 首次启动需要下载模型，确保网络畅通
# 如果使用 Docker，模型缓存位于 docker/cache/gz/
```

#### 无人机初始位置不对

**现象：** 无人机出现在错误的位置。

**原因：** `PX4_GZ_MODEL_POSE` 环境变量未正确设置。

**解决：**

- 检查 airframe 配置文件中的默认位姿：`ROMFS/px4fmu_common/init.d-posix/airframes/`
- 检查 Docker profile 中的 `--pose` 参数：`docker/gz_sitl_profiles.conf`
- 位姿格式：`x,y,z,roll,pitch,yaw`（6 个逗号分隔值）

#### 机械臂插件未加载

**现象：** 机械臂无法控制或 Gazebo 报插件加载错误。

**解决：**
```bash
# 手动编译并安装 Gamma 机械臂控制插件
cd windshape_dev/plugins/gamma_arm_control
cmake -S . -B build
cmake --build build
sudo cmake --install build
```

#### 仿真时序不同步

**现象：** 仿真运行速度异常（过快或过慢）。

**解决：** 确保使用锁步调度器：

```bash
EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
```

### PX4 运行时问题

#### 传感器校准失败

**现象：** PX4 启动后报传感器错误。

**解决：** 仿真环境下传感器由 Gazebo 模拟，检查：

- Gazebo 传感器插件是否正确加载
- 检查 `EKF2_SENS_EN` 参数是否包含所需传感器
- 仿真中跳过校准：`param set CAL_GYRO0_EN 0`

#### EKF 不收敛

**现象：** 飞行日志显示 EKF 状态异常或位置估计漂移。

**解决：**

- 检查 GPS 模拟是否正常
- 检查 IMU 噪声参数是否匹配模型
- 调整 EKF2 参数：`EKF2_*`

#### 控制器输出异常

**现象：** 无人机飞行姿态异常或无法悬停。

**解决：**

- 检查控制分配参数：`CA_AIRFRAME`、`CA_ROTOR*`
- 检查 PreGME 控制器参数：`USR_LAMBDA_Q_*`、`USR_I_YY` 等
- 确认 airframe 配置文件中的电机映射与 SDF 模型一致

### ROS2 / 通信问题

#### MAVROS 连接失败

**现象：** `ros2 launch mavros px4.launch` 无法连接。

**解决：**
```bash
# 确认 PX4 已启动并监听 UDP
# PX4 默认在 14540 端口监听
ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557
```

#### Zenoh 桥接不通

**现象：** Zenoh 节点之间无法通信。

**解决：** 检查 Zenoh 配置文件是否正确，确认网络端口开放。

#### uXRCE-DDS 代理断开

**现象：** ROS2 话题无法接收 PX4 数据。

**解决：** 重启 uXRCE-DDS 代理：

```bash
# 在容器内
MicroXRCEAgent udp4 -p 8888
```

## 调试技巧

### 启用详细日志

```bash
# PX4 详细日志
export PX4_DEBUG=1

# Gazebo 详细日志
export GZ_VERBOSE=3
```

### 使用 PX4 Flight Review 分析日志

1. 启动仿真并飞行后，日志保存在 `~/PX4/` 目录
2. 上传 `.ulog` 文件到 <http://logs.px4.io/>
3. 分析 EKF 状态、控制器性能、传感器数据

### Gazebo 调试工具

```bash
# 查看 Gazebo 主题列表
gz topic -l

# 查看特定主题数据
gz topic -e -t /world/<world_name>/model/<model_name>/pose

# 查看模型位姿
gz topic -e -t /gazebo/default/model/<model_name>/pose
```

## 回滚与恢复

### Docker 缓存清理

```bash
# 清理所有 Docker 构建缓存
docker compose -f docker/compose.yaml build --no-cache

# 清理本地缓存目录
rm -rf docker/cache/ccache/*
rm -rf docker/cache/gz/*
```

### 构建目录清理

```bash
# 清理 PX4 构建产物
rm -rf build/px4_sitl_default
rm -rf build/docker

# 清理 Gamma 插件构建产物
rm -rf windshape_dev/plugins/gamma_arm_control/build
```

### Git 回退策略

```bash
# 查看最近的提交
git log --oneline -10

# 回退到指定提交（本地）
git reset --hard <commit_hash>

# 撤销未提交的修改
git checkout -- <file>
git restore <file>
```

---

> 本指南将持续更新。如有未覆盖的问题，请提交 Issue 或联系维护人。
