# 快速验证

完成安装和启动后，通过以下步骤验证环境是否正常工作。

## 1. 验证 Docker 环境

```bash
# 检查 Docker 是否运行
docker info

# 检查 NVIDIA GPU 是否可用
docker run --rm --gpus all nvidia/cuda:12.2-base nvidia-smi
```

## 2. 验证 PX4 SITL

```bash
# 进入 Docker 容器
bash docker/into_gz_sitl.sh

# 检查 PX4 版本
px4 --version

# 列出可用的仿真目标
ninja -C build/px4_sitl_default -t targets | grep "^gz_"
```

## 3. 验证 Gazebo 仿真

```bash
# 启动最小化仿真
gz sim -r Tools/simulation/gz/worlds/laboratory_landingbox.sdf
```

应看到 Gazebo 窗口打开，显示实验室场景。

## 4. 验证 ROS 2 桥接

在另一个终端中：

```bash
# 启动数据桥接
bash windshape_dev/data_stream/gz_bridge/bridge_gz_ros.sh

# 检查话题
ros2 topic list
```

应能看到 `/gz/camera/...` 和 `/camera/...` 等桥接话题。

## 5. 验证 QGroundControl 连接

启动 QGroundControl 后，应能自动连接到 PX4 SITL（默认 UDP 端口 14550）。

检查连接状态：
- 车辆状态显示为 "Ready"
- 可以查看传感器数据
- 可以发送控制指令

## 6. 验证机械臂控制

```bash
# 启动 Gamma 臂 Web 控制
bash windshape_dev/arm_control/gamma_arm/gamma_arm_web_control.sh
```

在浏览器中打开显示的地址，应能看到机械臂控制面板。

## 验证清单

- [ ] Docker 镜像构建成功
- [ ] Gazebo 场景正常加载
- [ ] PX4 SITL 运行无报错
- [ ] ROS 2 话题桥接正常
- [ ] QGroundControl 可连接
- [ ] 机械臂控制界面可访问
