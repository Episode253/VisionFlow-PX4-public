# VisionFlow-PX4

## 检查可构建目标

ninja -C build/px4_sitl_default -t targets | grep gz_q940_ti

### 启动串联机械臂（季梦玉师姐）模型

PX4_GZ_WORLD=laboratory_landingbox make px4_sitl gz_q940_ti_laboratory_landingbox  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"

### 启动云台版基础模型

PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_x500_gimbal_laboratory_no_landingbox  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"

### 启动小车模型

PX4_GZ_MODEL_POSE="0,0,0.5,0,0,0" make px4_sitl gz_differential_rover_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"

## 启动数据桥接节点

bash windshape_dev/data_stream/gz_bridge/bridge_gz_ros.sh

### 运行图像可视化

bash windshape_dev/data_stream/image_stream/camera_stream.sh

## 启动机械臂控制节点

bash windshape_dev/arm_control/ti5_arm_web_control.sh
