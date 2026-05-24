# VisionFlow-PX4

## 启动仿真环境

### 检查可构建目标

ninja -C build/px4_sitl_default -t targets | grep gz_q940_ti

### 根据可构建目标启动仿真环境

PX4_GZ_WORLD=lab106_landingbox make px4_sitl gz_q940_ti_laboratory  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"

## 启动数据桥接节点

bash windshape_dev/video_stream/bridge_gz_ros.sh

### 运行图像可视化

ros2 run rqt_image_view rqt_image_view /depth_camera

## 启动机械臂控制节点

bash windshape_dev/arm_control/ti5_arm_web_control.sh
