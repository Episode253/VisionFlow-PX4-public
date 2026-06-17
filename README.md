# VisionFlow-PX4

## 检查可构建目标并启动仿真环境

ninja -C build/px4_sitl_default -t targets | grep gz_q940_ti

### 启动串联机械臂（季梦玉师姐）模型（基础环境）

PX4_GZ_WORLD=laboratory_landingbox make px4_sitl gz_q940_ti_laboratory_landingbox  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"

### 启动串联机械臂模型（vla_task0）

PX4_GZ_WORLD=laboratory_landingbox_vla_task0 make px4_sitl gz_q940_ti_laboratory_landingbox_vla_task0  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"

### 启动云台版基础模型

PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_x500_gimbal_laboratory_no_landingbox  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"

### 启动小车模型

PX4_GZ_MODEL_POSE="0,0,0.5,0,0,0" make px4_sitl gz_differential_rover_laboratory_no_landingbox EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"

### 启动公司平台

PX4_GZ_WORLD=laboratory_no_landingbox make px4_sitl gz_swan_gamma_laboratory_no_landingbox  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"

## 启动数据桥接节点

bash windshape_dev/data_stream/gz_bridge/bridge_gz_ros.sh

### 运行图像可视化

bash windshape_dev/data_stream/image_stream/camera_stream.sh

## 启动机械臂控制节点

bash windshape_dev/arm_control/ti5_arm_web_control.sh

## 启动 mavros 节点

source thirdparty/install/setup.bash && ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557

## 启动 HITL 节点（这部分较为复杂，将出单独的README做出说明）

gz sim -r Tools/simulation/gz/worlds/laboratory_landingbox_hitl.sdf
