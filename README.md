## 启动gz sim仿真

### 基础仿真环境测试
make px4_sitl gz_x500

### 室外场景
make px4_sitl gz_x500_gimbal

### 无头仿真环境测试
HEADLESS=1 make px4_sitl gz_x500_gimbal

### 室内场景
make px4_sitl gz_x500_depth_baylands

## 启动mavros通信

### Mavros主节点

ros2 launch mavros px4.launch

ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557

### mavros通信验证

ros2 topic echo /mavros/imu/data_raw

## gazebo数据流

### 查看所有的gazebo内部话题

gz topic -l

### 查看详细的gazebo话题数据流
gz topic --info --topic /world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image

## Gstream视频流操作

### 获取视频流---用于验证视频流正常

gst-launch-1.0 udpsrc port=5600 caps="application/x-rtp, media=video, encoding-name=H264, payload=96" ! rtpjitterbuffer !  rtph264depay ! avdec_h264 ! autovideosink

### 进入YOLO环境---目前使用系统YOLO环境

source yolo_stable/bin/activate

### 开启YOLO检测---包含视频流录制交互

python3 /home/renwang/PX4-Autopilot/visual_tracking/yolo/video_stream_capture/yolo_stream_version2.py

## offboard 测试

### 启用offboard官方测试例程

python3 /home/renwang/PX4-Autopilot/control/offboard/official_offboard.py

### echo无人机目前的状态

ros2 topic echo /mavros/state
