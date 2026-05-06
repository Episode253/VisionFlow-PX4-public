# 启动仿真环境
make px4_sitl gz_x500_gimbal

# 启动 mavros
ros2 launch mavros px4.launch

<arg name="fcu_url" default="udp://:14540@127.0.0.1:14557" />

<arg name="fcu_url" default="/dev/ttyACM0:57600" />

# setpoint_velocity
/**/setpoint_velocity:
  ros__parameters:
    mav_frame: BODY_NED
    # mav_frame:LOCAL_NED

# 启动 QGC
/home/renwang/桌面/QGroundControl-x86_64.AppImage

# 启动YOLO检测框发布节点
python3 /home/renwang/data_storage/PX4-Autopilot/develop/visual_tracking/yolo/yolo_msg_publish/yolo_msg_publish.py

# python3 /home/renwang/PX4-Autopilot/develop/visual_tracking/yolo/yolo_target_tracking/yolo_human_tracking.py

# 覆盖性输出日志进行保存，方便后续调试
python3 /home/renwang/PX4-Autopilot/develop/visual_tracking/yolo/yolo_target_tracking/yolo_human_tracking.py \
  > /home/renwang/PX4-Autopilot/develop/visual_tracking/yolo/cmd_log/yolo_human_tracking.log 2>&1
