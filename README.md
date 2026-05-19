# VisionFlow-PX4

## 启动仿真环境

PX4_GZ_WORLD=lab106_landingbox make px4_sitl gz_q940_ti  EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"

## 启动数据桥接节点

bash windshape_dev/video_stream/bridge_oakd.sh

## 运行图像可视化节点

ros2 run rqt_image_view rqt_image_view /depth_camera
