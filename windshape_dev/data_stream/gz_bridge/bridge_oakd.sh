#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

WORLD="/world/laboratory/model/q940_ti_0"

OAKD1="${WORLD}/model/oakd_lite_one/link/camera_link/sensor"
OAKD2="${WORLD}/model/oakd_lite_two/link/camera_link/sensor"
OAKD3="${WORLD}/model/oakd_lite_three/link/camera_link/sensor"
OAKD4="${WORLD}/model/oakd_lite_four/link/camera_link/sensor"

cleanup()
{
    echo ""
    echo "Stopping bridge and static TF..."

    if [ -n "$TF_PID" ]; then
        kill "$TF_PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id map \
    --child-frame-id camera_link &

TF_PID=$!

ros2 run ros_gz_bridge parameter_bridge \
"${OAKD1}/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image" \
"${OAKD1}/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" \
"${OAKD1}/StereoOV7251/depth_image@sensor_msgs/msg/Image@gz.msgs.Image" \
"${OAKD1}/StereoOV7251/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" \
"${OAKD1}/StereoOV7251/depth_image/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked" \
"${OAKD2}/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image" \
"${OAKD2}/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" \
"${OAKD2}/StereoOV7251/depth_image@sensor_msgs/msg/Image@gz.msgs.Image" \
"${OAKD2}/StereoOV7251/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" \
"${OAKD2}/StereoOV7251/depth_image/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked" \
"${OAKD3}/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image" \
"${OAKD3}/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" \
"${OAKD3}/StereoOV7251/depth_image@sensor_msgs/msg/Image@gz.msgs.Image" \
"${OAKD3}/StereoOV7251/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" \
"${OAKD3}/StereoOV7251/depth_image/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked" \
"${OAKD4}/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image" \
"${OAKD4}/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" \
"${OAKD4}/StereoOV7251/depth_image@sensor_msgs/msg/Image@gz.msgs.Image" \
"${OAKD4}/StereoOV7251/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" \
"${OAKD4}/StereoOV7251/depth_image/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked" \
--ros-args \
-r "${OAKD1}/IMX214/image:=/oakd1/rgb/image" \
-r "${OAKD1}/IMX214/camera_info:=/oakd1/rgb/camera_info" \
-r "${OAKD1}/StereoOV7251/depth_image:=/oakd1/depth/image" \
-r "${OAKD1}/StereoOV7251/camera_info:=/oakd1/depth/camera_info" \
-r "${OAKD1}/StereoOV7251/depth_image/points:=/oakd1/depth/points" \
-r "${OAKD2}/IMX214/image:=/oakd2/rgb/image" \
-r "${OAKD2}/IMX214/camera_info:=/oakd2/rgb/camera_info" \
-r "${OAKD2}/StereoOV7251/depth_image:=/oakd2/depth/image" \
-r "${OAKD2}/StereoOV7251/camera_info:=/oakd2/depth/camera_info" \
-r "${OAKD2}/StereoOV7251/depth_image/points:=/oakd2/depth/points" \
-r "${OAKD3}/IMX214/image:=/oakd3/rgb/image" \
-r "${OAKD3}/IMX214/camera_info:=/oakd3/rgb/camera_info" \
-r "${OAKD3}/StereoOV7251/depth_image:=/oakd3/depth/image" \
-r "${OAKD3}/StereoOV7251/camera_info:=/oakd3/depth/camera_info" \
-r "${OAKD3}/StereoOV7251/depth_image/points:=/oakd3/depth/points" \
-r "${OAKD4}/IMX214/image:=/oakd4/rgb/image" \
-r "${OAKD4}/IMX214/camera_info:=/oakd4/rgb/camera_info" \
-r "${OAKD4}/StereoOV7251/depth_image:=/oakd4/depth/image" \
-r "${OAKD4}/StereoOV7251/camera_info:=/oakd4/depth/camera_info" \
-r "${OAKD4}/StereoOV7251/depth_image/points:=/oakd4/depth/points"
