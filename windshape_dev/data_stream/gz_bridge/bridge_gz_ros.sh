#!/bin/bash
set -eo pipefail

source /opt/ros/humble/setup.bash

WORLD="/world/laboratory/model/q940_ti_0"

OAKD1="${WORLD}/model/oakd_lite_one/link/camera_link/sensor"
OAKD2="${WORLD}/model/oakd_lite_two/link/camera_link/sensor"
OAKD3="${WORLD}/model/oakd_lite_three/link/camera_link/sensor"
OAKD4="${WORLD}/model/oakd_lite_four/link/camera_link/sensor"

# UAV body ground-truth odometry
DRONE_GT_ODOM="/model/q940_ti_0/odometry"
DRONE_GT_ODOM_COV="/model/q940_ti_0/odometry_with_covariance"

# Gripper ground-truth odometry
GRIPPER_GT_ODOM="/model/q940_ti_0/model/gripper3/odometry"
GRIPPER_GT_ODOM_COV="/model/q940_ti_0/model/gripper3/odometry_with_covariance"

# Gripper joint state: Gazebo -> ROS 2
GRIPPER_JOINT_STATE="/gripper3/joint_state"

# Gripper position command: ROS 2 -> Gazebo
GRIPPER_POSITION_CMD="/joint/gripper3/position_cmd"

# Arm joint state: Gazebo -> ROS 2
ARM_JOINT_STATE="/model/ti5_arm/joint_state"

cleanup()
{
    echo ""
    echo "Stopping bridge and static TF..."

    if [ -n "${TF_CAMERA_PID:-}" ]; then
        kill "$TF_CAMERA_PID" 2>/dev/null || true
    fi

    if [ -n "${TF_WORLD_PID:-}" ]; then
        kill "$TF_WORLD_PID" 2>/dev/null || true
    fi

    if [ -n "${ARM_CMD_BRIDGE_PID:-}" ]; then
        kill "$ARM_CMD_BRIDGE_PID" 2>/dev/null || true
    fi

    if [ -n "${ARM_CMD_BRIDGE_YAML:-}" ] && [ -f "$ARM_CMD_BRIDGE_YAML" ]; then
        rm -f "$ARM_CMD_BRIDGE_YAML"
    fi
}

trap cleanup EXIT INT TERM

ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id map \
    --child-frame-id camera_link &

TF_CAMERA_PID=$!

ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id map \
    --child-frame-id world &

TF_WORLD_PID=$!

ARM_CMD_BRIDGE_YAML="$(mktemp /tmp/ti5_arm_cmd_bridge_XXXXXX.yaml)"

cat > "$ARM_CMD_BRIDGE_YAML" <<'EOF'
- ros_topic_name: "/ti5_arm/joint1/command"
  gz_topic_name: "/joint/1/position_cmd"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ
  lazy: false

- ros_topic_name: "/ti5_arm/joint2/command"
  gz_topic_name: "/joint/2/position_cmd"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ
  lazy: false

- ros_topic_name: "/ti5_arm/joint3/command"
  gz_topic_name: "/joint/3/position_cmd"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ
  lazy: false

- ros_topic_name: "/ti5_arm/joint4/command"
  gz_topic_name: "/joint/4/position_cmd"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ
  lazy: false

- ros_topic_name: "/ti5_arm/joint5/command"
  gz_topic_name: "/joint/5/position_cmd"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ
  lazy: false

- ros_topic_name: "/ti5_arm/joint6/command"
  gz_topic_name: "/joint/6/position_cmd"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ
  lazy: false
EOF

ros2 run ros_gz_bridge parameter_bridge \
    --ros-args \
    -p config_file:="$ARM_CMD_BRIDGE_YAML" \
    -r __node:=ti5_arm_cmd_bridge &

ARM_CMD_BRIDGE_PID=$!

ros2 run ros_gz_bridge parameter_bridge \
"${DRONE_GT_ODOM}@nav_msgs/msg/Odometry@gz.msgs.Odometry" \
"${DRONE_GT_ODOM_COV}@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance" \
"${GRIPPER_GT_ODOM}@nav_msgs/msg/Odometry@gz.msgs.Odometry" \
"${GRIPPER_GT_ODOM_COV}@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance" \
"${GRIPPER_JOINT_STATE}@sensor_msgs/msg/JointState@gz.msgs.Model" \
"${GRIPPER_POSITION_CMD}@std_msgs/msg/Float64]gz.msgs.Double" \
"${ARM_JOINT_STATE}@sensor_msgs/msg/JointState@gz.msgs.Model" \
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
-r "${DRONE_GT_ODOM}:=/q940/ground_truth/odom" \
-r "${DRONE_GT_ODOM_COV}:=/q940/ground_truth/odom_with_covariance" \
-r "${GRIPPER_GT_ODOM}:=/gripper3/ground_truth/odom" \
-r "${GRIPPER_GT_ODOM_COV}:=/gripper3/ground_truth/odom_with_covariance" \
-r "${GRIPPER_JOINT_STATE}:=/gripper3/joint_states" \
-r "${GRIPPER_POSITION_CMD}:=/gripper3/command/position" \
-r "${ARM_JOINT_STATE}:=/ti5_arm/joint_states" \
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
