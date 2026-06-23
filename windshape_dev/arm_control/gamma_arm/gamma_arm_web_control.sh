#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WEB_PORT="${WEB_PORT:-9000}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
WEB_HOST="${WEB_HOST:-127.0.0.1}"

URL="http://${WEB_HOST}:${WEB_PORT}/index.html"
ROSBRIDGE_URL="ws://${WEB_HOST}:${ROSBRIDGE_PORT}"

LOG_DIR="${SCRIPT_DIR}/log"
ROSBRIDGE_LOG="${LOG_DIR}/rosbridge_${ROSBRIDGE_PORT}.log"
WEB_LOG="${LOG_DIR}/web_${WEB_PORT}.log"
BRIDGE_LOG="${LOG_DIR}/gamma_arm_bridge.log"
BRIDGE_CONFIG="${LOG_DIR}/gamma_arm_bridge.yaml"

ROSBRIDGE_PID=""
HTTP_PID=""
BRIDGE_PID=""
ROSBRIDGE_STARTED_BY_SCRIPT="false"
GZ_KEEPALIVE_PIDS=()

mkdir -p "${LOG_DIR}"

cleanup() {
  echo ""
  echo "[INFO] Shutting down Gamma arm web/bridge layer..."

  if [[ -n "${HTTP_PID}" ]] && kill -0 "${HTTP_PID}" 2>/dev/null; then
    kill "${HTTP_PID}" 2>/dev/null || true
  fi

  if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    kill "${BRIDGE_PID}" 2>/dev/null || true
  fi

  if [[ "${ROSBRIDGE_STARTED_BY_SCRIPT}" == "true" ]] && [[ -n "${ROSBRIDGE_PID}" ]] && kill -0 "${ROSBRIDGE_PID}" 2>/dev/null; then
    kill "${ROSBRIDGE_PID}" 2>/dev/null || true
  fi

  if [[ ${#GZ_KEEPALIVE_PIDS[@]} -gt 0 ]]; then
    for pid in "${GZ_KEEPALIVE_PIDS[@]}"; do
      if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
        kill "${pid}" 2>/dev/null || true
      fi
    done
  fi

  echo "[INFO] Stopped."
}
trap cleanup EXIT INT TERM

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[ERROR] Command not found: $1"
    exit 1
  fi
}

port_in_use() {
  local port="$1"

  if command -v ss >/dev/null 2>&1; then
    ss -lnt 2>/dev/null | grep -q ":${port} "
    return $?
  fi

  if command -v netstat >/dev/null 2>&1; then
    netstat -lnt 2>/dev/null | grep -q ":${port} "
    return $?
  fi

  return 1
}

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi

need_cmd ros2
need_cmd python3

if ! ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then
  echo "[ERROR] rosbridge_server is not installed."
  echo "        Install inside the container:"
  echo "        sudo apt update && sudo apt install -y ros-humble-rosbridge-server"
  exit 1
fi

if ! ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
  echo "[ERROR] ros_gz_bridge is not available."
  echo "        For your Harmonic image, install/use:"
  echo "        sudo apt update && sudo apt install -y ros-humble-ros-gzharmonic"
  exit 1
fi

if [[ ! -f "${SCRIPT_DIR}/index.html" ]]; then
  echo "[ERROR] index.html was not found in: ${SCRIPT_DIR}"
  exit 1
fi

cat > "${BRIDGE_CONFIG}" <<'EOF'
# ROS2 side names must be legal ROS2 topic names.
# Gazebo side names keep the original GammaArmControlPlugin topics.

- ros_topic_name: "/gamma_arm/joint1/position_cmd"
  gz_topic_name: "/joint/gamma/1/position_cmd"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/gamma_arm/joint2/position_cmd"
  gz_topic_name: "/joint/gamma/2/position_cmd"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/gamma_arm/joint3/position_cmd"
  gz_topic_name: "/joint/gamma/3/position_cmd"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/gamma_arm/joint4/position_cmd"
  gz_topic_name: "/joint/gamma/4/position_cmd"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/gamma_arm/joint5/position_cmd"
  gz_topic_name: "/joint/gamma/5/position_cmd"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/gamma_arm/joint6/position_cmd"
  gz_topic_name: "/joint/gamma/6/position_cmd"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

# Feedback is optional. If this bridge is unsupported in your local ros_gz_bridge
# build, command control still works; only the green Actual curve remains empty.
- ros_topic_name: "/gamma_arm/joint_states"
  gz_topic_name: "/model/gamma_arm/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS
EOF

if port_in_use "${ROSBRIDGE_PORT}"; then
  echo "[INFO] rosbridge already running on port ${ROSBRIDGE_PORT}."
else
  echo "[INFO] Starting rosbridge on port ${ROSBRIDGE_PORT}..."
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    port:="${ROSBRIDGE_PORT}" \
    > "${ROSBRIDGE_LOG}" 2>&1 &

  ROSBRIDGE_PID=$!
  ROSBRIDGE_STARTED_BY_SCRIPT="true"
  sleep 2
fi

start_yaml_bridge() {
  echo "[INFO] Starting ros_gz_bridge with YAML config:"
  echo "       ${BRIDGE_CONFIG}"

  # Preferred path for ros_gz_bridge versions that provide the launch file.
  ros2 launch ros_gz_bridge ros_gz_bridge.launch.py \
    bridge_name:=gamma_arm_bridge \
    config_file:="${BRIDGE_CONFIG}" \
    > "${BRIDGE_LOG}" 2>&1 &

  BRIDGE_PID=$!
  sleep 2

  if kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    return 0
  fi

  echo "[WARN] ros2 launch ros_gz_bridge failed. Trying parameter_bridge fallback..."
  echo "[WARN] Previous log:"
  tail -n 50 "${BRIDGE_LOG}" || true

  ros2 run ros_gz_bridge parameter_bridge \
    --ros-args -p config_file:="${BRIDGE_CONFIG}" \
    > "${BRIDGE_LOG}" 2>&1 &

  BRIDGE_PID=$!
  sleep 2

  if ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    echo "[ERROR] ros_gz_bridge failed to start."
    echo "        Check log:"
    echo "        ${BRIDGE_LOG}"
    tail -n 100 "${BRIDGE_LOG}" || true
    exit 1
  fi
}


start_gz_keepalive_subscribers() {
  if ! command -v gz >/dev/null 2>&1; then
    echo "[WARN] gz command not found; skip Gazebo topic keepalive subscribers."
    return 0
  fi

  echo "[INFO] Starting Gazebo command topic keepalive subscribers..."

  for i in 1 2 3 4 5 6; do
    local topic="/joint/gamma/${i}/position_cmd"

    if pgrep -f "gz topic -e -t ${topic}" >/dev/null 2>&1; then
      echo "       ${topic} already has a keepalive subscriber."
      continue
    fi

    gz topic -e -t "${topic}" > /dev/null 2>&1 &
    GZ_KEEPALIVE_PIDS+=("$!")

    echo "       keepalive: ${topic}"
  done

  sleep 0.5
}

start_yaml_bridge
start_gz_keepalive_subscribers

if port_in_use "${WEB_PORT}"; then
  echo "[ERROR] Web port ${WEB_PORT} is already in use."
  echo "        Try another port, for example:"
  echo "        WEB_PORT=9001 bash ${0}"
  exit 1
fi

echo "[INFO] Starting web server on ${WEB_HOST}:${WEB_PORT}..."
cd "${SCRIPT_DIR}"

python3 -m http.server "${WEB_PORT}" --bind "${WEB_HOST}" \
  > "${WEB_LOG}" 2>&1 &

HTTP_PID=$!
sleep 1

echo ""
echo "Gamma Arm Web Control:"
echo "  ${URL}"
echo ""
echo "rosbridge websocket:"
echo "  ${ROSBRIDGE_URL}"
echo ""
echo "ROS2 command topics published by browser:"
for i in 1 2 3 4 5 6; do
  echo "  /gamma_arm/joint${i}/position_cmd"
done
echo ""
echo "Gazebo command topics received by GammaArmControlPlugin:"
for i in 1 2 3 4 5 6; do
  echo "  /joint/gamma/${i}/position_cmd"
done
echo ""
echo "Feedback topic:"
echo "  ROS2   : /gamma_arm/joint_states"
echo "  Gazebo : /model/gamma_arm/joint_state"
echo ""
echo "Bridge config:"
echo "  ${BRIDGE_CONFIG}"
echo ""
echo "Logs:"
echo "  ${LOG_DIR}"
echo ""
echo "Test without browser:"
echo "  ros2 topic pub --once /gamma_arm/joint1/position_cmd std_msgs/msg/Float64 '{data: 0.5}'"
echo ""
echo "Check Gazebo receiving:"
echo "  gz topic -e -t /joint/gamma/1/position_cmd"
echo ""
echo "Gazebo topic keepalive:"
echo "  Started automatically for /joint/gamma/1~6/position_cmd"
echo ""
echo "Press Ctrl+C to stop only the web/bridge layer. PX4/Gazebo keeps running in the first terminal."

if command -v xdg-open >/dev/null 2>&1; then
  xdg-open "${URL}" >/dev/null 2>&1 || true
fi

while true; do
  sleep 1
done
