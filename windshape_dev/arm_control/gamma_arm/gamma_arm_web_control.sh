#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WEB_PORT="${WEB_PORT:-9000}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
WEB_HOST="${WEB_HOST:-127.0.0.1}"
WEB_URL_HOST="${WEB_URL_HOST:-}"
BRIDGE_WAIT_SEC="${BRIDGE_WAIT_SEC:-5}"
HEALTH_INTERVAL_SEC="${HEALTH_INTERVAL_SEC:-5}"

# Long-term operation switches
GUI_ENABLE="${GUI_ENABLE:-1}"              # 1: open embedded QtWebEngine GUI, 0: backend only
GUI_REQUIRED="${GUI_REQUIRED:-0}"          # 1: fail if embedded GUI cannot start
LOG_ENABLE="${LOG_ENABLE:-0}"              # 1: write subprocess logs under ./log
GZ_KEEPALIVE="${GZ_KEEPALIVE:-1}"          # 1: keep Gazebo command topics subscribed
AUTO_RESTART="${AUTO_RESTART:-1}"          # 1: restart managed backend processes if they die

if [[ -z "${WEB_URL_HOST}" ]]; then
  if [[ "${WEB_HOST}" == "0.0.0.0" ]]; then
    WEB_URL_HOST="127.0.0.1"
  else
    WEB_URL_HOST="${WEB_HOST}"
  fi
fi

ROSBRIDGE_URL="ws://${WEB_URL_HOST}:${ROSBRIDGE_PORT}"
URL_BASE="http://${WEB_URL_HOST}:${WEB_PORT}/index.html"
URL="${URL_BASE}?rosbridge=$(ROSBRIDGE_URL_FOR_QUERY="${ROSBRIDGE_URL}" python3 - <<'PYURL'
from urllib.parse import quote
import os
print(quote(os.environ['ROSBRIDGE_URL_FOR_QUERY'], safe=''))
PYURL
)"

BRIDGE_CONFIG="/tmp/gamma_arm_bridge_${ROSBRIDGE_PORT}_${WEB_PORT}_$$.yaml"
LOG_DIR="${SCRIPT_DIR}/log"

ROSBRIDGE_PID=""
HTTP_PID=""
BRIDGE_PID=""
GUI_PID=""
ROSBRIDGE_STARTED_BY_SCRIPT="false"
declare -a GZ_KEEPALIVE_PIDS=()

log()  { echo "[INFO] $*"; }
warn() { echo "[WARN] $*"; }
err()  { echo "[ERROR] $*" >&2; exit 1; }

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || err "Required command not found: $1"
}

source_ros_setup() {
  local setup_file="$1"
  [[ -f "${setup_file}" ]] || return 0

  local nounset_was_on=0
  case "$-" in
    *u*) nounset_was_on=1; set +u ;;
  esac

  export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
  # shellcheck disable=SC1090
  source "${setup_file}"

  if [[ "${nounset_was_on}" == "1" ]]; then
    set -u
  fi
}

_port_in_use() {
  local port="$1"
  if command -v ss >/dev/null 2>&1; then
    ss -lnt 2>/dev/null | grep -q ":${port} " && return 0
  elif command -v netstat >/dev/null 2>&1; then
    netstat -lnt 2>/dev/null | grep -q ":${port} " && return 0
  fi
  return 1
}

wait_for_port() {
  local port="$1" timeout="$2"
  local end=$((SECONDS + timeout))
  while ! _port_in_use "${port}"; do
    sleep 0.25
    if (( SECONDS >= end )); then
      return 1
    fi
  done
  return 0
}

_kill() {
  local pid="${1:-}"
  [[ -n "${pid}" ]] || return 0
  if kill -0 "${pid}" 2>/dev/null; then
    kill "${pid}" 2>/dev/null || true
  fi
}

_bg() {
  local name="$1"; shift
  if [[ "${LOG_ENABLE}" == "1" ]]; then
    mkdir -p "${LOG_DIR}"
    "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  else
    "$@" >/dev/null 2>&1 &
  fi
  echo $!
}

cleanup() {
  echo ""
  log "Shutting down Gamma Arm web/bridge layer..."

  _kill "${GUI_PID}"
  _kill "${HTTP_PID}"
  _kill "${BRIDGE_PID}"

  if [[ "${ROSBRIDGE_STARTED_BY_SCRIPT}" == "true" ]]; then
    _kill "${ROSBRIDGE_PID}"
  fi

  for pid in "${GZ_KEEPALIVE_PIDS[@]:-}"; do
    _kill "${pid}"
  done

  rm -f "${BRIDGE_CONFIG}" 2>/dev/null || true
  log "Stopped."
}
trap cleanup EXIT INT TERM

need_cmd python3

source_ros_setup /opt/ros/humble/setup.bash

need_cmd ros2

ros2 pkg prefix rosbridge_server >/dev/null 2>&1 || err \
  "rosbridge_server is not installed. Install: sudo apt update && sudo apt install -y ros-humble-rosbridge-server"

ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1 || err \
  "ros_gz_bridge is not available. Install/use: sudo apt update && sudo apt install -y ros-humble-ros-gzharmonic"

[[ -f "${SCRIPT_DIR}/index.html" ]] || err "index.html not found in ${SCRIPT_DIR}"
[[ -f "${SCRIPT_DIR}/gamma_arm_gui.py" ]] || warn "gamma_arm_gui.py not found; embedded GUI will be unavailable."

write_bridge_config() {
  cat > "${BRIDGE_CONFIG}" <<'EOF'
# ROS2 ↔ Gazebo bridge config for Gamma Arm (6 DOF)
# Direction: ROS_TO_GZ for commands, GZ_TO_ROS for optional feedback.

- ros_topic_name: "/gamma_arm/joint1/position_cmd"
  gz_topic_name:  "/joint/gamma/1/position_cmd"
  ros_type_name:  "std_msgs/msg/Float64"
  gz_type_name:   "gz.msgs.Double"
  direction:       ROS_TO_GZ

- ros_topic_name: "/gamma_arm/joint2/position_cmd"
  gz_topic_name:  "/joint/gamma/2/position_cmd"
  ros_type_name:  "std_msgs/msg/Float64"
  gz_type_name:   "gz.msgs.Double"
  direction:       ROS_TO_GZ

- ros_topic_name: "/gamma_arm/joint3/position_cmd"
  gz_topic_name:  "/joint/gamma/3/position_cmd"
  ros_type_name:  "std_msgs/msg/Float64"
  gz_type_name:   "gz.msgs.Double"
  direction:       ROS_TO_GZ

- ros_topic_name: "/gamma_arm/joint4/position_cmd"
  gz_topic_name:  "/joint/gamma/4/position_cmd"
  ros_type_name:  "std_msgs/msg/Float64"
  gz_type_name:   "gz.msgs.Double"
  direction:       ROS_TO_GZ

- ros_topic_name: "/gamma_arm/joint5/position_cmd"
  gz_topic_name:  "/joint/gamma/5/position_cmd"
  ros_type_name:  "std_msgs/msg/Float64"
  gz_type_name:   "gz.msgs.Double"
  direction:       ROS_TO_GZ

- ros_topic_name: "/gamma_arm/joint6/position_cmd"
  gz_topic_name:  "/joint/gamma/6/position_cmd"
  ros_type_name:  "std_msgs/msg/Float64"
  gz_type_name:   "gz.msgs.Double"
  direction:       ROS_TO_GZ

# Optional feedback: Gazebo → ROS2.
# If your ros_gz_bridge build does not support gz.msgs.Model→JointState,
# command control still works; only the Actual curve remains empty.
- ros_topic_name: "/gamma_arm/joint_states"
  gz_topic_name:  "/model/gamma_arm/joint_state"
  ros_type_name:  "sensor_msgs/msg/JointState"
  gz_type_name:   "gz.msgs.Model"
  direction:       GZ_TO_ROS
EOF
}

start_rosbridge() {
  if _port_in_use "${ROSBRIDGE_PORT}"; then
    log "rosbridge already running on port ${ROSBRIDGE_PORT}; reusing it."
    ROSBRIDGE_STARTED_BY_SCRIPT="false"
    ROSBRIDGE_PID=""
    return 0
  fi

  log "Starting rosbridge on port ${ROSBRIDGE_PORT}..."
  ROSBRIDGE_PID="$(_bg rosbridge_${ROSBRIDGE_PORT} ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:="${ROSBRIDGE_PORT}")"
  ROSBRIDGE_STARTED_BY_SCRIPT="true"

  if ! wait_for_port "${ROSBRIDGE_PORT}" 10; then
    err "rosbridge did not bind port ${ROSBRIDGE_PORT} within 10 s. Enable LOG_ENABLE=1 to inspect logs."
  fi
  log "rosbridge ready, PID ${ROSBRIDGE_PID}."
}

start_bridge() {
  write_bridge_config

  log "Starting ros_gz_bridge using launch file..."
  BRIDGE_PID="$(_bg gamma_arm_bridge ros2 launch ros_gz_bridge ros_gz_bridge.launch.py \
    bridge_name:=gamma_arm_bridge \
    config_file:="${BRIDGE_CONFIG}")"

  sleep "${BRIDGE_WAIT_SEC}"
  if kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    log "ros_gz_bridge started, PID ${BRIDGE_PID}."
    return 0
  fi

  warn "ros_gz_bridge launch file failed. Trying parameter_bridge fallback..."
  BRIDGE_PID="$(_bg gamma_arm_bridge ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="${BRIDGE_CONFIG}")"
  sleep 2

  if kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    log "parameter_bridge started, PID ${BRIDGE_PID}."
    return 0
  fi

  err "ros_gz_bridge failed to start with both methods. Enable LOG_ENABLE=1 to inspect logs."
}

start_http() {
  if _port_in_use "${WEB_PORT}"; then
    err "Web port ${WEB_PORT} is already in use. Try: WEB_PORT=9001 bash ${0}"
  fi

  log "Starting local HTTP server on ${WEB_HOST}:${WEB_PORT}..."
  cd "${SCRIPT_DIR}"
  HTTP_PID="$(_bg web_${WEB_PORT} python3 -m http.server "${WEB_PORT}" --bind "${WEB_HOST}")"

  if ! wait_for_port "${WEB_PORT}" 5; then
    err "HTTP server did not start within 5 s. Enable LOG_ENABLE=1 to inspect logs."
  fi
  log "HTTP server ready, PID ${HTTP_PID}."
}

start_one_keepalive() {
  local i="$1" topic="/joint/gamma/${i}/position_cmd"
  if pgrep -f "gz topic -e -t ${topic}" >/dev/null 2>&1; then
    log "  ${topic}: keepalive already running."
    GZ_KEEPALIVE_PIDS[$i]=""
    return 0
  fi
  gz topic -e -t "${topic}" >/dev/null 2>&1 &
  GZ_KEEPALIVE_PIDS[$i]="$!"
  log "  ${topic}: keepalive started, PID ${GZ_KEEPALIVE_PIDS[$i]}."
}

start_gz_keepalive() {
  [[ "${GZ_KEEPALIVE}" == "1" ]] || { log "Gazebo keepalive disabled."; return 0; }
  if ! command -v gz >/dev/null 2>&1; then
    warn "gz not found; skipping Gazebo topic keepalive. Commands may be dropped if no subscriber exists."
    return 0
  fi

  log "Starting Gazebo topic keepalive subscribers..."
  local i
  for i in 1 2 3 4 5 6; do
    start_one_keepalive "${i}"
  done
}

can_start_gui() {
  [[ "${GUI_ENABLE}" == "1" ]] || return 1

  if [[ ! -f "${SCRIPT_DIR}/gamma_arm_gui.py" ]]; then
    warn "Embedded GUI skipped: gamma_arm_gui.py not found."
    return 1
  fi

  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    warn "Embedded GUI skipped: no DISPLAY/WAYLAND_DISPLAY. If running in Docker, expose X11/Wayland or use GUI_ENABLE=0."
    return 1
  fi

  if ! python3 - <<'PY' >/dev/null 2>&1
from PySide6.QtWebEngineWidgets import QWebEngineView
PY
  then
    warn "Embedded GUI skipped: PySide6 QtWebEngine is not installed. Install: pip install PySide6"
    return 1
  fi

  return 0
}

start_gui() {
  if can_start_gui; then
    log "Starting embedded GUI window..."
    python3 "${SCRIPT_DIR}/gamma_arm_gui.py" "${URL}" &
    GUI_PID="$!"
    log "Embedded GUI started, PID ${GUI_PID}."
  else
    if [[ "${GUI_REQUIRED}" == "1" ]]; then
      err "Embedded GUI is required but cannot start."
    fi
    warn "Backend is running. Manual URL if needed: ${URL}"
  fi
}

restart_bridge_if_needed() {
  if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    return 0
  fi
  warn "ros_gz_bridge died. Restarting..."
  start_bridge
}

restart_rosbridge_if_needed() {
  if [[ "${ROSBRIDGE_STARTED_BY_SCRIPT}" != "true" ]]; then
    return 0
  fi
  if [[ -n "${ROSBRIDGE_PID}" ]] && kill -0 "${ROSBRIDGE_PID}" 2>/dev/null && _port_in_use "${ROSBRIDGE_PORT}"; then
    return 0
  fi
  warn "rosbridge died. Restarting..."
  start_rosbridge
}

restart_http_if_needed() {
  if [[ -n "${HTTP_PID}" ]] && kill -0 "${HTTP_PID}" 2>/dev/null && _port_in_use "${WEB_PORT}"; then
    return 0
  fi
  warn "HTTP server died. Restarting..."
  start_http
}

restart_keepalive_if_needed() {
  [[ "${GZ_KEEPALIVE}" == "1" ]] || return 0
  command -v gz >/dev/null 2>&1 || return 0
  local i pid
  for i in 1 2 3 4 5 6; do
    pid="${GZ_KEEPALIVE_PIDS[$i]:-}"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      continue
    fi
    if pgrep -f "gz topic -e -t /joint/gamma/${i}/position_cmd" >/dev/null 2>&1; then
      continue
    fi
    warn "Gazebo keepalive for joint ${i} died. Restarting..."
    start_one_keepalive "${i}"
  done
}

start_rosbridge
start_bridge
start_gz_keepalive
start_http
start_gui

cat <<SUMMARY

  ┌─────────────────────────────────────────────────────┐
  │  Gamma Arm Web Control v2                           │
  ├─────────────────────────────────────────────────────┤
  │  Embedded GUI : ${GUI_ENABLE}
  │  Browser URL  : ${URL}
  │  rosbridge    : ${ROSBRIDGE_URL}
  │  HTTP bind    : ${WEB_HOST}:${WEB_PORT}
  │  Bridge config: ${BRIDGE_CONFIG}
  ├─────────────────────────────────────────────────────┤
  │  No CDN dependency: this page uses built-in MiniROSLIB.
  │  External browser is not launched by this script.
  ├─────────────────────────────────────────────────────┤
  │  Test command:
  │    ros2 topic pub --once /gamma_arm/joint1/position_cmd \
  │        std_msgs/msg/Float64 '{data: 0.5}'
  │  Check Gazebo receive:
  │    gz topic -e -t /joint/gamma/1/position_cmd
  ├─────────────────────────────────────────────────────┤
  │  Ctrl+C stops GUI + web + bridge layer only.         │
  │  PX4 / Gazebo keep running in the other terminal.   │
  └─────────────────────────────────────────────────────┘

SUMMARY

while true; do
  sleep "${HEALTH_INTERVAL_SEC}"

  if [[ "${AUTO_RESTART}" == "1" ]]; then
    restart_rosbridge_if_needed
    restart_bridge_if_needed
    restart_http_if_needed
    restart_keepalive_if_needed
  fi
done
