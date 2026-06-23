#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WEB_PORT="${WEB_PORT:-8000}"
VIDEO_PORT="${VIDEO_PORT:-8080}"
WEB_HOST="${WEB_HOST:-127.0.0.1}"
VIDEO_HOST="${VIDEO_HOST:-127.0.0.1}"

MONITOR_PAGE="${MONITOR_PAGE:-index.html}"

URL="http://${WEB_HOST}:${WEB_PORT}/${MONITOR_PAGE}"
VIDEO_URL="http://${VIDEO_HOST}:${VIDEO_PORT}"

LOG_DIR="${SCRIPT_DIR}/log"
WEB_VIDEO_LOG="${LOG_DIR}/web_video_server_${VIDEO_PORT}.log"
WEB_LOG="${LOG_DIR}/camera_monitor_web_${WEB_PORT}.log"

WEB_VIDEO_PID=""
HTTP_PID=""
WEB_VIDEO_STARTED_BY_SCRIPT="false"

mkdir -p "$LOG_DIR"

cleanup() {
  echo ""
  echo "[INFO] Shutting down camera monitor..."

  if [[ -n "$HTTP_PID" ]] && kill -0 "$HTTP_PID" 2>/dev/null; then
    kill "$HTTP_PID" 2>/dev/null || true
  fi

  if [[ "$WEB_VIDEO_STARTED_BY_SCRIPT" == "true" ]] && [[ -n "$WEB_VIDEO_PID" ]] && kill -0 "$WEB_VIDEO_PID" 2>/dev/null; then
    kill "$WEB_VIDEO_PID" 2>/dev/null || true
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

need_cmd ros2
need_cmd python3

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi

if ! ros2 pkg prefix web_video_server >/dev/null 2>&1; then
  echo "[ERROR] web_video_server is not installed."
  echo "        Please install it with:"
  echo "        sudo apt update"
  echo "        sudo apt install ros-humble-web-video-server"
  exit 1
fi

if [[ ! -f "${SCRIPT_DIR}/${MONITOR_PAGE}" ]]; then
  echo "[ERROR] Monitor page was not found:"
  echo "        ${SCRIPT_DIR}/${MONITOR_PAGE}"
  exit 1
fi

if ! grep -qE "8080|stream|web_video_server|videoServer" "${SCRIPT_DIR}/${MONITOR_PAGE}"; then
  echo "[WARN] ${MONITOR_PAGE} does not seem to contain stream monitor code."
  echo "       Please check whether this is the correct frontend file."
fi

if port_in_use "$VIDEO_PORT"; then
  echo "[INFO] web_video_server already running on port ${VIDEO_PORT}."
else
  echo "[INFO] Starting web_video_server on port ${VIDEO_PORT}..."

  ros2 run web_video_server web_video_server \
    --ros-args \
    -p port:="${VIDEO_PORT}" \
    > "$WEB_VIDEO_LOG" 2>&1 &

  WEB_VIDEO_PID=$!
  WEB_VIDEO_STARTED_BY_SCRIPT="true"
  sleep 2
fi

if port_in_use "$WEB_PORT"; then
  echo "[ERROR] Web port ${WEB_PORT} is already in use."
  echo "        Stop the old server or try another port:"
  echo "        WEB_PORT=8001 ./camera_stream.sh"
  exit 1
fi

echo "[INFO] Starting local web server on port ${WEB_PORT}..."
cd "$SCRIPT_DIR"

python3 -m http.server "$WEB_PORT" --bind "$WEB_HOST" \
  > "$WEB_LOG" 2>&1 &

HTTP_PID=$!

sleep 1

echo ""
echo "Camera monitor page:"
echo "  ${URL}"
echo ""
echo "Serving directory:"
echo "  ${SCRIPT_DIR}"
echo ""
echo "web_video_server:"
echo "  ${VIDEO_URL}"
echo ""
echo "Example stream:"
echo "  ${VIDEO_URL}/stream?topic=/oakd2/rgb/image"
echo ""
echo "Logs:"
echo "  ${LOG_DIR}"
echo ""
echo "Press Ctrl+C to stop."

if command -v xdg-open >/dev/null 2>&1; then
  xdg-open "$URL?t=$(date +%s)" >/dev/null 2>&1 || true
fi

while true; do
  sleep 1
done
