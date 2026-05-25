#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WEB_PORT="${WEB_PORT:-9000}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
WEB_HOST="127.0.0.1"

URL="http://${WEB_HOST}:${WEB_PORT}/index.html"
ROSBRIDGE_URL="ws://${WEB_HOST}:${ROSBRIDGE_PORT}"

LOG_DIR="${SCRIPT_DIR}/log"
ROSBRIDGE_LOG="${LOG_DIR}/rosbridge_${ROSBRIDGE_PORT}.log"
WEB_LOG="${LOG_DIR}/web_${WEB_PORT}.log"

ROSBRIDGE_PID=""
HTTP_PID=""
ROSBRIDGE_STARTED_BY_SCRIPT="false"

mkdir -p "$LOG_DIR"

cleanup() {
  echo ""
  echo "[INFO] Shutting down..."

  if [[ -n "$HTTP_PID" ]] && kill -0 "$HTTP_PID" 2>/dev/null; then
    kill "$HTTP_PID" 2>/dev/null || true
  fi

  if [[ "$ROSBRIDGE_STARTED_BY_SCRIPT" == "true" ]] && [[ -n "$ROSBRIDGE_PID" ]] && kill -0 "$ROSBRIDGE_PID" 2>/dev/null; then
    kill "$ROSBRIDGE_PID" 2>/dev/null || true
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

if ! ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then
  echo "[ERROR] rosbridge_server is not installed."
  echo "        Please install it with:"
  echo "        sudo apt update"
  echo "        sudo apt install ros-humble-rosbridge-server"
  exit 1
fi

if [[ ! -f "${SCRIPT_DIR}/index.html" ]]; then
  echo "[WARN] index.html was not found in:"
  echo "       ${SCRIPT_DIR}"
  echo "       The web server will still start, but the control page may not open correctly."
fi

if port_in_use "$ROSBRIDGE_PORT"; then
  echo "[INFO] rosbridge already running on port ${ROSBRIDGE_PORT}."
else
  echo "[INFO] Starting rosbridge on port ${ROSBRIDGE_PORT}..."
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    port:="${ROSBRIDGE_PORT}" \
    > "$ROSBRIDGE_LOG" 2>&1 &

  ROSBRIDGE_PID=$!
  ROSBRIDGE_STARTED_BY_SCRIPT="true"
  sleep 2
fi

if port_in_use "$WEB_PORT"; then
  echo "[ERROR] Web port ${WEB_PORT} is already in use."
  echo "        Try another port       "
  exit 1
fi

echo "[INFO] Starting web server on port ${WEB_PORT}..."
cd "$SCRIPT_DIR"

python3 -m http.server "$WEB_PORT" --bind "$WEB_HOST" \
  > "$WEB_LOG" 2>&1 &

HTTP_PID=$!

sleep 1

echo ""
echo "Web control panel:"
echo "  ${URL}"
echo ""
echo "rosbridge websocket:"
echo "  ${ROSBRIDGE_URL}"
echo ""
echo "Logs:"
echo "  ${LOG_DIR}"
echo ""
echo "Press Ctrl+C to stop."

if command -v xdg-open >/dev/null 2>&1; then
  xdg-open "$URL" >/dev/null 2>&1 || true
fi

while true; do
  sleep 1
done
