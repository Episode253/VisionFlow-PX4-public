#!/usr/bin/env bash
set -eo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-visionflow-px4-sitl}"
WORKDIR="${WORKDIR:-/workspace/VisionFlow-PX4}"

AUTO_WEB_CONTROL="${AUTO_WEB_CONTROL:-true}"
WEB_SESSION="${WEB_SESSION:-gamma_web}"
WEB_HOST="${WEB_HOST:-127.0.0.1}"
WEB_PORT="${WEB_PORT:-9000}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
WEB_SCRIPT="${WEB_SCRIPT:-${WORKDIR}/windshape_dev/arm_control/gamma_arm/gamma_arm_web_control.sh}"

GUI_ENABLE="${GUI_ENABLE:-1}"
LOG_ENABLE="${LOG_ENABLE:-0}"
AUTO_RESTART="${AUTO_RESTART:-1}"
GZ_KEEPALIVE="${GZ_KEEPALIVE:-0}"
QTWEBENGINE_DISABLE_SANDBOX="${QTWEBENGINE_DISABLE_SANDBOX:-1}"
QTWEBENGINE_CHROMIUM_FLAGS="${QTWEBENGINE_CHROMIUM_FLAGS:---no-sandbox --disable-gpu --disable-dev-shm-usage}"

usage() {
    cat <<USAGE
Usage:
  bash docker/into_gz_sitl.sh
  bash docker/into_gz_sitl.sh --web
  bash docker/into_gz_sitl.sh --no-web
  bash docker/into_gz_sitl.sh --web --gui
  bash docker/into_gz_sitl.sh --web --no-gui
  bash docker/into_gz_sitl.sh --web --gui=1
  bash docker/into_gz_sitl.sh --web --gui=0

Environment:
  GUI_ENABLE=0|1
  LOG_ENABLE=0|1
  AUTO_RESTART=0|1
  GZ_KEEPALIVE=0|1
  WEB_HOST=127.0.0.1
  WEB_PORT=9000
  ROSBRIDGE_PORT=9090
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-web)
            AUTO_WEB_CONTROL="false"; shift ;;
        --web)
            AUTO_WEB_CONTROL="true"; shift ;;
        --gui|--gui=1)
            GUI_ENABLE="1"; shift ;;
        --gui=0|--no-gui)
            GUI_ENABLE="0"; shift ;;
        AUTO_RESTART=*) AUTO_RESTART="${1#*=}"; shift ;;
        LOG_ENABLE=*) LOG_ENABLE="${1#*=}"; shift ;;
        GUI_ENABLE=*) GUI_ENABLE="${1#*=}"; shift ;;
        GZ_KEEPALIVE=*) GZ_KEEPALIVE="${1#*=}"; shift ;;
        WEB_PORT=*) WEB_PORT="${1#*=}"; shift ;;
        ROSBRIDGE_PORT=*) ROSBRIDGE_PORT="${1#*=}"; shift ;;
        WEB_HOST=*) WEB_HOST="${1#*=}"; shift ;;
        -h|--help)
            usage; exit 0 ;;
        *)
            echo "[ERROR] Unknown option: $1" >&2
            usage
            exit 1 ;;
    esac
done

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
    echo "[ERROR] Container is not running: ${CONTAINER_NAME}" >&2
    echo ""
    echo "Please start PX4/Gazebo SITL first:"
    echo "  bash docker/run_gz_sitl.sh --profile \"Entity 1\""
    echo ""
    echo "Current running containers:"
    docker ps --format '  {{.Names}}\t{{.Status}}'
    exit 1
fi

TMPDIR_HOST="$(mktemp -d)"
cleanup_tmp() { rm -rf "${TMPDIR_HOST}" 2>/dev/null || true; }
trap cleanup_tmp EXIT

cat > "${TMPDIR_HOST}/gamma_web_runner.sh" <<'RUNNER'
#!/usr/bin/env bash
set -eo pipefail

source_ros_setup() {
    local setup_file="${1:-/opt/ros/humble/setup.bash}"
    if [ -f "${setup_file}" ]; then
        set +u
        # shellcheck disable=SC1090
        source "${setup_file}"
        set +u
    fi
}

source_ros_setup /opt/ros/humble/setup.bash
if [ -f /workspace/VisionFlow-PX4/install/setup.bash ]; then
    source_ros_setup /workspace/VisionFlow-PX4/install/setup.bash
fi

# MAVROS workspace (thirdparty/mavros-humble), built with colcon.
if [ -f /workspace/VisionFlow-PX4/thirdparty/mavros-humble/install/setup.bash ]; then
    source_ros_setup /workspace/VisionFlow-PX4/thirdparty/mavros-humble/install/setup.bash
fi

cd "${WORKDIR:-/workspace/VisionFlow-PX4}" 2>/dev/null || cd /

if [ ! -f "${WEB_SCRIPT:-}" ]; then
    echo "[ERROR] Gamma arm web control script not found:"
    echo "        ${WEB_SCRIPT:-<empty>}"
    exit 1
fi

export WEB_HOST="${WEB_HOST:-127.0.0.1}"
export WEB_PORT="${WEB_PORT:-9000}"
export ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
export GUI_ENABLE="${GUI_ENABLE:-1}"
export LOG_ENABLE="${LOG_ENABLE:-0}"
export AUTO_RESTART="${AUTO_RESTART:-1}"
export GZ_KEEPALIVE="${GZ_KEEPALIVE:-0}"
export QTWEBENGINE_DISABLE_SANDBOX="${QTWEBENGINE_DISABLE_SANDBOX:-1}"
export QTWEBENGINE_CHROMIUM_FLAGS="${QTWEBENGINE_CHROMIUM_FLAGS:---no-sandbox --disable-gpu --disable-dev-shm-usage}"

mkdir -p "$(dirname "${WEB_SCRIPT}")/log" 2>/dev/null || true

echo "[runner] WORKDIR=${WORKDIR:-/workspace/VisionFlow-PX4}"
echo "[runner] WEB_SCRIPT=${WEB_SCRIPT}"
echo "[runner] WEB_HOST=${WEB_HOST} WEB_PORT=${WEB_PORT} ROSBRIDGE_PORT=${ROSBRIDGE_PORT}"
echo "[runner] GUI_ENABLE=${GUI_ENABLE} LOG_ENABLE=${LOG_ENABLE} AUTO_RESTART=${AUTO_RESTART} GZ_KEEPALIVE=${GZ_KEEPALIVE}"
echo "[runner] QTWEBENGINE_CHROMIUM_FLAGS=${QTWEBENGINE_CHROMIUM_FLAGS}"

exec bash "${WEB_SCRIPT}"
RUNNER

cat > "${TMPDIR_HOST}/visionflow_interactive_bashrc" <<'RC'

export TERM=xterm-256color
export FORCE_COLOR=1
export CLICOLOR=1
export WEB_SESSION="${WEB_SESSION:-gamma_web}"
export WORKDIR="${WORKDIR:-/workspace/VisionFlow-PX4}"
export WEB_HOST="${WEB_HOST:-127.0.0.1}"
export WEB_PORT="${WEB_PORT:-9000}"
export ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
export GUI_ENABLE="${GUI_ENABLE:-1}"
export LOG_ENABLE="${LOG_ENABLE:-0}"
export AUTO_RESTART="${AUTO_RESTART:-1}"
export GZ_KEEPALIVE="${GZ_KEEPALIVE:-0}"
export QTWEBENGINE_DISABLE_SANDBOX="${QTWEBENGINE_DISABLE_SANDBOX:-1}"
export QTWEBENGINE_CHROMIUM_FLAGS="${QTWEBENGINE_CHROMIUM_FLAGS:---no-sandbox --disable-gpu --disable-dev-shm-usage}"

source_ros_setup() {
    local setup_file="${1:-/opt/ros/humble/setup.bash}"
    if [ -f "${setup_file}" ]; then
        set +u
        # shellcheck disable=SC1090
        source "${setup_file}"
        set +u
    fi
}

source_ros_setup /opt/ros/humble/setup.bash
if [ -f /workspace/VisionFlow-PX4/install/setup.bash ]; then
    source_ros_setup /workspace/VisionFlow-PX4/install/setup.bash
fi
# MAVROS workspace (thirdparty/mavros-humble), built with colcon.
if [ -f /workspace/VisionFlow-PX4/thirdparty/mavros-humble/install/setup.bash ]; then
    source_ros_setup /workspace/VisionFlow-PX4/thirdparty/mavros-humble/install/setup.bash
fi

alias ls="ls --color=auto"
alias ll="ls -alF --color=auto"
alias la="ls -A --color=auto"
alias grep="grep --color=auto"
alias croot="cd /workspace/VisionFlow-PX4"
alias gzlist="gz topic -l"
alias gzps='ps aux | grep -E "gz|px4|ros|bridge" | grep -v grep'

_tmux_web_start() {
    env \
        WORKDIR="${WORKDIR:-/workspace/VisionFlow-PX4}" \
        WEB_SCRIPT="${WEB_SCRIPT:-/workspace/VisionFlow-PX4/windshape_dev/arm_control/gamma_arm/gamma_arm_web_control.sh}" \
        WEB_SESSION="${WEB_SESSION:-gamma_web}" \
        WEB_HOST="${WEB_HOST:-127.0.0.1}" \
        WEB_PORT="${WEB_PORT:-9000}" \
        ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}" \
        GUI_ENABLE="${GUI_ENABLE:-1}" \
        LOG_ENABLE="${LOG_ENABLE:-0}" \
        AUTO_RESTART="${AUTO_RESTART:-1}" \
        GZ_KEEPALIVE="${GZ_KEEPALIVE:-0}" \
        QTWEBENGINE_DISABLE_SANDBOX="${QTWEBENGINE_DISABLE_SANDBOX:-1}" \
        QTWEBENGINE_CHROMIUM_FLAGS="${QTWEBENGINE_CHROMIUM_FLAGS:---no-sandbox --disable-gpu --disable-dev-shm-usage}" \
        /tmp/gamma_web_runner.sh
}

webstart() {
    if ! command -v tmux >/dev/null 2>&1; then
        echo "[ERROR] tmux not installed."
        return 1
    fi

    if [ ! -x /tmp/gamma_web_runner.sh ]; then
        echo "[ERROR] /tmp/gamma_web_runner.sh not found. Re-run docker/into_gz_sitl.sh from host."
        return 1
    fi

    if tmux has-session -t "${WEB_SESSION:-gamma_web}" 2>/dev/null; then
        echo "[OK] Gamma web already running in tmux: ${WEB_SESSION:-gamma_web}"
    else
        tmux new-session -d -s "${WEB_SESSION:-gamma_web}" env \
            WORKDIR="${WORKDIR:-/workspace/VisionFlow-PX4}" \
            WEB_SCRIPT="${WEB_SCRIPT:-/workspace/VisionFlow-PX4/windshape_dev/arm_control/gamma_arm/gamma_arm_web_control.sh}" \
            WEB_SESSION="${WEB_SESSION:-gamma_web}" \
            WEB_HOST="${WEB_HOST:-127.0.0.1}" \
            WEB_PORT="${WEB_PORT:-9000}" \
            ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}" \
            GUI_ENABLE="${GUI_ENABLE:-1}" \
            LOG_ENABLE="${LOG_ENABLE:-0}" \
            AUTO_RESTART="${AUTO_RESTART:-1}" \
            GZ_KEEPALIVE="${GZ_KEEPALIVE:-0}" \
            QTWEBENGINE_DISABLE_SANDBOX="${QTWEBENGINE_DISABLE_SANDBOX:-1}" \
            QTWEBENGINE_CHROMIUM_FLAGS="${QTWEBENGINE_CHROMIUM_FLAGS:---no-sandbox --disable-gpu --disable-dev-shm-usage}" \
            /tmp/gamma_web_runner.sh
        sleep 1
        if tmux has-session -t "${WEB_SESSION:-gamma_web}" 2>/dev/null; then
            echo "[OK] Gamma web started in tmux: ${WEB_SESSION:-gamma_web}"
        else
            echo "[ERROR] Gamma web tmux session exited quickly."
            echo "        Run: weblog"
            return 1
        fi
    fi

    echo "[OK] Web UI    : http://${WEB_HOST:-127.0.0.1}:${WEB_PORT:-9000}/index.html"
    echo "[OK] rosbridge : ws://${WEB_HOST:-127.0.0.1}:${ROSBRIDGE_PORT:-9090}"
    echo "[OK] GUI_ENABLE=${GUI_ENABLE:-1} LOG_ENABLE=${LOG_ENABLE:-0} AUTO_RESTART=${AUTO_RESTART:-1} GZ_KEEPALIVE=${GZ_KEEPALIVE:-0}"
}

webstart_gui() {
    export GUI_ENABLE=1
    webstop >/dev/null 2>&1 || true
    webstart
}

webstart_headless() {
    export GUI_ENABLE=0
    webstop >/dev/null 2>&1 || true
    webstart
}

webstop() {
    if command -v tmux >/dev/null 2>&1 && tmux has-session -t "${WEB_SESSION:-gamma_web}" 2>/dev/null; then
        tmux kill-session -t "${WEB_SESSION:-gamma_web}"
        echo "[OK] Stopped tmux session: ${WEB_SESSION:-gamma_web}"
    fi
    pkill -f "gamma_arm_web_control.sh" 2>/dev/null || true
    pkill -f "gamma_arm_gui.py" 2>/dev/null || true
}

weblog() {
    if command -v tmux >/dev/null 2>&1 && tmux has-session -t "${WEB_SESSION:-gamma_web}" 2>/dev/null; then
        tmux capture-pane -pt "${WEB_SESSION:-gamma_web}" -S -260
    else
        echo "[WARN] tmux session not running: ${WEB_SESSION:-gamma_web}"
        echo "[INFO] Recent logs:"
        tail -n 160 /workspace/VisionFlow-PX4/windshape_dev/arm_control/gamma_arm/log/*.log 2>/dev/null || true
    fi
}

webattach() {
    tmux attach -t "${WEB_SESSION:-gamma_web}"
}

webps() {
    ps aux | grep -E "rosbridge|ros_gz_bridge|parameter_bridge|http.server|gamma_arm_web_control|gamma_arm_gui|QtWebEngine" | grep -v grep || true
}

webcheck() {
    echo "[check] tmux:"
    tmux ls 2>/dev/null || true
    echo ""
    echo "[check] processes:"
    webps
    echo ""
    echo "[check] HTTP:"
    curl -I "http://${WEB_HOST:-127.0.0.1}:${WEB_PORT:-9000}/index.html" 2>/dev/null | head -n 5 || true
    echo ""
    echo "[check] ROS topics:"
    ros2 topic list 2>/dev/null | grep -E "/gamma_arm|/joint_states" || true
    echo ""
    echo "[check] Gazebo topics:"
    gz topic -l 2>/dev/null | grep -E "/joint/gamma|/model/gamma_arm" || true
    echo ""
    echo "[check] DISPLAY=${DISPLAY:-<empty>}"
}

export PS1="\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "
cd "${WORKDIR:-/workspace/VisionFlow-PX4}" 2>/dev/null || cd ~

echo ""
echo "[OK] Entered PX4/Gazebo container: ${HOSTNAME}"
echo "[OK] ROS_DISTRO=${ROS_DISTRO:-unknown}"
echo "[OK] Workdir: $(pwd)"
echo "[OK] Helper rcfile: VISIONFLOW_HELPER_RC_V3"
echo ""
echo "Gamma web control:"
echo "  Web UI    : http://${WEB_HOST:-127.0.0.1}:${WEB_PORT:-9000}/index.html"
echo "  rosbridge : ws://${WEB_HOST:-127.0.0.1}:${ROSBRIDGE_PORT:-9090}"
echo "  GUI_ENABLE=${GUI_ENABLE:-1} LOG_ENABLE=${LOG_ENABLE:-0} AUTO_RESTART=${AUTO_RESTART:-1} GZ_KEEPALIVE=${GZ_KEEPALIVE:-0}"
echo ""
echo "Gamma web helper commands:"
echo "  webstart           # start with current GUI_ENABLE"
echo "  webstart_gui       # start embedded GUI"
echo "  webstart_headless  # start web backend only"
echo "  webstop            # stop web control"
echo "  weblog             # show tmux pane / logs"
echo "  webattach          # attach tmux session"
echo "  webps              # show related processes"
echo "  webcheck           # check processes and topics"
echo ""
RC

chmod +x "${TMPDIR_HOST}/gamma_web_runner.sh"
chmod 644 "${TMPDIR_HOST}/visionflow_interactive_bashrc"

# Copy helper scripts into the running container.
docker cp "${TMPDIR_HOST}/gamma_web_runner.sh" "${CONTAINER_NAME}:/tmp/gamma_web_runner.sh"
docker cp "${TMPDIR_HOST}/visionflow_interactive_bashrc" "${CONTAINER_NAME}:/tmp/visionflow_interactive_bashrc"
docker exec "${CONTAINER_NAME}" bash -c 'chmod +x /tmp/gamma_web_runner.sh && chmod 644 /tmp/visionflow_interactive_bashrc'

echo "[OK] Helper scripts installed in container."

# Start Gamma web control in container, before entering shell.
if [[ "${AUTO_WEB_CONTROL}" == "true" ]]; then
    docker exec \
        -e WORKDIR="${WORKDIR}" \
        -e WEB_SCRIPT="${WEB_SCRIPT}" \
        -e WEB_SESSION="${WEB_SESSION}" \
        -e WEB_HOST="${WEB_HOST}" \
        -e WEB_PORT="${WEB_PORT}" \
        -e ROSBRIDGE_PORT="${ROSBRIDGE_PORT}" \
        -e GUI_ENABLE="${GUI_ENABLE}" \
        -e LOG_ENABLE="${LOG_ENABLE}" \
        -e AUTO_RESTART="${AUTO_RESTART}" \
        -e GZ_KEEPALIVE="${GZ_KEEPALIVE}" \
        -e QTWEBENGINE_DISABLE_SANDBOX="${QTWEBENGINE_DISABLE_SANDBOX}" \
        -e QTWEBENGINE_CHROMIUM_FLAGS="${QTWEBENGINE_CHROMIUM_FLAGS}" \
        "${CONTAINER_NAME}" \
        bash -c '
set -eo pipefail
if ! command -v tmux >/dev/null 2>&1; then
    echo "[ERROR] tmux is not installed in container."
    exit 1
fi
if [ ! -x /tmp/gamma_web_runner.sh ]; then
    echo "[ERROR] /tmp/gamma_web_runner.sh not found or not executable."
    exit 1
fi
if tmux has-session -t "${WEB_SESSION}" 2>/dev/null; then
    echo "[OK] Gamma arm web control already running in tmux: ${WEB_SESSION}"
else
    tmux new-session -d -s "${WEB_SESSION}" env \
        WORKDIR="${WORKDIR}" \
        WEB_SCRIPT="${WEB_SCRIPT}" \
        WEB_SESSION="${WEB_SESSION}" \
        WEB_HOST="${WEB_HOST}" \
        WEB_PORT="${WEB_PORT}" \
        ROSBRIDGE_PORT="${ROSBRIDGE_PORT}" \
        GUI_ENABLE="${GUI_ENABLE}" \
        LOG_ENABLE="${LOG_ENABLE}" \
        AUTO_RESTART="${AUTO_RESTART}" \
        GZ_KEEPALIVE="${GZ_KEEPALIVE}" \
        QTWEBENGINE_DISABLE_SANDBOX="${QTWEBENGINE_DISABLE_SANDBOX}" \
        QTWEBENGINE_CHROMIUM_FLAGS="${QTWEBENGINE_CHROMIUM_FLAGS}" \
        /tmp/gamma_web_runner.sh
    sleep 1
    if tmux has-session -t "${WEB_SESSION}" 2>/dev/null; then
        echo "[OK] Gamma arm web control started in tmux: ${WEB_SESSION}"
    else
        echo "[ERROR] Gamma arm web control tmux session exited quickly."
        exit 1
    fi
fi

echo "[OK] Web Control UI : http://${WEB_HOST}:${WEB_PORT}/index.html"
echo "[OK] rosbridge     : ws://${WEB_HOST}:${ROSBRIDGE_PORT}"
echo "[OK] GUI_ENABLE    : ${GUI_ENABLE}"
echo "[OK] LOG_ENABLE    : ${LOG_ENABLE}"
echo "[OK] AUTO_RESTART  : ${AUTO_RESTART}"
echo "[OK] GZ_KEEPALIVE  : ${GZ_KEEPALIVE}"
'
fi

echo "[OK] Entering container interactive shell..."

# Enter the container with helper functions loaded.
docker exec -it \
    -e TERM=xterm-256color \
    -e WORKDIR="${WORKDIR}" \
    -e WEB_SCRIPT="${WEB_SCRIPT}" \
    -e WEB_SESSION="${WEB_SESSION}" \
    -e WEB_HOST="${WEB_HOST}" \
    -e WEB_PORT="${WEB_PORT}" \
    -e ROSBRIDGE_PORT="${ROSBRIDGE_PORT}" \
    -e GUI_ENABLE="${GUI_ENABLE}" \
    -e LOG_ENABLE="${LOG_ENABLE}" \
    -e AUTO_RESTART="${AUTO_RESTART}" \
    -e GZ_KEEPALIVE="${GZ_KEEPALIVE}" \
    -e QTWEBENGINE_DISABLE_SANDBOX="${QTWEBENGINE_DISABLE_SANDBOX}" \
    -e QTWEBENGINE_CHROMIUM_FLAGS="${QTWEBENGINE_CHROMIUM_FLAGS}" \
    "${CONTAINER_NAME}" \
    bash --rcfile /tmp/visionflow_interactive_bashrc -i
