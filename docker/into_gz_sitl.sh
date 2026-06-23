#!/usr/bin/env bash
set -e

CONTAINER_NAME="${CONTAINER_NAME:-visionflow-px4-sitl}"
WORKDIR="${WORKDIR:-/workspace/VisionFlow-PX4}"

AUTO_WEB_CONTROL="${AUTO_WEB_CONTROL:-true}"
WEB_SESSION="${WEB_SESSION:-gamma_web}"
WEB_HOST="${WEB_HOST:-127.0.0.1}"
WEB_PORT="${WEB_PORT:-9000}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
WEB_SCRIPT="${WORKDIR}/windshape_dev/arm_control/gamma_arm/gamma_arm_web_control.sh"

usage() {
    echo "Usage:"
    echo "  bash docker/into_gz_sitl.sh"
    echo "  bash docker/into_gz_sitl.sh --no-web"
    echo "  bash docker/into_gz_sitl.sh --web"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-web)
            AUTO_WEB_CONTROL="false"
            shift
            ;;
        --web)
            AUTO_WEB_CONTROL="true"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[ERROR] Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
    echo "[ERROR] Container is not running: ${CONTAINER_NAME}"
    echo ""
    echo "Please start PX4/Gazebo SITL first:"
    echo "  bash docker/run_gz_sitl.sh --profile \"Entity 3\""
    echo ""
    echo "Current running containers:"
    docker ps --format '  {{.Names}}\t{{.Status}}'
    exit 1
fi

# 1) Start Gamma web control layer in the container, without occupying this terminal.
if [[ "${AUTO_WEB_CONTROL}" == "true" ]]; then
    docker exec \
        -e WORKDIR="${WORKDIR}" \
        -e WEB_SCRIPT="${WEB_SCRIPT}" \
        -e WEB_SESSION="${WEB_SESSION}" \
        -e WEB_HOST="${WEB_HOST}" \
        -e WEB_PORT="${WEB_PORT}" \
        -e ROSBRIDGE_PORT="${ROSBRIDGE_PORT}" \
        "${CONTAINER_NAME}" \
        bash -lc '
set -e

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

cd "${WORKDIR}" 2>/dev/null || cd /

if [ ! -f "${WEB_SCRIPT}" ]; then
    echo "[WARN] Gamma arm web control script not found:"
    echo "       ${WEB_SCRIPT}"
    exit 0
fi

if command -v tmux >/dev/null 2>&1; then
    if tmux has-session -t "${WEB_SESSION}" 2>/dev/null; then
        echo "[OK] Gamma arm web control already running in tmux: ${WEB_SESSION}"
    else
        tmux new-session -d -s "${WEB_SESSION}" "bash -lc \"source /opt/ros/humble/setup.bash 2>/dev/null || true; cd ${WORKDIR}; WEB_HOST=${WEB_HOST} WEB_PORT=${WEB_PORT} ROSBRIDGE_PORT=${ROSBRIDGE_PORT} bash ${WEB_SCRIPT}; exec bash\""
        sleep 1

        if tmux has-session -t "${WEB_SESSION}" 2>/dev/null; then
            echo "[OK] Gamma arm web control started in tmux: ${WEB_SESSION}"
        else
            echo "[WARN] Gamma arm web control tmux session exited quickly."
            echo "[WARN] Run inside container to inspect:"
            echo "       tail -n 120 ${WORKDIR}/windshape_dev/arm_control/gamma_arm/log/gamma_arm_bridge.log"
        fi
    fi
else
    if pgrep -f "gamma_arm_web_control.sh" >/dev/null 2>&1; then
        echo "[OK] Gamma arm web control already seems running."
    else
        mkdir -p "${WORKDIR}/windshape_dev/arm_control/gamma_arm/log"
        nohup bash -lc "source /opt/ros/humble/setup.bash 2>/dev/null || true; cd ${WORKDIR}; WEB_HOST=${WEB_HOST} WEB_PORT=${WEB_PORT} ROSBRIDGE_PORT=${ROSBRIDGE_PORT} bash ${WEB_SCRIPT}" \
            > "${WORKDIR}/windshape_dev/arm_control/gamma_arm/log/gamma_web_nohup.log" 2>&1 &
        echo "[OK] Gamma arm web control started by nohup."
    fi
fi

echo "[OK] Open: http://${WEB_HOST}:${WEB_PORT}/index.html"
'
fi

# 2) Enter the container as an interactive shell.
docker exec -it \
    -e TERM=xterm-256color \
    -e WORKDIR="${WORKDIR}" \
    -e WEB_SESSION="${WEB_SESSION}" \
    -e WEB_HOST="${WEB_HOST}" \
    -e WEB_PORT="${WEB_PORT}" \
    -e ROSBRIDGE_PORT="${ROSBRIDGE_PORT}" \
    "${CONTAINER_NAME}" \
    bash -lc '
cat > /tmp/visionflow_interactive_bashrc <<RC
# ===== VisionFlow PX4/Gazebo interactive shell =====

export TERM=xterm-256color
export FORCE_COLOR=1
export CLICOLOR=1
export LS_COLORS="di=01;34:ln=01;36:so=01;35:pi=33:ex=01;32:bd=33;01:cd=33;01:su=37;41:sg=30;43:tw=30;42:ow=34;42"

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f /workspace/VisionFlow-PX4/install/setup.bash ]; then
    source /workspace/VisionFlow-PX4/install/setup.bash
fi

alias ls="ls --color=auto"
alias ll="ls -alF --color=auto"
alias la="ls -A --color=auto"
alias l="ls -CF --color=auto"
alias grep="grep --color=auto"
alias croot="cd /workspace/VisionFlow-PX4"
alias gzlist="gz topic -l"
alias gzps="ps aux | grep -E \"gz|px4|ros|bridge\" | grep -v grep"

webstart() {
    cd /workspace/VisionFlow-PX4 || return 1

    if command -v tmux >/dev/null 2>&1; then
        if tmux has-session -t "\${WEB_SESSION:-gamma_web}" 2>/dev/null; then
            echo "[OK] Gamma web already running: \${WEB_SESSION:-gamma_web}"
        else
            tmux new-session -d -s "\${WEB_SESSION:-gamma_web}" "bash -lc \"source /opt/ros/humble/setup.bash 2>/dev/null || true; cd /workspace/VisionFlow-PX4; WEB_HOST=\${WEB_HOST:-127.0.0.1} WEB_PORT=\${WEB_PORT:-9000} ROSBRIDGE_PORT=\${ROSBRIDGE_PORT:-9090} bash windshape_dev/arm_control/gamma_arm/gamma_arm_web_control.sh; exec bash\""
            echo "[OK] Gamma web started: \${WEB_SESSION:-gamma_web}"
        fi
    else
        nohup bash -lc "source /opt/ros/humble/setup.bash 2>/dev/null || true; cd /workspace/VisionFlow-PX4; WEB_HOST=\${WEB_HOST:-127.0.0.1} WEB_PORT=\${WEB_PORT:-9000} ROSBRIDGE_PORT=\${ROSBRIDGE_PORT:-9090} bash windshape_dev/arm_control/gamma_arm/gamma_arm_web_control.sh" \
            > /workspace/VisionFlow-PX4/windshape_dev/arm_control/gamma_arm/log/gamma_web_nohup.log 2>&1 &
        echo "[OK] Gamma web started by nohup."
    fi

    echo "[OK] Open: http://\${WEB_HOST:-127.0.0.1}:\${WEB_PORT:-9000}/index.html"
}

webstop() {
    if command -v tmux >/dev/null 2>&1 && tmux has-session -t "\${WEB_SESSION:-gamma_web}" 2>/dev/null; then
        tmux kill-session -t "\${WEB_SESSION:-gamma_web}"
        echo "[OK] Stopped tmux session: \${WEB_SESSION:-gamma_web}"
    fi
    pkill -f "gamma_arm_web_control.sh" 2>/dev/null || true
}

weblog() {
    if command -v tmux >/dev/null 2>&1 && tmux has-session -t "\${WEB_SESSION:-gamma_web}" 2>/dev/null; then
        tmux capture-pane -pt "\${WEB_SESSION:-gamma_web}" -S -180
    else
        tail -n 120 /workspace/VisionFlow-PX4/windshape_dev/arm_control/gamma_arm/log/gamma_arm_bridge.log 2>/dev/null || true
        tail -n 80 /workspace/VisionFlow-PX4/windshape_dev/arm_control/gamma_arm/log/gamma_web_nohup.log 2>/dev/null || true
    fi
}

webattach() {
    tmux attach -t "\${WEB_SESSION:-gamma_web}"
}

webps() {
    ps aux | grep -E "rosbridge|ros_gz_bridge|parameter_bridge|http.server|gamma_arm_web_control" | grep -v grep
}

export PS1="\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "

cd /workspace/VisionFlow-PX4 2>/dev/null || cd ~

echo ""
echo "[OK] Entered PX4/Gazebo container: visionflow-px4-sitl"
echo "[OK] ROS_DISTRO=\${ROS_DISTRO:-unknown}"
echo "[OK] Workdir: \$(pwd)"
echo ""
echo "Gamma web control:"
echo "  http://\${WEB_HOST:-127.0.0.1}:\${WEB_PORT:-9000}/index.html"
echo ""
echo "Useful commands:"
echo "  gz topic -l"
echo "  ros2 topic list"
echo "  gz topic -t /joint/gamma/1/position_cmd -m gz.msgs.Double -p \"data: 0.5\""
echo ""
echo "Gamma web helper commands:"
echo "  webstart   # start web control"
echo "  webstop    # stop web control"
echo "  weblog     # show web / bridge logs"
echo "  webattach  # attach tmux session"
echo "  webps      # show related processes"
echo ""
RC

exec bash --rcfile /tmp/visionflow_interactive_bashrc -i
'
