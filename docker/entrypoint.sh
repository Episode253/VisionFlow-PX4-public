#!/usr/bin/env bash
set -e

source_ros_setup() {
    local setup_file="$1"
    [ -f "${setup_file}" ] || return 0

    local nounset_was_on=0
    case "$-" in
        *u*) nounset_was_on=1; set +u ;;
    esac

    export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
    # shellcheck disable=SC1090
    source "${setup_file}"

    if [ "${nounset_was_on}" = "1" ]; then
        set -u
    fi
}

source_ros_setup /opt/ros/humble/setup.bash

# Source PX4 and MAVROS workspaces if available.
PX4_INSTALL="/workspace/VisionFlow-PX4/install/setup.bash"
[ -f "${PX4_INSTALL}" ] && source_ros_setup "${PX4_INSTALL}"

MAVROS_INSTALL="/workspace/VisionFlow-PX4/thirdparty/mavros-humble/install/setup.bash"
[ -f "${MAVROS_INSTALL}" ] && source_ros_setup "${MAVROS_INSTALL}"

export CCACHE_DIR="${CCACHE_DIR:-/home/px4/.ccache}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export QTWEBENGINE_DISABLE_SANDBOX="${QTWEBENGINE_DISABLE_SANDBOX:-1}"
export QTWEBENGINE_CHROMIUM_FLAGS="${QTWEBENGINE_CHROMIUM_FLAGS:---no-sandbox --disable-gpu --disable-dev-shm-usage}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-px4}"

mkdir -p "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}" 2>/dev/null || true

# Use custom Gazebo Sim GUI client config if available.
CUSTOM_GZ_GUI_CONFIG="/workspace/VisionFlow-PX4/windshape_dev/parameter/gazebo_sim_example.config"

if [ -f "${CUSTOM_GZ_GUI_CONFIG}" ]; then
    echo "[entrypoint] Use custom Gazebo GUI config: ${CUSTOM_GZ_GUI_CONFIG}"

    mkdir -p /home/px4/.gz/sim/8
    cp "${CUSTOM_GZ_GUI_CONFIG}" /home/px4/.gz/sim/8/gui.config

    mkdir -p /home/px4/.gz/sim
    cp "${CUSTOM_GZ_GUI_CONFIG}" /home/px4/.gz/sim/gui.config
else
    echo "[entrypoint] Custom Gazebo GUI config not found: ${CUSTOM_GZ_GUI_CONFIG}"
fi

if command -v ccache >/dev/null 2>&1; then
    ccache --set-config=max_size=20G || true
fi

exec "$@"
