#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

export CCACHE_DIR="${CCACHE_DIR:-/home/px4/.ccache}"
export QT_X11_NO_MITSHM=1

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
