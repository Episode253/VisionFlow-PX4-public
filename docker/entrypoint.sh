#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

export CCACHE_DIR="${CCACHE_DIR:-/home/px4/.ccache}"
export QT_X11_NO_MITSHM=1

if command -v ccache >/dev/null 2>&1; then
    ccache --set-config=max_size=20G || true
fi

exec "$@"
