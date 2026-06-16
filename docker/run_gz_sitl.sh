#!/usr/bin/env bash
set -e

cd "$(dirname "$0")/.."

REBUILD="${1:-}"
CONTAINER_NAME="visionflow-px4-sitl"

cleanup() {
    echo ""
    echo "[cleanup] Stop PX4/Gazebo container..."
    docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
    docker compose -f docker/compose.yaml down --remove-orphans >/dev/null 2>&1 || true
}

trap cleanup INT TERM

echo "[1/6] Create docker cache directories..."
mkdir -p docker/cache/ccache
mkdir -p docker/cache/gz

echo "[2/6] Fix script permissions..."
chmod +x docker/entrypoint.sh || true
chmod +x docker/shell.sh || true
chmod +x docker/run_gz_sitl.sh || true

echo "[3/6] Export user id..."
export USER_UID="$(id -u)"
export USER_GID="$(id -g)"

echo "[4/6] Allow Docker to access X11 display..."
xhost +local:docker >/dev/null 2>&1 || true

if [ "${REBUILD}" = "--build" ]; then
    echo "[5/6] Build docker image..."
    docker compose -f docker/compose.yaml build
else
    echo "[5/6] Skip docker build. Use '--build' if Dockerfile changed."
fi

echo "[cleanup] Remove old container if exists..."
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

echo "[6/6] Run PX4 Gazebo SITL..."
docker compose -f docker/compose.yaml run \
    --rm \
    --name "${CONTAINER_NAME}" \
    px4-humble-gz \
    bash -lc '
        source /opt/ros/humble/setup.bash
        cd /workspace/VisionFlow-PX4

        PX4_GZ_WORLD=laboratory_landingbox \
        make px4_sitl gz_q940_ti_laboratory_landingbox \
        EXTRA_CMAKE_ARGS="-DENABLE_LOCKSTEP_SCHEDULER=ON"
    '

cleanup
