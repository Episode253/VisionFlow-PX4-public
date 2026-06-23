#!/usr/bin/env bash
set -e

cd "$(dirname "$0")/.."

REBUILD="false"
REQUESTED_PROFILE=""
LIST_ONLY="false"

CONTAINER_NAME="visionflow-px4-sitl"
CONFIG_FILE="docker/gz_sitl_profiles.conf"
CLEANED_UP="false"

cleanup() {
    if [ "${CLEANED_UP}" = "true" ]; then
        return 0
    fi
    CLEANED_UP="true"

    echo ""
    echo "[cleanup] Stop PX4/Gazebo container..."
    docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
    docker compose -f docker/compose.yaml down --remove-orphans >/dev/null 2>&1 || true
}

handle_interrupt() {
    trap - EXIT INT TERM
    cleanup
    exit 130
}

trap cleanup EXIT
trap handle_interrupt INT TERM

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build)
            REBUILD="true"
            shift
            ;;
        --profile)
            REQUESTED_PROFILE="${2:-}"
            shift 2
            ;;
        --profile=*)
            REQUESTED_PROFILE="${1#*=}"
            shift
            ;;
        --list)
            LIST_ONLY="true"
            shift
            ;;
        -h|--help)
            echo "Usage:"
            echo "  bash docker/run_gz_sitl.sh --profile \"Entity 1\""
            echo "  bash docker/run_gz_sitl.sh --build --profile \"Entity 1\""
            echo "  bash docker/run_gz_sitl.sh --list"
            exit 0
            ;;
        *)
            echo "[error] Unknown argument: $1"
            exit 1
            ;;
    esac
done

if [ ! -f "${CONFIG_FILE}" ]; then
    echo "[error] SITL config file not found: ${CONFIG_FILE}"
    echo "Please create docker/gz_sitl_profiles.conf first."
    exit 1
fi

# shellcheck source=/dev/null
source "${CONFIG_FILE}"

if [ "${#SITL_PROFILES[@]}" -eq 0 ]; then
    echo "[error] No SITL profiles found in ${CONFIG_FILE}"
    exit 1
fi

print_profiles() {
    echo ""
    echo "Available SITL profiles:"
    echo ""

    local index=1
    local item
    for item in "${SITL_PROFILES[@]}"; do
        IFS='|' read -r PROFILE_ID PROFILE_NAME PROFILE_WORLD PROFILE_TARGET PROFILE_EXTRA <<< "${item}"

        if [ "${PROFILE_ID}" = "${DEFAULT_PROFILE:-}" ]; then
            echo "  ${index}) ${PROFILE_ID}  [default]"
        else
            echo "  ${index}) ${PROFILE_ID}"
        fi

        echo "     name   : ${PROFILE_NAME}"
        echo "     world  : ${PROFILE_WORLD}"
        echo "     target : ${PROFILE_TARGET}"
        echo "     extra  : ${PROFILE_EXTRA}"
        echo ""

        index=$((index + 1))
    done
}

find_profile_by_id() {
    local wanted_id="$1"
    local item

    for item in "${SITL_PROFILES[@]}"; do
        IFS='|' read -r PROFILE_ID PROFILE_NAME PROFILE_WORLD PROFILE_TARGET PROFILE_EXTRA <<< "${item}"

        if [ "${PROFILE_ID}" = "${wanted_id}" ]; then
            SELECTED_ID="${PROFILE_ID}"
            SELECTED_NAME="${PROFILE_NAME}"
            SELECTED_WORLD="${PROFILE_WORLD}"
            SELECTED_TARGET="${PROFILE_TARGET}"
            SELECTED_EXTRA="${PROFILE_EXTRA}"
            return 0
        fi
    done

    return 1
}

find_profile_by_index() {
    local wanted_index="$1"
    local current_index=1
    local item

    for item in "${SITL_PROFILES[@]}"; do
        IFS='|' read -r PROFILE_ID PROFILE_NAME PROFILE_WORLD PROFILE_TARGET PROFILE_EXTRA <<< "${item}"

        if [ "${current_index}" = "${wanted_index}" ]; then
            SELECTED_ID="${PROFILE_ID}"
            SELECTED_NAME="${PROFILE_NAME}"
            SELECTED_WORLD="${PROFILE_WORLD}"
            SELECTED_TARGET="${PROFILE_TARGET}"
            SELECTED_EXTRA="${PROFILE_EXTRA}"
            return 0
        fi

        current_index=$((current_index + 1))
    done

    return 1
}

if [ "${LIST_ONLY}" = "true" ]; then
    print_profiles
    exit 0
fi

if [ -n "${REQUESTED_PROFILE}" ]; then
    if ! find_profile_by_id "${REQUESTED_PROFILE}"; then
        echo "[error] Unknown SITL profile: ${REQUESTED_PROFILE}"
        print_profiles
        exit 1
    fi
else
    print_profiles

    if [ -t 0 ]; then
        echo "Select SITL profile."
        echo "Input number or profile id. Press Enter to use default: ${DEFAULT_PROFILE}"
        if ! read -r -p "> " USER_SELECTION; then
            echo ""
            exit 130
        fi
    else
        USER_SELECTION=""
    fi

    if [ -z "${USER_SELECTION}" ]; then
        if ! find_profile_by_id "${DEFAULT_PROFILE}"; then
            echo "[warn] Default profile '${DEFAULT_PROFILE}' not found. Use first profile."
            find_profile_by_index 1
        fi
    elif [[ "${USER_SELECTION}" =~ ^[0-9]+$ ]]; then
        if ! find_profile_by_index "${USER_SELECTION}"; then
            echo "[error] Invalid profile number: ${USER_SELECTION}"
            exit 1
        fi
    else
        if ! find_profile_by_id "${USER_SELECTION}"; then
            echo "[error] Unknown SITL profile id: ${USER_SELECTION}"
            exit 1
        fi
    fi
fi

echo ""
echo "[selected] ${SELECTED_ID}"
echo "  name   : ${SELECTED_NAME}"
echo "  world  : ${SELECTED_WORLD}"
echo "  target : ${SELECTED_TARGET}"
echo "  extra  : ${SELECTED_EXTRA}"
echo ""

echo "[1/6] Create docker cache directories..."
mkdir -p docker/cache/ccache
mkdir -p docker/cache/gz

echo "[2/6] Fix script permissions..."
chmod +x docker/entrypoint.sh || true
chmod +x docker/run_gz_sitl.sh || true

echo "[3/6] Export user id..."
export USER_UID="$(id -u)"
export USER_GID="$(id -g)"

echo "[4/6] Allow Docker to access X11 display..."
xhost +local:docker >/dev/null 2>&1 || true

if [ "${REBUILD}" = "true" ]; then
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
    -e PX4_SELECTED_WORLD="${SELECTED_WORLD}" \
    -e PX4_SELECTED_TARGET="${SELECTED_TARGET}" \
    -e PX4_SELECTED_EXTRA_CMAKE_ARGS="${SELECTED_EXTRA}" \
    px4-humble-gz \
    bash -lc '
        set -e
        set -o pipefail

        source /opt/ros/humble/setup.bash
        cd /workspace/VisionFlow-PX4

        echo "[container] PX4_GZ_WORLD=${PX4_SELECTED_WORLD}"
        echo "[container] PX4 target=${PX4_SELECTED_TARGET}"
        echo "[container] EXTRA_CMAKE_ARGS=${PX4_SELECTED_EXTRA_CMAKE_ARGS}"

        build_gamma_arm_control_plugin() {
            local plugin_dir="/workspace/VisionFlow-PX4/windshape_dev/plugins/gamma_arm_control"
            local build_dir="${plugin_dir}/build"
            local cache_file="${build_dir}/CMakeCache.txt"

            if [ ! -f "${plugin_dir}/CMakeLists.txt" ]; then
                echo "[container] Gamma arm control plugin source not found: ${plugin_dir}"
                return 0
            fi

            if [ -f "${cache_file}" ] && ! grep -q "^CMAKE_HOME_DIRECTORY:INTERNAL=${plugin_dir}$" "${cache_file}"; then
                echo "[container] Detected stale Gamma arm plugin CMake cache. Remove build/ and reconfigure..."
                rm -rf "${build_dir}"
            fi

            echo "[container] Build and install Gamma arm control Gazebo plugin..."
            cmake -S "${plugin_dir}" -B "${build_dir}"
            cmake --build "${build_dir}"
            sudo cmake --install "${build_dir}"
        }

        run_px4_make() {
            PX4_GZ_WORLD="${PX4_SELECTED_WORLD}" \
            make px4_sitl "${PX4_SELECTED_TARGET}" \
            EXTRA_CMAKE_ARGS="${PX4_SELECTED_EXTRA_CMAKE_ARGS}"
        }

        build_gamma_arm_control_plugin

        sudo ldconfig
        export GZ_SIM_SYSTEM_PLUGIN_PATH="/usr/local/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
        export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="/usr/local/lib:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}"
        export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
        export GAMMA_URDF_PATH="/workspace/VisionFlow-PX4/Tools/simulation/gz/models/gamma_arm/gamma_arm.urdf"

        BUILD_LOG="$(mktemp)"
        if ! run_px4_make 2>&1 | tee "${BUILD_LOG}"; then
            if grep -Eq "CMakeCache.txt.*is different than the directory|needed by .* missing and no known rule to make it" "${BUILD_LOG}"; then
                echo ""
                echo "[container] Detected stale PX4 SITL build cache. Remove build/px4_sitl_default and retry..."
                rm -rf /workspace/VisionFlow-PX4/build/px4_sitl_default
                run_px4_make
            else
                exit 1
            fi
        fi
    '
