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
        IFS='|' read -r PROFILE_ID PROFILE_NAME PROFILE_WORLD PROFILE_TARGET PROFILE_EXTRA PROFILE_POSE <<< "${item}"
        PROFILE_POSE="${PROFILE_POSE:-}"

        if [ "${PROFILE_ID}" = "${DEFAULT_PROFILE:-}" ]; then
            echo "  ${index}) ${PROFILE_ID}  [default]"
        else
            echo "  ${index}) ${PROFILE_ID}"
        fi

        echo "     name   : ${PROFILE_NAME}"
        echo "     world  : ${PROFILE_WORLD}"
        echo "     target : ${PROFILE_TARGET}"
        echo "     extra  : ${PROFILE_EXTRA}"
        if [ -n "${PROFILE_POSE}" ]; then
            echo "     pose   : ${PROFILE_POSE}"
        else
            echo "     pose   : <airframe default>"
        fi
        echo ""

        index=$((index + 1))
    done
}

find_profile_by_id() {
    local wanted_id="$1"
    local item

    for item in "${SITL_PROFILES[@]}"; do
        IFS='|' read -r PROFILE_ID PROFILE_NAME PROFILE_WORLD PROFILE_TARGET PROFILE_EXTRA PROFILE_POSE <<< "${item}"
        PROFILE_POSE="${PROFILE_POSE:-}"

        if [ "${PROFILE_ID}" = "${wanted_id}" ]; then
            SELECTED_ID="${PROFILE_ID}"
            SELECTED_NAME="${PROFILE_NAME}"
            SELECTED_WORLD="${PROFILE_WORLD}"
            SELECTED_TARGET="${PROFILE_TARGET}"
            SELECTED_EXTRA="${PROFILE_EXTRA}"
            SELECTED_POSE="${PROFILE_POSE}"
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
        IFS='|' read -r PROFILE_ID PROFILE_NAME PROFILE_WORLD PROFILE_TARGET PROFILE_EXTRA PROFILE_POSE <<< "${item}"
        PROFILE_POSE="${PROFILE_POSE:-}"

        if [ "${current_index}" = "${wanted_index}" ]; then
            SELECTED_ID="${PROFILE_ID}"
            SELECTED_NAME="${PROFILE_NAME}"
            SELECTED_WORLD="${PROFILE_WORLD}"
            SELECTED_TARGET="${PROFILE_TARGET}"
            SELECTED_EXTRA="${PROFILE_EXTRA}"
            SELECTED_POSE="${PROFILE_POSE}"
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
if [ -n "${SELECTED_POSE}" ]; then
    echo "  pose   : ${SELECTED_POSE}"
else
    echo "  pose   : <airframe default>"
fi
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
    # Use legacy builder to avoid BuildKit proxy routing issues.
    # Explicitly unset proxy env vars so build doesn't try to route through unavailable proxy.
    unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ftp_proxy FTP_PROXY no_proxy NO_PROXY
    DOCKER_BUILDKIT=0 docker compose -f docker/compose.yaml build
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
    -e PX4_SELECTED_MODEL_POSE="${SELECTED_POSE}" \
    px4-humble-gz \
    bash -lc '
        set -e
        set -o pipefail

        source /opt/ros/humble/setup.bash
        cd /workspace/VisionFlow-PX4

        echo "[container] PX4_GZ_WORLD=${PX4_SELECTED_WORLD}"
        echo "[container] PX4 target=${PX4_SELECTED_TARGET}"
        echo "[container] EXTRA_CMAKE_ARGS=${PX4_SELECTED_EXTRA_CMAKE_ARGS}"
        if [ -n "${PX4_SELECTED_MODEL_POSE}" ]; then
            echo "[container] PX4_GZ_MODEL_POSE=${PX4_SELECTED_MODEL_POSE}"
        else
            echo "[container] PX4_GZ_MODEL_POSE=<airframe default>"
        fi

        run_px4_make() {
            if [ -n "${PX4_SELECTED_MODEL_POSE}" ]; then
                PX4_GZ_MODEL_POSE="${PX4_SELECTED_MODEL_POSE}" \
                PX4_GZ_WORLD="${PX4_SELECTED_WORLD}" \
                make px4_sitl "${PX4_SELECTED_TARGET}" \
                BUILD_BASE_DIR=build/docker \
                EXTRA_CMAKE_ARGS="${PX4_SELECTED_EXTRA_CMAKE_ARGS}"
            else
                PX4_GZ_WORLD="${PX4_SELECTED_WORLD}" \
                make px4_sitl "${PX4_SELECTED_TARGET}" \
                BUILD_BASE_DIR=build/docker \
                EXTRA_CMAKE_ARGS="${PX4_SELECTED_EXTRA_CMAKE_ARGS}"
            fi
        }

        run_px4_make_with_ucdr_watchdog() {
            local build_log="$1"
            local attempt="$2"
            local stall_timeout="${PX4_UCDR_HEADER_STALL_TIMEOUT:-5}"
            local check_interval="${PX4_UCDR_HEADER_WATCH_INTERVAL:-5}"
            local watchdog_status
            local build_status
            local watchdog_pid

            : > "${build_log}"
            watchdog_status="$(mktemp)"
            echo "0" > "${watchdog_status}"

            echo "[container] PX4 build attempt ${attempt}: uORB ucdr watchdog ${stall_timeout}s"

            set +e
            (
                local last_size="0"
                local current_size="0"
                local stable_since
                local now

                stable_since="$(date +%s)"

                while true; do
                    if grep -q "Startup script returned successfully" "${build_log}" 2>/dev/null; then
                        exit 0
                    fi

                    if grep -q "Generating uORB topic ucdr headers" "${build_log}" 2>/dev/null; then
                        current_size="$(wc -c < "${build_log}" 2>/dev/null || echo 0)"

                        if [ "${current_size}" != "${last_size}" ]; then
                            last_size="${current_size}"
                            stable_since="$(date +%s)"
                        else
                            now="$(date +%s)"

                            if [ $((now - stable_since)) -ge "${stall_timeout}" ]; then
                                echo "124" > "${watchdog_status}"
                                echo ""
                                echo "[container] PX4 build appears stuck after Generating uORB topic ucdr headers for ${stall_timeout}s."
                                echo "[container] Stop this build attempt and retry automatically..."

                                pkill -TERM -f "make px4_sitl" >/dev/null 2>&1 || true
                                sleep 5

                                pkill -KILL -f "make px4_sitl" >/dev/null 2>&1 || true
                                exit 124
                            fi
                        fi
                    else
                        stable_since="$(date +%s)"
                        last_size="$(wc -c < "${build_log}" 2>/dev/null || echo 0)"
                    fi

                    sleep "${check_interval}"
                done
            ) &
            watchdog_pid="$!"

            set -o pipefail
            run_px4_make 2>&1 | tee "${build_log}"
            build_status="$?"
            set +o pipefail

            kill "${watchdog_pid}" >/dev/null 2>&1 || true
            wait "${watchdog_pid}" >/dev/null 2>&1 || true

            if [ "$(cat "${watchdog_status}" 2>/dev/null || echo 0)" = "124" ]; then
                rm -f "${watchdog_status}"
                set -e
                return 124
            fi

            rm -f "${watchdog_status}"
            set -e
            return "${build_status}"
        }

        BUILD_LOG="$(mktemp)"
        UCDR_RETRIES="${PX4_UCDR_HEADER_RETRIES:-1}"
        BUILD_ATTEMPT=1
        STALE_CACHE_RETRIED="false"

        while true; do
            if run_px4_make_with_ucdr_watchdog "${BUILD_LOG}" "${BUILD_ATTEMPT}"; then
                break
            fi

            BUILD_STATUS="$?"

            if [ "${BUILD_STATUS}" -eq 124 ] && [ "${BUILD_ATTEMPT}" -le "${UCDR_RETRIES}" ]; then
                BUILD_ATTEMPT=$((BUILD_ATTEMPT + 1))
                echo ""
                echo "[container] Retry PX4 build after uORB ucdr header stall (${BUILD_ATTEMPT}/$((UCDR_RETRIES + 1)))..."
                sleep 2
                continue
            fi

            if [ "${STALE_CACHE_RETRIED}" = "false" ] && grep -Eq "CMakeCache.txt.*is different than the directory|needed by .* missing and no known rule to make it" "${BUILD_LOG}"; then
                echo ""
                echo "[container] Detected stale PX4 SITL build cache. Remove build/docker/px4_sitl_default and retry..."
                rm -rf /workspace/VisionFlow-PX4/build/docker/px4_sitl_default
                STALE_CACHE_RETRIED="true"
                BUILD_ATTEMPT=$((BUILD_ATTEMPT + 1))
                continue
            fi

            exit "${BUILD_STATUS}"
        done
    '
