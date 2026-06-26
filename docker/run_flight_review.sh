#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")/.."

CONTAINER_NAME="${CONTAINER_NAME:-visionflow-flight-review}"
FR_DIR="${FR_DIR:-/workspace/VisionFlow-PX4/windshape_dev/flight_review}"
FR_PORT="${FR_PORT:-5006}"
COMPOSE_FILE="${COMPOSE_FILE:-docker/compose.yaml}"
SERVICE_NAME="${SERVICE_NAME:-px4-humble-gz}"
REBUILD="false"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build)
            REBUILD="true"
            shift
            ;;
        -h|--help)
            echo "Usage:"
            echo "  bash docker/run_flight_review.sh"
            echo "  bash docker/run_flight_review.sh --build"
            echo ""
            echo "Environment:"
            echo "  FR_PORT=5006"
            echo "  SERVICE_NAME=px4-humble-gz"
            echo "  CONTAINER_NAME=visionflow-flight-review"
            exit 0
            ;;
        *)
            echo "[ERROR] Unknown argument: $1"
            exit 1
            ;;
    esac
done

echo "[1/6] Check project structure..."

if [ ! -f "${COMPOSE_FILE}" ]; then
    echo "[ERROR] Docker compose file not found: ${COMPOSE_FILE}"
    echo "Please run this script from VisionFlow-PX4 root, or check docker/compose.yaml."
    exit 1
fi

if [ ! -d "windshape_dev/flight_review/app" ]; then
    echo "[ERROR] Flight Review app directory not found:"
    echo "        windshape_dev/flight_review/app"
    exit 1
fi

if ! docker compose -f "${COMPOSE_FILE}" config --services | grep -qx "${SERVICE_NAME}"; then
    echo "[ERROR] Docker compose service not found: ${SERVICE_NAME}"
    echo ""
    echo "Available services:"
    docker compose -f "${COMPOSE_FILE}" config --services | sed 's/^/  - /'
    echo ""
    echo "If your service name is different, run for example:"
    echo "  SERVICE_NAME=<actual-service-name> bash docker/run_flight_review.sh"
    exit 1
fi

echo "[2/6] Export user id..."
export USER_UID="$(id -u)"
export USER_GID="$(id -g)"

if [ "${REBUILD}" = "true" ]; then
    echo "[3/6] Build PX4 base docker image..."
    docker compose -f "${COMPOSE_FILE}" build
else
    echo "[3/6] Skip docker build. Use '--build' if Dockerfile changed."
fi

echo "[4/6] Prepare Flight Review local config..."
mkdir -p windshape_dev/flight_review/data/log_files
mkdir -p windshape_dev/flight_review/data/cache
mkdir -p windshape_dev/flight_review/data/cache/img

cat > windshape_dev/flight_review/app/config_user.ini <<CONFIG_EOF
[general]
domain_name = 127.0.0.1:${FR_PORT}
http_protocol = http
storage_path = ../data

[debug]
print_timing = 0
verbose_output = 1

[email]
smtpserver =
sender =
user_name =
password =

[email_notifications]
public_flightreport =
public_flightreport_bad =
CONFIG_EOF

echo "[5/6] Remove old Flight Review container if exists..."
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

echo "[6/6] Start Flight Review container..."

docker_run_cmd=(
    docker compose -f "${COMPOSE_FILE}" run
    --rm
    --name "${CONTAINER_NAME}"
    -e "FR_DIR=${FR_DIR}"
    -e "FR_PORT=${FR_PORT}"
    "${SERVICE_NAME}"
    bash -lc
)

docker_run_script='
set -eo pipefail

cd "${FR_DIR}"

if [ ! -d "app" ]; then
    echo "[ERROR] Flight Review app directory not found: ${FR_DIR}/app"
    exit 1
fi

echo "[container] Use image-level Flight Review virtual environment..."

if [ ! -x "/opt/flight_review_venv/bin/python" ]; then
    echo "[ERROR] /opt/flight_review_venv not found in Docker image."
    echo ""
    echo "This means the current PX4 Docker image was not built with Flight Review dependencies."
    echo ""
    echo "Please rebuild the image:"
    echo "  bash docker/run_flight_review.sh --build"
    echo ""
    echo "The script will not create venv or install pip packages at runtime."
    exit 1
fi

source /opt/flight_review_venv/bin/activate

if ! python -c "import bokeh, pyulog, tornado, scipy, pyfftw" >/dev/null 2>&1; then
    echo "[ERROR] Flight Review Python environment is incomplete."
    echo ""
    echo "Please rebuild the PX4 Docker image:"
    echo "  bash docker/run_flight_review.sh --build"
    echo ""
    echo "The script will not create venv or install pip packages at runtime."
    exit 1
fi

echo "[container] Flight Review Python packages already OK."

echo "[container] Initialize or upgrade database..."
cd app
./setup_db.py

echo "[container] Start Flight Review..."
echo "[OK] Upload page: http://127.0.0.1:${FR_PORT}/upload"
echo "[OK] If upload page does not auto-jump, manually open the printed /plot_app?log=xxx URL."
echo ""

exec ./serve.py \
    --address 0.0.0.0 \
    --port "${FR_PORT}" \
    --host "*" \
    --allow-websocket-origin "127.0.0.1:${FR_PORT}" \
    --allow-websocket-origin "localhost:${FR_PORT}"
'

"${docker_run_cmd[@]}" "${docker_run_script}"
