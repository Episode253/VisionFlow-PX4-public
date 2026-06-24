#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

ORIGINAL_URDF="${MODEL_DIR}/swan_gamma_v2.urdf"
RVIZ_URDF="${SCRIPT_DIR}/swan_gamma_v2_rviz.urdf"
RVIZ_CONFIG="${SCRIPT_DIR}/swan_gamma_v2_rviz.rviz"
MESH_DIR="${MODEL_DIR}/meshes"

ROS_SETUP="/opt/ros/humble/setup.bash"

RSP_PID=""
JSP_PID=""
RVIZ_PID=""

cleanup() {
  echo
  echo "[INFO] Cleaning up RViz preview processes..."

  if [ -n "${RVIZ_PID}" ]; then
    kill "${RVIZ_PID}" 2>/dev/null || true
  fi

  if [ -n "${JSP_PID}" ]; then
    kill "${JSP_PID}" 2>/dev/null || true
  fi

  if [ -n "${RSP_PID}" ]; then
    kill "${RSP_PID}" 2>/dev/null || true
  fi

  sleep 0.3

  if [ -f "${RVIZ_URDF}" ]; then
    echo "[INFO] Removing generated temporary URDF: ${RVIZ_URDF}"
    rm -f "${RVIZ_URDF}"
  fi

  echo "[INFO] Done."
}
trap cleanup EXIT INT TERM

echo "[INFO] Script directory : ${SCRIPT_DIR}"
echo "[INFO] Model directory  : ${MODEL_DIR}"
echo "[INFO] Source URDF      : ${ORIGINAL_URDF}"
echo "[INFO] Temporary URDF   : ${RVIZ_URDF}"
echo "[INFO] Mesh directory   : ${MESH_DIR}"
echo "[INFO] RViz config      : ${RVIZ_CONFIG}"

if [ ! -f "${ROS_SETUP}" ]; then
  echo "[ERROR] ROS setup file not found: ${ROS_SETUP}"
  exit 1
fi

# Source ROS environment.
# Keep this compatible with ROS setup scripts that may reference optional variables.
set +u
source "${ROS_SETUP}"
set -u

if [ ! -f "${ORIGINAL_URDF}" ]; then
  echo "[ERROR] Source URDF not found: ${ORIGINAL_URDF}"
  exit 1
fi

if [ ! -d "${MESH_DIR}" ]; then
  echo "[ERROR] Mesh directory not found: ${MESH_DIR}"
  exit 1
fi

if [ ! -f "${RVIZ_CONFIG}" ]; then
  echo "[ERROR] RViz config file not found: ${RVIZ_CONFIG}"
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] ros2 command not found. Please check your ROS environment."
  exit 1
fi

if ! command -v rviz2 >/dev/null 2>&1; then
  echo "[ERROR] rviz2 command not found. Please install or source RViz2 correctly."
  exit 1
fi

echo "[INFO] Generating temporary RViz URDF..."

python3 - <<PY
from pathlib import Path
import re
import sys

original = Path("${ORIGINAL_URDF}")
rviz_urdf = Path("${RVIZ_URDF}")
mesh_dir = Path("${MESH_DIR}").resolve()

text = original.read_text()

# Convert relative mesh paths to absolute file:// URIs for RViz.
text = text.replace('filename="meshes/', f'filename="file://{mesh_dir}/')
text = text.replace('filename="./meshes/', f'filename="file://{mesh_dir}/')
text = text.replace('filename="../meshes/', f'filename="file://{mesh_dir}/')

# Convert Gazebo model:// mesh URIs to the local iscca_model/meshes layout.
# Example:
#   model://swan_uav_v2/meshes/base_link.STL
# becomes:
#   file:///.../iscca_model/meshes/swan_uav_v2/base_link.STL
def replace_model_uri(match):
    model_name = match.group(1)
    file_name = match.group(2)
    return f'filename="file://{mesh_dir}/{model_name}/{file_name}"'

text = re.sub(
    r'filename="model://([^/]+)/meshes/([^"]+)"',
    replace_model_uri,
    text
)

rviz_urdf.write_text(text)
print(f"[INFO] Generated temporary URDF: {rviz_urdf}")

# Check mesh existence.
missing = []
mesh_uris = re.findall(r'filename="([^"]+)"', text)

if not mesh_uris:
    print("[WARN] No mesh filename entries were found in the generated URDF.")

for uri in mesh_uris:
    if uri.startswith("file://"):
        path = Path(uri.replace("file://", ""))
        if not path.exists():
            missing.append(str(path))

if missing:
    print("[ERROR] Missing mesh files:")
    for path in missing:
        print(f"  [MISS] {path}")
    sys.exit(2)

print("[INFO] Mesh path check passed.")
PY

echo "[INFO] Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(cat "${RVIZ_URDF}")" &
RSP_PID=$!

sleep 1

echo "[INFO] Starting joint_state_publisher_gui..."
echo "[INFO] Use this GUI to preview movable joints in RViz. It does not control Gazebo or real hardware."
ros2 run joint_state_publisher_gui joint_state_publisher_gui \
  --ros-args \
  -p robot_description:="$(cat "${RVIZ_URDF}")" &
JSP_PID=$!

sleep 1

echo "[INFO] Starting RViz2 with config: ${RVIZ_CONFIG}"
rviz2 -d "${RVIZ_CONFIG}" &
RVIZ_PID=$!

echo "[INFO] RViz preview is running."
echo "[INFO] Close RViz or press Ctrl+C in this terminal to stop everything."

wait "${RVIZ_PID}"
