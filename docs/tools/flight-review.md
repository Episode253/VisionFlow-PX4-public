# Flight Log Review

This page introduces the VisionFlow-PX4 flight log review tool, which provides a web-based interface for analyzing PX4 `.ulog` flight logs, visualizing key performance metrics, and detecting anomalies.

## Overview

Flight Review is a web application based on [PX4 Flight Review](http://logs.px4.io/) that runs inside the Docker container. It parses `.ulog` files and renders interactive plots for EKF status, controller outputs, sensor data, and more.

## Quick Start

```bash
# Launch Flight Review (uses existing Docker image)
bash docker/run_flight_review.sh

# Rebuild the Docker image first (if Dockerfile changed)
bash docker/run_flight_review.sh --build
```

After launch, open **http://127.0.0.1:5006/upload** in your browser.

## Uploading Log Files

1. Click **"Choose File"** on the upload page
2. Select a `.ulog` file from `~/PX4/` inside the container (or mount from host)
3. Click **"Upload"** — the page will automatically redirect to the analysis view
4. Use the top navigation bar to switch between:
   - **Overview** — flight timeline and key events
   - **3D** — 3D trajectory visualization (requires `--3d` flag)
   - **PID Analysis** — controller output and error plots

## Log File Location

Flight logs are saved inside the container at:

```
/home/px4/PX4/
```

Since the project directory is mounted at `/workspace/VisionFlow-PX4/`, host-side logs can be found at:

```bash
# On the host (adjust path if you use a different workspace root)
<PX4_ROOT>/build/docker/px4_sitl_default/tmp.px4_sitl_default/rootfs/PX4/log/
```

Or copy logs out of the container:

```bash
docker cp visionflow-px4-sitl:/home/px4/PX4/log_<date>_<time>.ulg ~/Downloads/
```

## Environment Variables

| Variable | Default | Description |
|------|--------|------|
| `FR_PORT` | `5006` | Web UI port |
| `CONTAINER_NAME` | `visionflow-flight-review` | Container name |
| `SERVICE_NAME` | `px4-humble-gz` | Docker Compose service |

## Related Pages

- [Tools Overview](index.md)
- [Docker Workflow](docker-workflow.md)
- [Data Streaming & Bridge](data-streaming.md)
