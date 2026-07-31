# Data Streaming & Bridge

This page covers VisionFlow-PX4's data streaming and bridge tools, including Gazebo-to-ROS2 topic bridging (`bridge_gz_ros.sh`), camera image streaming (`camera_stream.sh`), and local position data plotting (`odom_plotter.py`).

## Gazebo ↔ ROS2 Topic Bridge

The bridge script maps Gazebo simulation topics to ROS 2 topics (and vice versa), enabling ROS nodes to interact with the simulation.

### Launch the Bridge

```bash
bash windshape_dev/image_stream/bridge_gz_ros.sh
```

### What Gets Bridged

The script configures bidirectional mappings for:

| Direction | Gazebo Topic | ROS 2 Topic |
|-----------|-------------|-------------|
| Gazebo → ROS | `/model/<drone>/odometry` | `/mavros/local_position/odom` |
| Gazebo → ROS | `/model/<drone>/joint_state` | `/joint_states` |
| Gazebo → ROS | `/model/<gripper>/joint_state` | `/gripper3/joint_state` |
| ROS → Gazebo | `/joint/gripper3/position_cmd` | `/model/<gripper>/position_cmd` |
| ROS → Gazebo | Camera/image topics | Gazebo camera sensors |

The world and model names are configurable at the top of the script:

```bash
WORLD="/world/laboratory_landingbox/model/q940_ti_0"
```

### Verify the Bridge

```bash
# Inside the container, after launching the bridge
ros2 topic list | grep -E "camera|odometry|joint"
gz topic -l | grep -E "camera|model"
```

---

## Camera Stream Monitor

The camera stream script launches a web-based monitor that displays live camera feeds from the Gazebo simulation.

### Launch

```bash
bash windshape_dev/image_stream/camera_stream.sh
```

### Access

- Web monitor: **http://127.0.0.1:8000/index.html**
- Video stream: **http://127.0.0.1:8080**

### Environment Variables

| Variable | Default | Description |
|------|--------|------|
| `WEB_PORT` | `8000` | Web monitor port |
| `VIDEO_PORT` | `8080` | Video stream port |
| `WEB_HOST` | `127.0.0.1` | Bind address |
| `MONITOR_PAGE` | `index.html` | Monitor page filename |

---

## Local Position Plotter

The odom plotter visualizes the drone's local position trajectory in real time using ROS 2 odometry data.

### Launch

```bash
# Inside the Docker container
ros2 run <package> odom_plotter.py
# or directly:
python3 windshape_dev/data_plotting/local_position/odom_plotter.py
```

### Features

- Real-time X/Y/Z trajectory plot (dark theme)
- Moving statistics panel (mean, std deviation over a sliding window)
- Subscribes to `/mavros/local_position/odom`
- Uses `deque` with configurable history length (`PLOT_LEN=2000`, `CALC_LEN=300`)

### Environment

Requires a display (`DISPLAY` set, X11 forwarding active). Run inside the Docker container with GUI enabled:

```bash
bash docker/into_gz_sitl.sh
python3 windshape_dev/data_plotting/local_position/odom_plotter.py
```

---

## Related Pages

- [Tools Overview](index.md)
- [Docker Workflow](docker-workflow.md)
- [Flight Log Review](flight-review.md)
- [Communication Stack](../architecture/communication-stack.md)
