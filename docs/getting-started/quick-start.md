# Quick Verification

After completing installation and launch, verify that the environment is working properly through the following steps.

## 1. Verify Docker Environment

```bash
# Check if Docker is running
docker info

# Check if NVIDIA GPU is available
docker run --rm --gpus all nvidia/cuda:12.2-base nvidia-smi
```

## 2. Verify PX4 SITL

```bash
# Enter Docker container
bash docker/into_gz_sitl.sh

# Check PX4 version
px4 --version

# List available simulation targets
ninja -C build/px4_sitl_default -t targets | grep "^gz_"
```

## 3. Verify Gazebo Simulation

```bash
# Launch minimal simulation
gz sim -r Tools/simulation/gz/worlds/laboratory_landingbox.sdf
```

You should see a Gazebo window open, displaying the laboratory scene.

## 4. Verify ROS 2 Bridge

In another terminal:

```bash
# Start data bridge
bash windshape_dev/data_stream/gz_bridge/bridge_gz_ros.sh

# Check topics
ros2 topic list
```

You should see bridged topics such as `/gz/camera/...` and `/camera/...`.

## 5. Verify QGroundControl Connection

After launching QGroundControl, it should automatically connect to PX4 SITL (default UDP port 14550).

Check connection status:
- Vehicle status shows "Ready"
- Sensor data can be viewed
- Control commands can be sent

## 6. Verify Arm Control

```bash
# Launch Gamma arm web control
bash windshape_dev/arm_control/gamma_arm/gamma_arm_web_control.sh
```

Open the displayed address in a browser; you should see the arm control panel.

## Verification Checklist

- [ ] Docker image built successfully
- [ ] Gazebo scene loads correctly
- [ ] PX4 SITL runs without errors
- [ ] ROS 2 topic bridging works correctly
- [ ] QGroundControl can connect
- [ ] Arm control interface is accessible
