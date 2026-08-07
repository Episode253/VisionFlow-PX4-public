# Local Installation Guide

This guide walks you through setting up a complete native development
environment for VisionFlow-PX4 on **Ubuntu 22.04 (Jammy Jellyfish)**.
You will install the PX4 build toolchain, ROS 2 Humble, clone the repository,
and optionally configure WSL2 GPU acceleration for Gazebo simulation.

---

## 1. Install PX4 Development Environment

Follow the official PX4 Ubuntu setup to install the simulator and NuttX/Pixhawk
toolchains.

```bash
# Clone the main PX4 repository to get the setup script
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Run the setup script (install everything: simulation + NuttX)
bash Tools/setup/ubuntu.sh

# Restart your computer after the script completes
sudo reboot
```

> **Note:** The script is designed for clean Ubuntu LTS installations.
> If you already have some development tools installed, you may encounter
> conflicts. In that case, use the `--no-nuttx` or `--no-sim-tools` flags
> to skip specific components.

After rebooting, verify the toolchain:

```bash
# Check GCC (ARM cross-compiler)
arm-none-eabi-gcc --version

# Check Gazebo
gz sim --help
```

---

## 2. Install ROS 2 Humble

Install ROS 2 Humble Hawksbill (desktop-full variant) and the Gazebo-ROS bridge.

```bash
# 2a. Set up locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2b. Add ROS 2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 2c. Install ROS 2 Humble desktop
sudo apt update
sudo apt install -y ros-humble-desktop-full

# 2d. Install Gazebo-ROS bridge and related packages
sudo apt install -y ros-humble-ros-gz

# 2e. Source ROS 2 automatically
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc

# note: if you are running on WSL, you may need to start the daemon manually
ros2 daemon start
```

Verify the installation:

```bash
ros2 --version
ros2 topic list
```

---

## 3. Clone VisionFlow-PX4 and Initialise Submodules

```bash
# Clone the fork
git clone https://github.com/Renwang-Huang/VisionFlow-PX4.git
cd VisionFlow-PX4

# Initialise and update all submodules
git submodule update --init --recursive
```

> **Tip:** If you already cloned without `--recursive`, run the command above
> to pull all submodules.

Build the SITL target to confirm everything compiles:

```bash
make px4_sitl gz_q940_ti_gripper4_laboratory_landingbox
```

---

## 4. Install MicroXRCE-DDS Agent

The MicroXRCE-DDS Agent acts as a bridge between the PX4 uXRCE-DDS client
and the ROS 2 DDS network, enabling message passing between the flight
controller and ROS 2 nodes over UDP (or other transports).

```bash
# 4a. Clone the agent repository
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent

# 4b. Build from source
mkdir build && cd build
cmake ..
make

# 4c. Install system-wide
sudo make install
sudo ldconfig /usr/local/lib/

# 4d. Return to the project root
cd ../..
```

Verify the installation:

```bash
MicroXRCEAgent --help
```

The agent can then be started in a terminal (e.g. UDP on port 8888):

```bash
MicroXRCEAgent udp4 -p 8888
```

> **Note:** The agent must be running before launching PX4 SITL if you are
> using the uXRCE-DDS bridge (`uxrce_dds_client` module). The default PX4
> SITL configuration connects to `127.0.0.1:8888` over UDP.

---

## 5. (WSL2 Only) Force NVIDIA Driver for Gazebo

If you are running inside **WSL2 (Windows Subsystem for Linux)**, Gazebo
requires hardware-accelerated OpenGL through D3D12 translation.
Install the `kisak` Mesa PPA to get compatible Vulkan/OpenGL drivers.

```bash
# 5a. Add the Mesa PPA
sudo add-apt-repository -y ppa:kisak/turtle
sudo apt update

# 5b. Install Mesa Vulkan and OpenGL drivers
sudo apt install -y mesa-vulkan-drivers libgl1-mesa-dri

# 5c. Reinstall Mesa libraries to ensure correct versions
sudo apt install --reinstall -y \
  libgl1-mesa-dri \
  libglx-mesa0 \
  libgl1 \
  libglapi-mesa \
  mesa-vulkan-drivers

# 5d. Set environment variables to force D3D12 with NVIDIA adapter
echo 'export GALLIUM_DRIVER=d3d12' >> ~/.bashrc
echo 'export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA' >> ~/.bashrc
source ~/.bashrc
```

Verify that OpenGL is being rendered through the NVIDIA GPU:

```bash
glxinfo | grep "OpenGL renderer"
```

Expected output:

```
OpenGL renderer string: D3D12 (NVIDIA GeForce RTX ...)
```

If you see `llvmpipe` instead, the Mesa D3D12 driver is not active.
Double-check that WSL2 is up to date (`wsl --update` in Windows) and that
an NVIDIA driver with WSL support is installed on the Windows host.

---

## Next Steps

- [Launch a simulation (native)](native-launch.md)
- [Launch via Docker (recommended for first-time users)](docker-launch.md)
- [Quick verification checks](quick-start.md)
