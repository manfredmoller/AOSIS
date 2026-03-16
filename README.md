# AOSIS — Autonomous Optical Servoing & Inspection Station

A real-time robotic inspection system built on the NVIDIA Jetson Orin AGX using Isaac ROS NITROS for zero-copy GPU memory transport and TensorRT for edge AI inference.

## System Overview

AOSIS tracks high-heat effectors (soldering wands) using a dual-camera "Search and Magnify" hierarchy:
- **Intel RealSense D435i** — wide-field IR stereo + depth for 3D wand tip localization
- **Mokose UC70** — 4K optical zoom camera for stabilized close-range inspection

A three-axis motion stage (NEMA 23 steppers + TMC5160T Pro drivers) is controlled via a Teensy 4.1 running micro-ROS, closing the servo loop from camera detection to physical motion.

## Hardware

| Component | Details |
|---|---|
| Compute | NVIDIA Jetson Orin AGX 64GB, JetPack 6, MAXN mode |
| Tracking Camera | Intel RealSense D435i, firmware 5.13.0.50 |
| Inspection Camera | Mokose UC70 4K |
| Motion Controller | Teensy 4.1 (micro-ROS) |
| Stepper Drivers | TMC5160T Pro x3 |
| Motors | NEMA 23 stepper motors |

## Software Stack

| Component | Version |
|---|---|
| Isaac ROS | NITROS, Isaac ROS Humble |
| ROS 2 | Humble |
| realsense-ros | 4.55.1 |
| librealsense | 2.56.4 |
| Container base | nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble |

## Repository Structure
```
AOSIS/
├── src/
│   ├── aosis/                  # Primary ROS 2 package
│   │   ├── config/             # Sensor and pipeline parameters
│   │   └── launch/             # Launch files
│   └── microscope_bringup/     # Robot URDF and state publisher
├── docker/
│   └── Dockerfile.user         # Container customization layer
├── scripts/
│   └── setup.sh                # One-shot container environment setup
├── .env.template               # NGC credentials template
└── .gitignore
```

## Setup & Reproduction

### Prerequisites
- Jetson Orin AGX (or compatible Jetson) with JetPack 6
- NVIDIA NGC account (free) — [ngc.nvidia.com](https://ngc.nvidia.com)
- Docker with NGC credentials configured
- Intel RealSense D435i on USB 3.1+ port

### 1. Clone the repo
```bash
git clone https://github.com/manfredmoller/AOSIS.git
cd AOSIS
```

### 2. Configure NGC credentials
```bash
cp .env.template .env
# Edit .env and add your NGC API key
docker login nvcr.io
# Username: $oauthtoken
# Password: <your NGC API key>
```

### 3. Set up Isaac ROS workspace
```bash
mkdir -p ~/workspaces/isaac_ros-dev/src
cp -r src/* ~/workspaces/isaac_ros-dev/src/
```

### 4. Clone Isaac ROS dependencies
```bash
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/IntelRealSense/realsense-ros.git -b 4.51.1
```

### 5. Configure and launch container
```bash
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts
echo "CONFIG_IMAGE_KEY=ros2_humble.realsense" > .isaac_ros_common-config
./run_dev.sh
```

### 6. Build workspace
```bash
cd /workspaces/isaac_ros-dev
colcon build --packages-select realsense2_camera_msgs realsense2_camera realsense2_description \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 7. Launch RealSense
```bash
ros2 launch realsense2_camera rs_launch.py \
  --ros-args --params-file src/aosis/config/realsense_aosis.yaml
```

## Current Status

- [x] Isaac ROS container bringup on JetPack 6
- [x] RealSense D435i — IR stereo + depth streams verified at 30fps
- [ ] NITROS graph wiring to TensorRT inference pipeline
- [ ] Soldering wand tip detection model
- [ ] micro-ROS Teensy 4.1 node
- [ ] Closed-loop servo control via TMC5160T Pro
- [ ] Mokose UC70 integration

## Author

Manfred
