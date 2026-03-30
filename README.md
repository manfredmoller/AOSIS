# AOSIS — Autonomous Optical Servoing and Inspection Station

A real-time robotic inspection system built on the NVIDIA Jetson Orin AGX using Isaac ROS NITROS for zero-copy GPU memory transport and TensorRT for edge AI inference. AOSIS autonomously tracks a soldering iron and maintains a calibrated standoff distance using a dual-camera perception hierarchy and a three-axis ball-screw gantry.

---

## System Overview

AOSIS uses a layered perception architecture to track high-heat effectors during active soldering work:

- **Intel RealSense D435i** — wide-field IR stereo depth camera for 3D wand tip localization and spatial awareness
- **Mokose UC70 4K** — close-range inspection camera for high-resolution surface observation

Perception runs on a three-tier inference stack:

| Tier | Model | Hardware | Rate | Role |
|------|-------|----------|------|------|
| Fast | NanoOWL + Kalman filter | DLA | ~60 Hz | Real-time tracking |
| Mid | NanoOWL re-detection | GPU | 20–30 Hz | Drift correction |
| Slow | Quantized LLaVA | GPU | 0.5–2 Hz | Semantic scene understanding |

A three-axis motion stage (NEMA 23 steppers, TMC5160T Pro drivers, ball-screw linear rails) is controlled by a Teensy 4.1 running Micro-ROS, closing the servo loop from camera detection to physical gantry motion.

---

## Hardware

| Component | Details |
|-----------|---------|
| Compute | NVIDIA Jetson Orin AGX 64GB, JetPack 6, MAXN mode |
| Tracking camera | Intel RealSense D435i, firmware 5.13.0.50 |
| Inspection camera | Mokose UC70 4K |
| Motion controller | Teensy 4.1 (Micro-ROS over Ethernet) |
| Stepper drivers | BigTreeTech TMC5160T Pro x3 (StallGuard4 sensorless homing) |
| Motors | NEMA 23 stepper motors, 1.8° step angle, 3A/phase |
| Motion stage | Three-axis ball-screw linear rails |

---

## Software Stack

| Component | Details |
|-----------|---------|
| Container base | `nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble` |
| ROS 2 | Humble |
| Isaac ROS | NITROS zero-copy GPU transport |
| Micro-ROS | Humble, Ethernet UDP transport (192.168.10.0/24) |
| Teensy firmware | PlatformIO + micro_ros_platformio |
| Inference | TensorRT, ONNX, DLA |
| realsense-ros | 4.55.1 |
| librealsense | 2.56.4 |

---

## Repository Structure
```
AOSIS/
├── firmware/                   # Teensy 4.1 Micro-ROS firmware (PlatformIO)
│   ├── src/main.cpp            # Motion controller node
│   └── platformio.ini          # Build config (Humble, serial transport)
├── src/
│   ├── aosis/                  # Primary ROS 2 package
│   │   ├── config/             # Sensor parameters and RViz configs
│   │   └── launch/             # Launch files
│   └── aosis_bringup/          # Robot description and state publisher
│       ├── urdf/aosis.urdf     # Three-axis gantry URDF
│       └── launch/             # Bringup launch files
├── docker/
│   └── Dockerfile.user         # Isaac ROS container customization layer
├── scripts/
│   ├── aosis_start.sh          # Launch Isaac ROS development container
│   └── setup.sh                # One-shot workspace environment setup
├── .env.template               # NGC credentials template (never committed)
└── .gitignore
```

---

## ROS Topics

| Topic | Type | Direction | Rate |
|-------|------|-----------|------|
| `/joint_states` | `sensor_msgs/JointState` | Teensy → Orin | 50 Hz |
| `/aosis/cmd_vel` | `std_msgs/Float32MultiArray` | Orin → Teensy | on demand |

---

## Setup and Reproduction

### Prerequisites

- NVIDIA Jetson Orin AGX (or compatible Jetson) with JetPack 6
- NVIDIA NGC account — [ngc.nvidia.com](https://ngc.nvidia.com)
- Docker configured with NGC credentials
- Intel RealSense D435i connected to USB 3.1 or faster

### 1. Clone the repository
```bash
git clone https://github.com/manfredmoller/AOSIS.git
cd AOSIS
```

### 2. Configure NGC credentials
```bash
cp .env.template .env
# Add your NGC API key to .env
docker login nvcr.io
# Username: $oauthtoken
# Password: <your NGC API key>
```

### 3. Clone Isaac ROS dependencies
```bash
mkdir -p ~/workspaces/isaac_ros-dev/src && cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/IntelRealSense/realsense-ros.git -b 4.55.1
cp -r /path/to/AOSIS/src/* .
```

### 4. Launch the Isaac ROS container
```bash
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts
echo "CONFIG_IMAGE_KEY=ros2_humble.realsense" > .isaac_ros_common-config
./run_dev.sh
```

### 5. Build the workspace
```bash
cd /workspaces/isaac_ros-dev
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 6. Flash Teensy firmware
```bash
cd /path/to/AOSIS/firmware
pio run --target upload
```

### 7. Start the Micro-ROS agent
```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

### 8. Launch RealSense
```bash
ros2 launch aosis realsense_bringup.launch.py
```

---

## Build Status

| Subsystem | Status |
|-----------|--------|
| Isaac ROS container — JetPack 6 | ✅ Verified |
| RealSense D435i — IR stereo + depth at 30fps | ✅ Verified |
| Teensy 4.1 Micro-ROS node — 50Hz joint states over Ethernet UDP | ✅ Verified |
| Micro-ROS Ethernet transport — 50Hz UDP, 192.168.10.0/24 | ✅ Verified |
| TMC5160T Pro wiring and sensorless homing | 🔄 In progress |
| URDF — three-axis gantry tf2 tree | 🔄 In progress |
| RealSense D435i — Isaac ROS NITROS integration | ⬜ Planned |
| NanoOWL soldering iron tracking pipeline | ⬜ Planned |
| Mokose UC70 — NITROS dual-camera pipeline | ⬜ Planned |
| Closed-loop gantry servo control | ⬜ Planned |
| LLaVA semantic scene understanding | ⬜ Planned |

---

## Academic Context

AOSIS is a capstone project for ETI 4480 — Applied Robotics, Spring 2026. The system architecture is designed to support future AI capability expansion without structural changes, using a tiered inference model that separates real-time control from semantic reasoning.
