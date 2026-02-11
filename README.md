AOSIS: Autonomous Optical Servoing & Inspection Station

1. Project Overview

AOSIS (Autonomous Optical Servoing & Inspection Station) is a high-fidelity robotic system designed for micro-electronics assembly and inspection. 
It leverages a dual-camera "Search and Magnify" hierarchy to provide autonomous tracking of high-heat effectors (soldering wands) while maintaining a stabilized 4K optical field.

Developed on the NVIDIA Jetson Orin AGX, the system utilizes Isaac ROS NITROS for zero-copy GPU memory transport, ensuring ultra-low latency visual servoing.

Key Features

LWIR Spectral Tracking: Utilizes the RealSense D435i Infrared stream to isolate thermal signatures (>300°C).

4K Micro-Inspection: Targeted UC70 4K industrial optics for high-resolution digital magnification.

Deterministic Actuation: C++ based motor driver interfacing with a Teensy 4.1 and TMC5160 drivers.

Environmental Watchdog: Integrated VOC and thermal monitoring via the ROS2 Diagnostics framework.

2. Hardware Stack

Compute: NVIDIA Jetson Orin AGX (JetPack 6.1)

Vision: Intel RealSense D435i (Depth + IR)

Vision: UC70 4K Industrial Camera

MCU: Teensy 4.1 (600MHz ARM Cortex-M7)

Power: MeanWell LRS-350-15 (15V/23.2A) + 24V Stepper Rail

3. Installation & Setup

This project is designed to run within the isaac_ros-dev Docker environment.

Prerequisites

Ubuntu 22.04 LTS

NVIDIA JetPack 6.1

Docker with NVIDIA Isaac ROS
