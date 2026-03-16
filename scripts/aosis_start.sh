#!/bin/bash
xhost +local:root > /dev/null
docker rm -f isaac_ros_dev-aarch64-container 2>/dev/null
docker run -it --rm \
  --runtime nvidia \
  --network host \
  --privileged \
  --user $(id -u):$(id -g) \
  -v /mnt/nova_ssd/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /etc/passwd:/etc/passwd:ro \
  -v /etc/group:/etc/group:ro \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ISAAC_ROS_WS=/workspaces/isaac_ros-dev \
  -w /workspaces/isaac_ros-dev \
  --name isaac_ros_dev-aarch64-container \
  isaac_ros_dev-aarch64:latest \
  bash --init-file <(echo "source /opt/ros/humble/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null || true")
