#!/bin/bash
xhost +local:docker > /dev/null
docker rm -f isaac_ros_dev-aarch64-container 2>/dev/null
docker run -it --rm \
  --runtime nvidia \
  --network host \
  --privileged \
  --user admin \
  -v /mnt/nova_ssd/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
  -v /home/manfred/.ssh:/home/admin/.ssh:ro \
  -v /mnt/nova_ssd/aosis_pip_local:/home/admin/.local \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ISAAC_ROS_WS=/workspaces/isaac_ros-dev \
  -w /workspaces/isaac_ros-dev \
  --name isaac_ros_dev-aarch64-container \
  isaac_ros_dev-aarch64:latest \
  bash --init-file /home/admin/.bashrc
