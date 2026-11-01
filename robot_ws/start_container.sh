#!/bin/bash
# Script to start the docker container

docker run -d --name ros2_humble --restart unless-stopped \
  -u ros \
  --net=host \
  --privileged \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e TZ=Canada/Atlantic \
  -v /dev:/dev --device-cgroup-rule='c *:* rmw' \
  -v /usr/local/cuda-12.6:/usr/local/cuda-12.6:ro \
  -v ~/X3_ROS2_ws/robot_ws/gpu_libs:/usr/lib/gpu_libs:ro \
  -v ~/X3_ROS2_ws:/X3_ROS2_ws \
  -v ~/.ssh/authorized_keys:/home/ros/.ssh/authorized_keys:ro \
  ros2_humble_img bash -c "sudo mkdir -p /run/sshd && sudo /usr/sbin/sshd && exec sleep infinity"