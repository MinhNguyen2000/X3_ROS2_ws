# Script to start the docker container

#!/bin/bash
docker run -d --name ros2_humble --restart unless-stopped \
  -u ros \
  --net=host \
  --privileged \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /dev:/dev --device-cgroup-rule='c *:* rmw' \
  -v ~/X3_ROS2_ws:/X3_ROS2_ws \
  ros2_humble_img sleep infinity