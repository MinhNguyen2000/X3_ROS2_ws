#!/bin/bash
# Script to start the docker container

# The container user's GitHub key lives on the host in ~/.ssh_container, so it survives `docker rm`.
# Set it up once (the .pub file must be on GitHub, and the folder owned by uid 1000):
#   mkdir -m 700 ~/.ssh_container && cp <key> ~/.ssh_container/id_ed25519 && cp <key>.pub ~/.ssh_container/id_ed25519.pub && touch ~/.ssh_container/known_hosts && chmod 600 ~/.ssh_container/id_ed25519 ~/.ssh_container/known_hosts
# All three files must exist, otherwise docker would create root-owned directories at those paths.
KEY_DIR=~/.ssh_container
KEY_MOUNTS=()
if [ -f "$KEY_DIR/id_ed25519" ] && [ -f "$KEY_DIR/id_ed25519.pub" ] && [ -f "$KEY_DIR/known_hosts" ]; then
  KEY_MOUNTS=(
    -v "$KEY_DIR/id_ed25519:/home/ros/.ssh/id_ed25519:ro"
    -v "$KEY_DIR/id_ed25519.pub:/home/ros/.ssh/id_ed25519.pub:ro"
    -v "$KEY_DIR/known_hosts:/home/ros/.ssh/known_hosts"
  )
else
  echo "Warning: $KEY_DIR is missing id_ed25519, id_ed25519.pub or known_hosts: the container starts without a GitHub key" >&2
fi

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
  "${KEY_MOUNTS[@]}" \
  ros2_humble_img bash -c "sudo mkdir -p /run/sshd && sudo /usr/sbin/sshd && exec sleep infinity"