#!/bin/bash

# Script to set the environment variables (no need to pass arguments in the start container script)

export ROS_DOMAIN_ID=6
export ROS_LOCALHOST_ONLY=0
export ROS_IP=$(hostname -I | awk '{print $1}')
export LIDAR_TYPE=X3

# --- Lab DDS (added 2026-11-01): Fast DDS Discovery Server on the dev machine (192.168.0.104).
# Every terminal that sources this file uses it; LAB_DDS=off in a terminal selects default discovery.
if [ "${LAB_DDS:-server}" != "off" ]; then
  export ROS_DISCOVERY_SERVER=${LAB_DDS_SERVER:-192.168.0.104:11811}
else
  unset ROS_DISCOVERY_SERVER
fi

echo "--------------------------------------------------------"
echo -e "ROS_DOMAIN_ID: \033[32m$ROS_DOMAIN_ID\033[0m | ROS_IP: \033[32m$ROS_IP\033[0m"
echo -e "LIDAR_TYPE : \033[32m$LIDAR_TYPE\033[0m"
echo -e "DDS: \033[32m${ROS_DISCOVERY_SERVER:-default discovery}${ROS_SUPER_CLIENT:+ (super client)}\033[0m"
echo "--------------------------------------------------------"

# Source ROS so it's available in exec'd shells
source /opt/ros/humble/setup.bash

exec "$@"