#!/bin/bash

# Script to set environment variables

export ROS_DOMAIN_ID=6
export ROS_LOCALHOST_ONLY=0
# export ROS_IP=$(hostname -I | awk '{print $1}')
export ROS_IP=$(ip -4 -o addr show | grep 'wlxe' | grep -oP 'inet \K[\d.]+' | head -n1)
export ROS_IP=${ROS_IP:-$(ip -4 -o addr show | grep -E 'enp|eth' | grep -oP 'inet \K[\d.]+' | head -n1)}
export LIDAR_TYPE=x3

echo "----------"
echo -e "ROS_DOMAIN_ID:\033[32m$ROS_DOMAIN_ID\033[0m | ROS_IP:\033[32m$ROS_IP\033[0m"
echo -e "LIDAR_TYPE: \033[32m$LIDAR_TYPE\033[0m"
echo "----------"

source /opt/ros/humble/setup.bash

# Initialize submodules if not already done 
#   using rplidar_ros as submodule requires cloning with --recursive flag, if this was not 
#   done then the following command updates the submodule and make it available for use
git submodule update --init --recursive

source /X3_ROS2_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=/X3_ROS2_ws/install/x3_description/share:$IGN_GAZEBO_RESOURCE_PATH

# --- Lab DDS (added 2026-11-01): Fast DDS Discovery Server on the dev machine (192.168.0.104).
# Every terminal that sources this file uses it; LAB_DDS=off in a terminal selects default discovery.
if [ "${LAB_DDS:-server}" != "off" ]; then
  export ROS_DISCOVERY_SERVER=${LAB_DDS_SERVER:-192.168.0.104:11811}
  export ROS_SUPER_CLIENT=TRUE   # dev machine only: lets RViz and the ROS tools list every topic
else
  unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
fi
echo "DDS: ${ROS_DISCOVERY_SERVER:-default discovery}${ROS_SUPER_CLIENT:+ (super client)}"

exec "$@"