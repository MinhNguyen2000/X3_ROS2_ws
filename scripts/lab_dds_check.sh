#!/bin/bash
# Quick check of the lab ROS 2 discovery-server setup. Run on the dev machine (host or dev container).
# Prints whether the server is up and how many robot topics (/agent0/*, not counting the Pi's /agent0/odom_camera) and Pi camera (/camera/*) topics a super client can see.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=6 ROS_DISCOVERY_SERVER=${LAB_DDS_SERVER:-192.168.0.104:11811} ROS_SUPER_CLIENT=TRUE
up=$(ss -ulnp 2>/dev/null | grep -c ':11811\b')
echo "discovery server on this machine (UDP 11811): $([ "$up" -ge 1 ] && echo up || echo DOWN)"
topics=$(timeout 25 ros2 topic list --no-daemon 2>/dev/null)
robot=$(echo "$topics" | grep '^/agent0/' | grep -vc 'odom_camera'); cam=$(echo "$topics" | grep -c '^/camera/')
echo "robot topics (/agent0/*): $robot    Pi camera topics (/camera/*): $cam"
[ "$up" -lt 1 ] && { echo "RESULT: FAIL, start the server (see the notes) or use LAB_DDS=off"; exit 1; }
[ "$cam" -lt 1 ] && echo "WARN: no Pi camera topics; is the tracker running with the setting on (new shell shows 'DDS: 192.168.0.104:11811')?"
[ "$robot" -lt 1 ] && echo "WARN: no robot topics; is the robot bringup running from a new shell on the robot?"
[ "$cam" -ge 1 ] && [ "$robot" -ge 1 ] && echo "RESULT: OK" || echo "RESULT: partial"
