#!/bin/bash
# Functional test of run_robot_schedule.py without a robot: Gazebo's empty world (headless, in the devcontainer image) plays the
# robot, the runner performs the first N runs of stretch 1 with --auto (no operator). Checks the policy launch, the model switch,
# the sequence server and client, and the saved JSON. Needs this worktree built with colcon (see scripts/sim2real/README.md).
# usage: test_runner_in_gazebo.sh [N runs, default 3]
set -e
N=${1:-3}
WT=$(cd "$(dirname "$0")/../.." && pwd)
IMG=$(docker images --format '{{.Repository}}:{{.Tag}}' | grep '^vsc-x3_ros2_ws-.*-uid:' | head -1)
[ -n "$IMG" ] || { echo 'devcontainer image not found'; exit 1; }
GITDIR=$(git -C "$WT" rev-parse --git-common-dir)
docker run --rm --name runner-gazebo-test --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all,graphics -e GZ_HEADLESS=1 \
  -v "$WT":/X3_ROS2_ws -v "$GITDIR":"$GITDIR":ro -w /X3_ROS2_ws "$IMG" bash -lc "
  git config --global --add safe.directory '*' >/dev/null 2>&1
  export ROS_DOMAIN_ID=68 ROS_LOCALHOST_ONLY=0
  source /opt/ros/humble/setup.bash; source install/setup.bash
  ros2 launch x3_description gazebo.launch.py world:=empty_world.sdf > /tmp/gazebo.log 2>&1 &
  for i in \$(seq 1 60); do ros2 topic list 2>/dev/null | grep -q /agent0/scan && timeout 10 ros2 topic echo --once /agent0/odom >/dev/null 2>&1 && break; sleep 2; done
  echo 'sim ready'; sleep 5
  python3 scripts/sim2real/run_robot_schedule.py --auto --no-bringup --stretch 1 --max-runs $N
  echo '--- manifest'; cat scripts/sim2real/robot/manifest_robot.csv
  pkill -9 -f 'ign gazebo' || true
"
