#!/bin/bash
# Gazebo acceptance check of the deployment config (P10). Runs inside the x3_ros2_ws image, headless, on this worktree.
# usage: run_check.sh <config a|b|c|smoke|frozen|a40> [extra batch_multigoal.py args]
set -e
CFG=$1; shift
WT=$(cd "$(dirname "$0")/../../.." && pwd)   # the worktree this script is in
IMG=$(docker images --format '{{.Repository}}:{{.Tag}}' | grep '^vsc-x3_ros2_ws-.*-uid:' | head -1)   # the devcontainer image, no running container needed
[ -n "$IMG" ] || { echo 'devcontainer image not found'; exit 1; }
COMMON="-p max_lin_vel:=0.2 -p max_angular_vel:=0.3 -p normalized_actions:=true -p goal_timeout:=90.0"
case $CFG in
  a|smoke) PARAMS="$COMMON -p antennae_mask_dist:=0.0 -p rear_mask_deg:=45.8 -p rear_mask_fill:=train_mean -p range_clip:=11.10" ;;
  frozen)  PARAMS="$COMMON -p antennae_mask_dist:=0.0 -p rear_mask_deg:=40.0 -p rear_mask_fill:=train_mean -p range_clip:=11.10" ;;   # = robot.launch.py
  a40)     PARAMS="$COMMON -p antennae_mask_dist:=0.0 -p rear_mask_deg:=40.0 -p rear_mask_fill:=train_mean -p range_clip:=11.10" ;;   # config (a) with the deployed 40 degree mask: full 48 cells
  b)       PARAMS="$COMMON -p antennae_mask_dist:=0.0 -p rear_mask_deg:=45.8 -p rear_mask_fill:=range_max -p range_clip:=11.10" ;;
  c)       PARAMS="$COMMON" ;;
  *) echo "unknown config $CFG"; exit 1 ;;
esac
ARGS=""
for p in $PARAMS; do [ "$p" = "-p" ] || ARGS="$ARGS --planner-param $p"; done
GITDIR=$(git -C "$WT" rev-parse --git-common-dir)
docker run --rm --name p1check-$CFG --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all,graphics -e GZ_HEADLESS=1 \
  -v $WT:/X3_ROS2_ws -v $GITDIR:$GITDIR:ro -w /X3_ROS2_ws "$IMG" \
  bash -lc "git config --global --add safe.directory '*' >/dev/null 2>&1; python3 scripts/batch_multigoal.py --skip-apf --ros-domain-id 67 --manifest scripts/sim2real/check/manifest_$CFG.csv $ARGS $*"
