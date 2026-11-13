#!/bin/bash
# Gazebo repeats of the 8 deployed policies (P2), headless in the devcontainer image, on this worktree.
# usage: run_repeats.sh <lab|std> [extra batch_multigoal.py args]
#   lab: world_lab/path1 and path2, frozen deployment config (robot.launch.py / drl_policy.launch.py values, obstacle stop 0.205 m),
#        spawn at the lab odom origin. Default 10 repeats.   Manifest: manifest_world_lab.csv
#   std: the six standard multigoal scenarios with the SAME config as scripts/batch_runs/manifest.csv (policy_node defaults,
#        no planner parameters). Default 5 repeats.         Manifest: manifest_standard.csv
# Resumable: rerun the same command and finished jobs are skipped.
set -e
MODE=$1; shift
WT=$(cd "$(dirname "$0")/../../.." && pwd)
IMG=$(docker images --format '{{.Repository}}:{{.Tag}}' | grep '^vsc-x3_ros2_ws-.*-uid:' | head -1)
[ -n "$IMG" ] || { echo 'devcontainer image not found'; exit 1; }
MODELS="TD3_00400_1000 TD3_00402_1000 TD3_00399_900 TD3_00401_900 TD3_00390_1000 TD3_00395_1000 TD3_00391_800 TD3_00388_900"
SEL=""; for m in $MODELS; do SEL="$SEL --only-model $m"; done
case $MODE in
  lab) REPS=${REPS:-10}
       ARGS="--scenarios world_lab/path1 world_lab/path2 --gazebo-arg x_offset:=-1.6775 --gazebo-arg y_offset:=-0.7625"
       for p in max_lin_vel:=0.2 max_angular_vel:=0.3 normalized_actions:=true goal_timeout:=90.0 antennae_mask_dist:=0.0 \
                rear_mask_deg:=40.0 rear_mask_fill:=train_mean range_clip:=11.10 obstacle_tolerance:=0.205; do
         ARGS="$ARGS --planner-param $p"; done
       MAN=scripts/sim2real/gazebo_repeats/manifest_world_lab.csv ;;
  std) REPS=${REPS:-5}; ARGS=""; MAN=scripts/sim2real/gazebo_repeats/manifest_standard.csv ;;
  *) echo "unknown mode $MODE"; exit 1 ;;
esac
GITDIR=$(git -C "$WT" rev-parse --git-common-dir)
docker run --rm --name p2repeats-$MODE --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all,graphics -e GZ_HEADLESS=1 \
  -v $WT:/X3_ROS2_ws -v $GITDIR:$GITDIR:ro -w /X3_ROS2_ws "$IMG" \
  bash -lc "unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT; git config --global --add safe.directory '*' >/dev/null 2>&1; nice python3 scripts/batch_multigoal.py --skip-apf --ros-domain-id 68 --reps $REPS --kill-stragglers --manifest $MAN $SEL $ARGS $*"
