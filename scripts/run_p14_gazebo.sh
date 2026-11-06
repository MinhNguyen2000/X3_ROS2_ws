#!/bin/bash
# P14: Gazebo multi-goal batch for the A1 policies and the same-code controls (protocol: personal_hub notes/a1_gazebo_protocol_20261106.md).
# Runs inside the dev container relaxed_shaw; resumable (batch_multigoal.py skips jobs already ok in the manifest).
# Usage on the host: scripts/run_p14_gazebo.sh [a1|apf|paper|all] [extra batch_multigoal.py args, e.g. --limit 1]
set -u
STAGE=${1:-all}; shift || true
WT=/X3_ROS2_ws/.claude/worktrees/p14-gazebo
A1="TD3_01101_1500 TD3_01102_1500 TD3_01103_1500 TD3_01101_1300 TD3_01103_1400"
SC="world_1/path1 world_1/path2 world_1/path3 world_2/path1 world_3/path1 world_4/path1 world_5/path1"
BASE="GZ_HEADLESS=1 python3 scripts/batch_multigoal.py --manifest scripts/batch_runs/manifest_p14.csv --scenarios $SC"
run() { echo "$(date +%F_%T) stage $1"; docker exec relaxed_shaw bash -c "cd $WT && $2"; }
if [ "$STAGE" = a1 ] || [ "$STAGE" = all ]; then
    args=""; for m in $A1; do args+=" --extra-model $m --only-model $m"; done
    run a1 "$BASE --skip-apf $args --planner-param control_rate:=10.0 $*"
fi
if [ "$STAGE" = apf ] || [ "$STAGE" = all ]; then
    run apf "$BASE --only-model APF --apf-reps 5 $*"
fi
if [ "$STAGE" = paper ] || [ "$STAGE" = all ]; then
    args=""; for i in $(seq 396 405); do args+=" --only-model TD3_00${i}_1000"; done
    run paper "$BASE --skip-apf $args $*"
fi
echo "$(date +%F_%T) done"
