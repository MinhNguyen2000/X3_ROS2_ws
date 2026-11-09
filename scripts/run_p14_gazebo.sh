#!/bin/bash
# P14: Gazebo multi-goal batch for the A1 policies and the same-code controls (protocol: personal_hub notes/a1_gazebo_protocol_20261106.md).
# Runs inside the dev container relaxed_shaw; resumable (batch_multigoal.py skips jobs already ok in the manifest).
# Usage on the host: scripts/run_p14_gazebo.sh [a1|apf|paper|all|a2|a2dir|a2delta|paperdelta] [extra batch_multigoal.py args, e.g. --limit 1]
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
# A2 batch (personal_hub notes/a2_eval_protocol_20261109.md; not part of "all"): stage a2 runs a2dir, a2delta and
# paperdelta in that order into manifest_a2.csv. Final and Best of each seed, Best = Final listed once. A2_DELTA_MODELS
# overrides the delta list (the pilot smoke was A2_DELTA_MODELS=TD3_01301_1500 into manifest_a2_smoke.csv).
A2DIR="TD3_01201_1500 TD3_01202_1500 TD3_01203_1500 TD3_01204_1500 TD3_01205_1500 TD3_01206_1500 TD3_01207_1500
 TD3_01208_1500 TD3_01209_1500 TD3_01210_1500 TD3_01201_1100 TD3_01202_1000 TD3_01203_1100 TD3_01205_1000 TD3_01206_1100
 TD3_01209_1400 TD3_01210_1400"
A2DELTA=${A2_DELTA_MODELS:-"TD3_01301_1500 TD3_01302_1500 TD3_01303_1500 TD3_01304_1500 TD3_01305_1500 TD3_01306_1500
 TD3_01307_1500 TD3_01308_1500 TD3_01309_1500 TD3_01310_1500 TD3_01301_1200 TD3_01302_900 TD3_01303_1100 TD3_01304_1300
 TD3_01305_1000 TD3_01307_1400 TD3_01308_1300"}
A2_DELTA_BOUND=${A2_DELTA_BOUND:-0.1}
A2BASE="${BASE/manifest_p14.csv/manifest_a2.csv} --skip-apf"
if [ "$STAGE" = a2dir ] || [ "$STAGE" = a2 ]; then
    args=""; for m in $A2DIR; do args+=" --extra-model $m --only-model $m"; done
    run a2dir "$A2BASE $args --planner-param control_rate:=10.0 --planner-param action_mode:=direct $*"
fi
if [ "$STAGE" = a2delta ] || [ "$STAGE" = a2 ]; then
    b=$A2_DELTA_BOUND; args=""; for m in $A2DELTA; do args+=" --extra-model $m --only-model $m"; done
    for x in control_rate:=10.0 action_mode:=delta delta_act_linear:=$b delta_act_angular:=$b \
             delta_act_scale_linear:=1.0 delta_act_scale_angular:=1.0; do args+=" --planner-param $x"; done
    run a2delta "$A2BASE $args $*"
fi
if [ "$STAGE" = paperdelta ] || [ "$STAGE" = a2 ]; then    # paper TD3_delta Final at node defaults
    args=""; for i in $(seq 386 395); do args+=" --only-model TD3_00${i}_1000"; done
    run paperdelta "$A2BASE $args $*"
fi
if [ "$STAGE" = apf ] || [ "$STAGE" = all ]; then
    run apf "$BASE --only-model APF --apf-reps 5 $*"
fi
if [ "$STAGE" = paper ] || [ "$STAGE" = all ]; then
    args=""; for i in $(seq 396 405); do args+=" --only-model TD3_00${i}_1000"; done
    run paper "$BASE --skip-apf $args $*"
fi
echo "$(date +%F_%T) done"
