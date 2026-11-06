#!/bin/bash
# P14: Gazebo goal-to-goal batches (trial_orchestrator, world_1, 139 goals) for the A1 policies and the same-code
# controls (protocol: personal_hub notes/a1_g2g_protocol_20261106.md). One batch per unit: a fresh headless
# Gazebo, one trial_orchestrator run, teardown. Resumable: a unit whose folder already holds a complete CSV
# (139 trials) is skipped; an incomplete folder is moved to trial_results_p14/aborted/ (never deleted) and rerun.
# Runs inside the dev container; on the host: docker exec relaxed_shaw bash <worktree>/scripts/run_p14_g2g.sh [stage]
# Stages: a1_final, apf, a1_best, paper, all (in that order), or a single unit name (e.g. TD3_01101_1500, APF_run3).
WT=$(cd "$(dirname "$0")/.." && pwd)
OUT_ROOT=$WT/trial_results_p14
N_GOALS=139
STAGE=${1:-all}
source /opt/ros/humble/setup.bash && source "$WT/install/setup.bash"   # ROS setup files fail under set -u
set -u
export ROS_DOMAIN_ID=66 ROS_LOCALHOST_ONLY=0 LIDAR_TYPE=x3 GZ_HEADLESS=1
export IGN_GAZEBO_RESOURCE_PATH=$WT/install/x3_description/share:${IGN_GAZEBO_RESOURCE_PATH:-}

A1_FINAL="TD3_01101_1500 TD3_01102_1500 TD3_01103_1500"
APF="APF_run1 APF_run2 APF_run3 APF_run4 APF_run5"
A1_BEST="TD3_01101_1300 TD3_01103_1400"      # TD3_01102_1500 is both Final and Best
PAPER=$(for i in $(seq 396 405); do printf 'TD3_00%s_1000 ' "$i"; done)
case $STAGE in
    a1_final) UNITS=$A1_FINAL ;; apf) UNITS=$APF ;; a1_best) UNITS=$A1_BEST ;; paper) UNITS=$PAPER ;;
    all) UNITS="$A1_FINAL $APF $A1_BEST $PAPER" ;; *) UNITS=$STAGE ;;
esac

complete() {  # a CSV in $1 with N_GOALS data rows
    for f in "$1"/world_1_*.csv; do
        [ -f "$f" ] && [ "$(($(wc -l < "$f") - 1))" -eq "$N_GOALS" ] && return 0
    done
    return 1
}

teardown() {
    [ -n "${GZ_PID:-}" ] && kill -INT -- "-$GZ_PID" 2>/dev/null
    sleep 5
    pkill -f '[t]rial_orchestrator'; pkill -f '[p]olicy_node'; pkill -f '[a]pf_node'
    pkill -f '[r]f2o_laser_odometry'; pkill -f '[e]kf_node'; pkill -f '[c]ovariance_filter'
    pkill -f '[g]z sim'; pkill -f '[i]gn gazebo'; sleep 3
    pkill -9 -f '[g]z sim'; pkill -9 -f '[i]gn gazebo'; sleep 1
    GZ_PID=
}

mkdir -p "$OUT_ROOT/aborted"
for unit in $UNITS; do
    out=$OUT_ROOT/$unit
    if complete "$out"; then echo "$(date +%F_%T) $unit already complete, skipped"; continue; fi
    if [ -d "$out" ]; then mv "$out" "$OUT_ROOT/aborted/${unit}_$(date +%Y%m%d_%H%M%S)"; fi
    mkdir -p "$out"
    case $unit in
        APF_*) args="--planner APF" ;;
        TD3_011*) args="--planner DRL --model $unit --planner-param control_rate:=10.0" ;;
        TD3_*) args="--planner DRL --model $unit" ;;
    esac
    teardown
    echo "$(date +%F_%T) $unit start ($args)"
    setsid ros2 launch x3_description gazebo.launch.py world:=world_1.sdf launch_ekf_odom:=false \
        > "$out/gazebo.log" 2>&1 &
    GZ_PID=$!
    sleep 60
    # shellcheck disable=SC2086
    timeout 4h ros2 run x3_experiment_runner trial_orchestrator --world_name world_1 --output_dir "$out" $args \
        > "$out/orchestrator.log" 2>&1
    rc=$?
    teardown
    if complete "$out"; then
        f=$(ls "$out"/world_1_*.csv | head -1)
        echo "$(date +%F_%T) $unit ok rc=$rc success=$(awk -F, 'NR>1 && $8=="True"' "$f" | wc -l)/$N_GOALS"
    else
        echo "$(date +%F_%T) $unit INCOMPLETE rc=$rc (rerun the same command to retry)"
    fi
done
echo "$(date +%F_%T) done $STAGE"
