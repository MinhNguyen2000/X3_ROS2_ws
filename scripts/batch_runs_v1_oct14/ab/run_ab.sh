#!/bin/bash
# A/B of policy_node action/lidar settings on TD3_00390_800 (6 scenarios per variant)
cd /X3_ROS2_ws
run() { v=$1; shift
  python3 scripts/batch_multigoal.py --only-model TD3_00390_800 --manifest scripts/batch_runs/ab/$v.csv "$@" > scripts/batch_runs/ab/$v.out 2>&1; }
ACT_OCT="--planner-param normalized_actions:=false --planner-param delta_act_linear:=0.05 --planner-param delta_act_angular:=0.05 --planner-param delta_act_scale_linear:=1.0 --planner-param delta_act_scale_angular:=1.0"
LID_OCT="--planner-param antennae_mask_dist:=0.0 --planner-param rear_mask_deg:=40.0 --planner-param obstacle_tolerance:=0.15"
LID_SEP="--planner-param antennae_mask_dist:=0.20 --planner-param rear_mask_deg:=0.0 --planner-param obstacle_tolerance:=0.21"
run A0 $ACT_OCT $LID_OCT
run B  $LID_OCT
run C
run D  $ACT_OCT $LID_SEP
