#!/bin/bash
# 3 repeats of variant C (policy_node defaults) on TD3_00390_800 world_4/path1
cd /X3_ROS2_ws
for r in 1 2 3; do
  python3 scripts/batch_multigoal.py --only-model TD3_00390_800 --only-scenario world_4/path1 \
    --manifest scripts/batch_runs/ab/C_w4_r$r.csv > scripts/batch_runs/ab/C_w4_r$r.out 2>&1
done
