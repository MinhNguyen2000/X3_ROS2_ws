#!/bin/bash
# a, b, c one after another; each resumes from its manifest if restarted
cd /home/controlslab/X3_ROS2_ws_p1check
MODELS="--only-model TD3_00391_800 --only-model TD3_00388_900 --only-model TD3_00390_1000 --only-model TD3_00395_1000 --only-model TD3_00399_900 --only-model TD3_00401_900 --only-model TD3_00400_1000 --only-model TD3_00402_1000"
SCEN="--scenarios world_1/path1 world_1/path2 world_1/path3 world_2/path1 world_3/path1 world_5/path1"
for c in a b c; do
  scripts/sim2real/check/run_check.sh $c $MODELS $SCEN > scripts/sim2real/check/stdout_$c.log 2>&1
done
