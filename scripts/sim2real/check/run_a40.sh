#!/bin/bash
# config (a) with the deployed +-40 degree rear mask on the 8 selected policies x 6 scenarios (48 cells), resumable from manifest_a40.csv
cd "$(dirname "$0")/../../.."
MODELS="--only-model TD3_00391_800 --only-model TD3_00388_900 --only-model TD3_00390_1000 --only-model TD3_00395_1000 --only-model TD3_00399_900 --only-model TD3_00401_900 --only-model TD3_00400_1000 --only-model TD3_00402_1000"
SCEN="--scenarios world_1/path1 world_1/path2 world_1/path3 world_2/path1 world_3/path1 world_5/path1"
scripts/sim2real/check/run_check.sh a40 $MODELS $SCEN > scripts/sim2real/check/stdout_a40.log 2>&1
