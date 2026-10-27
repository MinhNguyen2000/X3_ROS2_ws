#!/bin/bash
# Re-run of APF batch 5. Earlier attempts: aborted_partial_5 (crash), aborted_hung_5 / aborted_dup_stacks_5
# (stale sim helper nodes from earlier sessions were still alive on domain 66).
source /opt/ros/humble/setup.bash && source /X3_ROS2_ws/install/setup.bash
export ROS_DOMAIN_ID=66 ROS_LOCALHOST_ONLY=0 LIDAR_TYPE=x3
export IGN_GAZEBO_RESOURCE_PATH=/X3_ROS2_ws/install/x3_description/share:$IGN_GAZEBO_RESOURCE_PATH
cd /X3_ROS2_ws
OUT=trial_results/APF_variance
echo "nodes visible before launch: $(ros2 node list 2>/dev/null | wc -l)"
# own process group so the whole sim tree (bridges, odom helpers, rsp) can be killed at the end
setsid ros2 launch x3_description gazebo.launch.py world:=world_1.sdf launch_ekf_odom:=false > $OUT/gazebo_run5.log 2>&1 &
GZ=$!
sleep 60
echo "=== run 5 (rerun) start $(date)"
ros2 run x3_experiment_runner trial_orchestrator --planner APF --world_name world_1 --output_dir $OUT/run5 > $OUT/run5.log 2>&1
echo "=== run 5 (rerun) exit $? $(date)"
kill -INT -- -$GZ; sleep 10; kill -TERM -- -$GZ 2>/dev/null
echo RERUN_DONE
