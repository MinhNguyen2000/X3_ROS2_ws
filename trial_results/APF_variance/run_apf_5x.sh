#!/bin/bash
# Runs inside the container. 5 sequential APF goal-to-goal batches on world_1.
source /opt/ros/humble/setup.bash && source /X3_ROS2_ws/install/setup.bash
export ROS_DOMAIN_ID=66 ROS_LOCALHOST_ONLY=0 LIDAR_TYPE=x3
export IGN_GAZEBO_RESOURCE_PATH=/X3_ROS2_ws/install/x3_description/share:$IGN_GAZEBO_RESOURCE_PATH
cd /X3_ROS2_ws
OUT=trial_results/APF_variance
ros2 launch x3_description gazebo.launch.py world:=world_1.sdf launch_ekf_odom:=false > $OUT/gazebo.log 2>&1 &
GZ=$!
sleep 60
for k in 1 2 3 4 5; do
  echo "=== run $k start $(date)"
  ros2 run x3_experiment_runner trial_orchestrator --planner APF --world_name world_1 --output_dir $OUT/run$k > $OUT/run$k.log 2>&1
  echo "=== run $k exit $? $(date)"
done
kill $GZ; pkill -f "gz sim"; pkill -f "ign gazebo"
echo ALL_DONE
