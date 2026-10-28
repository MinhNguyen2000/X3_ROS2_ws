# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ROS 2 Humble colcon workspace for a Yahboom ROS Master X3 (mecanum) robot doing autonomous navigation. It compares navigation policies (TD3 deep-RL actors in `direct` and `delta` action modes, and an Artificial Potential Field planner) in Gazebo, then runs the same planner nodes on the physical robot (Jetson Orin, JetPack 6.2.1). Full robot/Docker setup is in `docs/SETUP.md`.

## Build / run

Everything runs inside the `ros2_humble` Docker container, with the repo volume-mounted at `/X3_ROS2_ws` (`robot_ws/build.sh` -> `robot_ws/setup_gpu_libs.sh` -> `robot_ws/start_container.sh`). `ros_entrypoint.sh` sets `ROS_DOMAIN_ID=6`, sources ROS and the workspace, and inits submodules.

```bash
docker exec -it ros2_humble bash
cd /X3_ROS2_ws && colcon build --symlink-install && source install/setup.bash
colcon build --symlink-install --packages-select x3_drl_policy   # one package
```

- `--symlink-install` means Python edits take effect without rebuilding. Launch/config/policy files are installed to `share/`, so after adding one, rebuild.
- Keyboard teleop (`x3_control/x3_keyboard`) must be started with `ros2 run` in a TTY (`docker exec -it`), never through `ros2 launch`.
- Python packages only carry the ament lint tests (`test_flake8`, `test_pep257`, `test_copyright`). There are no unit tests. Run one with `colcon test --packages-select x3_apf_planner` or `pytest src/x3_apf_planner/test/test_flake8.py`.
- Submodules: `rplidar_ros`, `rf2o_laser_odometry`, `fisheye_calibration`. `astra_camera*` is vendored, not a submodule.

Typical sim run (each in its own `docker exec -it` shell): `ros2 launch x3_description gazebo.launch.py world:=world_2.sdf` (the `world` arg takes the filename incl. `.sdf`, default `world_1.sdf`; other args: `agent_name`, `use_sim_time`, `launch_ekf_odom`, `x_offset`, `y_offset`), then a planner node (`x3_drl_policy` / `x3_apf_planner`), then `ros2 run x3_nav_bringup goal_client` or `goal_sequence_client`. `ROS_DOMAIN_ID` and `ROS_IP` are set in `.devcontainer/ros_entrypoint.sh`.

## Architecture

**Planner interface is the seam between sim and real.** `x3_drl_policy/policy_node` and `x3_apf_planner/apf_node` expose identical interfaces under namespace `<agent_name>` (default `agent0`): subscribe `odom` (Odometry) and `scan` (LaserScan); publish `cmd_vel` (**TwistStamped**) and `active_planner` (latched String); serve the `navigate_to_goal` action (`x3_nav_interfaces/NavigateToGoal`). Anything that supplies those topics can host a planner, which is why the same node runs on Gazebo and on hardware.
- `policy_node` loads `x3_drl_policy/policies/<model_name>/actor_weights.pt` (+ `norm_stats.npz`). `action_mode` (`delta` vs `direct`) is inferred from the numeric model id in the name (`TD3_<id>_<checkpoint>`; ids 384 and 386-395 are delta). A new model with a different id range needs that rule updated. The 18 lidar ray groups are hardcoded (TODO in code).

**Simulation** (`x3_description`): `launch/gazebo.launch.py` starts Gazebo (`worlds/world_1..5`, `world_lab`), robot_state_publisher from the xacro URDF, `ros_gz_bridge`, ros2_control spawners, `odom_offset_node` (ground-truth `/odom` shifted to the spawn pose) and `odom_to_vel_raw_node` (fake wheel `vel_raw`). `launch_odom.py` runs the estimation chain.

**Odometry chain** (same in sim and real): `rf2o_laser_odometry` -> `x3_covariance_filter` (adds covariances to IMU / rf2o / wheel velocity) -> `robot_localization` EKF -> `odom_ekf`. In sim, planners use ground-truth `/odom` and `odom_ekf` is only logged for sim2real analysis. On hardware, `x3_bringup/launch/odom.launch.py` provides the odometry.

**Hardware** (`x3_bringup/launch/robot.launch.py`): rplidar (`/dev/rplidar`) -> `lidar_rear_mask` (`scan` -> `scan_filtered`), Astra camera, `mcnamu_driver` (Rosmaster serial board: IMU, encoders, motors), `x3_visual` face/emotion nodes, and a `policy_node` with lower velocity caps than sim. Hardware assumes the Rosmaster driver from `robot_ws/rosmaster_driver_install`.

**Driving goals**: `x3_nav_bringup` has `goal_client` (single goal) and `goal_sequence_server`/`goal_sequence_client` (`NavigateToGoalSequence` action; waypoint lists from `paths/paths_world_*.json`; `planner_choice` = `drl`/`apf`; records executed paths and exposes a `save_path` Trigger service).

**Batch experiments** (`x3_experiment_runner/trial_orchestrator`, run with `--planner {DRL,APF}`): keeps one headless Gazebo world alive and, per goal in `goals/goals_world_*.json`, restarts the planner and the odom stack as `ManagedProcess` subprocesses (`ros2 run`/`ros2 launch`) to clear stale state. Launch Gazebo with `launch_ekf_odom:=false` for this mode. Outputs a summary CSV plus per-trial JSON time series in `trial_results/`.

## Analysis scripts (`scripts/`)

Not ROS packages. `scripts/viz/` plots recorded runs. `run_registry.py` lists runs as `'<model_folder>/<path>_<world>_<ddmmyy>_<HHMM>'` (no `.json`), so renaming files in `recorded_paths/` requires updating it. `G2G_analysis.ipynb` analyzes trial results.

## Recorded-path data conventions (`scripts/recorded_paths/`)

- Layout: `<APF|TD3_<id>_<checkpoint>>/<pathN>_<world>_<ddmmyy>_<HHMM>.json`. World tag is `world_lab` (not `worldlab`) or `world_<N>` / `world<N>`.
- Real-robot logs have non-empty `poses`, `velocities` **and** `poses_gt`. Simulation logs have empty `poses_gt`. A log with empty `poses` and `velocities` is invalid and should be removed.
- Recorded results are committed. Treat them as experimental data and don't rewrite their contents.

## Gotchas

- All `x3_*` `package.xml` descriptions are still `TODO`.
- `x3_bringup/launch/robot.launch.py` references an undeclared `orbbec_cameracamera_name` argument, only launches the DRL planner, and includes `odom.launch.py` without forwarding `agent_name`.
- `build/`, `install/`, `log/`, `robot_ws/gpu_libs` and `*.engine` are gitignored.

## Project context

At the start of a session, read `../personal_hub/CLAUDE.md`, `../personal_hub/projects/springer_autonomous_robots_2026/tasks.md` and the notes index `../personal_hub/projects/springer_autonomous_robots_2026/notes/README.md`. Open an individual note only when the task needs it. Task updates and wrap-up go in tasks.md, following personal_hub/CLAUDE.md. This workspace feeds the paper in `../springer_autonomous_robots_2026`.

Write research notes (findings, design advice, selection rationale) to `../personal_hub/projects/springer_autonomous_robots_2026/notes/`, never inside this repo. This repo holds code, configs and experiment data only.

## Experiment integrity

- Don't modify trained policies (`actor_weights.pt`, `norm_stats.npz`) or the contents of `scripts/recorded_paths/` and `trial_results/`. Ask before deleting a log, even an invalid one.
- Analysis scripts and notebooks read logs and write new output files. Each output records the git commit hash and date, so the paper's tables and figures can be traced under Provenance in the project's tasks.md.
- Seed and checkpoint selection is defined on simulation data only, before any real-world deployment. Never select seeds or checkpoints using real-world outcomes.
- Don't start Gazebo, `trial_orchestrator`, planner nodes, training, or anything in `x3_bringup` (hardware) without asking me first.