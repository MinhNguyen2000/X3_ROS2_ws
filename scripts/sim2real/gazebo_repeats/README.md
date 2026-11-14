# scripts/sim2real/gazebo_repeats

Gazebo repeats of the eight deployed policies (planning task P2). Numbers below are snapshots of the CSV files in this folder; regenerate them with `analyze_gazebo_repeats.py` before quoting them. Code `world_5` is the paper's "World 4". Code `world_4/path1` is excluded everywhere (as `EXCLUDED_SCENARIOS` of `scripts/metrics/extract_leg_metrics.py`): its 40 runs are in `manifest_standard.csv` and the run JSONs but not in any output.

## Job A: is the Gazebo ranking stable over repeats? (valid)
Purpose: in the one-repeat batch a single collision decided a rank (TD3_delta Final M, 84.4 percent goal completion, one collision in world_1/path3). Job A reruns the same scenarios with 5 repeats per policy and scenario.
- Policies: the eight deployed ones (`scripts/sim2real/robot/robot_policy_stats.csv`).
- Scenarios: the paper's six: world_1 path1, path2, path3, world_2/path1, world_3/path1, world_5/path1. 8 x 6 x 5 = 240 runs, all `ok`, no launch failures, no retries.
- Config: the same as the original batch (`scripts/batch_runs/manifest.csv`, `manifest_world5.csv`): `policy_node` defaults, no planner parameters, goal tolerance 0.3 m (client default), headless Gazebo in the devcontainer image, ROS domain 68, discovery server unset. The original batch ran `policy_node.py` as of commit af91cae (hash acccf75392); these runs used the file at HEAD (hash e3e2892cc7), which only adds parameters whose defaults leave the processing unchanged (lidar_processing refactor, control_rate, action_mode, delta_down).
- Wall time: world_1 to world_3 and world_4 batch 2026-11-13 03:32 to 07:51 (240 runs, about 1.1 min per run), world_5 batch 13:31 to 14:18 (40 runs). Summed sequence time 3.75 h.
- Anomalies: none. 45 of 48 policy-by-scenario cells end identically in all 5 repeats (the simulation is nearly deterministic; largest sequence-time spread in a cell 23 s).
- Scores: goal completion = goals reached / goals planned over the six scenarios; collisions and timeouts per 100 attempted legs (reached legs plus the failed leg); SPL as in `robot_analysis.py` (shortest path of the world's sdf from the pose where the leg started, leg boundaries from the poses; failed and not attempted legs score 0). "Old" columns are the one-repeat batch (`scripts/metrics/output/multigoal_units.csv`, geodesic SPL by server-log boundaries, a slightly different leg split). Rank = competition rank by goal completion among the eight policies (ties share the best rank).

| Policy | Tier | Model | Runs ok | Goals % | Rank | Collisions /100 legs | Timeouts /100 legs | SPL all | SPL reached | Old goals % | Old rank |
|---|---|---|---|---|---|---|---|---|---|---|---|
| TD3_direct Final | B | TD3_00400_1000 | 30/30 | 100.0 | 1 | 0.00 | 0.00 | 0.920 | 0.919 | 100.0 | 1 |
| TD3_direct Final | M | TD3_00402_1000 | 25/30 | 92.5 | 6 | 3.27 | 0.00 | 0.826 | 0.896 | 100.0 | 1 |
| TD3_direct Best | B | TD3_00399_900 | 30/30 | 100.0 | 1 | 0.00 | 0.00 | 0.897 | 0.896 | 100.0 | 1 |
| TD3_direct Best | M | TD3_00401_900 | 21/30 | 91.9 | 7 | 5.77 | 0.00 | 0.825 | 0.901 | 96.9 | 7 |
| TD3_delta Final | B | TD3_00390_1000 | 30/30 | 100.0 | 1 | 0.00 | 0.00 | 0.872 | 0.871 | 100.0 | 1 |
| TD3_delta Final | M | TD3_00395_1000 | 25/30 | 84.4 | 8 | 3.57 | 0.00 | 0.745 | 0.863 | 84.4 | 8 |
| TD3_delta Best | B | TD3_00391_800 | 30/30 | 100.0 | 1 | 0.00 | 0.00 | 0.896 | 0.895 | 100.0 | 1 |
| TD3_delta Best | M | TD3_00388_900 | 30/30 | 100.0 | 1 | 0.00 | 0.00 | 0.869 | 0.866 | 100.0 | 1 |

Reading: TD3_delta Final M (TD3_00395_1000) keeps rank 8 of 8 with 84.4 percent goal completion over 30 runs; its 5 failures are all collisions in world_1/path3 (the same 84.4 percent as the one-repeat batch, because the simulation repeats itself). TD3_direct Final M (TD3_00402_1000) is 92.5 percent (rank 6; one repeat gave 100 percent) and TD3_direct Best M (TD3_00401_900) 91.9 percent (rank 7). The other five policies complete every leg in every run. So the single-collision ranking does not change with repeats for TD3_delta Final M, while TD3_direct Final M drops from a tie at 100 percent to rank 6 (collisions in world_3/path1 and world_5/path1).

## Job B: the deployed policies in the lab world (world_lab), SUSPECT, to be rerun
Purpose: separate the effect of the arena from the sim-to-real effect: same two goal sequences (`path1_world_lab`, `path2_world_lab`) and start pose as the robot, 10 repeats per policy and sequence (8 x 2 x 10 = 160 runs).
- Config: the frozen deployment config (`robot.launch.py` values: max_lin_vel 0.2, max_angular_vel 0.3, normalized actions, goal_timeout 90 s, antennae_mask_dist 0, rear_mask_deg 40 with training-mean fill, range_clip 11.10) plus obstacle_tolerance 0.205 (the LiDAR stop of `drl_policy.launch.py`), goal tolerance 0.3 m, spawn at the lab odometry origin (`x_offset:=-1.6775 y_offset:=-0.7625`). Gazebo world is `world_lab.sdf`; SPL uses its shortest path.
- Runs: 160 `ok`, plus 6 `launch_fail` rows (not counted as runs and retried). Wall time 2026-11-13 00:31 to 03:31, summed sequence time 1.78 h; a pilot run took 68 s.

Caveats, why Job B is suspect and will be rerun:
- The devcontainer sets `ROS_DISCOVERY_SERVER=192.168.0.104:11811` (the dev machine, the same discovery server the robot uses; the server ignores the ROS domain id). It was set for the first 45 runs (commits 87fbb20 and e56140c in `code_state`); it is unknown whether simulated traffic reached the robot's topics. Runs 46 on (commit d5d9c75) ran with it unset. The robot was not touched.
- After 45 good runs every launch failed ("sim not ready"); stray ROS nodes of a failed launch (odom_offset_node, covariance_filter) survived the teardown. `--kill-stragglers` was added to `batch_multigoal.py` and the discovery server was unset; the manifest keeps the 6 failed attempts.
- Only 23 of 160 runs completed all goals, against 114 of 160 on the robot (`robot_policy_stats.csv`); mean goal completion over the eight policies is 64.6 percent in Gazebo against 87.8 percent on the robot. Almost every failure is a LiDAR stop at 0.202 to 0.205 m. Whether this is the arena, the simulated LiDAR or a setup error is not established.
- Repeats are nearly deterministic: 12 of 16 policy-by-path cells end identically in all 10 repeats, so 10 repeats add little.

| Policy | Tier | Model | Runs ok | Goals % | Collisions /100 legs | Timeouts /100 legs | SPL all | SPL reached | Robot goals % | Robot SPL all | Robot SPL reached |
|---|---|---|---|---|---|---|---|---|---|---|---|
| TD3_direct Final | B | TD3_00400_1000 | 10/20 | 88.9 | 11.11 | 0.00 | 0.809 | 0.926 | 96.7 | 0.945 | 0.975 |
| TD3_direct Final | M | TD3_00402_1000 | 0/20 | 74.4 | 22.99 | 0.00 | 0.618 | 0.846 | 67.8 | 0.687 | 1.000 |
| TD3_direct Best | B | TD3_00399_900 | 0/20 | 35.6 | 38.46 | 0.00 | 0.345 | 1.000 | 93.3 | 0.889 | 0.956 |
| TD3_direct Best | M | TD3_00401_900 | 0/20 | 66.7 | 25.00 | 0.00 | 0.659 | 0.978 | 84.4 | 0.799 | 0.974 |
| TD3_delta Final | B | TD3_00390_1000 | 0/20 | 55.6 | 14.29 | 14.29 | 0.496 | 0.913 | 65.6 | 0.664 | 0.998 |
| TD3_delta Final | M | TD3_00395_1000 | 0/20 | 55.6 | 28.57 | 0.00 | 0.482 | 0.915 | 96.7 | 0.916 | 0.946 |
| TD3_delta Best | B | TD3_00391_800 | 0/20 | 55.6 | 28.57 | 0.00 | 0.521 | 0.953 | 100.0 | 0.973 | 0.973 |
| TD3_delta Best | M | TD3_00388_900 | 13/20 | 84.4 | 8.43 | 0.00 | 0.716 | 0.875 | 97.8 | 0.963 | 0.983 |

SPL of reached legs, mean over the eight policies: Gazebo world_lab 0.926 against robot 0.976 (earlier Gazebo values 0.86 to 0.92), so the Gazebo value is closer to the earlier Gazebo values than to the robot's, with the caveats above.

## Files
| File | What it is |
|---|---|
| `run_repeats.sh` | `run_repeats.sh lab` (Job B, `REPS` default 10) or `run_repeats.sh std` (Job A, default 5, `--scenarios` replaces the scenario list). Runs `scripts/batch_multigoal.py` headless in the devcontainer image on this worktree, resumable. |
| `manifest_standard.csv`, `stdout_standard.log` | Job A manifest (280 rows: the paper's six scenarios and the 40 code world_4 runs) and progress log. |
| `manifest_world_lab.csv`, `stdout_world_lab.log` | Job B manifest (160 `ok`, 6 `launch_fail`) and progress log. |
| `standard_runs.csv`, `standard_policies.csv`, `standard_summary.json` | Job A outputs (six scenarios only): per run, per policy, summary with code states, wall times, determinism. |
| `lab_runs.csv`, `lab_policies.csv`, `lab_summary.json` | Job B outputs, per policy next to the robot columns. |
| `../analyze_gazebo_repeats.py`, `../test_analyze_gazebo_repeats.py` | Analysis and its tests. |

Run JSONs are in `scripts/recorded_paths/<model>/`, client logs in `scripts/batch_runs/logs/` (the other logs are not tracked, as for earlier batches).

## Reproduce
```
cd <worktree of branch p2-gazebo-repeats>      # needs colcon build --symlink-install of x3_nav_interfaces, x3_description, x3_nav_bringup, x3_drl_policy, x3_covariance_filter, rf2o_laser_odometry
scripts/sim2real/gazebo_repeats/run_repeats.sh std                              # Job A (code world_4 included; add --scenarios world_1/path1 world_1/path2 world_1/path3 world_2/path1 world_3/path1 world_5/path1 for the paper's six)
scripts/sim2real/gazebo_repeats/run_repeats.sh std --scenarios world_5/path1    # the world_5 part
scripts/sim2real/gazebo_repeats/run_repeats.sh lab                              # Job B
python3 -I scripts/sim2real/analyze_gazebo_repeats.py --job standard            # or --job lab
python3 -m pytest -q -p no:anyio scripts/sim2real/test_analyze_gazebo_repeats.py
```
Code state: runs carry `5479304`/`d5d9c75`(Job A) or `87fbb20`/`e56140c`/`d5d9c75` (Job B) plus the `policy_node.py` hash in the manifest `code_state`; the analysis commit is in each `*_summary.json`.
