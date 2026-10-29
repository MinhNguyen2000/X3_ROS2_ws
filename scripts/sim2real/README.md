# scripts/sim2real

Scripts and data of the sim-to-real work (planning repo task IDs P1, P7, P10). The code changes behind them are
described in `docs/sim2real_deployment_config.md`. Numbers printed or stored here are snapshots of the data named in
each file; regenerate them with the script before quoting them.

Run Python scripts with an interpreter that has numpy and scipy (the system scipy on the lab host is broken against
numpy 2; the anaconda interpreter works).

## Policy selection and run schedule
| File | What it is |
|---|---|
| `select_policies_bm.py` | Ranks every model of each variant (delta/direct x best/final) on the Gazebo multi-goal batch (`scripts/batch_runs/aggregate_7_scenarios.csv`, APF excluded, world_4/path1 dropped) by mean waypoint ratio, then shorter mean path length. Picks B = rank 1 and M = rank ceil((n+1)/2). Writes `seed_selection_bm.csv`. |
| `seed_selection_bm.csv` | Full ranking per variant with tier B / M / W. |
| `make_schedule.py` | Run schedule for the confirmatory robot runs: the 8 B/M policies x 2 lab paths x 10 blocks = 160 runs, seed 20261028, shuffled policy order per block, path order alternating, 40 runs per 2-hour stretch. Writes `schedule.csv`. |
| `schedule.csv` | The committed schedule. |

## Deployment config
| File | What it is |
|---|---|
| `antenna_sector.py` | Finds the scan sector that is the robot's own structure from a static scan in open space. `--topic /agent0/scan --scans 30 --save scan.json` on the robot, or `--json scan.json`. Reports the rear half-width, the sector and the touched observation groups. |
| `static_scan_20261028.json` | The static scan of 2026-10-28 behind the 40-degree rear mask (30 scans, 1800 beams; `range_max` is not stored). |

## Robot runs (P7)
| File | What it is |
|---|---|
| `run_robot_schedule.py` | Robot-side runner for `schedule.csv`: after the operator places the robot on the start marks it relaunches the bringup (new odometry origin), starts the policy node (`drl_policy.launch.py`, model per run), checks the policy's published config against the frozen one and that the run started at the odometry origin, starts the sequence server and client, reads the online result from the saved JSON, asks the operator for a fault, repeats faulted runs, resumes from `robot/manifest_robot.csv`. Operator steps: `docs/real_robot_run_protocol.md`. `--dry-run` lists the pending runs. |
| `test_run_robot_schedule.py` | Unit tests of its pure logic (schedule, resume, config check, result parsing): `cd scripts/sim2real && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test_run_robot_schedule.py`. |
| `test_runner_in_gazebo.sh` | Functional test without a robot: Gazebo's empty world in the devcontainer image plays the robot and the runner performs the first N runs with `--auto --no-bringup`. Needs this worktree built with colcon. The simulated robot is not put back on the start marks between runs, so a run can start away from the origin. |

## Scouting re-analysis
| File | What it is |
|---|---|
| `scouting_reanalysis.py` | Re-analysis of the 2026-10-13..15 real-robot runs in `scripts/recorded_paths/*/path*_world_lab_*.json`: goals reached from `poses` (policy odometry; tolerance 0.3 m plus 0.01 m slack for the unlogged decision-time pose) and from `poses_gt` (camera, with yaw = 2 asin(logged value)), pipeline signature per run, frame check of camera against odometry, correlation of the sim ranking with the real goal fraction. `--note PATH` tests which count an old scouting note recorded. |
| `scouting_runs.csv`, `scouting_reanalysis_output.txt` | Per-run table and printed output of the last run. |

The scouting is a pilot under mixed pipeline states and the camera is not a ground truth; do not use it to rank policies.

## Gazebo acceptance check (`check/`)
Runs the 8 selected policies on the 6 scenarios of the sim ranking (world_1 paths 1 to 3, world_2/path1, world_3/path1,
world_5/path1) in Docker, headless, on a separate git worktree so that `scripts/recorded_paths` of the main workspace is
untouched and the batch's `pkill` of Gazebo cannot hit a session on the host.

| File | What it is |
|---|---|
| `run_check.sh` | `run_check.sh <a|b|c|smoke|frozen> [batch_multigoal.py args]`. Configs: a = the hardware config (+-45.8 degree mask, training-mean fill, clip 11.10); b = as a with the masked wedge set to the maximum range; c = old processing at the same scales; smoke = a; frozen = the `robot.launch.py` values (+-40 degrees). |
| `run_all.sh`, `run_bc.sh` | Drivers that run configs in sequence (resumable from the manifests). |
| `compare_configs.py` | Paired comparison of two configs from their manifests: cells worse/equal/better with a sign test, per-policy differences with sign test and bootstrap intervals, McNemar test on full-sequence success, outcome types, per variant and scenario. `compare_configs.py a b`. |
| `manifest_{a,b,c}.csv`, `stdout_*.log` | Results and progress of the three configs; the score is the server's online waypoint count. |
| `manifest_b_aborted_stale_gazebo.csv` | The aborted first run of (b), kept as evidence. |
| `compare_*.txt`, `acceptance.txt` | Output of `compare_configs.py` and of the acceptance rule. |

The scripts in `check/` have the check worktree path (`/home/controlslab/X3_ROS2_ws_p1check`) and the devcontainer image
name hard-coded; adjust `WT` and the image lookup in `run_check.sh` and the driver scripts before reusing them. The
worktree needs `colcon build --symlink-install` for `x3_nav_interfaces`, `x3_description`, `x3_nav_bringup`,
`x3_drl_policy`, `x3_covariance_filter` and `rf2o_laser_odometry` (the last is a submodule that is empty in a plain
worktree). Raw per-run JSONs and process logs of the check are not in git.
