# Real-robot confirmatory runs: operator protocol (P7)

For the 160 runs of `scripts/sim2real/schedule.csv` (8 policies x 2 lab paths x 10 blocks, 4 stretches of 40 runs, about 2 h each).
Selection rule: `personal_hub/.../notes/seed_selection_protocol_20261028.md`; scoring rule: `notes/scoring_rule_decision_20261028.md`
(the score of a run is the server's online goal count, `result.waypoints_completed`); config: `docs/sim2real_deployment_config.md`.
Items marked **DECIDE** are open owner decisions; the value shown is only a suggestion.

## 1. Once, before stretch 1 (not part of the 160 runs)
1. Robot checkout contains the frozen config (`robot.launch.py` / `drl_policy.launch.py` from `main`); rebuild `x3_bringup`,
   `x3_drl_policy` and `x3_nav_bringup`. Policy weights are installed from the package share directory, so the rebuild is needed.
2. Stop any running bringup: the runner launches it itself (`robot.launch.py start_policy:=false`) before every run, so a run takes the bringup start-up time on top of the run itself; check the 40 runs per 2-hour stretch against the real cycle time.
3. Check the scan once: `ros2 topic echo --once /agent0/scan --field range_max` (and `range_min`); note both.
4. Short check of the policy node on its own: `ros2 launch x3_bringup drl_policy.launch.py model_name:=TD3_00390_1000`; the log shows
   `Config:` with `normalized_actions=True max_lin_vel=0.2 max_angular_vel=0.3 antennae_mask_dist=0.0 rear_mask_deg=40.0
   rear_mask_fill=train_mean range_clip=11.1`.
5. Two or three runs with `scripts/sim2real/run_robot_schedule.py --max-runs 3` (their rows go to the manifest: delete `scripts/sim2real/robot/`
   afterwards so they do not count). Passing means: no immediate `Obstacle hit` abort (the self-returns just above the old 0.20 m mask limit
   are covered by the 40 degree wedge), the saved JSON has `result`, `planner_config`, `goal_results` and `code` = the frozen commit, and the
   runner reports no config mismatch.

## 2. Start of every stretch (pre-flight)
- LiDAR and antennae mounted as in the static scan of 2026-10-28. If anything was moved or replaced, repeat
  `scripts/sim2real/antenna_sector.py --topic /agent0/scan --scans 30 --save scan.json` in open space and re-derive `rear_mask_deg`
  (it is 40.0 for self returns spanning -29.1 to +36.9 degrees behind the sensor).
- Wheel odometry, IMU and rf2o running; `/agent0/odom` and `/agent0/scan` publishing; battery above the cutoff for a 2 h stretch **DECIDE: swap threshold**.
- Arena as in the scouting runs; nobody inside the arena during a run.
- Start the runner from a shell with ROS and the workspace sourced: `python3 scripts/sim2real/run_robot_schedule.py --stretch N`.

## 3. Every run
0. **Odometry origin.** The odometry frame (rf2o + EKF + the driver's wheel odometry) has its origin where the bringup started and the code
   has no reset; goals and the policy's heading input are in that frame. As in the 2026-10 scouting runs, the robot is put on the start
   marks and the whole bringup is relaunched before every run; `run_robot_schedule.py` does the relaunch after you place the robot and press
   Enter, and marks a run `start_ok` 0 (not kept, repeated) when its first logged pose is more than 0.15 m or 0.15 rad from the origin.
1. **Start pose.** Goals are relative to the start pose (the odometry frame starts at the robot), so the start position and heading must be
   the same every run. Mark the start position and heading on the floor and place the robot on them. **DECIDE: placement tolerance**
   (suggestion: a few cm and a few degrees); the ceiling camera is optional and not a ground truth.
2. Press Enter at the prompt; the runner relaunches the bringup, the policy node and the sequence server, then starts the sequence client. Stay clear of the arena; the e-stop is the operator's, Ctrl-C in the
   runner stops the client, server and policy node (the policy node sends a zero command).
3. After the run the runner prints the result; answer the fault prompt.

## 4. Faults (pre-declared)
A run is repeated, with the reason logged, only for a non-policy fault: **person**, **lidar** (dropout), **odom** (dropout or jump),
**battery** (cutoff), **driver** (motor driver fault), **other** (say what). Everything else is kept, including collisions, timeouts and
runs where the robot never moves. A faulted run, or a run whose `planner_config` differs from the frozen config, stays pending and is
run again after the unattempted runs.

## 5. After every stretch
- `scripts/sim2real/robot/manifest_robot.csv` has one row per attempt: 40 kept runs per stretch plus any repeats; every row has `code` = the
  frozen commit and `config_ok` 1.
- Commit the manifest and the run JSONs (`scripts/recorded_paths/<model>/path*_world_lab_*.json`) so the raw runs are in version control.
- Report per policy (variant, B/M): goals reached per run, full-sequence success, failure type; the quantity is "goals reached according to
  the robot's own odometry" and has no external verification.
