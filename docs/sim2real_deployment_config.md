# Sim-to-real deployment config (P10)

What the branch `p1-deploy-config` changed in the code that runs the trained TD3 policies, why, and how to use and
roll back each change. Written 2026-10-28 for the merge into `main`. The research reasoning, the numbers and their
provenance live in the planning repo (`personal_hub/projects/springer_autonomous_robots_2026/notes/`), cited by file
name below; this file only describes the code.

## 1. Why

The policies are trained in MuJoCo (2 m/s, 3 rad/s, 500 Hz policy steps, full 360-degree LiDAR with no masking) and
ranked in Gazebo (0.5 m/s, 1.0 rad/s, 50 Hz). The robot differs in four ways that change what the policy sees or does:

1. the robot's own structure returns a scan sector behind the LiDAR that training never has;
2. masked, invalid and no-return beams were set to the sensor's `range_max`, a value far outside the training
   distribution (+7 to +10 standard deviations in the rear groups);
3. a global 0.20 m mask treated every close return as free space, so a real obstacle closer than 0.20 m was hidden;
4. the velocity scales (0.2 m/s and 0.5 rad/s) did not reproduce the per-step motion the policy was trained with.

The branch makes the processing configurable, sets one frozen hardware config, and makes every run log enough to be
scored and audited. See `deployment_pipeline_advice_20261028.md` (18 ranked differences), `gazebo_hwconfig_check_20261028.md`
and `hwconfig_freeze_20261028.md`.

## 2. Code changes

### `x3_drl_policy`
- **`x3_drl_policy/lidar_processing.py` (new, pure numpy).** `preprocess_scan` rotates the scan so index 0 is forward,
  cleans it and flips it to the training beam order; `pool_groups` min-pools the 18 observation groups, ignoring masked
  beams and using a supplied fill value for fully masked groups. With default arguments it reproduces the previous
  node processing exactly.
- **`policy_node.py`.** Uses the module above. New parameters, defaults chosen so behaviour is unchanged:

  | Parameter | Default | Meaning |
  |---|---|---|
  | `rear_mask_fill` | `range_max` | `range_max`: masked rear beams set to the maximum range. `train_mean`: masked beams ignored when pooling; a fully masked group takes the policy's training mean for that group (from `norm_stats.npz`) |
  | `range_clip` | `0.0` | maximum range the policy sees; 0 uses the sensor `range_max`. Invalid, no-return and masked beams are set to this value. 11.10 m is the observation bound of the training environment |

  Existing parameters used by the config: `normalized_actions`, `max_lin_vel`, `max_angular_vel`, `antennae_mask_dist`,
  `rear_mask_deg` (half-width of the wedge behind the LiDAR), `goal_timeout`.
- **Startup log and latched topic.** The node logs a `Config:` line and publishes the same settings as JSON on the
  latched topic `planner_config` (std_msgs/String), which the sequence server saves with each run.
- **Result message.** A reached goal reports `Goal reached. Distance to goal 0.298`: the distance at the control tick
  that triggered it. That pose is never logged afterwards (the check happens before the feedback is published), so a
  recount from logged poses is one control tick short for the last goal.
- **Tests.** `test/test_lidar_processing.py` (8 tests): exact equality with the previous processing for four default-style
  configs, an obstacle in front is not hidden without the 0.20 m mask, mean fill and partial-group pooling, range clip.
  Run with `cd src/x3_drl_policy && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test/test_lidar_processing.py`
  (the plugin switch avoids a ROS pytest plugin that needs packages not on every host).

### `x3_nav_bringup/goal_sequence_server.py`
New keys in the saved run JSON (`scripts/recorded_paths/<model>/<path>_<world>_<ddmmyy>_<HHMM>.json`):

| Key | Content |
|---|---|
| `result` | `waypoints_completed`, `total_waypoints`, `success`, `message` of the sequence: the online goal count |
| `goal_results` | per goal `{goal, success, message}`; for a reached goal the message carries the decision-time distance |
| `planner_config` | the policy node's settings (JSON from `planner_config`); `null` for APF |
| `code` | git short hash of the workspace at save time |
| `gt_yaw_unit` | `"rad"` |

Bug fix: the camera pose was saved with `"yaw": orientation.z`, the quaternion z component (the sine of half the angle),
not the angle. It now saves the yaw in radians. **Files saved before 2026-10-28 hold `orientation.z`; convert with
`2 * asin(z)`.** Verified against odometry yaw over 166 scouting runs (slope 0.99, against 2.04 for the raw value).

Other things to know about the JSON: `total_distance` is not reliable (1.45 m for a 132 s Gazebo run); take path length
from `poses`. `poses_gt` is only present when the ceiling camera ran (it is not a ground truth: its fisheye calibration is
imperfect and it overestimates position far from the start pose) and is empty for 73 real runs of 2026-10-14, so identify
real-robot runs by the `world_lab` path name, not by `poses_gt`.

### `x3_bringup/launch/robot.launch.py` (changes the hardware defaults)
| Parameter | Before | Now |
|---|---|---|
| `normalized_actions` | default (true) | `True` |
| `max_lin_vel` / `max_angular_vel` | 0.2 / 0.5 | 0.2 / 0.3 |
| `goal_timeout` | 30 s | 90 s |
| `antennae_mask_dist` | default 0.20 | 0.0 (off) |
| `rear_mask_deg` | default 0 (off) | 40.0 |
| `rear_mask_fill` | n/a | `train_mean` |
| `range_clip` | n/a | 11.10 |

Reasons: 0.2 m/s and 0.3 rad/s at the 50 Hz control rate reproduce the per-step motion of 2 m/s and 3 rad/s at 500 Hz
(4 mm and 6 mrad) and the 0.667 m turning radius. The 40-degree half-width comes from a static scan of the robot in open
space (`scripts/sim2real/static_scan_20261028.json`, analysed with `antenna_sector.py`): self returns span -29.1 to +36.9
degrees behind the sensor, all 324 self beams are covered with about 3 degrees of margin. **To roll back to the previous
hardware behaviour set `max_angular_vel` 0.5, `goal_timeout` 30, `antennae_mask_dist` 0.20, `rear_mask_deg` 0.**

### `x3_description/launch/launch_world.py`
`GZ_HEADLESS=1` starts Gazebo server-only with headless rendering (for batch runs in a container without a display).
Without the variable the launch is unchanged.

### `scripts/batch_multigoal.py`
Teardown now sends SIGKILL to a Gazebo process that survives SIGTERM. A Gazebo server hung in start-up ignored SIGTERM,
stayed alive across jobs and made every later launch fail or run in the wrong world (found 2026-10-28; see section 4).

## 3. `scripts/sim2real/`
See `scripts/sim2real/README.md` for the file index and how to re-run each script.

## 4. Verification done on this branch
- Unit tests of the LiDAR processing (section 2).
- Offline: the frozen processing applied to the real static scan gives a lowest pooled value of 0.344 m (the collision
  stop is 0.205 m), fully masked groups 7 to 9, and no self-return beam left uncovered, for any assumed `range_max`.
- Gazebo acceptance check (8 policies x 6 scenarios x 3 configs, headless in a container): (a) the config above with
  +-45.8 degrees, (b) as (a) with the masked wedge set to the maximum range, (c) old processing at the same scales.
  Pre-declared rule: adopt (a) if its 8-policy mean waypoint ratio is at least (c) minus 0.05. Result 0.896 against
  0.922: met, with a margin (+0.024) that the cell-level spread does not make firm. (b) is far worse (0.251), so the fill
  value of the masked wedge matters. The first run of (b) was aborted by the stale-Gazebo problem above and rerun from
  scratch (its first 27 jobs reproduce exactly). Raw manifests and comparison outputs are in `scripts/sim2real/check/`.
- Smoke test of the frozen commit `e51357f` (tag `p10-hwconfig-frozen-20261028`): 4 of 4 runs complete, new JSON fields
  present, no tracebacks.

## 5. Not verified
- The robot has never run this config. The first on-robot check (launch, `Config:` line, 2 to 3 short runs, no immediate
  collision abort) is still to do.
- The Gazebo check used a +-45.8 degree mask; the robot launch uses +-40 degrees, which was only smoke tested.
- The RPLidar S2 `range_max` is not in the recorded scan. The frozen config clips at 11.10 m, so it does not depend on it.
- Gazebo has no antennae: the check shows what masking costs and what fill value does, not that the mask removes the
  robot's own returns.
