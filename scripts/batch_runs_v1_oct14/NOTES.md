# Multi-goal Gazebo batch v1 (pre-revert `policy_node.py`)

**Status: superseded.** The DRL results in this batch are attributed to breaking changes in
`policy_node.py` and were rerun as v2 (`scripts/batch_runs/`) after the user reverted those changes.
Do not use the TD3 results here as the reported performance of the policies. The APF runs are
unaffected (APF does not use `policy_node`) and stay valid.

## What was run
- Runner: `scripts/batch_multigoal.py`, started 2026-10-24 18:40, last job started 22:36
  (all 246 valid jobs; manifest `manifest.csv`, per-job logs in `logs/`).
- Scenarios: world_1 path1-3, world_2 path1, world_3 path1, world_4 path1.
- Policies: TD3_delta 00386-00395 (19 checkpoints), TD3_direct 00396-00405 (17 checkpoints),
  where `_1000` = final and any other suffix = best; APF x5 per scenario.
- Result JSONs were written by `goal_sequence_server` to `scripts/recorded_paths/<planner>/`
  with `241026_*` timestamps. The `saved_file` column of `manifest.csv` identifies exactly
  which JSONs belong to this batch. New (v2) runs land in the same folders, so use the manifest
  to keep v1 and v2 apart.

## Headline results (from `manifest.csv`)
| Planner | Runs | Full successes | Waypoints completed |
|---|---|---|---|
| APF | 30 | 30 | 100% |
| TD3_delta | 114 | 22 | 50% |
| TD3_direct | 102 | 56 | 71% |

## Why the DRL policies underperformed (attributed cause)
The author attributes the DRL underperformance to breaking changes introduced in commit
`8572c72` (2026-10-14 10:09, "[x3_drl_policy] update DRL policy node parameters and improve LiDAR
data handling"), which changed `src/x3_drl_policy/x3_drl_policy/policy_node.py`. From the author's
Notion analysis ("DRL work" page):
- **Action scaling semantics (most likely culprit):** the policy used to act in normalized units
  with `v = action * max_lin_vel`; after the commit the action is clipped directly in m/s and
  rad/s with no multiply. Delta mode ramps twice as fast and the observed `vx` no longer matches
  `norm_stats.npz`; angular is numerically unchanged because `max_angular_vel` = 1.0.
- **Obstacle abort and LiDAR changes:** `obstacle_tolerance` 0.21 -> 0.15, the 0.20 m antennae
  mask removed, and a new +/-40 degree rear mask added in `lidar_callback`.

**Which code v1 ran (corrected):** v1 did NOT run the committed HEAD version. It ran an
uncommitted working-tree `policy_node.py` with `max_lin_vel` 0.5 / `max_angular_vel` 1.0 and the
`scan` topic (HEAD has 0.2 / 0.3 and `scan_filtered`). The recorded v1 commands confirm the caps:
maximum |vx| = 0.5 and |vyaw| = 1.0 in the runs checked. That exact file was not saved, and the v1
manifest has no `code_state` column. An earlier version of this note wrongly said v1 used the
committed code; that was inferred from the file's modification time, which only shows the last edit.

The author then reverted the action-scaling and LiDAR changes in the working tree (still uncommitted),
and the DRL jobs were rerun as v2 with a `code_state` column in the manifest.

This attribution is the author's judgement. It has not been verified by isolating each change;
the planned one-factor A/B runs on TD3_00390_800 would do that.

## Observed symptoms in v1 (for reference)
- Many DRL runs ended within 3-8 s with `Obstacle hit` at about 0.14 m (world_1 path2/path3,
  world_3 path1 across several seeds), while APF succeeded on every scenario.
- TD3_delta seeds 00386, 00389, 00390 and 00391 never completed a scenario, and several produced
  identical waypoint counts per scenario.
