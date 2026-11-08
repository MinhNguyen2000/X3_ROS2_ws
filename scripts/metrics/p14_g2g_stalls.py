#!/usr/bin/env python3
"""P14: classify the timeouts of the goal-to-goal batches as stalls or not (A1 Final and APF rerun).

A timeout is a stall when, over its last 30 s, the robot stayed within 0.1 m (spread of the ground-truth
position) and the mean commanded forward speed over the last third of the trial was under 0.06 m/s. It is
'at start' when the robot ended within 0.6 m of the odom origin. Reads trial_results_p14/ and
scripts/metrics/output_p14_g2g/p14_g2g_trials.csv (p14_g2g.py), writes output_p14_g2g/p14_g2g_stalls.csv.

Usage: scripts/metrics/p14_g2g_stalls.py
"""
import json
import os

import numpy as np
import pandas as pd

import extract_leg_metrics as E

OUT = os.path.join(E.HERE, 'output_p14_g2g')
DATA = os.path.join(E.REPO, 'trial_results_p14')


def main():
    t = pd.read_csv(os.path.join(OUT, 'p14_g2g_trials.csv'))
    rows = []
    for kind in ('a1_final', 'apf_rerun'):
        for r in t[(t['kind'] == kind) & (t['failure_type'] == 'timeout')].itertuples():
            name = r.unit if kind == 'a1_final' else 'APF'
            j = json.load(open(os.path.join(DATA, r.unit, 'timeseries', f'world_1_{name}_trial{r.trial_idx:03d}.json')))
            g = np.array([[x['t'], x['x'], x['y']] for x in j['gt_odom']])
            v = np.array([x['vx'] for x in j['cmd_vel']])
            tail = g[g[:, 0] > g[-1, 0] - 30]
            spread = float(np.hypot(*(tail[:, 1:].max(0) - tail[:, 1:].min(0))))
            v_end = float(v[-max(len(v) // 3, 1):].mean())
            rows.append(dict(kind=kind, unit=r.unit, trial_idx=r.trial_idx, goal_x=r.goal_x, goal_y=r.goal_y,
                             bearing_deg=float(np.degrees(np.arctan2(r.goal_y, r.goal_x))),
                             goal_dist=float(np.hypot(r.goal_x, r.goal_y)), spread_30s=spread, v_end=v_end,
                             stalled=bool(spread < 0.1 and v_end < 0.06),
                             at_start=bool(np.hypot(g[-1, 1], g[-1, 2]) < 0.6),
                             end_to_goal=float(np.hypot(g[-1, 1] - r.goal_x, g[-1, 2] - r.goal_y))))
    d = pd.DataFrame(rows)
    d.to_csv(os.path.join(OUT, 'p14_g2g_stalls.csv'), index=False, float_format='%.4g')
    print(d.groupby(['kind', 'unit']).agg(timeouts=('stalled', 'size'), stalled=('stalled', 'sum'),
                                          at_start=('at_start', 'sum')).to_string())


if __name__ == '__main__':
    main()
