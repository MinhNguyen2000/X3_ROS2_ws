#!/usr/bin/env python3
"""Motion smoothness of the commanded velocities in the six multi-goal scenarios (paper table
table:APF_TD3_navigation_benchmark and the across-seed values in the Motion smoothness text).

Metrics come from scripts/viz/visualize_vel.py compute_accel (unchanged): finite-difference linear and
angular accelerations of the commanded v_x and omega_z over the recorded timestamps (about 50 Hz),
their mean absolute value and RMS, the mean |omega_z|, and the omega_z sign changes per second.

Each run is evaluated three ways (column `variant`):
  all          every sample, as compute_accel reports it
  excl_narrow  samples within -0.1 to +0.1 s of each goal arrival (and the run end) left out, which
               removes the step in the command when the robot stops at a goal
  excl_wide    samples within -0.5 to +2.0 s of each goal arrival left out
A goal arrival is the first recorded pose inside the goal tolerance of goal i, searched after arrival i-1.

Runs: every run of the six paper scenarios in scripts/batch_runs/aggregate_7_scenarios.csv (world_4/path1
excluded). `representative` marks the runs drawn in the multi-goal path figure: the median TD3_direct and
TD3_delta policies of scripts/metrics/multigoal_efficiency.py and the APF repeat closest to the median
path length (visualize_comparison_figures.pick_apf_run).

Writes to --out (default scripts/metrics/output):
  smoothness_per_run.csv         one row per run and variant
  smoothness_representative.csv  the representative runs only (the paper table)
  smoothness_aggregate.csv       mean and sample SD per scenario, planner and variant, over all runs
                                 (subset all_policies) or successful runs only (subset successful_only)

Ported from the 2026-10-27 job script (P9); it reproduces those CSVs. Usage: compute_smoothness.py [--out DIR]
"""
import argparse
import json
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'scripts', 'viz'))

import visualize_vel as vv                          # noqa: E402  metric definitions (unchanged)
import visualize_comparison_figures as vcf          # noqa: E402  APF run pick, run keys, scenario order
from viz_common import paths_json_path              # noqa: E402
from multigoal_efficiency import AGG_CSV, load_runs, representatives  # noqa: E402

RP = os.path.join(REPO, 'scripts', 'recorded_paths')
vv.SAVED_PATHS_DIR = RP                             # the module default points at scripts/scripts/recorded_paths
SCEN = [s for group in vcf.FIGURES.values() for s in group]
WINDOWS = {'narrow': (-0.1, 0.1), 'wide': (-0.5, 2.0)}   # s around each goal arrival (and run end)


def load(key):
    with open(os.path.join(RP, key + '.json')) as f:
        return json.load(f)


def arrivals(data, world, path):
    """Goal-switch times: first pose inside goal_tolerance of goal i, searched after arrival i-1."""
    goals = json.load(open(paths_json_path(world)))[f'{path}_{world}']
    tol = data.get('goal_tolerance') or 0.3
    po = data['poses']
    t = np.array([p['t'] for p in po]); x = np.array([p['x'] for p in po]); y = np.array([p['y'] for p in po])
    i0, out = 0, []
    for gx, gy in goals:
        d = np.hypot(x[i0:] - gx, y[i0:] - gy)
        hit = np.nonzero(d < tol)[0]
        if not len(hit):
            break
        i0 += hit[0]
        out.append(t[i0])
    vel_t_end = data['velocities'][-1]['t']
    if not out or abs(out[-1] - vel_t_end) > 0.5:
        out.append(vel_t_end)                       # treat the end of the record as a stop as well
    return np.array(out)


def metrics(data, world, path, window=None):
    r = vv.compute_accel(data['velocities'])
    s = r['stats']
    t, vyaw, ta, al, aa = r['t'], r['vyaw'], r['t_accel'], r['a_lin'], r['a_ang']
    if window is None:
        return dict(mean_abs_ax=s['mean_abs_a_lin'], rms_ax=s['rms_a_lin'],
                    mean_abs_alpha=s['mean_abs_a_ang'], rms_alpha=s['rms_a_ang'],
                    mean_abs_omega=float(np.mean(np.abs(vyaw))), mean_signed_omega=s['mean_signed_vyaw'],
                    osc_per_s=s['oscillations_per_s'], duration_s=float(t[-1] - t[0]),
                    rate_hz=s['sample_rate_hz'], n_arrivals=np.nan, excluded_frac=0.0)
    arr = arrivals(data, world, path)
    lo, hi = window

    def excl(tt):
        m = np.zeros(len(tt), bool)
        for a in arr:
            m |= (tt >= a + lo) & (tt <= a + hi)
        return m
    ka = ~excl(ta)                                  # accel samples (interval midpoints) kept
    kv = ~excl(t)                                   # velocity samples kept
    flips = np.abs(np.diff(np.sign(vyaw))) / 2      # same counting as compute_accel, per interval
    dt = np.diff(t)
    kept_dur = float(np.nansum(np.where(ka, dt, 0.0)))
    return dict(mean_abs_ax=float(np.nanmean(np.abs(al[ka]))), rms_ax=float(np.sqrt(np.nanmean(al[ka] ** 2))),
                mean_abs_alpha=float(np.nanmean(np.abs(aa[ka]))), rms_alpha=float(np.sqrt(np.nanmean(aa[ka] ** 2))),
                mean_abs_omega=float(np.mean(np.abs(vyaw[kv]))), mean_signed_omega=float(np.mean(vyaw[kv])),
                osc_per_s=float(flips[ka].sum() / kept_dur), duration_s=kept_dur,
                rate_hz=s['sample_rate_hz'], n_arrivals=len(arr), excluded_frac=float(1 - ka.mean()))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.join(HERE, 'output'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    df = pd.read_csv(AGG_CSV)
    reps = representatives(load_runs())
    chosen = {m: reps[(m, 'median')] for m in ('TD3_direct', 'TD3_delta')}
    apf = df[df.planner == 'apf']
    print('representative TD3 policies:', chosen)
    df6 = df[[(w, p) in SCEN for w, p in zip(df.world, df.path)]].copy()
    df6['key'] = df6.saved_file.map(vcf.run_key)
    df6['label'] = df6.planner.map({'apf': 'APF', 'direct': 'TD3_direct', 'delta': 'TD3_delta'})

    rows = []
    for _, r in df6.iterrows():
        data = load(r.key)
        rep_key = (vcf.pick_apf_run(apf, r.world, r.path) if r.label == 'APF'
                   else vcf.drl_run(df, chosen[r.label], r.world, r.path))
        for variant, win in [('all', None)] + [(f'excl_{k}', v) for k, v in WINDOWS.items()]:
            m = metrics(data, r.world, r.path, win)
            rows.append(dict(world=r.world, path=r.path, planner=r.label, model=r.model, kind=r.kind,
                             source=r.source, success=int(r.success), representative=(r.key == rep_key),
                             variant=variant, key=r.key, **m))
    per = pd.DataFrame(rows)
    per.to_csv(os.path.join(args.out, 'smoothness_per_run.csv'), index=False)

    rep = per[per.representative].copy()
    rep.to_csv(os.path.join(args.out, 'smoothness_representative.csv'), index=False)

    cols = ['mean_abs_ax', 'rms_ax', 'mean_abs_alpha', 'rms_alpha', 'mean_abs_omega', 'mean_signed_omega', 'osc_per_s']
    agg = []
    for subset, sel in [('all_policies', per), ('successful_only', per[per.success == 1])]:
        for (w, p, pl, var), g in sel.groupby(['world', 'path', 'planner', 'variant']):
            d = dict(subset=subset, world=w, path=p, planner=pl, variant=var, n=len(g))
            for c in cols:
                d[c + '_mean'] = g[c].mean(); d[c + '_sd'] = g[c].std(ddof=1)
            agg.append(d)
    pd.DataFrame(agg).to_csv(os.path.join(args.out, 'smoothness_aggregate.csv'), index=False)
    print('wrote smoothness_per_run.csv, smoothness_representative.csv, smoothness_aggregate.csv to', args.out)


if __name__ == '__main__':
    main()
