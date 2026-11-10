#!/usr/bin/env python3
"""Motion smoothness of the commanded velocities in the six multi-goal scenarios (paper table
table:APF_TD3_navigation_benchmark and the across-seed values in the Motion smoothness text).

Metrics come from scripts/viz/visualize_vel.py compute_accel (unchanged): finite-difference linear and
angular accelerations of the commanded v_x and omega_z over the recorded timestamps (about 50 Hz),
their mean absolute value and RMS, the mean |omega_z|, and the omega_z sign changes per second (osc_per_s,
every sign change). reversals_per_s counts a turning-direction reversal only when omega_z crosses from above
+OMEGA_BAND to below -OMEGA_BAND or back (0.05 rad/s), so small commands dithering around zero do not count;
the paper uses this count.

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
  table_smoothness.tex           tabular body of table:APF_TD3_navigation_benchmark (representative runs,
                                 variant TABLE_VARIANT = excl_narrow, reversals_per_s in the last column)
  smoothness_summary.txt         the ranges and ratios the paper text quotes, for variants all and excl_narrow, and
                                 the per-run ordering between planners over successful runs (variant excl_narrow)

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
vv.SAVED_PATHS_DIR = RP
SCEN = [s for group in vcf.FIGURES.values() for s in group]
WINDOWS = {'narrow': (-0.1, 0.1), 'wide': (-0.5, 2.0)}   # s around each goal arrival (and run end)
# A turning-direction reversal counts only when omega_z crosses from above +OMEGA_BAND to below -OMEGA_BAND
# (or back), so small commands that dither around zero are not counted; 5 percent of the 1 rad/s limit.
OMEGA_BAND = 0.05
TABLE_VARIANT = 'excl_narrow'      # the paper table leaves out the goal-stop transients
PLANNERS = ['APF', 'TD3_direct', 'TD3_delta']


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


def reversal_times(t, w, band=OMEGA_BAND):
    """Times at which omega_z completes a reversal across the band (+band to -band or back)."""
    out, side = [], 0
    for ti, wi in zip(t, w):
        s = 1 if wi > band else (-1 if wi < -band else 0)
        if s:
            if side and s != side:
                out.append(ti)
            side = s
    return np.array(out)


def metrics(data, world, path, window=None):
    r = vv.compute_accel(data['velocities'])
    s = r['stats']
    t, vyaw, ta, al, aa = r['t'], r['vyaw'], r['t_accel'], r['a_lin'], r['a_ang']
    t, vyaw = np.asarray(t), np.asarray(vyaw)
    rev = reversal_times(t, vyaw)
    if window is None:
        dur = float(t[-1] - t[0])
        return dict(mean_abs_ax=s['mean_abs_a_lin'], rms_ax=s['rms_a_lin'],
                    mean_abs_alpha=s['mean_abs_a_ang'], rms_alpha=s['rms_a_ang'],
                    mean_abs_omega=float(np.mean(np.abs(vyaw))), mean_signed_omega=s['mean_signed_vyaw'],
                    osc_per_s=s['oscillations_per_s'], reversals_per_s=len(rev) / dur, duration_s=dur,
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
                osc_per_s=float(flips[ka].sum() / kept_dur),
                reversals_per_s=float((~excl(rev)).sum() / kept_dur) if len(rev) else 0.0, duration_s=kept_dur,
                rate_hz=s['sample_rate_hz'], n_arrivals=len(arr), excluded_frac=float(1 - ka.mean()))


def representative_keys(df: pd.DataFrame) -> dict:
    """{(world, path, planner): run key} of the runs drawn in the multi-goal path figure, for the six scenarios:
    the median TD3_direct and TD3_delta policies and the APF repeat closest to the median path length.
    Also used by scripts/viz/regen_vel_figures.py, so the appendix velocity figures show the same runs."""
    reps = representatives(load_runs())
    chosen = {m: reps[(m, 'median')] for m in ('TD3_direct', 'TD3_delta')}
    print('representative TD3 policies:', chosen)
    apf = df[df.planner == 'apf']
    out = {}
    for world, path in SCEN:
        out[(world, path, 'APF')] = vcf.pick_apf_run(apf, world, path)
        for pl, model in chosen.items():
            out[(world, path, pl)] = vcf.drl_run(df, model, world, path)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.join(HERE, 'output'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    df = pd.read_csv(AGG_CSV)
    rep_keys = representative_keys(df)
    df6 = df[[(w, p) in SCEN for w, p in zip(df.world, df.path)]].copy()
    df6['key'] = df6.saved_file.map(vcf.run_key)
    df6['label'] = df6.planner.map({'apf': 'APF', 'direct': 'TD3_direct', 'delta': 'TD3_delta'})

    rows = []
    for _, r in df6.iterrows():
        data = load(r.key)
        rep_key = rep_keys[(r.world, r.path, r.label)]
        for variant, win in [('all', None)] + [(f'excl_{k}', v) for k, v in WINDOWS.items()]:
            m = metrics(data, r.world, r.path, win)
            rows.append(dict(world=r.world, path=r.path, planner=r.label, model=r.model, kind=r.kind,
                             source=r.source, success=int(r.success), representative=(r.key == rep_key),
                             variant=variant, key=r.key, **m))
    per = pd.DataFrame(rows)
    per.to_csv(os.path.join(args.out, 'smoothness_per_run.csv'), index=False)

    rep = per[per.representative].copy()
    rep.to_csv(os.path.join(args.out, 'smoothness_representative.csv'), index=False)

    cols = ['mean_abs_ax', 'rms_ax', 'mean_abs_alpha', 'rms_alpha', 'mean_abs_omega', 'mean_signed_omega', 'osc_per_s',
            'reversals_per_s']
    agg = []
    for subset, sel in [('all_policies', per), ('successful_only', per[per.success == 1])]:
        for (w, p, pl, var), g in sel.groupby(['world', 'path', 'planner', 'variant']):
            d = dict(subset=subset, world=w, path=p, planner=pl, variant=var, n=len(g))
            for c in cols:
                d[c + '_mean'] = g[c].mean(); d[c + '_sd'] = g[c].std(ddof=1)
            agg.append(d)
    agg = pd.DataFrame(agg)
    agg.to_csv(os.path.join(args.out, 'smoothness_aggregate.csv'), index=False)

    with open(os.path.join(args.out, 'table_smoothness.tex'), 'w') as f:
        f.write(table_tex(rep))
    summary = text_summary(rep, agg) + ordering_summary(per)
    with open(os.path.join(args.out, 'smoothness_summary.txt'), 'w') as f:
        f.write(summary)
    print(summary)
    print('wrote smoothness_per_run.csv, smoothness_representative.csv, smoothness_aggregate.csv, '
          'table_smoothness.tex, smoothness_summary.txt to', args.out)


TABLE_COLS = [('mean_abs_ax', 3), ('rms_ax', 3), ('mean_abs_alpha', 3), ('rms_alpha', 3), ('mean_abs_omega', 3),
              ('reversals_per_s', 2)]


def table_tex(rep: pd.DataFrame) -> str:
    """Tabular body of table:APF_TD3_navigation_benchmark: representative runs, TABLE_VARIANT samples."""
    r = rep[rep.variant == TABLE_VARIANT].set_index(['world', 'path', 'planner'])
    lines = [r'\begin{tabular}{llcccccc}', r'\midrule[1.5pt]',
             r'\textbf{Scenario} & \textbf{Planner} & \makecell{$\overline{|a_x|}$\\(m/s$^2$)} & '
             r'\makecell{$a_{x,\text{RMS}}$\\(m/s$^2$)} & \makecell{$\overline{|\alpha_z|}$\\(rad/s$^2$)} & '
             r'\makecell{$\alpha_{z,\text{RMS}}$\\(rad/s$^2$)} & \makecell{$\overline{|\omega_z|}$\\(rad/s)} & '
             r'\makecell{$\omega_z$ reversals\\per s} \\', r'\midrule[1.5pt]']
    for k, (world, path) in enumerate(SCEN):
        name = f"{vcf.DISPLAY_WORLD.get(world, world).replace('_', ' ').capitalize()}, {path.replace('path', 'path ')}"
        for i, pl in enumerate(PLANNERS):
            x = r.loc[(world, path, pl)]
            head = (r'\multirow{3}{*}{' + name + '}') if i == 0 else ''
            lines.append(f"{head} & {pl.replace('_', chr(92) + '_')} & "
                         + ' & '.join(f'{x[c]:.{d}f}' for c, d in TABLE_COLS) + r' \\')
        lines.append(r'\midrule' if k < len(SCEN) - 1 else r'\midrule[1.5pt]')
    lines += [r'\end{tabular}', '']
    return '\n'.join(lines)


def text_summary(rep: pd.DataFrame, agg: pd.DataFrame) -> str:
    """Ranges over the six scenarios that the paper text quotes, per planner and variant."""
    out = [f'Ranges over the six scenarios (min to max); reversals use the {OMEGA_BAND} rad/s band.', '']
    for var in ['all', TABLE_VARIANT]:
        out.append(f'Representative runs, variant {var}:')
        r = rep[rep.variant == var]
        for pl in PLANNERS:
            g = r[r.planner == pl]
            out.append(f'  {pl:10s} ' + '  '.join(
                f'{c} {g[c].min():.{d}f} to {g[c].max():.{d}f}' for c, d in TABLE_COLS + [('osc_per_s', 2)]))
        for c in ['reversals_per_s', 'mean_abs_alpha', 'rms_alpha', 'rms_ax']:
            ratio = r.pivot_table(index=['world', 'path'], columns='planner', values=c)
            for num, den in [('TD3_direct', 'APF'), ('TD3_delta', 'APF'), ('TD3_direct', 'TD3_delta')]:
                q = ratio[num] / ratio[den]
                out.append(f'  {c} {num}/{den} per scenario: {q.min():.2f} to {q.max():.2f}')
        out.append('')
        a = agg[(agg.subset == 'successful_only') & (agg.variant == var)]
        out.append(f'Successful runs of all final and best checkpoints (APF: 5 repeats), variant {var}, mean +- sample SD per scenario:')
        for pl in PLANNERS:
            g = a[a.planner == pl]
            for c, d in [('rms_alpha', 2), ('reversals_per_s', 2), ('rms_ax', 2)]:
                lo = g.loc[g[c + '_mean'].idxmin()]; hi = g.loc[g[c + '_mean'].idxmax()]
                out.append(f'  {pl:10s} {c}: {lo[c + "_mean"]:.{d}f} +- {lo[c + "_sd"]:.{d}f} to '
                           f'{hi[c + "_mean"]:.{d}f} +- {hi[c + "_sd"]:.{d}f}  (n {int(g.n.min())} to {int(g.n.max())})')
        out.append('')
    return '\n'.join(out)


def ordering_summary(per: pd.DataFrame) -> str:
    """For each metric and planner pair (A, B), the number of scenarios in which every successful run of A is above
    every successful run of B (min over A > max over B), for TABLE_VARIANT. Lets the text state an ordering that
    holds for all seeds and checkpoints without quoting statistics."""
    d = per[(per.variant == TABLE_VARIANT) & (per.success == 1)]
    n = d.groupby('planner').size()
    out = [f'Per-run ordering over successful runs of all final and best checkpoints (APF: 5 repeats), variant {TABLE_VARIANT}:',
           '  runs per planner over the six scenarios: ' + ', '.join(f'{pl} {n[pl]}' for pl in PLANNERS),
           '  "A > B k/6" means every run of A is above every run of B in k of the six scenarios']
    pairs = [('TD3_direct', 'APF'), ('TD3_direct', 'TD3_delta'), ('TD3_delta', 'APF'), ('APF', 'TD3_delta')]
    for c, _ in TABLE_COLS:
        k = {pr: 0 for pr in pairs}
        for _, g in d.groupby(['world', 'path']):
            for a, b in pairs:
                k[(a, b)] += int(g[g.planner == a][c].min() > g[g.planner == b][c].max())
        out.append(f'  {c:16s} ' + '  '.join(f'{a} > {b} {k[(a, b)]}/6' for a, b in pairs))
    out.append('')
    return '\n'.join(out)


if __name__ == '__main__':
    main()
