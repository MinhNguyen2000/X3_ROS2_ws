#!/usr/bin/env python3
"""P14 goal-to-goal metrics: the A1 policies and the same-code reruns of APF and TD3_direct Final.

Reads trial_results_p14/<unit>/world_1_*.csv (written by scripts/run_p14_g2g.sh) and computes the goal-to-goal
measures of extract_leg_metrics.py (shortest-path SPL from the odom origin, failure types, per-unit aggregation,
median [Q1, Q3] and bootstrap summary), so the numbers are defined as in Table spl_aggregate.

Kinds:
  - a1_final: TD3_011<seed>_1500; a1_best: the MuJoCo-selected checkpoint per seed (BEST below); a policy that
    is both counts in both rows, as extract_leg_metrics.py does;
  - apf_rerun: APF_run1..5, one unit per repeat; direct_final_rerun: TD3_00396_1000 to TD3_00405_1000.
The published units (scripts/metrics/output/g2g_units.csv, kinds apf and direct_final) are added to the summary
as apf_paper and direct_final_paper when that file exists. Units with an incomplete CSV are left out and listed.

Usage: scripts/metrics/p14_g2g.py [--out scripts/metrics/output_p14_g2g] [--n-boot 10000]
"""
import argparse
import datetime
import glob
import json
import os

import numpy as np
import pandas as pd

import extract_leg_metrics as E
from shortest_path import OBSTACLE_TOLERANCE, ShortestPath

DATA = os.path.join(E.REPO, 'trial_results_p14')
N_GOALS = 139
BEST = {'101': '1300', '102': '1500', '103': '1400'}     # notes/a1_eval_protocol_20261106.md, eval seed 11


def kinds_of(unit):
    if unit.startswith('APF_'):
        return 'apf', ['apf_rerun']
    _, sid, ckpt = unit.split('_')
    if sid.startswith('011'):
        seed = sid[2:]
        return 'a1', [k for k, hit in (('a1_final', ckpt == '1500'), ('a1_best', BEST.get(seed) == ckpt)) if hit]
    return 'direct', ['direct_final_rerun'] if ckpt == '1000' else []


def load_trials(sp):
    rows, incomplete = [], []
    for d in sorted(glob.glob(os.path.join(DATA, '*'))):
        unit = os.path.basename(d)
        if unit == 'aborted' or not os.path.isdir(d):
            continue
        csvs = sorted(glob.glob(os.path.join(d, 'world_1_*.csv')))
        df = pd.read_csv(csvs[-1]) if csvs else None
        if df is None or len(df) != N_GOALS:
            incomplete.append(unit)
            continue
        planner, kinds = kinds_of(unit)
        rows += [df.assign(planner=planner, kind=k, unit=unit, seed=np.nan) for k in kinds]
    df = pd.concat(rows, ignore_index=True)
    df['success'] = df['success'].astype(bool)
    straight = np.hypot(df['goal_x'], df['goal_y'])
    assert np.allclose(straight, df['straight_line_distance'], atol=1e-3), 'G2G start is not odom (0, 0)'
    goals = df[['goal_x', 'goal_y']].drop_duplicates()
    geo = {(x, y): sp.length((0.0, 0.0), (x, y)) for x, y in goals.itertuples(index=False)}
    df['L_straight'] = df['straight_line_distance']
    df['L_geo'] = np.maximum([geo[(x, y)][0] for x, y in zip(df['goal_x'], df['goal_y'])], df['L_straight'])
    df['geo_flags'] = [';'.join(geo[(x, y)][1]) for x, y in zip(df['goal_x'], df['goal_y'])]
    df['P'] = df['distance_traveled_odom']
    df['failure_type'] = np.where(df['success'], '', df['failure_reason'].map(E.failure_type))
    df['spl_straight'] = E.spl(df['success'], df['L_straight'], df['P'])
    df['spl_geo'] = E.spl(df['success'], df['L_geo'], df['P'])
    cols = ['kind', 'planner', 'unit', 'seed', 'trial_idx', 'goal_x', 'goal_y', 'success', 'failure_type',
            'failure_reason', 'travel_time', 'P', 'L_straight', 'L_geo', 'geo_flags', 'spl_straight', 'spl_geo']
    return df[cols], incomplete


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default=os.path.join(E.HERE, 'output_p14_g2g'))
    ap.add_argument('--n-boot', type=int, default=10000)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rng = np.random.default_rng(0)

    trials, incomplete = load_trials(ShortestPath('world_1', OBSTACLE_TOLERANCE))
    units = E.g2g_units(trials)
    # SPL over successful goals only, as a secondary efficiency measure
    units = units.merge(trials[trials['success']].groupby(['kind', 'unit'])['spl_geo'].mean()
                        .rename('spl_geo_success').reset_index(), on=['kind', 'unit'], how='left')

    metrics = ['success_rate', 'n_collision', 'n_timeout', 'n_other', 'spl_geo', 'spl_geo_success', 'spl_straight']
    summary, units_all = E.summarize(units, metrics, args.n_boot, rng), units
    published = os.path.join(E.HERE, 'output', 'g2g_units.csv')
    if os.path.exists(published):    # the published units have no spl_geo_success
        pub = pd.read_csv(published)
        pub = pub[pub['kind'].isin(['apf', 'direct_final'])].copy()
        pub['kind'] = pub['kind'] + '_paper'
        pub = pub[units.columns.intersection(pub.columns)]
        units_all = pd.concat([units, pub], ignore_index=True)
        summary = pd.concat([summary, E.summarize(pub, [m for m in metrics if m in pub.columns], args.n_boot, rng)],
                            ignore_index=True)

    for name, df in {'p14_g2g_trials.csv': trials, 'p14_g2g_units.csv': units_all,
                     'p14_g2g_summary.csv': summary}.items():
        df.to_csv(os.path.join(args.out, name), index=False, float_format='%.6g')
    meta = dict(script='scripts/metrics/p14_g2g.py', code_commit=E.code_commit(),
                generated=datetime.datetime.now().isoformat(timespec='seconds'), data=DATA,
                complete_units=sorted(trials['unit'].unique().tolist()), incomplete_units=incomplete,
                radius_m=OBSTACLE_TOLERANCE, n_boot=args.n_boot, bootstrap_rng_seed=0, best_checkpoints=BEST)
    json.dump(meta, open(os.path.join(args.out, 'meta.json'), 'w'), indent=2)

    print(json.dumps({k: meta[k] for k in ('code_commit', 'complete_units', 'incomplete_units')}))
    pd.set_option('display.width', 200)
    s = summary.assign(v=lambda d: d.apply(lambda r: f"{r['median']:.4g} [{r['q1']:.4g}, {r['q3']:.4g}]", axis=1))
    print(s.pivot(index='kind', columns='metric', values='v')[[m for m in metrics if m in set(s['metric'])]]
          .to_string())
    print(units.sort_values(['kind', 'unit'])[['kind', 'unit', 'n_goals', 'success_rate', 'n_collision',
                                                'n_timeout', 'n_other', 'spl_geo']].to_string(index=False))


if __name__ == '__main__':
    main()
