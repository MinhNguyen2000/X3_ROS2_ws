#!/usr/bin/env python3
"""Collision, timeout and SPL per leg from the Gazebo logs (P2), for goal-to-goal and multi-goal.

Goal-to-goal (G2G, Table spl_aggregate), one row per goal attempt:
  - TD3: trial_results/world_1_TD3_<id>_<ckpt>_*.csv, ids 386-395 delta, 396-405 direct;
    Final = _1000, Best = the checkpoint lists in G2G_analysis.ipynb (copied below).
  - APF: trial_results/APF_variance/run1..5/*.csv, one unit per run.
  - Start is the docking pose, odom (0, 0); P = distance_traveled_odom, as in G2G_analysis.ipynb.

Multi-goal (Table multigoal_success), one row per leg:
  - runs: scripts/batch_runs/aggregate_7_scenarios.csv; each run's goal_sequence_server log is
    found through the manifests by saved_file (v1 APF logs in batch_runs_v1_oct14/logs).
  - a leg runs from "Navigating to waypoint k" to "Waypoint k reached / failed" in the server
    log; its path length P and duration come from the recorded poses in that window, and its
    start is the robot pose at the start of the window (the B2 protocol in review-merged.md).
  - runs stop at the first failure (stop_on_fail), so later legs are 'not_attempted'.
  - world_4/path1 is kept in the per-leg and per-run outputs but excluded from the per-policy
    and summary outputs, as in the paper (6 scenarios).

Failure type: 'collision' if the planner message contains "Obstacle hit", 'timeout' if it
contains "timeout", else 'other'.

SPL per leg = S * L / max(L, P), with L either the straight-line distance (spl_straight, the
draft's definition) or the shortest collision-free path with clearance --radius (spl_geo,
shortest_path.py). Failed and not-attempted legs score 0. Run SPL = mean over all planned legs.

Unit of replication: the policy (seed x checkpoint) for TD3, the repeat for APF. Summary rows
give mean ± sample SD, median [Q1, Q3] and a 95% percentile bootstrap CI over these units.

Usage: extract_leg_metrics.py [--data-root DIR] [--out DIR] [--radius M] [--n-boot N]
--data-root holds scripts/ and trial_results/ (default: this repo; the batch logs are
gitignored, so point it at the checkout that has them).
"""
import argparse
import datetime
import glob
import json
import os
import re
import subprocess

import numpy as np
import pandas as pd

from shortest_path import OBSTACLE_TOLERANCE, ShortestPath

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
PATHS_DIR = os.path.join(REPO, 'src', 'x3_nav_bringup', 'paths')

# G2G Best checkpoints, from G2G_analysis.ipynb (TD3_direct_star_runs, TD3_delta_star_runs)
G2G_BEST = ['TD3_00396_500', 'TD3_00397_1000', 'TD3_00398_1000', 'TD3_00399_900', 'TD3_00400_700',
            'TD3_00401_900', 'TD3_00402_700', 'TD3_00403_900', 'TD3_00404_500', 'TD3_00405_1000',
            'TD3_00386_1000', 'TD3_00387_500', 'TD3_00388_900', 'TD3_00389_900', 'TD3_00390_800',
            'TD3_00391_800', 'TD3_00392_600', 'TD3_00393_900', 'TD3_00394_800', 'TD3_00395_900']
EXCLUDED_SCENARIOS = {('world_4', 'path1')}

LOG_RE = re.compile(r'^\[(\w+)\] \[(\d+\.\d+)\] \[[^\]]+\]: (.*)$')


def failure_type(msg):
    if not isinstance(msg, str) or not msg.strip():
        return ''
    if 'Obstacle hit' in msg:
        return 'collision'
    if 'timeout' in msg.lower():
        return 'timeout'
    return 'other'


def spl(success, L, P):
    return np.where(success, L / np.maximum(L, P), 0.0)


def code_commit():
    try:
        head = subprocess.run(['git', '-C', REPO, 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True, timeout=10).stdout.strip()
        dirty = subprocess.run(['git', '-C', REPO, 'status', '--porcelain', '--', 'scripts/metrics'],
                               capture_output=True, text=True, timeout=10).stdout.strip()
        return head + ('+dirty' if dirty else '')
    except Exception:
        return 'nogit'


# ----------------------------------------------------------------------------- G2G
def model_kind(model):
    seed = int(model.split('_')[1])
    return seed, 'delta' if seed <= 395 else 'direct'


def checkpoint_kinds(model):
    '''Rows a TD3 policy counts in: Final if _1000, Best if in G2G_BEST, both when they coincide.'''
    _, variant = model_kind(model)
    return [f'{variant}_{c}' for c, hit in (('final', model.endswith('_1000')),
                                            ('best', model in G2G_BEST)) if hit]


def load_g2g(root, sp):
    rows = []
    for path in sorted(glob.glob(os.path.join(root, 'trial_results', 'world_1_TD3_*.csv'))):
        model = re.match(r'world_1_(TD3_\d+_\d+)_\d{8}_\d{6}\.csv$', os.path.basename(path)).group(1)
        seed, variant = model_kind(model)
        if not 386 <= seed <= 405:
            continue
        df = pd.read_csv(path)
        for kind in checkpoint_kinds(model):
            rows.append(df.assign(planner=variant, kind=kind, unit=model, seed=seed))
    for path in sorted(glob.glob(os.path.join(root, 'trial_results', 'APF_variance', 'run*', '*.csv'))):
        run = re.search(r'(run\d+)', path).group(1)
        rows.append(pd.read_csv(path).assign(planner='apf', kind='apf', unit=f'APF_{run}', seed=np.nan))
    df = pd.concat(rows, ignore_index=True)
    df['success'] = df['success'].astype(bool)

    straight = np.hypot(df['goal_x'], df['goal_y'])
    assert np.allclose(straight, df['straight_line_distance'], atol=1e-3), 'G2G start is not odom (0, 0)'
    goals = df[['goal_x', 'goal_y']].drop_duplicates()
    geo = {(x, y): sp.length((0.0, 0.0), (x, y)) for x, y in goals.itertuples(index=False)}
    df['L_straight'] = df['straight_line_distance']
    # the CSV rounds straight_line_distance to 1 mm; the geodesic is never shorter than the line
    df['L_geo'] = np.maximum([geo[(x, y)][0] for x, y in zip(df['goal_x'], df['goal_y'])], df['L_straight'])
    df['geo_flags'] = [';'.join(geo[(x, y)][1]) for x, y in zip(df['goal_x'], df['goal_y'])]
    df['P'] = df['distance_traveled_odom']
    df['failure_type'] = np.where(df['success'], '', df['failure_reason'].map(failure_type))
    df['spl_straight'] = spl(df['success'], df['L_straight'], df['P'])
    df['spl_geo'] = spl(df['success'], df['L_geo'], df['P'])
    cols = ['kind', 'planner', 'unit', 'seed', 'trial_idx', 'goal_x', 'goal_y', 'success',
            'failure_type', 'failure_reason', 'travel_time', 'P', 'L_straight', 'L_geo',
            'geo_flags', 'spl_straight', 'spl_geo']
    return df[cols]


def g2g_units(trials):
    g = trials.groupby(['kind', 'planner', 'unit', 'seed'], dropna=False)
    out = g.agg(n_goals=('success', 'size'), success_rate=('success', 'mean'),
                spl_straight=('spl_straight', 'mean'), spl_geo=('spl_geo', 'mean'),
                n_collision=('failure_type', lambda s: (s == 'collision').sum()),
                n_timeout=('failure_type', lambda s: (s == 'timeout').sum()),
                n_other=('failure_type', lambda s: (s == 'other').sum())).reset_index()
    out['success_rate'] *= 100
    return out


# ----------------------------------------------------------------------------- multi-goal
def index_server_logs(root):
    '''saved_file -> server log path, from every manifest.'''
    log_dirs = [os.path.join(root, 'scripts', 'batch_runs', 'logs'),
                os.path.join(root, 'scripts', 'batch_runs_v1_oct14', 'logs')]
    index = {}
    for man in glob.glob(os.path.join(root, 'scripts', 'batch_runs*', 'manifest*.csv')):
        for r in pd.read_csv(man).itertuples():
            if not isinstance(r.saved_file, str) or not isinstance(r.log_file, str):
                continue
            name = os.path.basename(r.log_file).replace('_client.log', '_server.log')
            for d in log_dirs:
                if os.path.exists(os.path.join(d, name)):
                    index[r.saved_file] = os.path.join(d, name)
    return index


def parse_server_log(path):
    '''Start time and, per attempted leg: waypoint, t_start, t_end, outcome, message.'''
    start, legs = None, []
    for line in open(path, errors='replace'):
        m = LOG_RE.match(line.strip())
        if not m:
            continue
        ts, msg = float(m.group(2)), m.group(3)
        if msg.startswith('Start time:'):
            start = float(msg.split(':', 1)[1])
        elif (w := re.match(r'Navigating to waypoint\s+(\d+)/\s*(\d+): \(\s*([-\d.]+),\s*([-\d.]+)\)', msg)):
            legs.append(dict(leg=int(w.group(1)), n_legs=int(w.group(2)), wx=float(w.group(3)),
                             wy=float(w.group(4)), t_start=ts, t_end=np.nan, outcome='', message=''))
        elif (w := re.match(r'Waypoint (\d+)/\d+ reached', msg)):
            assert legs and legs[-1]['leg'] == int(w.group(1)), path
            legs[-1].update(t_end=ts, outcome='reached')
        elif (w := re.match(r'Waypoint (\d+)/\d+ failed: (.*)', msg)):
            assert legs and legs[-1]['leg'] == int(w.group(1)), path
            legs[-1].update(t_end=ts, outcome=failure_type(w.group(2)), message=w.group(2))
    assert start is not None, f'no start time in {path}'
    for leg in legs:
        leg['t_start'] -= start
        leg['t_end'] -= start
    return legs


def load_multigoal(root, sp_by_world):
    agg = pd.read_csv(os.path.join(root, 'scripts', 'batch_runs', 'aggregate_7_scenarios.csv'))
    logs = index_server_logs(root)
    paths = {}
    for f in glob.glob(os.path.join(PATHS_DIR, 'paths_world_*.json')):
        paths.update(json.load(open(f)))
    leg_rows = []
    for run in agg.itertuples():
        planned = paths[f'{run.path}_{run.world}']
        assert len(planned) == run.total_waypoints, run.saved_file
        legs = parse_server_log(logs[run.saved_file])
        poses = json.load(open(os.path.join(root, run.saved_file)))['poses']
        t = np.array([p['t'] for p in poses])
        xy = np.array([[p['x'], p['y']] for p in poses])
        sp = sp_by_world[run.world]
        unit = run.model if run.planner != 'apf' else f'APF_rep{run.rep}'
        seed = int(run.model.split('_')[1]) if run.planner != 'apf' else np.nan
        # the aggregate labels a coinciding Best/Final policy only as final; count it in both rows
        kinds = checkpoint_kinds(run.model) if run.planner != 'apf' else ['apf']
        assert run.kind in kinds, run.saved_file
        run_rows = []
        for k, (wx, wy) in enumerate(planned, 1):
            row = dict(kind=None, planner=run.planner, unit=unit, seed=seed, model=run.model,
                       world=run.world, path=run.path, rep=run.rep, source=run.source,
                       leg=k, n_legs=len(planned), wx=wx, wy=wy, outcome='not_attempted',
                       message='', t_start=np.nan, duration=np.nan, sx=np.nan, sy=np.nan,
                       P=np.nan, L_straight=np.nan, L_geo=np.nan, geo_flags='')
            if k <= len(legs):
                lg = legs[k - 1]
                assert (lg['leg'], lg['wx'], lg['wy']) == (k, wx, wy), run.saved_file
                win = (t >= lg['t_start']) & (t <= lg['t_end'])
                i0 = max(np.searchsorted(t, lg['t_start'], side='right') - 1, 0)
                seg = np.concatenate([xy[i0:i0 + 1], xy[win]])
                sx, sy = seg[0]
                L_geo, flags = sp.length((sx, sy), (wx, wy))
                row.update(outcome=lg['outcome'] or 'unknown', message=lg['message'],
                           t_start=lg['t_start'], duration=lg['t_end'] - lg['t_start'],
                           sx=sx, sy=sy, P=np.linalg.norm(np.diff(seg, axis=0), axis=1).sum(),
                           L_straight=float(np.hypot(wx - sx, wy - sy)),
                           L_geo=max(L_geo, float(np.hypot(wx - sx, wy - sy))),
                           geo_flags=';'.join(flags))
            run_rows.append(row)
        leg_rows += [dict(r, kind=kind) for kind in kinds for r in run_rows]
    legs = pd.DataFrame(leg_rows)
    ok = legs['outcome'] == 'reached'
    legs['spl_straight'] = spl(ok, legs['L_straight'].fillna(1), legs['P'].fillna(0))
    legs['spl_geo'] = spl(ok, legs['L_geo'].fillna(1), legs['P'].fillna(0))
    return agg, legs


def multigoal_runs(agg, legs):
    g = legs.groupby(['kind', 'planner', 'unit', 'seed', 'model', 'world', 'path', 'rep', 'source'],
                     dropna=False)
    runs = g[['outcome', 'leg', 'P', 'duration', 'spl_straight', 'spl_geo']].apply(lambda d: pd.Series(dict(
        n_legs=len(d),
        goals_reached=int((d['outcome'] == 'reached').sum()),
        legs_attempted=int((d['outcome'] != 'not_attempted').sum()),
        failure_type=next(iter(d.loc[~d['outcome'].isin(['reached', 'not_attempted']), 'outcome']), ''),
        failed_leg=next(iter(d.loc[~d['outcome'].isin(['reached', 'not_attempted']), 'leg']), np.nan),
        P_total=d['P'].sum(), time_total=d['duration'].sum(),
        spl_straight=d['spl_straight'].mean(), spl_geo=d['spl_geo'].mean(),
        spl_geo_reached=d.loc[d['outcome'] == 'reached', 'spl_geo'].mean()))).reset_index()
    runs['success'] = (runs['goals_reached'] == runs['n_legs']).astype(int)

    # cross-check against the aggregate the paper's Table multigoal_success is built from
    chk = runs.merge(agg[['model', 'world', 'path', 'rep', 'success', 'waypoints_completed',
                          'path_len_m']], on=['model', 'world', 'path', 'rep'], how='left',
                     suffixes=('', '_agg'), validate='many_to_one')
    assert (chk['success'] == chk['success_agg']).all(), 'success differs from aggregate'
    assert (chk['goals_reached'] == chk['waypoints_completed']).all(), 'goals differ from aggregate'
    runs['path_len_m_agg'] = chk['path_len_m'].to_numpy()
    return runs


def multigoal_units(legs, runs):
    keep = ~runs.set_index(['world', 'path']).index.isin(list(EXCLUDED_SCENARIOS))
    r = runs[keep]
    lk = legs[~legs.set_index(['world', 'path']).index.isin(list(EXCLUDED_SCENARIOS))]
    att = lk[lk['outcome'] != 'not_attempted']
    g = r.groupby(['kind', 'planner', 'unit', 'seed'], dropna=False)
    out = g.agg(n_runs=('success', 'size'), run_success=('success', 'mean'),
                goals_reached=('goals_reached', 'sum'), goals_planned=('n_legs', 'sum'),
                spl_straight=('spl_straight', 'mean'), spl_geo=('spl_geo', 'mean')).reset_index()
    ga = att.groupby(['kind', 'unit'])
    reached = att[att['outcome'] == 'reached'].groupby(['kind', 'unit'])
    per = pd.DataFrame(dict(
        legs_attempted=ga.size(),
        n_collision=ga['outcome'].apply(lambda s: (s == 'collision').sum()),
        n_timeout=ga['outcome'].apply(lambda s: (s == 'timeout').sum()),
        n_other=ga['outcome'].apply(lambda s: (~s.isin(['reached', 'collision', 'timeout'])).sum()),
        spl_geo_reached=reached['spl_geo'].mean(),
        spl_straight_reached=reached['spl_straight'].mean(),
    )).reset_index()
    out = out.merge(per, on=['kind', 'unit'])
    out['run_success'] *= 100
    out['goal_completion'] = 100 * out['goals_reached'] / out['goals_planned']
    out['collisions_per_100_legs'] = 100 * out['n_collision'] / out['legs_attempted']
    out['timeouts_per_100_legs'] = 100 * out['n_timeout'] / out['legs_attempted']
    return out


# ----------------------------------------------------------------------------- summary
def summarize(units, metrics, n_boot, rng):
    rows = []
    for kind, d in units.groupby('kind'):
        for m in metrics:
            x = d[m].dropna().to_numpy(float)
            q1, med, q3 = np.percentile(x, [25, 50, 75])
            boot = rng.choice(x, size=(n_boot, len(x)), replace=True).mean(axis=1)
            lo, hi = np.percentile(boot, [2.5, 97.5])
            rows.append(dict(kind=kind, metric=m, n=len(x), mean=x.mean(),
                             sd=x.std(ddof=1) if len(x) > 1 else np.nan,
                             median=med, q1=q1, q3=q3, ci95_lo=lo, ci95_hi=hi))
    return pd.DataFrame(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--data-root', default=REPO)
    ap.add_argument('--out', default=os.path.join(HERE, 'output'))
    ap.add_argument('--radius', type=float, default=OBSTACLE_TOLERANCE)
    ap.add_argument('--n-boot', type=int, default=10000)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rng = np.random.default_rng(0)

    worlds = ['world_1', 'world_2', 'world_3', 'world_4', 'world_5']
    sp_by_world = {w: ShortestPath(w, args.radius) for w in worlds}

    g2g = load_g2g(args.data_root, sp_by_world['world_1'])
    g2g_u = g2g_units(g2g)
    agg, legs = load_multigoal(args.data_root, sp_by_world)
    runs = multigoal_runs(agg, legs)
    mg_u = multigoal_units(legs, runs)

    outputs = {
        'g2g_trials.csv': g2g,
        'g2g_units.csv': g2g_u,
        'g2g_summary.csv': summarize(g2g_u, ['success_rate', 'spl_straight', 'spl_geo',
                                             'n_collision', 'n_timeout'], args.n_boot, rng),
        'multigoal_legs.csv': legs,
        'multigoal_runs.csv': runs,
        'multigoal_units.csv': mg_u,
        'multigoal_summary.csv': summarize(mg_u, ['run_success', 'goal_completion',
                                                  'collisions_per_100_legs', 'timeouts_per_100_legs',
                                                  'spl_straight', 'spl_geo', 'spl_geo_reached'],
                                           args.n_boot, rng),
    }
    for name, df in outputs.items():
        df.to_csv(os.path.join(args.out, name), index=False, float_format='%.6g')
    meta = dict(script='scripts/metrics/extract_leg_metrics.py', code_commit=code_commit(),
                generated=datetime.datetime.now().isoformat(timespec='seconds'),
                data_root=os.path.abspath(args.data_root), radius_m=args.radius,
                n_boot=args.n_boot, bootstrap_rng_seed=0,
                excluded_scenarios=sorted('/'.join(s) for s in EXCLUDED_SCENARIOS),
                outputs=sorted(outputs))
    json.dump(meta, open(os.path.join(args.out, 'meta.json'), 'w'), indent=2)

    print(f"radius {args.radius} m | commit {meta['code_commit']} | wrote {len(outputs)} CSVs to {args.out}")
    print('\nG2G failures (all candidate policies / APF runs, pooled):')
    print(g2g[~g2g['success']].groupby('kind')['failure_type'].value_counts().unstack(fill_value=0))
    print('\nMulti-goal run outcomes (6 scenarios):')
    r6 = runs[~runs.set_index(['world', 'path']).index.isin(list(EXCLUDED_SCENARIOS))]
    print(r6.groupby('kind').agg(runs=('success', 'size'), success=('success', 'sum'),
                                 collision=('failure_type', lambda s: (s == 'collision').sum()),
                                 timeout=('failure_type', lambda s: (s == 'timeout').sum())))
    flagged = (g2g['geo_flags'] != '').sum(), (legs['geo_flags'] != '').sum()
    print(f'\nend points inside the inflated map: G2G {flagged[0]} trials, multi-goal {flagged[1]} legs')


if __name__ == '__main__':
    main()
