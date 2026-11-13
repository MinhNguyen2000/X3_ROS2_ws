#!/usr/bin/env python3
"""Per-policy results of the Gazebo repeats of the 8 deployed policies (P2), from a batch manifest and its run JSONs.

Job B (--job lab):      world_lab/path1 and path2, 10 repeats, frozen deployment config; shown next to the robot values of
                        robot/robot_policy_stats.csv (goals_pct, spl_all, spl_reached).
Job A (--job standard): the six standard multigoal scenarios, 5 repeats, original batch config; shown next to the one-repeat
                        values of the original batch (scripts/metrics/output/multigoal_units.csv).

Per policy: runs, success runs (all legs reached), goal completion (goals reached / goals planned), collisions and timeouts per
100 attempted legs (attempted = reached legs + the failed leg; failure type from result.message of the run JSON, as in
robot_results.failure_type), SPL over all planned legs (failed and not attempted legs score 0) and over reached legs.
SPL per leg = L / max(L, P): L = shortest collision-free path (scripts/metrics/shortest_path.py, clearance 0.21 m, the world's .sdf) from
the pose where the leg started to the goal, P = travelled length of the leg; leg boundaries from the poses (first pose within goal
tolerance + 0.01 m of each goal), as robot_analysis.py. Job A also reports the subset without world_4/path1, which the original
batch tables exclude (multigoal_units.csv).

Usage: analyze_gazebo_repeats.py --job lab|standard [--manifest CSV] [--out DIR]
Outputs (default scripts/sim2real/gazebo_repeats/): <job>_runs.csv, <job>_policies.csv, <job>_summary.json
"""
import argparse
import csv
import json
import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.normpath(os.path.join(HERE, '..', '..'))
for sub in ('', '../viz', '../metrics'):
    sys.path.insert(0, os.path.normpath(os.path.join(HERE, sub)))
import shortest_path as sps  # noqa: E402
import viz_common as vc  # noqa: E402

GOAL_SLACK = 0.01
ROBOT_STATS = os.path.join(HERE, 'robot', 'robot_policy_stats.csv')
OLD_UNITS = os.path.join(WS, 'scripts', 'metrics', 'output', 'multigoal_units.csv')
OUT = os.path.join(HERE, 'gazebo_repeats')
MANIFESTS = {'lab': os.path.join(OUT, 'manifest_world_lab.csv'), 'standard': os.path.join(OUT, 'manifest_standard.csv')}
EXCLUDED = {('world_4', 'path1')}


def failure_type(result):
    if result.get('success'):
        return 'none'
    msg = (result.get('message') or '').lower()
    if 'obstacle hit' in msg:
        return 'collision'
    if 'timeout' in msg:
        return 'timeout'
    return 'other'


def find_boundaries(x, y, goals, tol, margin=GOAL_SLACK):
    """Index of the first pose within tol + margin of each goal, searched after the previous one (closest approach otherwise)."""
    bounds, start = [], 0
    for gx, gy in goals:
        d = np.hypot(x[start:] - gx, y[start:] - gy)
        hit = np.flatnonzero(d <= tol + margin)
        i = start + (int(hit[0]) if hit.size else int(np.argmin(d)))
        bounds.append(i)
        start = i
    return bounds


def leg_spl(x, y, goals, k, tol, shortest):
    """SPL of each of the k reached legs; shortest(p, q) gives the shortest-path length in the odom frame."""
    if k == 0:
        return []
    out, start = [], 0
    for i, e in enumerate(find_boundaries(x, y, goals[:k], tol)):
        P = float(np.hypot(np.diff(x[start:e + 1]), np.diff(y[start:e + 1])).sum())
        L = shortest((x[start], y[start]), goals[i])
        out.append(L / max(L, P, 1e-9))
        start = e
    return out


class Worlds:
    """Goal lists and shortest-path planners per world, in the odom frame of the planners."""

    def __init__(self):
        self.sp, self.shift, self.paths = {}, {}, {}

    def get(self, world):
        if world not in self.sp:
            cfg = vc.world_config(world)
            self.shift[world] = (cfg['odom_init'][0] - sps.SPAWN_XY[0], cfg['odom_init'][1] - sps.SPAWN_XY[1])
            self.sp[world] = sps.ShortestPath(world)
            with open(vc.paths_json_path(world)) as f:
                self.paths[world] = {k: np.array(v, float) for k, v in json.load(f).items()}
        return self.sp[world], self.shift[world], self.paths[world]

    def shortest_fn(self, world):
        sp, (dx, dy), _ = self.get(world)
        return lambda p, q: sp.length((p[0] + dx, p[1] + dy), (q[0] + dx, q[1] + dy))[0]


def run_record(row, worlds, root=WS):
    d = json.load(open(os.path.join(root, row['saved_file'])))
    res = d.get('result') or {}
    world, path = row['world'], row['path']
    goals = worlds.get(world)[2][f'{path}_{world}']
    poses = [p for p in d['poses'] if np.isfinite(p.get('x', np.nan)) and np.isfinite(p.get('y', np.nan))]
    x, y = np.array([p['x'] for p in poses]), np.array([p['y'] for p in poses])
    k, total = int(res['waypoints_completed']), int(res['total_waypoints'])
    legs = leg_spl(x, y, goals, k, d.get('goal_tolerance') or 0.3, worlds.shortest_fn(world))
    fail = failure_type(res)
    return dict(model=row['model'], world=world, path=path, rep=int(row['rep']), success=int(bool(res['success'])),
                goals=k, total=total, legs_attempted=k + (0 if res.get('success') else 1), failure=fail,
                spl_all_sum=float(sum(legs)), spl_reached_sum=float(sum(legs)),
                time_s=float(d.get('elapsed_time', float('nan'))), saved_file=row['saved_file'])


def load_runs(manifest, worlds):
    rows = list(csv.DictReader(open(manifest)))
    ok = [r for r in rows if r['status'] == 'ok']
    recs = [run_record(r, worlds) for r in ok]
    return rows, recs


def policy_table(recs, models):
    """models: {model: (variant, tier)}. One row per policy."""
    out = []
    for m, (v, t) in models.items():
        rs = [r for r in recs if r['model'] == m]
        if not rs:
            continue
        att = sum(r['legs_attempted'] for r in rs)
        reached = sum(r['goals'] for r in rs)
        out.append(dict(
            variant=v, tier=t, model=m, n_runs=len(rs), n_success=sum(r['success'] for r in rs),
            goals_pct=100 * reached / sum(r['total'] for r in rs),
            collisions_per_100_legs=100 * sum(r['failure'] == 'collision' for r in rs) / att,
            timeouts_per_100_legs=100 * sum(r['failure'] == 'timeout' for r in rs) / att,
            n_collision=sum(r['failure'] == 'collision' for r in rs), n_timeout=sum(r['failure'] == 'timeout' for r in rs),
            n_other=sum(r['failure'] == 'other' for r in rs),
            spl_all=sum(r['spl_all_sum'] / r['total'] for r in rs) / len(rs),
            spl_reached=(sum(r['spl_reached_sum'] for r in rs) / reached) if reached else float('nan')))
    return out


def determinism(recs):
    '''Run-to-run variation inside each (policy, scenario) cell: cells, cells whose repeats all end the same way (goals reached
    and failure type), and the largest spread of the sequence time in a cell [s].'''
    cells = {}
    for r in recs:
        cells.setdefault((r['model'], r['world'], r['path']), []).append(r)
    same = sum(len({(x['goals'], x['failure']) for x in v}) == 1 for v in cells.values())
    spread = max(max(x['time_s'] for x in v) - min(x['time_s'] for x in v) for v in cells.values())
    return dict(cells=len(cells), cells_identical_outcome=same, max_time_spread_s=spread,
                reps_per_cell_min=min(len(v) for v in cells.values()), reps_per_cell_max=max(len(v) for v in cells.values()))


def rank_by_goals(rows):
    """1 = best; ties share the lowest rank (competition ranking)."""
    return {r['model']: 1 + sum(o['goals_pct'] > r['goals_pct'] + 1e-9 for o in rows) for r in rows}


def git_head():
    try:
        return subprocess.run(['git', '-C', WS, 'rev-parse', '--short', 'HEAD'], capture_output=True, text=True).stdout.strip()
    except Exception:
        return 'nogit'


def fnum(x):
    return '' if x is None or (isinstance(x, float) and math.isnan(x)) else x


def write_csv(path, rows):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--job', required=True, choices=['lab', 'standard'])
    ap.add_argument('--manifest')
    ap.add_argument('--out', default=OUT)
    a = ap.parse_args()
    manifest = a.manifest or MANIFESTS[a.job]
    os.makedirs(a.out, exist_ok=True)
    robot = {r['model']: r for r in csv.DictReader(open(ROBOT_STATS))}
    models = {m: (r['variant'], r['tier']) for m, r in robot.items()}
    worlds = Worlds()
    rows, recs = load_runs(manifest, worlds)
    stamps = sorted(r['timestamp'] for r in rows if r['timestamp'])
    summary = dict(job=a.job, manifest=os.path.relpath(manifest, WS), analysis_commit=git_head(),
                   code_states=sorted({r['code_state'] for r in rows}), manifest_rows=len(rows),
                   status_counts={s: sum(r['status'] == s for r in rows) for s in sorted({r['status'] for r in rows})},
                   first_timestamp=stamps[0] if stamps else '', last_timestamp=stamps[-1] if stamps else '',
                   sequence_time_h=sum(float(r['elapsed_s'] or 0) for r in rows) / 3600)
    summary['determinism'] = determinism(recs)
    write_csv(os.path.join(a.out, f'{a.job}_runs.csv'), recs)
    pol = policy_table(recs, models)
    if a.job == 'lab':
        for r in pol:
            rb = robot[r['model']]
            r.update(robot_goals_pct=float(rb['goals_pct']), robot_spl_all=float(rb['spl_all']),
                     robot_spl_reached=float(rb['spl_reached']), robot_n_success=int(rb['n_success']), robot_n_runs=int(rb['n_runs']))
        summary['gazebo_mean_spl_reached'] = float(np.mean([r['spl_reached'] for r in pol]))
        summary['robot_mean_spl_reached'] = float(np.mean([r['robot_spl_reached'] for r in pol]))
    else:
        sub = [r for r in recs if (r['world'], r['path']) not in EXCLUDED]
        pol_sub = policy_table(sub, models)
        old = {}
        for r in csv.DictReader(open(OLD_UNITS)):
            if r['unit'] in models and r['kind'] in ('delta_final', 'delta_best', 'direct_final', 'direct_best'):
                old[r['unit']] = r
        rk_new, rk_old = rank_by_goals(pol_sub), None
        old_rows = [dict(model=m, goals_pct=float(o['goal_completion'])) for m, o in old.items()]
        rk_old = rank_by_goals(old_rows)
        for r in pol_sub:
            o = old.get(r['model'], {})
            r.update(rank_goals=rk_new[r['model']], old_goals_pct=float(o.get('goal_completion', 'nan')),
                     old_rank_goals=rk_old.get(r['model']), old_collisions_per_100_legs=float(o.get('collisions_per_100_legs', 'nan')),
                     old_spl_geo=float(o.get('spl_geo', 'nan')), old_spl_geo_reached=float(o.get('spl_geo_reached', 'nan')))
        write_csv(os.path.join(a.out, 'standard_policies_without_world_4.csv'), pol_sub)
        summary['ranking_without_world_4'] = {r['model']: dict(variant=r['variant'], tier=r['tier'], rank=r['rank_goals'],
                                                               old_rank=r['old_rank_goals']) for r in pol_sub}
    write_csv(os.path.join(a.out, f'{a.job}_policies.csv'), pol)
    json.dump(summary, open(os.path.join(a.out, f'{a.job}_summary.json'), 'w'), indent=1)
    for r in pol:
        print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})


if __name__ == '__main__':
    main()
