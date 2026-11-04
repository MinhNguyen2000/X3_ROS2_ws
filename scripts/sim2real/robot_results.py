#!/usr/bin/env python3
"""Result table and progress report of the confirmatory real-robot runs (P7), from robot/manifest_robot.csv and the run JSONs.

Scoring rule (personal_hub notes/scoring_rule_decision_20261028.md): the score of a run is the server's online goal count
(`result.waypoints_completed` of `total_waypoints`, "goals reached according to the robot's own odometry"); full-sequence success is
`result.success`; failure type comes from `result.message`; path length is the summed step length of `poses` (not `total_distance`) and
time is `elapsed_time`. No external verification of the odometry exists. Only kept attempts count (`is_kept` of run_robot_schedule.py).

Inputs : --manifest (default robot/manifest_robot.csv), --root (the workspace that `saved_file` is relative to).
Outputs (--out, default robot/): robot_runs.csv (one row per kept run), robot_units.csv (policy x path and policy pooled),
         table_robot.tex (policy rows, both paths pooled), table_robot_path.tex (policy x path rows), robot_meta.json.
Partial data are labelled with the number of runs per policy; the script prints the progress against schedule.csv first.

Usage: robot_results.py [--manifest CSV] [--root DIR] [--schedule CSV] [--out DIR]
"""
import argparse
import collections
import csv
import datetime
import hashlib
import json
import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import run_robot_schedule as rrs  # noqa: E402

# row order of table:deployed_policies (make_tables.py): variant blocks, B before M
VARIANTS = ['direct_final', 'direct_best', 'delta_final', 'delta_best']
LABEL = {'direct_final': 'TD3\\_direct Final', 'direct_best': 'TD3\\_direct Best',
         'delta_final': 'TD3\\_delta Final', 'delta_best': 'TD3\\_delta Best'}
PATHS = ['path1_world_lab', 'path2_world_lab']


def failure_type(result):
    if result.get('success'):
        return 'none'
    msg = (result.get('message') or '').lower()
    if 'obstacle hit' in msg:
        return 'collision'
    if 'timeout' in msg:
        return 'timeout'
    return 'other'


def path_length(poses):
    """Summed step length of the logged odometry poses (m)."""
    if not poses or len(poses) < 2:
        return 0.0
    xy = np.array([[p['x'], p['y']] for p in poses], float)
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())


def kept_runs(manifest):
    """Last kept attempt per run number."""
    out = {}
    for r in manifest:
        if rrs.is_kept(r):
            out[int(r['run'])] = r
    return out


def run_record(row, root):
    d = json.load(open(os.path.join(root, row['saved_file'])))
    res = d.get('result') or {}
    return dict(run=int(row['run']), stretch=row['stretch'], block=row['block'], variant=row['variant'], tier=row['tier'],
                model=row['model'], path=row['path'], goals=int(res['waypoints_completed']), total=int(res['total_waypoints']),
                success=int(bool(res['success'])), failure=failure_type(res), time_s=float(d.get('elapsed_time', float('nan'))),
                path_m=path_length(d.get('poses')), battery_v=row.get('battery_v', ''), saved_file=row['saved_file'])


def quart(v):
    v = np.asarray(v, float)
    if not len(v):
        return float('nan'), float('nan'), float('nan')
    return tuple(float(x) for x in np.percentile(v, [50, 25, 75]))


def unit_row(rs, **key):
    ok = [r for r in rs if r['success']]
    n = len(rs)
    g, t = sum(r['goals'] for r in rs), sum(r['total'] for r in rs)
    pm, pq1, pq3 = quart([r['path_m'] for r in ok])
    tm, tq1, tq3 = quart([r['time_s'] for r in ok])
    gm, gq1, gq3 = quart([r['goals'] / r['total'] * 100 for r in rs])
    return dict(key, n_runs=n, n_success=len(ok), goals_pct=100 * g / t if t else float('nan'), goals_pct_median=gm, goals_pct_q1=gq1,
                goals_pct_q3=gq3, collisions=sum(r['failure'] == 'collision' for r in rs), timeouts=sum(r['failure'] == 'timeout' for r in rs),
                other=sum(r['failure'] == 'other' for r in rs), path_median=pm, path_q1=pq1, path_q3=pq3, time_median=tm, time_q1=tq1,
                time_q3=tq3)


def build_units(recs):
    units = []
    for v in VARIANTS:
        for tier in ('B', 'M'):
            sel = [r for r in recs if r['variant'] == v and r['tier'] == tier]
            if not sel:
                continue
            base = dict(variant=v, tier=tier, model=sel[0]['model'])
            units.append(unit_row(sel, path='both', **base))
            for p in PATHS:
                s = [r for r in sel if r['path'] == p]
                if s:
                    units.append(unit_row(s, path=p, **base))
    return units


def fmt_iqr(m, a, b, nd=1):
    return '--' if math.isnan(m) else f'{m:.{nd}f} [{a:.{nd}f}, {b:.{nd}f}]'


def tex_tables(units):
    def block(rows, header, spec):
        out = [rf'\begin{{tabular}}{{{spec}}}', r'\midrule[1.5pt]', header, r'\midrule[1.5pt]']
        prev = None
        for v, line in rows:
            if prev is not None and v != prev:
                out.append(r'\midrule')
            out.append(line)
            prev = v
        return '\n'.join(out + [r'\midrule[1.5pt]', r'\end{tabular}', ''])

    both = [u for u in units if u['path'] == 'both']
    t1 = [(u['variant'], f"{LABEL[u['variant']]} & {u['tier']} & {u['model'].replace('_', chr(92) + '_')} & {u['n_runs']} & "
           f"{u['n_success']}/{u['n_runs']} & {u['goals_pct']:.1f} & {u['collisions']} & {u['timeouts']} & {u['other']} \\\\") for u in both]
    h1 = (r'\textbf{Variant} & \textbf{Role} & \textbf{Policy} & \textbf{Runs} & \textbf{Full sequence} & \textbf{Goals (\%)} & '
          r'\textbf{Coll.} & \textbf{Timeouts} & \textbf{Other} \\')
    per = [u for u in units if u['path'] != 'both']
    t2 = [(u['variant'], f"{LABEL[u['variant']]} & {u['tier']} & {u['path'][:5]} & {u['n_runs']} & {u['n_success']}/{u['n_runs']} & "
           f"{fmt_iqr(u['path_median'], u['path_q1'], u['path_q3'], 2)} & {fmt_iqr(u['time_median'], u['time_q1'], u['time_q3'], 1)} \\\\")
          for u in per]
    h2 = (r'\textbf{Variant} & \textbf{Role} & \textbf{Path} & \textbf{Runs} & \textbf{Full sequence} & '
          r'\textbf{Path length (m)} & \textbf{Time (s)} \\')
    return block(t1, h1, 'lllcccccc'), block(t2, h2, 'lllccll')


def progress(schedule, manifest, kept):
    print(f'{len(schedule)} scheduled runs, {len(kept)} kept, {len(manifest)} attempts, {len(manifest) - len(kept)} not kept')
    want = collections.Counter((s['variant'], s['tier'], s['path']) for s in schedule)
    have = collections.Counter((r['variant'], r['tier'], r['path']) for r in kept.values())
    for v in VARIANTS:
        for tier in ('B', 'M'):
            cells = '  '.join(f"{p[:5]} {have[(v, tier, p)]}/{want[(v, tier, p)]}" for p in PATHS)
            print(f'  {v:13s} {tier}  {cells}')
    faults = collections.Counter(r['fault'] for r in manifest if r['fault'])
    anomalies = {k: collections.Counter(r[k] for r in manifest) for k in ('status', 'config_ok', 'start_ok', 'dirty', 'code')}
    print('  faults:', dict(faults) or 'none', '| anomalies:', {k: dict(v) for k, v in anomalies.items()})


def git_head():
    try:
        return subprocess.check_output(['git', '-C', HERE, 'rev-parse', '--short', 'HEAD'], text=True).strip()
    except Exception:
        return 'unknown'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--manifest', default=rrs.MANIFEST)
    ap.add_argument('--root', default=rrs.WS, help='directory that saved_file is relative to')
    ap.add_argument('--schedule', default=rrs.SCHEDULE)
    ap.add_argument('--out', default=rrs.OUT_DIR)
    a = ap.parse_args()
    manifest, schedule = rrs.load_manifest(a.manifest), rrs.load_schedule(a.schedule)
    kept = kept_runs(manifest)
    progress(schedule, manifest, kept)
    recs = [run_record(r, a.root) for _, r in sorted(kept.items())]
    units = build_units(recs)
    os.makedirs(a.out, exist_ok=True)
    for name, rows in (('robot_runs.csv', recs), ('robot_units.csv', units)):
        with open(os.path.join(a.out, name), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    t1, t2 = tex_tables(units)
    open(os.path.join(a.out, 'table_robot.tex'), 'w').write(t1)
    open(os.path.join(a.out, 'table_robot_path.tex'), 'w').write(t2)
    json.dump(dict(date=datetime.date.today().isoformat(), script_commit=git_head(), n_kept=len(kept), n_scheduled=len(schedule),
                   complete=len(kept) == len(schedule), manifest=os.path.abspath(a.manifest),
                   manifest_sha1=hashlib.sha1(open(a.manifest, 'rb').read()).hexdigest(),
                   run_code={r['code'] for r in kept.values()} and sorted({r['code'] for r in kept.values()})),
              open(os.path.join(a.out, 'robot_meta.json'), 'w'), indent=1)
    print('wrote', a.out)


if __name__ == '__main__':
    main()
