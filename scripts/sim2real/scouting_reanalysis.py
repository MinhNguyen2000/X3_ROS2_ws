#!/usr/bin/env python3
"""Re-analysis of the 2026-10-13..15 real-robot scouting runs (P1).

For every scripts/recorded_paths/<model>/path{1,2}_world_lab_<DDMMYY>_<HHMM>.json:
  - k_odom: goals reached according to `poses` (the policy's own odometry; this is what the goal-sequence
            server counted online), sequential, tolerance 0.3 m
  - k_gt:   goals reached according to `poses_gt` (ceiling camera tracker, only when recorded), mapped into
            the start-relative frame the waypoints are defined in:
            p_odom_frame = R(-yaw0) (p_gt - p_gt[0]), with yaw0 = 2 asin(logged "yaw") because the recorder
            stores the quaternion z component under that key
  - pipeline signature: day, max |vyaw| and |vx| of the commanded velocities, first commanded vx (a delta
    policy's first step: 0.01 = normalized actions at 0.2 m/s, 0.05 = increments applied in m/s, 0.2 = saturated)
  - closest approach of the gt track to the first goal it missed

Joins the per-model sim ranking (seed_selection_bm.csv) and, optionally, the hand-entered scores of the old
scouting note, and prints strata summaries. Writes scouting_runs.csv.

Usage: scouting_reanalysis.py [--note PATH_TO_seed_selection_result_20261027_2328.md]
"""
import argparse
import csv
import datetime
import glob
import json
import os
import re
from collections import Counter, defaultdict

import numpy as np
from scipy.stats import spearmanr

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..'))
WAYPOINTS = os.path.join(WS, 'src', 'x3_nav_bringup', 'paths', 'paths_world_lab.json')
RUNS = os.path.join(WS, 'scripts', 'recorded_paths')
TOL = 0.3


def goals_reached(xy, waypoints, tol=TOL):
    k, closest = 0, np.inf
    for p in xy:
        if k == len(waypoints):
            break
        d = np.hypot(*(p - waypoints[k]))
        closest = min(closest, d)
        if d <= tol:
            k, closest = k + 1, np.inf
    return k, (closest if k < len(waypoints) else float('nan'))


def gt_to_start_frame(g):
    x0, y0, th = g[0, 1], g[0, 2], g[0, 3]
    c, s = np.cos(-th), np.sin(-th)
    d = g[:, 1:3] - [x0, y0]
    return np.c_[c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]]


def analyse_run(f, wp_all):
    d = json.load(open(f))
    base = os.path.basename(f)
    path = base[:5]
    W = np.array(wp_all[path + '_world_lab'])
    odom = np.array([[q['x'], q['y']] for q in d['poses']]) if d['poses'] else np.zeros((0, 2))
    gt = np.array([[q['t'], q['x'], q['y'], q['yaw']] for q in d['poses_gt']]) if d['poses_gt'] else None
    if gt is not None:
        # goal_sequence_server logs `orientation.z` of the camera pose under the key "yaw": the sine of half the
        # yaw, not the angle. For a planar pose yaw = 2 asin(z).
        gt[:, 3] = 2.0 * np.arcsin(np.clip(gt[:, 3], -1.0, 1.0))
    vel = d['velocities'] or []
    stamp = datetime.datetime.strptime(base[:-5].split('_', 3)[3], '%d%m%y_%H%M')
    k_odom, _ = goals_reached(odom, W)
    row = dict(model=f.split(os.sep)[-2], path=path, day=stamp.strftime('%d'), time=stamp.strftime('%H:%M'),
               elapsed_s=round(d['elapsed_time'], 1), n_goals=len(W), k_odom=k_odom,
               vyaw_cap=round(max((abs(v['vyaw']) for v in vel), default=0.0), 2),
               vx_cap=round(max((abs(v['vx']) for v in vel), default=0.0), 2),
               first_vx=round(vel[0]['vx'], 3) if vel else '', has_gt=gt is not None,
               k_gt='', gt_closest_to_missed_m='', odom_gt_end_err_m='', fit_rms_m='', fit_rot_deg='', fit_extent_ratio='', start_map_rms_m='')
    if gt is not None and len(gt) > 1:
        g = gt_to_start_frame(gt)
        k_gt, closest = goals_reached(g, W)
        row.update(k_gt=k_gt, gt_closest_to_missed_m=round(closest, 2) if k_gt < len(W) else '',
                   odom_gt_end_err_m=round(float(np.hypot(*(g[-1] - odom[-1]))), 2))
        fit = fit_gt_to_odom(np.array([q['t'] for q in d['poses']]), odom, gt)
        if fit:
            row.update(fit_rms_m=round(fit[0], 3), fit_rot_deg=round(fit[1], 1), fit_extent_ratio=round(fit[2], 3),
                       start_map_rms_m=round(fit[3], 3))
    return row


def fit_gt_to_odom(odom_t, odom_xy, gt):
    """Best-fit rigid transform (Kabsch) of the gt track onto the odometry track over the common time span.
    Returns (rms residual [m], fitted rotation minus start-yaw rotation [deg], gt/odom extent ratio,
    rms residual [m] of the start-pose mapping used for the goal counts) or None."""
    tg = gt[:, 0] - gt[0, 0]
    to = odom_t - odom_t[0]
    m = tg <= to[-1]
    if m.sum() < 20:
        return None
    G = gt[m, 1:3]
    O = np.c_[np.interp(tg[m], to, odom_xy[:, 0]), np.interp(tg[m], to, odom_xy[:, 1])]
    if np.ptp(O[:, 0]) + np.ptp(O[:, 1]) < 0.8:
        return None
    Gc, Oc = G - G.mean(0), O - O.mean(0)
    U, _, Vt = np.linalg.svd(Gc.T @ Oc)
    if np.linalg.det(Vt.T @ U.T) < 0:
        Vt[-1] *= -1
    R = Vt.T @ U.T
    rms = float(np.sqrt(np.mean(np.sum(((R @ Gc.T).T - Oc) ** 2, 1))))
    th = gt[0, 3]
    A = G - G[0]
    A = np.c_[np.cos(-th) * A[:, 0] - np.sin(-th) * A[:, 1], np.sin(-th) * A[:, 0] + np.cos(-th) * A[:, 1]]
    rms_start = float(np.sqrt(np.mean(np.sum((A - O) ** 2, 1))))
    rot = float(np.degrees(np.arctan2(R[1, 0], R[0, 0]) + gt[0, 3]))
    ext = float(np.linalg.norm(Gc, axis=1).sum() / max(np.linalg.norm(Oc, axis=1).sum(), 1e-9))
    return rms, (rot + 180) % 360 - 180, ext, rms_start


def parse_old_note(path):
    """model -> (rw_p1, rw_p2, rw_score) from the hand-entered table of the 2026-10-27 scouting note."""
    out = {}
    for line in open(path):
        m = re.match(r'\|\s*\d+\s*\|[^|]*\|\s*\**(TD3_\d+_\d+)\**\s*\|(?:[^|]*\|){3}\s*(\d+)\s*\|\s*(\d+)\s*\|\s*([\d.]+)\s*\|', line)
        if m:
            out[m.group(1)] = (int(m.group(2)), int(m.group(3)), float(m.group(4)))
    return out


def model_scores(rows, key, path):
    """model -> mean goal fraction over the runs on `path`, using column `key` (k_odom / k_gt)."""
    acc = defaultdict(list)
    for r in rows:
        if r['path'] == path and r[key] != '':
            acc[r['model']].append(r[key] / r['n_goals'])
    return {m: float(np.mean(v)) for m, v in acc.items()}


def rho(sim, real, models):
    ms = [m for m in models if m in sim and m in real]
    if len(ms) < 6:
        return f'n={len(ms)} (too few)'
    r, p = spearmanr([sim[m] for m in ms], [real[m] for m in ms])
    return f'rho={r:+.2f} p={p:.2f} n={len(ms)}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--note', default=None, help='old scouting note, to test which count it recorded')
    ap.add_argument('--out', default=os.path.join(HERE, 'scouting_runs.csv'))
    args = ap.parse_args()

    wp_all = json.load(open(WAYPOINTS))
    rows = [analyse_run(f, wp_all) for f in sorted(glob.glob(os.path.join(RUNS, '*', 'path*_world_lab_*.json')))]
    rows.sort(key=lambda r: (r['day'], r['time']))
    with open(args.out, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    sim_rows = list(csv.DictReader(open(os.path.join(HERE, 'seed_selection_bm.csv'))))
    sim = {r['model']: float(r['mean_wp_ratio']) for r in sim_rows}
    variant = {r['model']: r['variant'] for r in sim_rows}

    print(f'{len(rows)} runs, {sum(r["has_gt"] for r in rows)} with poses_gt, '
          f'{len({r["model"] for r in rows})} model dirs, {len(sim)} models in the sim ranking')
    print('runs per day / has_gt:', sorted(Counter((r['day'], r['has_gt']) for r in rows).items()))
    print('first commanded vx, delta models only, by day:',
          sorted(Counter((r['day'], r['first_vx']) for r in rows if variant.get(r['model'], '').startswith('delta')).items()))
    print('vyaw cap by day:', sorted(Counter((r['day'], r['vyaw_cap']) for r in rows if r['vyaw_cap'] in (0.3, 0.4)).items()))

    for p in ('path1', 'path2'):
        done = [r['elapsed_s'] for r in rows if r['path'] == p and r['k_odom'] == r['n_goals']]
        print(f'{p}: {len(done)} runs where the odometry count reached every goal, median elapsed {np.median(done):.0f} s '
              f'(all {sum(r["path"] == p for r in rows)} runs: median {np.median([r["elapsed_s"] for r in rows if r["path"] == p]):.0f} s)')

    both = [r for r in rows if r['k_gt'] != '']
    print(f'\nodom vs start-yaw-aligned camera gt, {len(both)} runs with both: '
          f'mean k_odom - k_gt = {np.mean([r["k_odom"] - r["k_gt"] for r in both]):+.2f} goals; '
          f'odom counts more goals than gt in {sum(r["k_odom"] > r["k_gt"] for r in both)}, '
          f'fewer in {sum(r["k_odom"] < r["k_gt"] for r in both)}; '
          f'median end-position disagreement {np.median([r["odom_gt_end_err_m"] for r in both]):.2f} m')
    for p in ('path1', 'path2'):
        sub = [r for r in both if r['path'] == p]
        print(f'  {p}: k_gt histogram {sorted(Counter(r["k_gt"] for r in sub).items())}, '
              f'k_odom histogram {sorted(Counter(r["k_odom"] for r in sub).items())}')
        miss = [r['gt_closest_to_missed_m'] for r in sub if r['gt_closest_to_missed_m'] != '']
        if miss:
            print(f'  {p}: closest approach to the first missed goal (gt): median {np.median(miss):.2f} m, '
                  f'{sum(m <= 0.5 for m in miss)}/{len(miss)} within 0.5 m')

    fits = [r for r in rows if r['fit_rms_m'] != '']
    print(f'\nframe check, {len(fits)} runs with a usable gt track: RMS odom-vs-gt after best-fit rotation+translation '
          f'median {np.median([r["fit_rms_m"] for r in fits]):.2f} m (start-pose mapping used for the counts: '
          f'{np.median([r["start_map_rms_m"] for r in fits]):.2f} m); fitted rotation minus start-yaw alignment '
          f'median {np.median([r["fit_rot_deg"] for r in fits]):+.1f} deg (IQR {np.percentile([r["fit_rot_deg"] for r in fits], 25):+.1f}'
          f'..{np.percentile([r["fit_rot_deg"] for r in fits], 75):+.1f}); gt/odom extent ratio '
          f'median {np.median([r["fit_extent_ratio"] for r in fits]):.2f}')

    print('\nsim wp_ratio vs per-model mean goal fraction on one path (Spearman, models with >=1 run)')
    strata = (('all runs', lambda r: True),
              ('vyaw cap 0.3', lambda r: r['vyaw_cap'] == 0.3),
              ('vyaw cap 0.4', lambda r: r['vyaw_cap'] == 0.4))
    variants = ('delta_best', 'delta_final', 'direct_best', 'direct_final')
    for key, label in (('k_odom', 'odom'), ('k_gt', 'gt')):
        for path in ('path1', 'path2'):
            for name, sel in strata:
                real = model_scores([r for r in rows if sel(r)], key, path)
                print(f'  {label:<4}{path} {name:<13} pooled {rho(sim, real, sim)}')
                print('      ' + ' | '.join(f'{v} {rho(sim, real, [m for m in sim if variant[m] == v])}' for v in variants))

    print('\ndelta models, first commanded vx vs mean goal fraction (a normalized 0.2 m/s controller starts at 0.01)')
    for path in ('path1', 'path2'):
        for fv in sorted({r['first_vx'] for r in rows if variant.get(r['model'], '').startswith('delta') and r['path'] == path}):
            sub = [r for r in rows if variant.get(r['model'], '').startswith('delta') and r['path'] == path and r['first_vx'] == fv]
            g = [r['k_gt'] / r['n_goals'] for r in sub if r['k_gt'] != '']
            print(f'  {path} first_vx={fv}: {len(sub)} runs over {len({r["model"] for r in sub})} models, '
                  f'odom goal fraction {np.mean([r["k_odom"] / r["n_goals"] for r in sub]):.2f}, '
                  f'gt goal fraction {np.mean(g) if g else float("nan"):.2f} (n={len(g)}), '
                  f'median elapsed {np.median([r["elapsed_s"] for r in sub]):.0f} s')

    if args.note:
        old = parse_old_note(args.note)
        per = defaultdict(lambda: defaultdict(set))
        for r in rows:
            per[r['model']][(r['path'], 'odom')].add(r['k_odom'])
            if r['k_gt'] != '':
                per[r['model']][(r['path'], 'gt')].add(r['k_gt'])
        print(f'\nold note: {len(old)} models parsed; does the recorded count appear among the runs of that model?')
        for src in ('odom', 'gt'):
            hit = tot = 0
            for m, (p1, p2, _) in old.items():
                for path, val in (('path1', p1), ('path2', p2)):
                    if (path, src) in per[m]:
                        tot += 1
                        hit += val in per[m][(path, src)]
            print(f'  {src}: {hit}/{tot} model-path entries have a run with exactly that count')


if __name__ == '__main__':
    main()
