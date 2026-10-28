#!/usr/bin/env python3
"""Paired comparison of two Gazebo check configs from their manifests.

usage: compare_configs.py X Y            (e.g. a b: how does Y differ from X)

Unit: one cell = one (policy, scenario) run. Both configs run the same 8 policies on the same 6 scenarios, one
run per cell (Gazebo reruns are identical, checked on 27 repeated jobs), so cells are paired by (model, world, path).
Metric per cell: waypoint ratio = online waypoints_completed / total_waypoints (the server's online count, not a
recount from poses). Only status 'ok' rows are used; a cell missing in either config is dropped.

Reported:
  1. cell level: number of cells where Y is worse / equal / better than X, mean paired difference, exact two-sided
     sign test on the non-tied cells
  2. cluster level, because cells of one policy or one scenario are not independent: per-policy mean differences
     (8 values), exact sign test over policies, 95% percentile bootstrap CI of the mean difference resampling
     policies, and the same resampling scenarios (6 values)
  3. full-sequence success (every goal reached) per config, McNemar exact test on the discordant cells
  4. failure types per config (collision / timeout) from the client logs
  5. per variant and tier (B = rank 1, M = median of the sim ranking) and per scenario means
"""
import csv
import math
import os
import re
import sys
from collections import Counter, defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = os.path.join(HERE, '..', '..', 'batch_runs', 'logs')
LAB = {'TD3_00391_800': ('delta_best', 'B'), 'TD3_00388_900': ('delta_best', 'M'),
       'TD3_00390_1000': ('delta_final', 'B'), 'TD3_00395_1000': ('delta_final', 'M'),
       'TD3_00399_900': ('direct_best', 'B'), 'TD3_00401_900': ('direct_best', 'M'),
       'TD3_00400_1000': ('direct_final', 'B'), 'TD3_00402_1000': ('direct_final', 'M')}
ORDER = sorted(LAB, key=lambda m: LAB[m])
rng = np.random.default_rng(0)


def load(cfg):
    rows = [r for r in csv.DictReader(open(os.path.join(HERE, f'manifest_{cfg}.csv'))) if r['status'] == 'ok']
    return {(r['model'], r['world'], r['path']): r for r in rows}      # a later ok row of a rerun replaces an earlier one


def ratio(r):
    return int(r['waypoints_completed']) / int(r['total_waypoints'])


def complete(r):
    return r['waypoints_completed'] == r['total_waypoints']


def fail_type(r):
    if complete(r):
        return 'complete'
    t = open(os.path.join(LOGS, os.path.basename(r['log_file'])), errors='ignore').read()
    m = re.findall(r'\[FAILED\] Stopped at waypoint \d+: (Obstacle hit|Goal timeout)', t)
    return 'collision' if m and m[-1].startswith('Obst') else 'timeout' if m else 'other'


def sign_test(neg, pos):
    n = neg + pos
    if n == 0:
        return 1.0
    k = min(neg, pos)
    return min(1.0, 2 * sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n)


def boot_ci(values, n=10000):
    v = np.asarray(values, float)
    means = [rng.choice(v, len(v)).mean() for _ in range(n)]
    return np.percentile(means, [2.5, 97.5])


def main(x, y):
    X, Y = load(x), load(y)
    cells = sorted(set(X) & set(Y), key=lambda k: (LAB[k[0]], k[1], k[2]))
    print(f'config ({x}) {len(X)} ok runs, config ({y}) {len(Y)} ok runs, {len(cells)} paired cells '
          f'(dropped: {len(set(X) ^ set(Y))})')
    d = np.array([ratio(Y[k]) - ratio(X[k]) for k in cells])
    worse, eq, better = int((d < 0).sum()), int((d == 0).sum()), int((d > 0).sum())
    print(f'\n1. cells: ({y}) worse than ({x}) in {worse}, equal in {eq}, better in {better}; '
          f'mean ({y}) - ({x}) = {d.mean():+.3f}; sign test p = {sign_test(worse, better):.2g}')

    by_pol = defaultdict(list)
    by_sc = defaultdict(list)
    for k, v in zip(cells, d):
        by_pol[k[0]].append(v)
        by_sc[(k[1], k[2])].append(v)
    pm = {m: float(np.mean(v)) for m, v in by_pol.items()}
    sm = {s: float(np.mean(v)) for s, v in by_sc.items()}
    lo, hi = boot_ci(list(pm.values()))
    lo2, hi2 = boot_ci(list(sm.values()))
    npos = sum(v > 0 for v in pm.values())
    nneg = sum(v < 0 for v in pm.values())
    print(f'2. policies: mean difference per policy, ({y}) worse in {nneg}, better in {npos} of {len(pm)}; '
          f'sign test p = {sign_test(nneg, npos):.2g}')
    print(f'   bootstrap 95% CI of the mean difference: resampling policies [{lo:+.3f}, {hi:+.3f}], '
          f'resampling scenarios [{lo2:+.3f}, {hi2:+.3f}]')

    cx = {k: complete(X[k]) for k in cells}
    cy = {k: complete(Y[k]) for k in cells}
    b = sum(cx[k] and not cy[k] for k in cells)
    c = sum(cy[k] and not cx[k] for k in cells)
    print(f'3. every goal reached: ({x}) {sum(cx.values())}/{len(cells)}, ({y}) {sum(cy.values())}/{len(cells)}; '
          f'discordant cells {b} only in ({x}), {c} only in ({y}); McNemar exact p = {sign_test(b, c):.2g}')

    print('4. outcomes: ' + '; '.join(f'({n}) ' + ', '.join(f'{k} {v}' for k, v in sorted(Counter(fail_type(R[k]) for k in cells).items()))
                                      for n, R in ((x, X), (y, Y))))

    print(f'\n5. per policy (variant, tier): mean waypoint ratio ({x}) -> ({y}), difference')
    for m in ORDER:
        if m in pm:
            a = np.mean([ratio(X[k]) for k in cells if k[0] == m])
            bb = np.mean([ratio(Y[k]) for k in cells if k[0] == m])
            print(f'   {LAB[m][0]:13s} {LAB[m][1]}  {m:15s} {a:.3f} -> {bb:.3f}  ({bb - a:+.3f})')
    print(f'   8-policy mean: {np.mean([ratio(X[k]) for k in cells]):.3f} -> {np.mean([ratio(Y[k]) for k in cells]):.3f}')
    print(f'\n   per scenario (world/path): ' + '  '.join(
        f'{s[0][-1]}/{s[1][-1]}: {np.mean([ratio(X[k]) for k in cells if k[1:] == s]):.2f}->{np.mean([ratio(Y[k]) for k in cells if k[1:] == s]):.2f}'
        for s in sorted(sm)))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
