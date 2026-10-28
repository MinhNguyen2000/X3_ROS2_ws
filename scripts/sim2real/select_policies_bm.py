#!/usr/bin/env python3
"""Seed selection protocol (P1): best (B) and median (M) policy per variant from the Gazebo multi-goal ranking.

Rule, fixed before any confirmatory hardware run:
  - source: scripts/batch_runs/aggregate_7_scenarios.csv (DRL runs, APF excluded)
  - drop world_4/path1 (6 scenarios remain; every model has n = 6)
  - per model: mean_wp_ratio = mean(waypoints_completed / total_waypoints), mean_path_len_m
  - rank within each variant (delta_best, delta_final, direct_best, direct_final) by highest
    mean_wp_ratio, then lowest mean_path_len_m
  - the pool is every model of the variant, no exclusions
  - B = rank 1, M = rank ceil((n + 1) / 2)

Usage: select_policies_bm.py [--csv PATH] [--out DIR]
Writes <out>/seed_selection_bm.csv (full ranking) and prints the picks.
"""
import argparse
import collections
import csv
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CSV = os.path.join(HERE, '..', 'batch_runs', 'aggregate_7_scenarios.csv')
VARIANTS = ['delta_best', 'delta_final', 'direct_best', 'direct_final']


def load_stats(path):
    rows = list(csv.DictReader(open(path)))
    rows = [r for r in rows if not (r['world'] == 'world_4' and r['path'] == 'path1')]
    rows = [r for r in rows if r['planner'] != 'apf']
    per_model = collections.defaultdict(list)
    for r in rows:
        per_model[(r['kind'], r['model'])].append(r)
    stats = {}
    for key, v in per_model.items():
        n = len(v)
        stats[key] = dict(
            n=n,
            wp=sum(int(r['waypoints_completed']) / int(r['total_waypoints']) for r in v) / n,
            path=sum(float(r['path_len_m']) for r in v) / n,
            success=sum(float(r['success']) for r in v) / n,
        )
    return stats


def rank_variant(stats, kind):
    cand = [(k[1], s) for k, s in stats.items() if k[0] == kind]
    cand.sort(key=lambda t: (-round(t[1]['wp'], 9), t[1]['path']))
    return cand


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default=DEFAULT_CSV)
    ap.add_argument('--out', default=HERE)
    args = ap.parse_args()

    stats = load_stats(args.csv)
    out_rows, picks = [], {}
    for kind in VARIANTS:
        ranked = rank_variant(stats, kind)
        n = len(ranked)
        m_rank = math.ceil((n + 1) / 2)
        for i, (model, s) in enumerate(ranked, 1):
            tier = 'B' if i == 1 else 'M' if i == m_rank else 'W' if i == n else ''
            out_rows.append(dict(variant=kind, rank=i, n_pool=n, tier=tier, model=model,
                                 n_scenarios=s['n'], mean_wp_ratio=f"{s['wp']:.4f}",
                                 mean_path_len_m=f"{s['path']:.3f}", mean_success=f"{s['success']:.4f}"))
        picks[kind] = (ranked[0][0], ranked[m_rank - 1][0], n, m_rank)

    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, 'seed_selection_bm.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(out_rows[0]))
        w.writeheader()
        w.writerows(out_rows)

    print(f'{"variant":<14}{"n":>3}  {"B (rank 1)":<16}{"M (rank)":<20}')
    for kind, (b, m, n, mr) in picks.items():
        print(f'{kind:<14}{n:>3}  {b:<16}{m} ({mr})')
    print('wrote', path)


if __name__ == '__main__':
    main()
