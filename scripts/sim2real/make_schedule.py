#!/usr/bin/env python3
"""Run schedule for the confirmatory real-robot runs (P1 / P7).

Policies: the B and M picks per variant from seed_selection_bm.csv (select_policies_bm.py), 8 in total.
Design: REPS blocks. In every block each policy runs once on each lab path. Policy order is shuffled per
block with a fixed seed; the policy node is loaded once per policy and both paths run back to back, with
the path order alternating between blocks. RUNS_PER_STRETCH consecutive runs form one data-collection stretch.

Usage: make_schedule.py [--picks CSV] [--reps 10] [--seed 20261028] [--out schedule.csv]
"""
import argparse
import csv
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))
PATHS = ['path1_world_lab', 'path2_world_lab']
RUNS_PER_STRETCH = 40


def load_picks(path):
    picks = []
    for r in csv.DictReader(open(path)):
        if r['tier'] in ('B', 'M'):
            picks.append((r['variant'], r['tier'], r['model']))
    return picks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--picks', default=os.path.join(HERE, 'seed_selection_bm.csv'))
    ap.add_argument('--reps', type=int, default=10)
    ap.add_argument('--seed', type=int, default=20261028)
    ap.add_argument('--out', default=os.path.join(HERE, 'schedule.csv'))
    args = ap.parse_args()

    picks = load_picks(args.picks)
    rng = random.Random(args.seed)
    rows = []
    for block in range(1, args.reps + 1):
        order = picks[:]
        rng.shuffle(order)
        paths = PATHS if block % 2 == 1 else PATHS[::-1]
        for variant, tier, model in order:
            for path in paths:
                rows.append(dict(run=len(rows) + 1, block=block, variant=variant, tier=tier,
                                 model=model, path=path))
    for r in rows:
        r['stretch'] = (r['run'] - 1) // RUNS_PER_STRETCH + 1

    fields = ['run', 'stretch', 'block', 'variant', 'tier', 'model', 'path']
    with open(args.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    print(f'{len(picks)} policies x {len(PATHS)} paths x {args.reps} blocks = {len(rows)} runs, '
          f'{rows[-1]["stretch"]} stretches of {RUNS_PER_STRETCH}; wrote {args.out}')


if __name__ == '__main__':
    main()
