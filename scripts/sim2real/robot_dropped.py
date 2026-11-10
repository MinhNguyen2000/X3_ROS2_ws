#!/usr/bin/env python3
"""Attempts of the confirmatory robot runs that are not in robot/manifest_robot.csv (P7): characterisation and sensitivity of the result.

During stretches 2 to 4 the obstacle stop was triggered by spurious LiDAR points (phantom points, index 6 in the stop message) and the
operator dropped those attempts without declaring a fault, so the manifest has no row for them. Their run JSONs were archived
(~/archive/x3_dropped_attempts_{,stretch3_,stretch4_}20261103.tar.gz, 30 JSONs); this script reads them from the tar files without
extracting, classifies each attempt by its recorded outcome and, per policy, compares the primary success rate (kept runs only) with
  - 'all attempts': kept runs plus every dropped attempt with its recorded outcome,
  - 'without phantom': kept runs plus the dropped attempts whose stop was not at index 6.
The primary analysis (robot_analysis.py) keeps only the manifest's runs; this is a sensitivity analysis, not a replacement.

Outputs (--out, default robot/): robot_dropped_attempts.csv (one row per dropped attempt), robot_dropped_policies.csv (one row per
policy), table_robot_dropped.tex.

Usage: robot_dropped.py [--archives TAR ...] [--out DIR]
"""
import argparse
import csv
import json
import os
import re
import sys
import tarfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import robot_results as rr  # noqa: E402

ARCHIVE_DIR = os.path.expanduser('~/archive')
ARCHIVES = [os.path.join(ARCHIVE_DIR, f'x3_dropped_attempts_{s}20261103.tar.gz') for s in ('', 'stretch3_', 'stretch4_')]
PHANTOM_INDEX = 6
HIT = re.compile(r'Obstacle hit with distance\s+([0-9.]+),\s*index\s+(\d+)')
KINDS = ('success', 'phantom_index_6', 'collision_other_index', 'other_failure')


def classify(result):
    """Recorded outcome of an attempt: success, stop at the phantom beam (index 6), stop at another beam, other failure."""
    if result.get('success'):
        return 'success'
    m = HIT.search(result.get('message') or '')
    if not m:
        return 'other_failure'
    return 'phantom_index_6' if int(m.group(2)) == PHANTOM_INDEX else 'collision_other_index'


def read_attempts(archives):
    """Dropped attempts from the run JSONs inside the tar files (read in memory, nothing is extracted)."""
    rows = []
    for a in archives:
        with tarfile.open(a) as t:
            for m in t.getmembers():
                if not (m.isfile() and m.name.endswith('.json') and '/recorded_paths/' in m.name):
                    continue
                d = json.load(t.extractfile(m))
                r = d.get('result') or {}
                hit = HIT.search(r.get('message') or '')
                parts = m.name.split('/')
                rows.append(dict(archive=os.path.basename(a), file=parts[-1], model=parts[-2], path=parts[-1].split('_')[0],
                                 goals=r.get('waypoints_completed', ''), total=r.get('total_waypoints', ''), success=int(bool(r.get('success'))),
                                 kind=classify(r), hit_index=int(hit.group(2)) if hit else '', hit_distance=float(hit.group(1)) if hit else '',
                                 elapsed_s=d.get('elapsed_time', ''), code=d.get('code', ''), message=r.get('message', '')))
    return sorted(rows, key=lambda r: (r['model'], r['file']))


def per_policy(schedule, kept_rows, dropped):
    """One row per policy: kept runs and successes, dropped attempts by kind, and the success rate under each treatment."""
    info = {}
    for s in schedule:
        info.setdefault(s['model'], (s['variant'], s['tier']))
    rows = []
    for model, (variant, tier) in info.items():
        kept = [r for r in kept_rows if r['model'] == model]
        drop = [r for r in dropped if r['model'] == model]
        ks, kn = sum(int(r['success']) for r in kept), len(kept)
        n = {k: sum(r['kind'] == k for r in drop) for k in KINDS}
        ds = n['success']
        rows.append(dict(variant=variant, tier=tier, model=model, kept_n=kn, kept_success=ks, dropped_n=len(drop),
                         dropped_success=n['success'], dropped_phantom_index_6=n['phantom_index_6'],
                         dropped_collision_other_index=n['collision_other_index'], dropped_other_failure=n['other_failure'],
                         success_primary=ks / kn if kn else float('nan'),
                         success_all_attempts=(ks + ds) / (kn + len(drop)) if kn + len(drop) else float('nan'),
                         success_without_phantom=(ks + ds) / (kn + len(drop) - n['phantom_index_6']) if kn + len(drop) - n['phantom_index_6'] else float('nan')))
    order = {k: i for i, k in enumerate((v, t) for v in rr.VARIANTS for t in ('B', 'M'))}
    return sorted(rows, key=lambda r: order[(r['variant'], r['tier'])])


def table_dropped(rows):
    esc = lambda s: s.replace('_', r'\_')
    head = [r'\textbf{Variant} & \textbf{Role} & \textbf{Policy} & \textbf{Kept} & \textbf{Dropped} & \textbf{Stop at index 6} & '
            r'\textbf{Other stop} & \textbf{Completed} & \textbf{Other} \\']
    out = [r'\begin{tabular}{lllcccccc}', r'\midrule[1.5pt]', *head, r'\midrule[1.5pt]']
    prev = None
    for r in rows:
        if prev is not None and r['variant'] != prev:
            out.append(r'\midrule')
        out.append(f"{rr.LABEL[r['variant']]} & {r['tier']} & {esc(r['model'])} & {r['kept_n']} & {r['dropped_n']} & "
                   f"{r['dropped_phantom_index_6']} & {r['dropped_collision_other_index']} & {r['dropped_success']} & {r['dropped_other_failure']} \\\\")
        prev = r['variant']
    return '\n'.join(out + [r'\midrule[1.5pt]', r'\end{tabular}', ''])


def write_csv(path, rows):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--archives', nargs='+', default=ARCHIVES)
    ap.add_argument('--out', default=rr.rrs.OUT_DIR)
    a = ap.parse_args()
    manifest = rr.rrs.load_manifest()
    kept = [r for r in manifest if rr.rrs.is_kept(r)]
    dropped = read_attempts(a.archives)
    rows = per_policy(rr.rrs.load_schedule(), kept, dropped)
    os.makedirs(a.out, exist_ok=True)
    write_csv(os.path.join(a.out, 'robot_dropped_attempts.csv'), dropped)
    write_csv(os.path.join(a.out, 'robot_dropped_policies.csv'), rows)
    open(os.path.join(a.out, 'table_robot_dropped.tex'), 'w').write(table_dropped(rows))
    print(f'{len(dropped)} dropped attempts, {sum(r["dropped_phantom_index_6"] for r in rows)} stopped at index {PHANTOM_INDEX}')
    for r in rows:
        print(f"  {r['variant']:13s} {r['tier']}  kept {r['kept_success']}/{r['kept_n']}  dropped {r['dropped_n']} (idx6 {r['dropped_phantom_index_6']}, "
              f"other stop {r['dropped_collision_other_index']}, completed {r['dropped_success']}, other {r['dropped_other_failure']})  "
              f"success primary {r['success_primary']:.2f}, all attempts {r['success_all_attempts']:.2f}, without phantom {r['success_without_phantom']:.2f}")


if __name__ == '__main__':
    main()
