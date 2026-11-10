#!/usr/bin/env python3
"""P14 multi-goal metrics: the A1 and A2 policies and the same-code reruns of APF and the paper's TD3.

Reads scripts/batch_runs/manifest_p14.csv (written by scripts/run_p14_gazebo.sh; more with --manifest) and
computes the multi-goal measures of extract_leg_metrics.py with its own functions (leg parsing, shortest-path SPL,
per-unit aggregation, median [Q1, Q3] and bootstrap summary), so the numbers are defined as in
Table multigoal_success. world_4/path1 is run but left out of the per-unit and summary outputs.

Kinds:
  - <group>_final: TD3_0SSxx_1500 with group a1 (011), a2dir (012), a2delta (013) or a2ad (014, asymmetric delta
    bound); <group>_best: the
    MuJoCo-selected checkpoint per seed (BEST below), a policy that is both counts in both rows, as
    extract_leg_metrics.py does;
  - apf_rerun: APF, one unit per repeat; direct_final_rerun: TD3_00396_1000 to TD3_00405_1000;
    delta_final_rerun: TD3_00386_1000 to TD3_00395_1000.
The published units (scripts/metrics/output/multigoal_units.csv, kinds apf, direct_final and delta_final) are
added to the summary as <kind>_paper when that file exists. With scipy available, a two-sided Mann-Whitney U
test of each A2 Final group against apf_rerun, and of a2ad_final against the other A2 Final groups, is written to
mwu.csv (notes/a2_eval_protocol_20261109.md, notes/a2_ad_eval_protocol_20261109.md).

Usage: scripts/metrics/p14_multigoal.py [--manifest scripts/batch_runs/manifest_p14.csv ...]
                                        [--out scripts/metrics/output_p14] [--n-boot 10000]
"""
import argparse
import datetime
import json
import os

import numpy as np
import pandas as pd

import extract_leg_metrics as E
from shortest_path import OBSTACLE_TOLERANCE, ShortestPath

MANIFEST = os.path.join(E.REPO, 'scripts', 'batch_runs', 'manifest_p14.csv')
LOG_DIR = os.path.join(E.REPO, 'scripts', 'batch_runs', 'logs')
GROUP = {'011': 'a1', '012': 'a2dir', '013': 'a2delta', '014': 'a2ad'}
BEST = {'01101': '1300', '01102': '1500', '01103': '1400',      # notes/a1_eval_protocol_20261106.md, eval seed 11
        # notes/a2_mujoco_result_20261109.md, eval seed 11
        '01201': '1100', '01202': '1000', '01203': '1100', '01204': '1500', '01205': '1000',
        '01206': '1100', '01207': '1500', '01208': '1500', '01209': '1400', '01210': '1400',
        '01301': '1200', '01302': '900', '01303': '1100', '01304': '1300', '01305': '1000',
        '01306': '1500', '01307': '1400', '01308': '1300', '01309': '1500', '01310': '1500',
        # notes/a2_ad_mujoco_result_20261109.md, eval seed 11
        '01401': '1300', '01402': '1400', '01403': '1500', '01404': '1500', '01405': '1400',
        '01406': '1300', '01407': '1500', '01408': '1500', '01409': '1500', '01410': '1300'}


def kinds_of(model):
    if model == 'APF':
        return 'apf', ['apf_rerun']
    _, sid, ckpt = model.split('_')
    g = GROUP.get(sid[:3])
    if g:
        return g, [k for k, hit in ((f'{g}_final', ckpt == '1500'), (f'{g}_best', BEST.get(sid) == ckpt)) if hit]
    p = 'delta' if 386 <= int(sid) <= 395 else 'direct'
    return p, [f'{p}_final_rerun'] if ckpt == '1000' else []


def load_runs(manifests):
    man = pd.concat([pd.read_csv(m, dtype={'rep': int}) for m in manifests], ignore_index=True)
    ok = man[man['status'] == 'ok'].copy()
    # one row per job: the last ok row (resumed batches skip jobs already ok, so duplicates are reruns)
    ok = ok.drop_duplicates(['model', 'world', 'path', 'rep'], keep='last')
    return man, ok


def legs_of(ok, sp_by_world):
    paths = {}
    for f in os.listdir(E.PATHS_DIR):
        if f.startswith('paths_world_') and f.endswith('.json'):
            paths.update(json.load(open(os.path.join(E.PATHS_DIR, f))))
    rows = []
    for run in ok.itertuples():
        planner, kinds = kinds_of(run.model)
        if not kinds:
            continue
        planned = paths[f'{run.path}_{run.world}']
        log = os.path.join(LOG_DIR, os.path.basename(run.log_file).replace('_client.log', '_server.log'))
        legs = E.parse_server_log(log)
        poses = json.load(open(os.path.join(E.REPO, run.saved_file)))['poses']
        t = np.array([p['t'] for p in poses])
        xy = np.array([[p['x'], p['y']] for p in poses])
        sp = sp_by_world[run.world]
        unit = run.model if planner != 'apf' else f'APF_rep{run.rep}'
        run_rows = []
        for k, (wx, wy) in enumerate(planned, 1):
            row = dict(planner=planner, unit=unit, model=run.model, world=run.world, path=run.path,
                       rep=run.rep, leg=k, n_legs=len(planned), wx=wx, wy=wy, outcome='not_attempted',
                       message='', duration=np.nan, P=np.nan, L_straight=np.nan, L_geo=np.nan, geo_flags='')
            if k <= len(legs):
                lg = legs[k - 1]
                assert (lg['leg'], lg['wx'], lg['wy']) == (k, wx, wy), run.saved_file
                win = (t >= lg['t_start']) & (t <= lg['t_end'])
                i0 = max(np.searchsorted(t, lg['t_start'], side='right') - 1, 0)
                seg = np.concatenate([xy[i0:i0 + 1], xy[win]])
                sx, sy = seg[0]
                L_geo, flags = sp.length((sx, sy), (wx, wy))
                L_straight = float(np.hypot(wx - sx, wy - sy))
                row.update(outcome=lg['outcome'] or 'unknown', message=lg['message'],
                           duration=lg['t_end'] - lg['t_start'],
                           P=np.linalg.norm(np.diff(seg, axis=0), axis=1).sum(),
                           L_straight=L_straight, L_geo=max(L_geo, L_straight), geo_flags=';'.join(flags))
            run_rows.append(row)
        rows += [dict(r, kind=kind, seed=np.nan) for kind in kinds for r in run_rows]
    legs = pd.DataFrame(rows)
    ok_leg = legs['outcome'] == 'reached'
    legs['spl_straight'] = E.spl(ok_leg, legs['L_straight'].fillna(1), legs['P'].fillna(0))
    legs['spl_geo'] = E.spl(ok_leg, legs['L_geo'].fillna(1), legs['P'].fillna(0))
    return legs


def runs_of(legs):
    g = legs.groupby(['kind', 'planner', 'unit', 'seed', 'model', 'world', 'path', 'rep'], dropna=False)
    runs = g[['outcome', 'leg', 'P', 'duration', 'spl_straight', 'spl_geo']].apply(lambda d: pd.Series(dict(
        n_legs=len(d), goals_reached=int((d['outcome'] == 'reached').sum()),
        failure_type=next(iter(d.loc[~d['outcome'].isin(['reached', 'not_attempted']), 'outcome']), ''),
        failed_leg=next(iter(d.loc[~d['outcome'].isin(['reached', 'not_attempted']), 'leg']), np.nan),
        P_total=d['P'].sum(), time_total=d['duration'].sum(),
        spl_straight=d['spl_straight'].mean(), spl_geo=d['spl_geo'].mean()))).reset_index()
    runs['success'] = (runs['goals_reached'] == runs['n_legs']).astype(int)
    return runs


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--manifest', nargs='+', default=[MANIFEST])
    ap.add_argument('--out', default=os.path.join(E.HERE, 'output_p14'))
    ap.add_argument('--n-boot', type=int, default=10000)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rng = np.random.default_rng(0)

    man, ok = load_runs(args.manifest)
    sp_by_world = {w: ShortestPath(w, OBSTACLE_TOLERANCE) for w in sorted(ok['world'].unique())}
    legs = legs_of(ok, sp_by_world)
    runs = runs_of(legs)
    units = E.multigoal_units(legs, runs)

    published = os.path.join(E.HERE, 'output', 'multigoal_units.csv')
    if os.path.exists(published):
        pub = pd.read_csv(published)
        pub = pub[pub['kind'].isin(['apf', 'direct_final', 'delta_final'])].copy()
        pub['kind'] = pub['kind'] + '_paper'
        units_all = pd.concat([units, pub[units.columns.intersection(pub.columns)]], ignore_index=True)
    else:
        units_all = units
    metrics = ['run_success', 'goal_completion', 'collisions_per_100_legs', 'timeouts_per_100_legs',
               'spl_geo', 'spl_geo_reached', 'spl_straight']
    summary = E.summarize(units_all, metrics, args.n_boot, rng)

    for name, df in {'p14_legs.csv': legs, 'p14_runs.csv': runs, 'p14_units.csv': units_all,
                     'p14_summary.csv': summary}.items():
        df.to_csv(os.path.join(args.out, name), index=False, float_format='%.6g')
    mwu = []
    try:
        from scipy.stats import mannwhitneyu
        pairs = [('a2dir_final', 'apf_rerun'), ('a2delta_final', 'apf_rerun'), ('a2ad_final', 'apf_rerun'),
                 ('a2ad_final', 'a2delta_final'), ('a2ad_final', 'a2dir_final')]  # notes/a2_ad_eval_protocol_20261109.md
        for kind, vs in pairs:
            for m in ('goal_completion', 'spl_geo'):
                x = units.loc[units['kind'] == kind, m].dropna()
                y = units.loc[units['kind'] == vs, m].dropna()
                if len(x) and len(y):
                    r = mannwhitneyu(x, y, alternative='two-sided')
                    mwu.append(dict(kind=kind, vs=vs, metric=m, n=len(x), n_vs=len(y), U=r.statistic, p=r.pvalue))
        pd.DataFrame(mwu).to_csv(os.path.join(args.out, 'mwu.csv'), index=False, float_format='%.6g')
    except Exception as e:      # the system scipy may not load against numpy 2; the test is then skipped
        print('Mann-Whitney U skipped:', e)

    counts = ok['model'].map(lambda m: kinds_of(m)[0]).value_counts().to_dict()
    meta = dict(script='scripts/metrics/p14_multigoal.py', code_commit=E.code_commit(),
                generated=datetime.datetime.now().isoformat(timespec='seconds'), manifest=args.manifest,
                manifest_rows=len(man), ok_jobs=counts,
                not_ok_rows=int((man['status'] != 'ok').sum()), radius_m=OBSTACLE_TOLERANCE,
                n_boot=args.n_boot, bootstrap_rng_seed=0, best_checkpoints=BEST,
                excluded_scenarios=sorted('/'.join(s) for s in E.EXCLUDED_SCENARIOS))
    json.dump(meta, open(os.path.join(args.out, 'meta.json'), 'w'), indent=2)

    print(json.dumps({k: meta[k] for k in ('code_commit', 'ok_jobs', 'not_ok_rows')}))
    if mwu:
        print(pd.DataFrame(mwu).to_string(index=False))
    pd.set_option('display.width', 200)
    s = summary.assign(v=lambda d: d.apply(lambda r: f"{r['median']:.4g} [{r['q1']:.4g}, {r['q3']:.4g}]", axis=1))
    print(s.pivot(index='kind', columns='metric', values='v')[metrics].to_string())
    r6 = runs[~runs.set_index(['world', 'path']).index.isin(list(E.EXCLUDED_SCENARIOS))]
    print(r6.groupby(['kind', 'unit']).agg(runs=('success', 'size'), success=('success', 'sum'),
                                           goals=('goals_reached', 'sum'), legs=('n_legs', 'sum'),
                                           fail=('failure_type', lambda x: ','.join(v for v in x if v))).to_string())


if __name__ == '__main__':
    main()
