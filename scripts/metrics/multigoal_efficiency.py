#!/usr/bin/env python3
"""Multi-goal path length and travel time relative to APF, and the policies drawn in the path figure.

Reads scripts/batch_runs/aggregate_7_scenarios.csv (one row per run) for the six paper scenarios
(world_4/path1 excluded, as in extract_leg_metrics.py). Every ratio is a run's path length or
travel time divided by the APF mean of the same scenario (five repeats).

Method ratios (paper text after fig:path_comparison_APF_TD3): over the successful runs, the ratio
is averaged within each scenario and then over the six scenarios.

Representative policies (fig:path_comparison_APF_TD3), per method:
  1. pool      : every distinct policy (Final and Best checkpoint of each seed; a Best equal to
                 Final counts once), since the MuJoCo Best is not guaranteed to beat Final here
  2. eligible  : succeeded in all six scenarios, so every panel can show the policy
  3. len_r, t_r: the policy's mean path length and travel time ratio over the six scenarios
  4. z         : len_r and t_r standardized over the eligible policies of the method
  median       : smallest |z(len_r)| + |z(t_r)|, the typical efficiency (rule fixed before
                 inspecting any trajectory)
  most efficient: smallest z(len_r) + z(t_r), the best case among the eligible policies
  APF          : the repeat whose path length is closest to the five-repeat median, per scenario
                 (chosen in visualize_comparison_figures.py).

Writes to --out (default scripts/metrics/output):
  multigoal_efficiency_policies.csv  one row per TD3 policy: eligible, len_r, t_r, z, role
  multigoal_efficiency_methods.csv   per method: ratios averaged per scenario, then over scenarios

Usage: multigoal_efficiency.py [--out DIR]
"""
import argparse
import os

import numpy as np
import pandas as pd

from extract_leg_metrics import EXCLUDED_SCENARIOS

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
AGG_CSV = os.path.join(REPO, 'scripts', 'batch_runs', 'aggregate_7_scenarios.csv')
ROLES = ('median', 'most_efficient')


def load_runs(csv=AGG_CSV) -> pd.DataFrame:
    """Runs of the six paper scenarios with len_r and t_r relative to the APF mean of the scenario."""
    df = pd.read_csv(csv)
    df = df[[(w, p) not in EXCLUDED_SCENARIOS for w, p in zip(df.world, df.path)]].copy()
    apf = df[df.planner == 'apf']
    ref_len = apf.groupby(['world', 'path']).path_len_m.mean()
    ref_t = apf.groupby(['world', 'path']).elapsed_s.mean()
    idx = list(zip(df.world, df.path))
    df['len_r'] = df.path_len_m.values / ref_len.reindex(idx).values
    df['t_r'] = df.elapsed_s.values / ref_t.reindex(idx).values
    return df


def method_ratios(df: pd.DataFrame) -> pd.DataFrame:
    ok = df[(df.planner != 'apf') & (df.success == 1)]
    per_scenario = ok.groupby(['planner', 'world', 'path'])[['len_r', 't_r']].mean()
    out = per_scenario.groupby('planner').mean()
    out['n_runs'] = ok.groupby('planner').size()
    return out.reset_index()


def policy_efficiency(df: pd.DataFrame) -> pd.DataFrame:
    """One row per distinct TD3 policy, with the median and most efficient policy of each method marked."""
    drl = df[df.planner != 'apf'].drop_duplicates(['model', 'world', 'path'])
    n_scen = drl.groupby(['world', 'path']).ngroups
    per = (drl.groupby(['planner', 'model'])
              .agg(n_runs=('success', 'size'), n_success=('success', 'sum'),
                   len_r=('len_r', 'mean'), t_r=('t_r', 'mean'))
              .reset_index())
    per['eligible'] = per.n_success == n_scen
    per['z_len'] = np.nan
    per['z_t'] = np.nan
    per['role'] = ''
    for planner, g in per[per.eligible].groupby('planner'):
        z = lambda s: (s - s.median()) / (s.std() if s.std() > 0 else 1.0)
        per.loc[g.index, 'z_len'] = z(g.len_r)
        per.loc[g.index, 'z_t'] = z(g.t_r)
        e = per.loc[g.index]
        median = (e.z_len.abs() + e.z_t.abs()).idxmin()
        best = (e.z_len + e.z_t).idxmin()
        per.loc[median, 'role'] = 'median'
        per.loc[best, 'role'] = (per.loc[best, 'role'] + ';most_efficient').lstrip(';')
    return per.sort_values(['planner', 'role', 'model'], ascending=[True, False, True])


def representatives(df: pd.DataFrame) -> dict:
    """{('TD3_direct' | 'TD3_delta', 'median' | 'most_efficient'): model}"""
    per = policy_efficiency(df)
    out = {}
    for _, r in per[per.role != ''].iterrows():
        for role in r.role.split(';'):
            out[('TD3_' + r.planner, role)] = r.model
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.join(HERE, 'output'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    df = load_runs()
    methods = method_ratios(df)
    policies = policy_efficiency(df)
    methods.to_csv(os.path.join(args.out, 'multigoal_efficiency_methods.csv'), index=False, float_format='%.6g')
    policies.to_csv(os.path.join(args.out, 'multigoal_efficiency_policies.csv'), index=False, float_format='%.6g')

    print('method ratios (successful runs, per scenario then averaged):')
    print(methods.round(3).to_string(index=False))
    print('\nrepresentative policies:')
    print(policies[policies.role != ''][['planner', 'model', 'role', 'len_r', 't_r']].round(3).to_string(index=False))


if __name__ == '__main__':
    main()
