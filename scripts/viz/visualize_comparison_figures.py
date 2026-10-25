"""
Comparative path figures (APF vs TD3_direct vs TD3_delta) for the paper, with the plotted TD3
policies chosen by a fixed rule from aggregate_7_scenarios.csv instead of by hand.

Rule (per method):
  1. pool      : every policy (Final and distinct Best checkpoint of each seed; 19 delta, 17 direct), since
                 the MuJoCo-selected Best is not guaranteed to beat Final in the multi-goal test
  2. eligible  : succeeded on every plotted scenario
  3. pick      : eligible policy closest to the method's median in mean path length and mean time,
                 both relative to the APF mean for the same scenario (sum of |standardized deviation|)
  APF          : the repeat whose path length is closest to the 5-repeat median, per scenario.

Outputs <FIG_DIR>/MultipleGoal_APF_TD3_<direct>_<delta>_<tag>.{pdf,png}. Does not touch run_registry.py.
"""
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from viz_common import (WS_DIR, load_run, load_obstacles, load_goals, run_path_name,
                        draw_obstacles, draw_goals, apply_arena_style)

CSV     = os.path.join(WS_DIR, 'scripts', 'batch_runs', 'aggregate_7_scenarios.csv')
FIG_DIR = os.path.join(WS_DIR, 'scripts', 'figures')
ARROW_EVERY_N = 500
COLORS = {'APF': 'tab:blue', 'TD3_direct': 'tab:orange', 'TD3_delta': 'tab:green'}

# (world, path) scenarios that define "plotted" and the figure groups; world 4 path 1 is excluded
FIGURES = {
    'world1':    [('world_1', 'path1'), ('world_1', 'path2'), ('world_1', 'path3')],
    'world235':  [('world_2', 'path1'), ('world_3', 'path1'), ('world_5', 'path1')],
}
SCENARIOS = [s for group in FIGURES.values() for s in group]


def run_key(saved_file: str) -> str:
    """'scripts/recorded_paths/TD3_00386_1000/path2_world_1_241026_2358.json' -> 'TD3_00386_1000/path2_world_1_241026_2358'"""
    return saved_file.split('recorded_paths/')[1][:-len('.json')]


def select_policies(df: pd.DataFrame) -> dict:
    df = df[[(w, p) in SCENARIOS for w, p in zip(df.world, df.path)]].copy()
    apf = df[df.planner == 'apf']
    ref_len = apf.groupby(['world', 'path']).path_len_m.mean()
    ref_t = apf.groupby(['world', 'path']).elapsed_s.mean()

    drl = df[df.planner != 'apf'].copy()
    drl['seed'] = drl.model.str.extract(r'TD3_(\d+)_')[0].astype(int)
    drl = drl.drop_duplicates(['model', 'world', 'path'])    # a policy whose Best == Final is one policy

    idx = list(zip(drl.world, drl.path))
    drl['len_r'] = drl.path_len_m.values / ref_len.reindex(idx).values
    drl['t_r'] = drl.elapsed_s.values / ref_t.reindex(idx).values

    chosen = {}
    for planner, g in drl.groupby('planner'):
        per = g.groupby('model').agg(ok=('success', 'min'), len_r=('len_r', 'mean'), t_r=('t_r', 'mean'))
        n_total = len(per)
        per = per[per.ok == 1].copy()
        z = lambda s: (s - s.median()) / (s.std() if s.std() > 0 else 1.0)
        per['dist'] = z(per.len_r).abs() + z(per.t_r).abs()
        per = per.sort_values('dist')
        print(f'{planner}: {len(per)}/{n_total} eligible; closest to median:\n{per.round(3).head(3)}\n')
        chosen['TD3_' + planner] = per.index[0]
    return chosen, apf


def pick_apf_run(apf: pd.DataFrame, world: str, path: str) -> str:
    g = apf[(apf.world == world) & (apf.path == path)]
    r = g.iloc[(g.path_len_m - g.path_len_m.median()).abs().argmin()]
    return run_key(r.saved_file)


def drl_run(df: pd.DataFrame, model: str, world: str, path: str) -> str:
    r = df[(df.model == model) & (df.world == world) & (df.path == path)]
    assert len(r) == 1, (model, world, path, len(r))
    return run_key(r.iloc[0].saved_file)


def plot_group(tag, scenarios, chosen, df, apf):
    fig, axes = plt.subplots(1, len(scenarios), figsize=(6.0 * len(scenarios), 6.4))
    for ax, (world, path) in zip(np.atleast_1d(axes), scenarios):
        keys = [('APF', pick_apf_run(apf, world, path)),
                ('TD3_direct', drl_run(df, chosen['TD3_direct'], world, path)),
                ('TD3_delta', drl_run(df, chosen['TD3_delta'], world, path))]
        for label, key in keys:
            run = load_run(key)
            line = ax.plot(run['x'], run['y'], '--', color=COLORS[label], alpha=0.8, lw=1.5, zorder=1,
                           label=f"{label} | {run['elapsed_time']:.1f} s | {run['path_length']:.1f} m")
            i = range(0, len(run['x']), ARROW_EVERY_N)
            ax.quiver([run['x'][j] for j in i], [run['y'][j] for j in i],
                      [np.cos(run['yaw'][j]) for j in i], [np.sin(run['yaw'][j]) for j in i],
                      color=line[0].get_color(), scale=50, width=0.004, headwidth=4, headlength=5, alpha=0.8)
        draw_obstacles(ax, load_obstacles(world))
        draw_goals(ax, load_goals(world, run_path_name(keys[0][1])))
        apply_arena_style(ax, world)
        ax.set_title(f"{world.replace('_', ' ').capitalize()}, {path.replace('path', 'path ')}", fontsize=14)
        ax.legend(loc='upper left', fontsize=10)
    fig.tight_layout()
    base = f"MultipleGoal_APF_TD3_{chosen['TD3_direct'][4:]}_{chosen['TD3_delta'][4:]}_{tag}"
    os.makedirs(FIG_DIR, exist_ok=True)
    for ext in ('pdf', 'png'):
        fig.savefig(os.path.join(FIG_DIR, f'{base}.{ext}'), dpi=200, bbox_inches='tight')
    plt.close(fig)
    print('wrote', os.path.join(FIG_DIR, base + '.{pdf,png}'))


def main():
    df = pd.read_csv(CSV)
    chosen, apf = select_policies(df)
    print('chosen:', chosen)
    for tag, scenarios in FIGURES.items():
        plot_group(tag, scenarios, chosen, df, apf)


if __name__ == '__main__':
    main()
