"""
Comparative path figures (APF vs TD3_direct vs TD3_delta) for the paper, with the plotted TD3
policies chosen by a fixed rule instead of by hand.

The TD3 policies come from scripts/metrics/multigoal_efficiency.py (representatives), per method:
  median         : the eligible policy (all six scenarios completed) closest to the method's median
                   in mean path length and travel time relative to APF, the typical efficiency
  most efficient : the eligible policy with the shortest and fastest runs relative to APF, the
                   best case
APF: the repeat whose path length is closest to the 5-repeat median, per scenario.

Default: one 2x3 figure of the six scenarios (world 1 paths 1 to 3 on top, worlds 2, 3 and 4
below). Each panel draws APF, then for each TD3 method the median policy (dashed, with heading
arrows) and the most efficient policy (dotted, no arrows), and one shared legend sits below the
panels. The travel time and path length of every drawn run go to a CSV next to the figure
instead of the legend.
  -> <FIG_DIR>/MultipleGoal_APF_TD3_<direct>_<delta>_eff_<direct>_<delta>_grid.{pdf,png,csv}
--median-only: the earlier figure, two rows of three panels with the median policies alone and
a legend per panel (time and length in the labels).
  -> <FIG_DIR>/MultipleGoal_APF_TD3_<direct>_<delta>_<tag>.{pdf,png}
Does not touch run_registry.py.

Usage: visualize_comparison_figures.py [--median-only]
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from viz_common import (WS_DIR, load_run, load_obstacles, load_goals, run_path_name,
                        draw_obstacles, draw_goals, apply_arena_style)

sys.path.insert(0, os.path.join(WS_DIR, 'scripts', 'metrics'))
from multigoal_efficiency import AGG_CSV, load_runs, representatives  # noqa: E402

FIG_DIR = os.path.join(WS_DIR, 'scripts', 'figures')
ARROW_EVERY_N = 500
COLORS = {'APF': 'tab:blue', 'TD3_direct': 'tab:orange', 'TD3_delta': 'tab:green'}
STYLE = {'median': dict(ls='--', lw=1.5, arrows=True, name='median'),
         'most_efficient': dict(ls=':', lw=2.0, arrows=False, name='most efficient')}

# (world, path) scenarios of each figure; world 4 path 1 is excluded
FIGURES = {
    'world1':    [('world_1', 'path1'), ('world_1', 'path2'), ('world_1', 'path3')],
    'world235':  [('world_2', 'path1'), ('world_3', 'path1'), ('world_5', 'path1')],
}
# the paper calls world_5 "world 4" (world_4/path1 was an experimental setting and is excluded)
DISPLAY_WORLD = {'world_5': 'world_4'}


def run_key(saved_file: str) -> str:
    """'scripts/recorded_paths/TD3_00386_1000/path2_world_1_241026_2358.json' -> 'TD3_00386_1000/path2_world_1_241026_2358'"""
    return saved_file.split('recorded_paths/')[1][:-len('.json')]


def pick_apf_run(apf: pd.DataFrame, world: str, path: str) -> str:
    g = apf[(apf.world == world) & (apf.path == path)]
    r = g.iloc[(g.path_len_m - g.path_len_m.median()).abs().argmin()]
    return run_key(r.saved_file)


def drl_run(df: pd.DataFrame, model: str, world: str, path: str) -> str:
    r = df[(df.model == model) & (df.world == world) & (df.path == path)]
    assert len(r) == 1, (model, world, path, len(r))
    return run_key(r.iloc[0].saved_file)


def draw_run(ax, key, color, label, ls, lw, arrows, stats_in_label=True):
    run = load_run(key)
    if stats_in_label:
        label = f"{label} | {run['elapsed_time']:.1f} s | {run['path_length']:.1f} m"
    ax.plot(run['x'], run['y'], ls, color=color, alpha=0.8, lw=lw, zorder=1, label=label)
    if arrows:
        i = range(0, len(run['x']), ARROW_EVERY_N)
        ax.quiver([run['x'][j] for j in i], [run['y'][j] for j in i],
                  [np.cos(run['yaw'][j]) for j in i], [np.sin(run['yaw'][j]) for j in i],
                  color=color, scale=50, width=0.004, headwidth=4, headlength=5, alpha=0.8)
    return run


def panel_title(world, path):
    return f"{DISPLAY_WORLD.get(world, world).replace('_', ' ').capitalize()}, {path.replace('path', 'path ')}"


def plot_grid(chosen, df, apf):
    """All six scenarios in one 2x3 figure with one legend below the panels."""
    rows = list(FIGURES.values())
    fig, axes = plt.subplots(len(rows), len(rows[0]), figsize=(6.0 * len(rows[0]), 6.0 * len(rows) + 0.8))
    stats = []
    for ax_row, scenarios in zip(axes, rows):
        for ax, (world, path) in zip(ax_row, scenarios):
            apf_key = pick_apf_run(apf, world, path)
            drawn = [('APF', '', apf_key, draw_run(ax, apf_key, COLORS['APF'], 'APF', '--', 1.5, True, False))]
            for method in ('TD3_direct', 'TD3_delta'):
                for role in ('median', 'most_efficient'):
                    s = STYLE[role]
                    key = drl_run(df, chosen[(method, role)], world, path)
                    run = draw_run(ax, key, COLORS[method], f"{method} {s['name']}",
                                   s['ls'], s['lw'], s['arrows'], False)
                    drawn.append((method, role, key, run))
            for planner, role, key, run in drawn:
                stats.append(dict(world=world, path=path, panel=panel_title(world, path), planner=planner,
                                  role=role, run=key, elapsed_s=round(run['elapsed_time'], 2),
                                  path_length_m=round(run['path_length'], 3)))
            draw_obstacles(ax, load_obstacles(world))
            draw_goals(ax, load_goals(world, run_path_name(apf_key)))
            apply_arena_style(ax, world)
            ax.set_title(panel_title(world, path), fontsize=14)
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncol=len(labels), fontsize=13, frameon=False,
               bbox_to_anchor=(0.5, 0.0), handlelength=3.0)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    ids = lambda role: f"{chosen[('TD3_direct', role)][4:]}_{chosen[('TD3_delta', role)][4:]}"
    base = f"MultipleGoal_APF_TD3_{ids('median')}_eff_{ids('most_efficient')}_grid"
    os.makedirs(FIG_DIR, exist_ok=True)
    for ext in ('pdf', 'png'):
        fig.savefig(os.path.join(FIG_DIR, f'{base}.{ext}'), dpi=200, bbox_inches='tight')
    plt.close(fig)
    pd.DataFrame(stats).to_csv(os.path.join(FIG_DIR, f'{base}.csv'), index=False)
    print('wrote', os.path.join(FIG_DIR, base + '.{pdf,png,csv}'))


def plot_group(tag, scenarios, chosen, roles, df, apf):
    fig, axes = plt.subplots(1, len(scenarios), figsize=(6.0 * len(scenarios), 6.4))
    for ax, (world, path) in zip(np.atleast_1d(axes), scenarios):
        apf_key = pick_apf_run(apf, world, path)
        draw_run(ax, apf_key, COLORS['APF'], 'APF', '--', 1.5, True)
        for method in ('TD3_direct', 'TD3_delta'):
            for role in roles:
                s = STYLE[role]
                label = method if roles == ('median',) else f"{method} {s['name']}"
                draw_run(ax, drl_run(df, chosen[(method, role)], world, path), COLORS[method],
                         label, s['ls'], s['lw'], s['arrows'])
        draw_obstacles(ax, load_obstacles(world))
        draw_goals(ax, load_goals(world, run_path_name(apf_key)))
        apply_arena_style(ax, world)
        ax.set_title(f"{DISPLAY_WORLD.get(world, world).replace('_', ' ').capitalize()}, {path.replace('path', 'path ')}", fontsize=14)
        ax.legend(loc='upper left', fontsize=10)
    fig.tight_layout()
    ids = lambda role: f"{chosen[('TD3_direct', role)][4:]}_{chosen[('TD3_delta', role)][4:]}"
    base = 'MultipleGoal_APF_TD3_' + ids('median')
    if 'most_efficient' in roles:
        base += '_eff_' + ids('most_efficient')
    base += f'_{tag}'
    os.makedirs(FIG_DIR, exist_ok=True)
    for ext in ('pdf', 'png'):
        fig.savefig(os.path.join(FIG_DIR, f'{base}.{ext}'), dpi=200, bbox_inches='tight')
    plt.close(fig)
    print('wrote', os.path.join(FIG_DIR, base + '.{pdf,png}'))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--median-only', action='store_true', help='draw only the median policies')
    args = ap.parse_args()
    roles = ('median',) if args.median_only else ('median', 'most_efficient')

    df = pd.read_csv(AGG_CSV)
    chosen = representatives(load_runs())
    apf = df[df.planner == 'apf']
    print('chosen:', chosen)
    if args.median_only:
        for tag, scenarios in FIGURES.items():
            plot_group(tag, scenarios, chosen, roles, df, apf)
    else:
        plot_grid(chosen, df, apf)


if __name__ == '__main__':
    main()
