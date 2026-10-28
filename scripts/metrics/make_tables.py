#!/usr/bin/env python3
"""LaTeX table bodies and the per-seed strip plot for the P2 reporting layout.

Reads scripts/metrics/output/{g2g_units,multigoal_units}.csv (extract_leg_metrics.py) and
scripts/sim2real/seed_selection_bm.csv (P1 ranking). Writes to the same output folder:
  table_g2g.tex           table:spl_aggregate       success, SPL (shortest path), SPL (straight line)
  table_failure_modes.tex table:failure_modes       collisions and timeouts, G2G and multi-goal
  table_multigoal.tex     table:multigoal_success   runs, 6/6 policies, goal completion, SPL
  table_deployed.tex      Physical Deployment       the B / M policies, labelled as selected in-sample
  fig_multigoal_seeds.{pdf,png}                     one dot per policy, goal completion and SPL
  tables_meta.json                                  commit, date, inputs

Layout (decision 2026-10-28): every seed is reported; the statistic is median [Q1, Q3] across
the units (policies, or the 5 APF repeats), with mean +- sample SD as a secondary column.
Shortest-path SPL is primary and straight-line SPL secondary. The B / M policies of the P1 rule
are shown separately: they are ranked on these same scenarios, so B is best of 10 by construction.
Only table bodies (tabular) are written; captions and placement belong to the paper.

Usage: make_tables.py [--out DIR]
"""
import argparse
import datetime
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from extract_leg_metrics import code_commit

HERE = os.path.dirname(os.path.abspath(__file__))
BM_CSV = os.path.join(HERE, '..', 'sim2real', 'seed_selection_bm.csv')
KINDS = ['apf', 'direct_final', 'direct_best', 'delta_final', 'delta_best']
LABEL = {'apf': 'APF', 'direct_final': 'direct\nFinal', 'direct_best': 'direct\nBest',
         'delta_final': 'delta\nFinal', 'delta_best': 'delta\nBest'}
PLANNER_STYLE = {'apf': ('#1f77b4', 'o', 'APF'), 'direct': ('#ff7f0e', 's', 'TD3_direct'),
                 'delta': ('#2ca02c', '^', 'TD3_delta')}      # colours of the paper's trajectory figures
INK, MUTED, GRID = '#222222', '#666666', '#d9d9d9'


def ms(x, d):
    return f'${np.mean(x):.{d}f} \\pm {np.std(x, ddof=1):.{d}f}$'


def med(x, d):
    q1, m, q3 = np.percentile(x, [25, 50, 75])
    return f'{m:.{d}f} [{q1:.{d}f}, {q3:.{d}f}]'


def tabular(colspec, header, cells):
    '''header: list of LaTeX lines; cells(kind) -> list of cell strings for that row.'''
    def row(prefix, kind):
        return f'{prefix} & ' + ' & '.join(cells(kind)) + r' \\'
    return '\n'.join([
        r'\begin{tabular}{' + colspec + '}', r'\midrule[1.5pt]', *header, r'\midrule[1.5pt]',
        row(r'APF & ---', 'apf'), r'\midrule',
        row(r'\multirow{2}{*}{TD3\_direct} & Final', 'direct_final'),
        row(r'                             & Best ', 'direct_best'), r'\midrule',
        row(r'\multirow{2}{*}{TD3\_delta}  & Final', 'delta_final'),
        row(r'                             & Best ', 'delta_best'),
        r'\midrule[1.5pt]', r'\end{tabular}', ''])


def table_g2g(g):
    by = {k: g[g.kind == k] for k in KINDS}
    header = [r' & & \multicolumn{2}{c|}{\textbf{Success Rate (\%)}} & \multicolumn{2}{c|}{\textbf{SPL (shortest path)}} & \textbf{SPL (straight line)} \\',
              r'\textbf{Planner} & \textbf{} & Mean $\pm$ SD & Median [Q1, Q3] & Mean $\pm$ SD & Median [Q1, Q3] & Median [Q1, Q3] \\']
    return tabular('ll|cc|cc|c', header, lambda k: [
        ms(by[k].success_rate, 2), med(by[k].success_rate, 2),
        ms(by[k].spl_geo, 3), med(by[k].spl_geo, 3), med(by[k].spl_straight, 3)])


def table_failure_modes(g, m):
    gb = {k: g[g.kind == k] for k in KINDS}
    mb = {k: m[m.kind == k] for k in KINDS}
    header = [r' & & \multicolumn{2}{c|}{\textbf{Goal-to-goal (per 139 goals)}} & \multicolumn{2}{c}{\textbf{Multi-goal (per 100 legs)}} \\',
              r'\textbf{Planner} & \textbf{} & Collisions & Timeouts & Collisions & Timeouts \\']
    return tabular('ll|cc|cc', header, lambda k: [
        med(gb[k].n_collision, 1), med(gb[k].n_timeout, 1),
        med(mb[k].collisions_per_100_legs, 2), med(mb[k].timeouts_per_100_legs, 2)])


def table_multigoal(m):
    mb = {k: m[m.kind == k] for k in KINDS}

    def runs(k):
        d = mb[k]
        return f'{int(round((d.run_success / 100 * d.n_runs).sum()))}/{int(d.n_runs.sum())}'

    def full(k):
        return '---' if k == 'apf' else f'{int((mb[k].run_success >= 100 - 1e-9).sum())}/{len(mb[k])}'

    header = [r' & & \multicolumn{2}{c|}{\textbf{Runs}} & \multicolumn{2}{c|}{\textbf{Goal completion (\%)}} & \multicolumn{2}{c}{\textbf{SPL (shortest path)}} \\',
              r'\textbf{Planner} & \textbf{} & Successful & Policies 6/6 & Mean $\pm$ SD & Median [Q1, Q3] & All legs & Reached legs \\',
              r' & & & & & & \multicolumn{2}{c}{Median [Q1, Q3]} \\']
    return tabular('ll|cc|cc|cc', header, lambda k: [
        runs(k), full(k), ms(mb[k].goal_completion, 1), med(mb[k].goal_completion, 1),
        med(mb[k].spl_geo, 3), med(mb[k].spl_geo_reached, 3)])


def table_deployed(m, bm):
    rows = []
    for kind in KINDS[1:]:
        for tier in ('B', 'M'):
            r = bm[(bm.variant == kind) & (bm.tier == tier)].iloc[0]
            u = m[(m.kind == kind) & (m.unit == r.model)].iloc[0]
            name = r.model.replace('_', r'\_')
            planner, ckpt = kind.split('_')
            rows.append(f'TD3\\_{planner} {ckpt.capitalize()} & {tier} & {name} & {int(r["rank"])}/{int(r.n_pool)} & '
                        f'{int(round(u.run_success / 100 * u.n_runs))}/{int(u.n_runs)} & {u.goal_completion:.1f} & '
                        f'{u.collisions_per_100_legs:.2f} & {u.spl_geo:.3f} \\\\')
    return '\n'.join([
        r'\begin{tabular}{llllcccc}', r'\midrule[1.5pt]',
        r'\textbf{Variant} & \textbf{Role} & \textbf{Policy} & \textbf{Rank} & \textbf{Runs} & \textbf{Goals (\%)} & \textbf{Coll./100 legs} & \textbf{SPL} \\',
        r'\midrule[1.5pt]', *rows[:2], r'\midrule', *rows[2:4], r'\midrule', *rows[4:6], r'\midrule', *rows[6:],
        r'\midrule[1.5pt]', r'\end{tabular}', ''])


def strip_plot(m, path):
    plt.rcParams.update({'font.size': 8, 'text.color': INK, 'axes.labelcolor': INK,
                         'xtick.color': MUTED, 'ytick.color': MUTED, 'axes.edgecolor': GRID,
                         'pdf.fonttype': 42, 'ps.fonttype': 42})
    xpos = dict(zip(KINDS, [0.0, 1.9, 3.3, 5.2, 6.6]))
    panels = [('goal_completion', 'Goal completion (%)', (0, 102), np.arange(0, 101, 20)),
              ('spl_geo', 'SPL, shortest path, all legs', (0, 1.0), np.arange(0, 1.01, 0.2))]
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.7), sharex=True)
    for ax, (col, ylabel, ylim, yticks), letter in zip(axes, panels, '(a)(b)'.replace(')(', ') (').split()):
        for kind in KINDS:
            color, marker, _ = PLANNER_STYLE[kind.split('_')[0]]
            v = np.sort(m.loc[m.kind == kind, col].to_numpy(float))
            # values closer than one marker height share a row; spread each row sideways by one marker width
            key = np.floor(v / ((ylim[1] - ylim[0]) * 0.035))
            off = np.zeros_like(v)
            for kv in np.unique(key):
                idx = np.where(key == kv)[0]
                off[idx] = (np.arange(len(idx)) - (len(idx) - 1) / 2) * 0.17
            half = max(0.4, np.abs(off).max() + 0.15)
            ax.scatter(xpos[kind] + off, v, s=20, marker=marker, color=color, edgecolor='white',
                       linewidth=0.5, zorder=3)
            ax.hlines(np.median(v), xpos[kind] - half, xpos[kind] + half, color=INK, linewidth=1.5, zorder=4)
        ax.set_title(letter, loc='left', fontsize=8, color=INK)
        ax.set_ylim(*ylim)
        ax.set_yticks(yticks)
        ax.set_ylabel(ylabel)
        ax.set_xticks([xpos[k] for k in KINDS])
        ax.set_xticklabels([LABEL[k] for k in KINDS])
        ax.set_xlim(-1.0, 7.6)
        ax.grid(axis='y', color=GRID, linewidth=0.5, zorder=0)
        ax.set_axisbelow(True)
        for s in ('top', 'right'):
            ax.spines[s].set_visible(False)
        ax.tick_params(length=0)
    handles = [plt.Line2D([], [], marker=mk, color=c, linestyle='', markersize=6, markeredgecolor='white',
                          label=lab) for c, mk, lab in PLANNER_STYLE.values()]
    handles.append(plt.Line2D([], [], color=INK, linewidth=1.5, label='median'))
    fig.legend(handles=handles, loc='lower center', ncol=4, frameon=False, bbox_to_anchor=(0.5, -0.01))
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(path + '.pdf')
    fig.savefig(path + '.png', dpi=300)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default=os.path.join(HERE, 'output'))
    args = ap.parse_args()
    g = pd.read_csv(os.path.join(args.out, 'g2g_units.csv'))
    m = pd.read_csv(os.path.join(args.out, 'multigoal_units.csv'))
    bm = pd.read_csv(BM_CSV)

    outputs = {'table_g2g.tex': table_g2g(g), 'table_failure_modes.tex': table_failure_modes(g, m),
               'table_multigoal.tex': table_multigoal(m), 'table_deployed.tex': table_deployed(m, bm)}
    for name, text in outputs.items():
        open(os.path.join(args.out, name), 'w').write(text)
    strip_plot(m, os.path.join(args.out, 'fig_multigoal_seeds'))
    meta = dict(script='scripts/metrics/make_tables.py', code_commit=code_commit(),
                generated=datetime.datetime.now().isoformat(timespec='seconds'),
                inputs=['output/g2g_units.csv', 'output/multigoal_units.csv', 'scripts/sim2real/seed_selection_bm.csv'],
                outputs=sorted(outputs) + ['fig_multigoal_seeds.pdf', 'fig_multigoal_seeds.png'])
    json.dump(meta, open(os.path.join(args.out, 'tables_meta.json'), 'w'), indent=2)
    print(f"commit {meta['code_commit']} | wrote {len(outputs)} tables and the figure to {args.out}")


if __name__ == '__main__':
    main()
