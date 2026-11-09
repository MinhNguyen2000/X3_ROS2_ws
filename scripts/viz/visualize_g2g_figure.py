#!/usr/bin/env python3
"""
Compose the 3-panel goal-to-goal (G2G) figure: (a) APF, (b) TD3_direct, (c) TD3_delta.

Panel drawing follows visualize_batch_trials.py (same layers and styling). The three policies are the
representative ones picked by the median-success-rate rule (closest to the median G2G success rate of the
method's pool of final + best checkpoints; APF: the repeat closest to the median of its 5 repeats).

Usage (from the workspace root):
    python3 scripts/viz/visualize_g2g_figure.py
    python3 scripts/viz/visualize_g2g_figure.py --apf <csv> --direct <csv> --delta <csv> --out <pdf>
"""
import os
import argparse

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.lines import Line2D

from viz_common import CURRENT_DIR, WS_DIR, sdf_path, parse_sdf_obstacles, draw_obstacles
import json

ODOM_INIT = {'x': -3.0, 'y': -3.0}      # odom origin in the plotting frame (world_1..5)
GOAL_TOLERANCE = 0.20                   # m, trial_orchestrator goal_tolerance
WORLD = 'world_1'
OK_COLOR, FAIL_COLOR = '#228B22', '#CD5C5C'
LEGEND_FONTSIZE = 22.4    # 5.9 pt in the paper (24 in figure scaled to the 455 pt text width), same as the multi-goal legend, which sets this limit; with two-line labels this legend fits one row up to 24.2 (6.4 pt)
# legend labels, each on two lines (subject, then what happened) so the one-row legend is narrower
LEGEND_LABELS = ['Successful\ntrajectory', 'Failed\ntrajectory', 'Goal\nreached', 'Goal\nnot reached',
                 'Collision\npoint', 'Goal\ntimeout', 'Obstacles\n(box & cylinder)']

# Median-rule selections (see seed_selection in the paper repo / G2G pool ranking)
DEFAULTS = {
    'apf':    'trial_results/APF_variance/run3/world_1_APF_20261025_220328.csv',
    'direct': 'trial_results/world_1_TD3_00404_1000_20260910_235516.csv',
    'delta':  'trial_results/world_1_TD3_00395_1000_20260910_143411.csv',
}
DEFAULT_OUT = os.path.join(CURRENT_DIR, 'figures', 'G2G_APF_TD3_00404_1000_TD3_00395_1000_world1.pdf')


def draw_g2g_panel(ax, csv_path, obstacles):
    """Draw obstacles, trajectories, failure markers, goals and start pose of one G2G batch. Returns stats."""
    df = pd.read_csv(csv_path)
    draw_obstacles(ax, obstacles)

    n_collision = n_timeout = n_missing = 0
    for idx, row in df.iterrows():
        json_path = os.path.join(WS_DIR, row['timeseries_log_path'])
        if not os.path.exists(json_path):
            print(f'[WARNING] JSON not found: {json_path}')
            n_missing += 1
            continue
        with open(json_path, 'r') as f:
            trial = json.load(f)
        if not trial.get('gt_odom'):
            print(f'[WARNING] No trajectory data for trial {idx}')
            n_missing += 1
            continue

        x = np.array([p['x'] + ODOM_INIT['x'] for p in trial['gt_odom']])
        y = np.array([p['y'] + ODOM_INIT['y'] for p in trial['gt_odom']])

        if row['success']:
            ax.plot(x, y, alpha=0.25, linewidth=1.0, linestyle='-', color=OK_COLOR)
            continue

        ax.plot(x, y, alpha=0.50, linewidth=1.0, linestyle='--', color=FAIL_COLOR)
        end_x, end_y = x[-1], y[-1]
        reason = str(row['failure_reason'])
        if 'Obstacle' in reason:
            ax.plot(end_x, end_y, marker='x', markersize=10, color='red', markeredgewidth=2, zorder=10)
            n_collision += 1
        if 'timeout' in reason:
            ax.plot(end_x, end_y, marker='o', markersize=3, color='red', markeredgewidth=2, zorder=10)
            n_timeout += 1
        gx, gy = row['goal_x'] + ODOM_INIT['x'], row['goal_y'] + ODOM_INIT['y']
        ax.plot([gx, end_x], [gy, end_y], color='red', alpha=0.25, linewidth=1.0, linestyle=':', zorder=5)

    # goals: dot + tolerance circle (same scatter-size conversion as the original script)
    goal_scatter_size = GOAL_TOLERANCE * 100 / (2.54 / 72)
    for _, row in df.iterrows():
        gx, gy = row['goal_x'] + ODOM_INIT['x'], row['goal_y'] + ODOM_INIT['y']
        color = OK_COLOR if row['success'] else FAIL_COLOR
        ax.scatter(gx, gy, c=color, edgecolors=color, s=10, alpha=0.3, zorder=2)
        ax.scatter(gx, gy, linewidths=1, linestyle='--', c=color, edgecolors=color,
                   s=goal_scatter_size, alpha=0.3, zorder=2)

    # start pose
    ax.quiver(-3, -3, 1, 0, color='green', scale=50, width=0.004, headwidth=4, headlength=5, alpha=0.8)

    ax.set_xlim(-4.0, 4.0)
    ax.set_ylim(-4.0, 4.0)
    ax.set_aspect('equal')
    ax.minorticks_on()
    ax.tick_params(axis='both', which='both', direction='in', labelsize=12)
    ax.set_xlabel('X (m)', fontsize=12)
    ax.grid(False)

    return {'n': len(df), 'success': int(df['success'].sum()), 'collision': n_collision,
            'timeout': n_timeout, 'missing': n_missing}


def main():
    ap = argparse.ArgumentParser(description='3-panel G2G figure (APF / TD3_direct / TD3_delta)')
    for k in DEFAULTS:
        ap.add_argument(f'--{k}', default=DEFAULTS[k], help=f'{k} batch CSV (relative to workspace root)')
    ap.add_argument('--out', default=DEFAULT_OUT, help='output PDF (a PNG preview is saved next to it)')
    args = ap.parse_args()

    obstacles = parse_sdf_obstacles(sdf_path(WORLD))
    panels = [('(a) APF', args.apf), ('(b) TD3_direct', args.direct), ('(c) TD3_delta', args.delta)]

    fig, axes = plt.subplots(1, 3, figsize=(24, 8.6))
    for ax, (title, csv) in zip(axes, panels):
        csv_path = csv if os.path.isabs(csv) else os.path.join(WS_DIR, csv)
        if not os.path.exists(csv_path):
            raise FileNotFoundError(csv_path)
        s = draw_g2g_panel(ax, obstacles=obstacles, csv_path=csv_path)
        ax.text(0.5, -0.12, title, transform=ax.transAxes, ha='center', va='top', fontsize=14)
        print(f"{title:15s} n={s['n']} success={s['success']} ({100 * s['success'] / s['n']:.2f}%) "
              f"collision={s['collision']} timeout={s['timeout']} missing={s['missing']}  <- {os.path.basename(csv_path)}")
    axes[0].set_ylabel('Y (m)', fontsize=12)

    legends = [
        Line2D([0], [0], color=OK_COLOR, alpha=0.25, linewidth=2.0, linestyle='-', label='Successful trajectory'),
        Line2D([0], [0], color=FAIL_COLOR, alpha=0.50, linewidth=2.0, linestyle='--', label='Failed trajectory'),
        Line2D([0], [0], marker='o', color=OK_COLOR, markerfacecolor=OK_COLOR, markersize=8, alpha=0.5,
               linestyle='None', label='Goal reached'),
        Line2D([0], [0], marker='o', color=FAIL_COLOR, markerfacecolor=FAIL_COLOR, markersize=8, alpha=0.5,
               linestyle='None', markeredgewidth=1.5, label='Goal not reached'),
        Line2D([0], [0], marker='x', color='red', markersize=10, linestyle='None', markeredgewidth=2,
               label='Collision point'),
        Line2D([0], [0], marker='o', color='red', markersize=3, linestyle='None', markeredgewidth=2,
               label='Goal timeout'),
        patches.Patch(facecolor='red', alpha=0.5, edgecolor='red', linewidth=1.5, label='Obstacles (box & cylinder)'),
    ]
    for handle, label in zip(legends, LEGEND_LABELS):
        handle.set_label(label)
    # one legend below the panels without a frame, as in the multi-goal path figure
    leg = fig.legend(handles=legends, loc='lower center', ncol=len(legends), fontsize=LEGEND_FONTSIZE, frameon=False,
                     handlelength=2.0, bbox_to_anchor=(0.5, 0.0))
    # the bottom margin holds the legend plus the (a) to (c) panel labels below the x axis labels
    fig.canvas.draw()
    leg_h = leg.get_window_extent().height / fig.bbox.height
    fig.tight_layout(rect=(0, leg_h + 0.10, 1, 1))

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    fig.savefig(args.out)
    fig.savefig(os.path.splitext(args.out)[0] + '.png', dpi=150)
    print('saved', args.out)


if __name__ == '__main__':
    main()
