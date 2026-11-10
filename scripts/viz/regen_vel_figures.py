"""
Velocity figures of the paper appendix (fig:velocity_profiles_path<N>_world<W>), one pair per multi-goal scenario:
  <FIG_DIR>/APF_TD3_00401_1000_00390_1000_action_path<N>_world<W>.pdf          commanded velocity and acceleration
  <FIG_DIR>/APF_TD3_00401_1000_00390_1000_command_actual_path<N>_world<W>.pdf  commanded against executed velocity
for the six scenarios of table:APF_TD3_navigation_benchmark. <W> is the paper's world number (run directory
world_5 is world 4 in the paper, visualize_comparison_figures.DISPLAY_WORLD).

Runs: scripts/metrics/compute_smoothness.representative_keys, the same runs as the smoothness table and the
multi-goal path figure (median TD3_direct and TD3_delta policies, APF repeat closest to the median path length).
Plotted in the order APF, TD3_direct, TD3_delta, so the colours are C0 (blue), C1 (orange) and C2 (green).

Shading: the samples compute_smoothness leaves out of the table (variant excl_narrow, WINDOWS['narrow'] around each
goal arrival and the run end) are shaded in the colour of their run, on every panel of the action figure and in
that planner's column of the command figure.

Reuses visualize_vel.main() unchanged for the plots; only the run list, figure size, shading and savefig are set here.
Usage: regen_vel_figures.py
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(CURRENT_DIR, '..', 'metrics'))

import visualize_vel as vv                     # noqa: E402
import visualize_comparison_figures as vcf     # noqa: E402
import compute_smoothness as cs                # noqa: E402

FIG_DIR = os.path.join(CURRENT_DIR, '..', 'figures')
TAG = 'APF_TD3_00401_1000_00390_1000'
ACTION_FIGSIZE = (19.2, 10)  # wide aspect (~1.92) of the previously used vel/accel PDFs, so they fit on one page with the command/actual plot
PLANNERS = ['APF', 'TD3_direct', 'TD3_delta']
SHADE_ALPHA = 0.3

# The world 1 runs of the figures before this script picked them itself (paper 68ff6f1); the selection must not change.
WORLD1_BEFORE = {
    'path1': ['APF/path1_world_1_241026_1847', 'TD3_00401_1000/path1_world_1_251026_0253', 'TD3_00390_1000/path1_world_1_251026_0043'],
    'path2': ['APF/path2_world_1_241026_1852', 'TD3_00401_1000/path2_world_1_251026_0254', 'TD3_00390_1000/path2_world_1_251026_0045'],
    'path3': ['APF/path3_world_1_241026_1859', 'TD3_00401_1000/path3_world_1_251026_0255', 'TD3_00390_1000/path3_world_1_251026_0046'],
}


def shade(ax, arrivals, color):
    lo, hi = cs.WINDOWS['narrow']
    for a in arrivals:
        ax.axvspan(a + lo, a + hi, color=color, alpha=SHADE_ALPHA, lw=0, zorder=0)


def main():
    rep = cs.representative_keys(pd.read_csv(cs.AGG_CSV))
    models = [rep[('world_1', 'path1', pl)].split('/')[0] for pl in PLANNERS[1:]]
    assert TAG == 'APF_' + '_'.join(m[len('TD3_'):] if i else m for i, m in enumerate(models)), \
        ('representative policies changed, rename TAG and the paper assets', models)
    os.makedirs(FIG_DIR, exist_ok=True)

    for world, path in cs.SCEN:
        runs = [rep[(world, path, pl)] for pl in PLANNERS]
        if world == 'world_1':
            assert runs == WORLD1_BEFORE[path], (path, runs)
        name = f"{path}_{vcf.DISPLAY_WORLD.get(world, world).replace('_', '')}"
        print(f'===== {name} (runs {world}) =====')

        plt.close('all')
        vv.path_list = runs
        plt.show = lambda: None
        vv.main()
        fig_action, fig_cmd = (plt.figure(i) for i in plt.get_fignums()[:2])

        for i, key in enumerate(runs):
            arr = cs.arrivals(cs.load(key), world, path)
            for ax in fig_action.axes:
                shade(ax, arr, f'C{i}')
            for ax in fig_cmd.axes[i::len(runs)]:      # column i of the 3 x n_planners grid (row-major axes)
                shade(ax, arr, f'C{i}')

        fig_action.set_size_inches(*ACTION_FIGSIZE)
        fig_action.tight_layout()
        fig_action.savefig(os.path.join(FIG_DIR, f'{TAG}_action_{name}.pdf'), bbox_inches='tight')
        fig_cmd.savefig(os.path.join(FIG_DIR, f'{TAG}_command_actual_{name}.pdf'), bbox_inches='tight')


if __name__ == '__main__':
    main()
