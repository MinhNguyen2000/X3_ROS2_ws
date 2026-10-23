import os, json
import matplotlib.pyplot as plt
import numpy as np

from viz_common import (load_run, load_obstacles, load_goals, run_path_name,
                        draw_obstacles, draw_goals, apply_arena_style, make_arena_fig)
from run_registry import get_path_list

PATH = 'path_1'
WORLD = 'world_1'
ARROW_EVERY_N = 500
path_list = get_path_list(PATH, WORLD)

def main():
    fig, ax = make_arena_fig(WORLD)

    for run_key in path_list:
        run = load_run(run_key)
        # linespec = 'r--' if run['planner']=='TD3_delta' else 'b-.'
        line = ax.plot(run['x'], run['y'], '--', alpha=0.7, lw=1.5, zorder=1,
                       label=f"{run['planner']} | {run['elapsed_time']: 6.2f}s | {run['path_length']: 6.2f}m")

        idx = range(0, len(run['x']), ARROW_EVERY_N)
        ax.quiver([run['x'][i] for i in idx], [run['y'][i] for i in idx],
                  [np.cos(run['yaw'][i]) for i in idx], [np.sin(run['yaw'][i]) for i in idx],
                  color=line[0].get_color(), scale=50, width=0.004,
                  headwidth=4, headlength=5, alpha=0.8)

    draw_obstacles(ax, load_obstacles(WORLD))

    path_name = run_path_name(path_list[0])
    print(f'Comparing the policy performance on {path_name}')
    draw_goals(ax, load_goals(WORLD, path_name))

    apply_arena_style(ax, WORLD)
    ax.legend(loc='upper left', fontsize=14)
    plt.show()

if __name__ == "__main__":
    main()