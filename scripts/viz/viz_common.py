"""
Shared configuration, loaders and drawing helpers for the path visualization scripts
(visualize_path.py, visualize_path_variance.py).

Place this file in the same folder as visualize_path.py so the relative paths below
resolve exactly as they did before.
"""
import os, re, json
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# ===== Paths (resolved relative to this file) =====
CURRENT_DIR     = os.path.dirname(os.path.abspath(__file__))
WS_DIR          = os.path.abspath(os.path.join(CURRENT_DIR, '..', '..'))
SAVED_PATHS_DIR = os.path.join(WS_DIR, 'scripts', 'recorded_paths')
PATHS_JSON_DIR  = os.path.join(WS_DIR, 'src', 'x3_nav_bringup', 'paths')
WORLDS_DIR      = os.path.join(WS_DIR, 'src', 'x3_description', 'worlds')


def paths_json_path(world: str) -> str:
    return os.path.join(PATHS_JSON_DIR, f'paths_{world}.json')


def sdf_path(world: str) -> str:
    return os.path.join(WORLDS_DIR, f'{world}.sdf')


# ===== Frame / style constants =====
FT = 0.305                                        # feet -> m

# odom_init: odom origin expressed in the plotting frame (m); arena_w / arena_l: y / x extent (m)
WORLD_CONFIGS = {
    'sim':       {'odom_init': (-3.0, -3.0),            'arena_w': 8.0,    'arena_l': 8.0},      # world_1..5
    'world_lab': {'odom_init': (-5.5 * FT, -2.5 * FT),  'arena_w': 9 * FT, 'arena_l': 15 * FT},  # physical test bed
}


def world_config(world: str) -> dict:
    """Frame/arena parameters for a world tag ('world_lab', 'world_3', 'world3', ...)."""
    if world == 'world_lab':
        return WORLD_CONFIGS['world_lab']
    if re.fullmatch(r'world_?\d+', world):
        return WORLD_CONFIGS['sim']
    raise ValueError(f"Unknown world '{world}': add it to WORLD_CONFIGS in viz_common.py")


def world_from_run_key(run_key: str) -> str:
    """'TD3_00395_1000/path1_world_lab_151026_2036' -> 'world_lab'."""
    parts = run_key.split('/')[1].split('_')[1:-2]
    return '_'.join(parts)


GOAL_TOLERANCE  = 0.30                            # goal circle radius (m)
DELTA_MAX_ID    = 395                             # TD3 model ids <= this are delta, above are direct


# ===== Run naming =====
def planner_label(model_folder: str) -> str:
    """
    Categorize the TD3 policies as TD3_delta or TD3_direct
    'TD3_00395_1000' -> 'TD3_delta', 'TD3_00404_500' -> 'TD3_direct', 'APF' -> 'APF'.
    """
    parts = model_folder.split('_')[:2]
    if len(parts) > 1:
        parts[1] = 'delta' if int(parts[1]) <= DELTA_MAX_ID else 'direct'
    return '_'.join(parts)


def run_path_name(run_key: str) -> str:
    """'TD3_00395_1000/path1_world_lab_151026_2036' -> 'path1_world_lab' (key into paths_<world>.json)."""
    return '_'.join(run_key.split('/')[1].split('_')[:3])


# ===== Loading =====
def load_run(run_key: str, world: str = None) -> dict:
    """
    Load one recorded run, with poses shifted into the plotting frame.
    `world` selects the plotting-frame offset; if omitted it is parsed from the run key.
    Returns arrays x, y, yaw, t plus metadata. path_length is recomputed from the poses
    (the saved total_distance field is not trusted).
    """
    # world decides the odom -> plotting frame offset
    world = world or world_from_run_key(run_key)
    odom_init = world_config(world)['odom_init']
    with open(os.path.join(SAVED_PATHS_DIR, f'{run_key}.json'), 'r') as f:
        data = json.load(f)

    # poses -> arrays, shifted from the odom frame into the plotting frame (yaw/t are frame-independent)
    poses = data['poses']
    nan = float('nan')
    x   = np.array([p.get('x', nan) for p in poses], dtype=float) + odom_init[0]
    y   = np.array([p.get('y', nan) for p in poses], dtype=float) + odom_init[1]
    yaw = np.array([p.get('yaw', nan) for p in poses], dtype=float)
    t   = np.array([p.get('t', nan) for p in poses], dtype=float)

    # path length is summed over valid (non-NaN) poses only
    ok = np.isfinite(x) & np.isfinite(y)
    path_length = float(np.hypot(np.diff(x[ok]), np.diff(y[ok])).sum())

    return {
        'key':            run_key,
        'planner':        planner_label(run_key.split('/')[0]),
        'path_name':      run_path_name(run_key),
        'x': x, 'y': y, 'yaw': yaw, 't': t,
        'elapsed_time':   data['elapsed_time'],
        'total_distance': data['total_distance'],     # as saved; see path_length
        'path_length':    path_length,
        'goal_tolerance': data.get('goal_tolerance'),
        'raw':            data,
    }


def load_goals(world: str, path_name: str) -> np.ndarray:
    """Waypoints of `path_name` in the plotting frame, shape (N, 2)."""
    # waypoints are stored in the odom frame; shift them into the plotting frame
    with open(paths_json_path(world), 'r') as f:
        available = json.load(f)
    goals = np.array(available[path_name], dtype=float)
    odom_init = world_config(world)['odom_init']
    goals[:, 0] += odom_init[0]
    goals[:, 1] += odom_init[1]
    return goals


# ===== Geometry =====
def rotated_box_corners(cx: float, cy: float, sx: float, sy: float, yaw: float) -> np.ndarray:
    """4 corners of a box centered at (cx, cy), size (sx, sy), rotated by yaw (rad, CCW) about its center."""
    # corners in the box's local frame, then rotate by yaw and translate to the center
    hx, hy = sx / 2.0, sy / 2.0
    local = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]])
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    return local @ R.T + np.array([cx, cy])


def parse_sdf_obstacles(path: str) -> list:
    """Parse box/cylinder obstacles from an SDF world. Returns dicts with type, x, y, geometry, yaw."""
    root = ET.parse(path).getroot()
    obstacles = []

    for elem in root.iter('model'):
        name = elem.get('name', '')
        is_box = name.startswith('box')
        is_cylinder = name.startswith('cylinder')
        if not (is_box or is_cylinder):
            continue

        # pose text is 'x y z roll pitch yaw'; only x, y and yaw matter for a top-down plot
        vals = [float(v) for v in elem.find('pose').text.strip().split()]
        x, y = vals[0], vals[1]
        yaw = vals[5] if len(vals) >= 6 else 0.0

        # collision geometry gives the true obstacle size
        if is_box:
            sx, sy, _ = [float(v) for v in elem.find('.//collision/geometry/box/size').text.strip().split()]
            obstacles.append({'name': name, 'type': 'box', 'x': x, 'y': y, 'sx': sx, 'sy': sy, 'yaw': yaw})
        else:
            radius = float(elem.find('.//collision/geometry/cylinder/radius').text.strip())
            obstacles.append({'name': name, 'type': 'cylinder', 'x': x, 'y': y, 'radius': radius, 'yaw': yaw})

    return obstacles


def load_obstacles(world: str) -> list:
    return parse_sdf_obstacles(sdf_path(world))


# ===== Drawing =====
def draw_obstacles(ax, obstacles: list):
    # obstacles are semi-transparent red so overlapping paths stay visible
    for obs in obstacles:
        if obs['type'] == 'box':
            corners = rotated_box_corners(obs['x'], obs['y'], obs['sx'], obs['sy'], obs.get('yaw', 0.0))
            ax.add_patch(patches.Polygon(corners, closed=True, color='red', alpha=0.5, linewidth=2))
        elif obs['type'] == 'cylinder':
            ax.add_patch(patches.Circle((obs['x'], obs['y']), obs['radius'], color='red', alpha=0.5, linewidth=2))


def draw_goals(ax, goals: np.ndarray, tol: float = GOAL_TOLERANCE):
    """Numbered goal tolerance circles (radius `tol` in data units, so they scale with the arena)."""
    # one tolerance circle plus its 1-based visiting order per goal
    for i, (gx, gy) in enumerate(goals):
        ax.add_patch(patches.Circle((gx, gy), tol, linewidth=1, linestyle='--', edgecolor='#1C7826',
                                    facecolor='#3EA047', alpha=0.4, zorder=10))
        ax.annotate(str(i + 1), xy=(gx, gy), xycoords='data', ha='center', va='center',
                    fontsize=14, fontweight='bold', color='black', zorder=3)


def make_arena_fig(world: str, long_side: float = 12.0):
    """Figure + axes whose aspect ratio matches the arena of `world` (long side = `long_side` in)."""
    cfg = world_config(world)
    return plt.subplots(figsize=(long_side, long_side * cfg['arena_w'] / cfg['arena_l']))


def apply_arena_style(ax, world: str):
    """Axis limits, ticks and labels shared by every figure."""
    cfg = world_config(world)
    # arena is centered on the plotting-frame origin
    ax.set_xlim(-cfg['arena_l'] / 2, cfg['arena_l'] / 2)
    ax.set_ylim(-cfg['arena_w'] / 2, cfg['arena_w'] / 2)
    # inward ticks, no grid, equal aspect so metres look the same on both axes
    ax.minorticks_on()
    ax.tick_params(axis='both', which='minor', direction='in', labelsize=14)
    ax.tick_params(axis='both', which='major', direction='in', labelsize=14)
    ax.grid(False)
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_aspect('equal')