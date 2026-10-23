"""
Repeatability figures for ONE policy on ONE waypoint sequence (n repeated runs).

Outputs (saved to ./figures/):
  repeat_<path>_<world>_<policy>.png/pdf   ensemble map: faint runs, median path, percentile band per leg
  spread_<path>_<world>_<policy>.png/pdf   cross-track band width vs normalized progress, per leg
  repeat_<...>_runs.csv / _summary.csv     per-run metrics and mean/SD/CV summary
  (--compare) repeat_<path>_<world>_compare (maps overlaid), spread_<path>_<world>_compare,
              and a Mann-Whitney U table between policies

Usage:
  python visualize_repeats.py                 # PATH / WORLD / POLICY set below
  python visualize_repeats.py --all           # every batch in REPEAT_BATCHES (no pop-up windows)
  python visualize_repeats.py --compare       # also overlay policies (map + spread) + Mann-Whitney U
  python visualize_repeats.py --debug         # also save the leg-boundary diagnostic plot

Method: each run is split into legs (first pose within goal tolerance + margin, searched only after
the previous boundary), each leg is resampled by normalized arc length, the median path is the
pointwise coordinate-wise median, and the band is the percentile range of the signed cross-track
offset from the median path. Spread therefore includes EKF odom error; all runs share one docked start.
"""
import os, csv, argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.ticker import AutoMinorLocator

from viz_common import (CURRENT_DIR, GOAL_TOLERANCE, load_run, run_path_name, load_obstacles,
                        load_goals, draw_obstacles, draw_goals, apply_arena_style, make_arena_fig)
from run_registry import REPEAT_BATCHES, get_repeat_batch

# ===== Selection (edit like PATH/WORLD in visualize_path.py) =====
PATH, WORLD, POLICY = 'path_1', 'world_lab', 'TD3_delta'

# ===== Settings =====
N_RESAMPLE   = 200              # points per leg after arc-length resampling
BAND_PCT     = (10, 90)         # percentile band of the cross-track offset
TOL_MARGIN   = 0.00             # m, boundary = first pose within goal tolerance + margin
SMOOTH_WIN   = 7                # odd; light smoothing of the median path (samples)
ROBOT_RADIUS = 0.0              # m, subtracted from clearance (0 = robot centre to obstacle surface)
COLOR_BY_LEG = False
SAVE_DPI     = 300
FIG_DIR      = os.path.join(CURRENT_DIR, 'figures')
POLICY_COLORS = {'TD3_delta': 'C0', 'TD3_direct': 'C1'}


# ===================== Preparation =====================
def clean(run: dict) -> dict:
    """Drop poses with NaN x/y."""
    ok = np.isfinite(run['x']) & np.isfinite(run['y'])
    out = dict(run)
    for k in ('x', 'y', 'yaw', 't'):
        out[k] = run[k][ok]
    return out


def find_boundaries(x, y, goals, tol, margin=TOL_MARGIN):
    """
    Index of the pose that ends each leg. Goal k is searched only after the boundary of goal k-1,
    so a path that crosses an earlier goal's circle again cannot cut a leg short.
    Falls back to the closest approach (flagged) if no pose gets within tol + margin.
    """
    bounds, flags, start = [], [], 0
    for gx, gy in goals:
        # distance of the remaining poses to this goal
        d = np.hypot(x[start:] - gx, y[start:] - gy)
        # first pose inside the tolerance circle ends the leg; otherwise use the closest approach
        hit = np.flatnonzero(d <= tol + margin)
        if hit.size:
            i, flag = start + int(hit[0]), False
        else:
            i, flag = start + int(np.argmin(d)), True
        bounds.append(i)
        flags.append(flag)
        start = i
    return bounds, flags


def leg_slices(bounds):
    starts = [0] + list(bounds[:-1])
    return list(zip(starts, bounds))        # inclusive end index


def resample_leg(x, y, n=N_RESAMPLE):
    """Resample a leg to n points uniformly spaced in normalized arc length."""
    # drop repeated poses (robot standing still) so arc length is strictly increasing
    P = np.column_stack([x, y])
    keep = np.r_[True, np.hypot(*np.diff(P, axis=0).T) > 1e-6]
    P = P[keep]
    if len(P) < 2:
        raise ValueError('leg has fewer than 2 distinct poses')
    # normalized arc length u in [0, 1], then interpolate x and y on a uniform u grid
    s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
    u = s / s[-1]
    g = np.linspace(0.0, 1.0, n)
    return np.column_stack([np.interp(g, u, P[:, 0]), np.interp(g, u, P[:, 1])])


def smooth(a, w):
    """1D convolution moving average"""
    if w <= 1:
        return a
    pad = w // 2
    return np.convolve(np.pad(a, (pad, pad), mode='edge'), np.ones(w) / w, mode='valid')


def leg_stats(R):
    """R: (n_runs, N, 2) resampled leg. Median path + signed cross-track offsets (+ = left of travel)."""
    # median path across runs, lightly smoothed
    med = np.median(R, axis=0)
    med = np.column_stack([smooth(med[:, 0], SMOOTH_WIN), smooth(med[:, 1], SMOOTH_WIN)])
    # unit tangent and left-pointing normal of the median path
    tan = np.gradient(med, axis=0)
    tan /= np.linalg.norm(tan, axis=1, keepdims=True) + 1e-12
    nrm = np.column_stack([-tan[:, 1], tan[:, 0]])
    # signed cross-track offset of every run from the median, projected on the normal
    off = np.einsum('rnk,nk->rn', R - med, nrm)
    # percentile band of the offsets at each point along the leg
    lo, hi = np.percentile(off, BAND_PCT, axis=0)
    return {'median': med, 'normal': nrm, 'offsets': off, 'lo': lo, 'hi': hi, 'width': hi - lo}


def min_clearance(x, y, obstacles):
    """Minimum signed distance from the robot centre to any obstacle surface (negative = inside)."""
    ds = []
    for o in obstacles:
        # signed distance to the surface of each obstacle (cylinder: radius, box: rounded-box SDF)
        if o['type'] == 'cylinder':
            d = np.hypot(x - o['x'], y - o['y']) - o['radius']
        else:
            # express the point in the box's local frame so the box is axis-aligned
            c, s = np.cos(o['yaw']), np.sin(o['yaw'])
            dx, dy = x - o['x'], y - o['y']
            lx, ly = c * dx + s * dy, -s * dx + c * dy
            qx, qy = np.abs(lx) - o['sx'] / 2, np.abs(ly) - o['sy'] / 2
            d = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0)
        ds.append(d)
    # closest obstacle wins
    return float(np.min(ds)) - ROBOT_RADIUS


def run_metrics(r, bounds, obstacles):
    x, y, t = r['x'], r['y'], r['t']
    m = {'Duration (s)': float(r['elapsed_time']),
         'Path length (m)': float(r['path_length']),
         'Min clearance (m)': min_clearance(x, y, obstacles)}
    # per-leg time and travelled length (leg 1 starts at t=0, later legs at the previous boundary)
    for k, (s, e) in enumerate(leg_slices(bounds)):
        m[f'Leg {k + 1} time (s)'] = float(t[e] - (0.0 if k == 0 else t[s]))
        m[f'Leg {k + 1} length (m)'] = float(np.hypot(np.diff(x[s:e + 1]), np.diff(y[s:e + 1])).sum())
    return m


def leg_label(k):
    return f"{'S' if k == 0 else k}→{k + 1}"


# ===================== Analysis =====================
def analyze(path, world, policy):
    # 1. load and clean the repeated runs of one policy on one sequence
    keys = get_repeat_batch(path, world, policy)
    runs = [clean(load_run(k, world)) for k in keys]

    # 2. goals, obstacles and goal tolerance; all runs must share them
    names = {r['path_name'] for r in runs}
    assert len(names) == 1, f'mixed sequences in batch: {names}'
    goals = load_goals(world, names.pop())
    obstacles = load_obstacles(world)

    tols = {r['goal_tolerance'] for r in runs if r['goal_tolerance'] is not None}
    assert len(tols) <= 1, f'runs used different goal tolerances: {tols}'
    tol = tols.pop() if tols else GOAL_TOLERANCE

    # 3. split every run into legs (start->goal1, goal1->goal2, ...), flagging runs that never reached a goal
    bounds_all, flagged = [], []
    for k, r in zip(keys, runs):
        b, f = find_boundaries(r['x'], r['y'], goals, tol)
        if any(f):
            flagged.append((k, [i + 1 for i, fl in enumerate(f) if fl]))
        bounds_all.append(b)

    # 4. resample each leg to a common length and compute median path + band, per leg
    n_legs = len(goals)
    legs_R = []
    for leg in range(n_legs):
        legs_R.append(np.stack([resample_leg(r['x'][leg_slices(b)[leg][0]:leg_slices(b)[leg][1] + 1],
                                             r['y'][leg_slices(b)[leg][0]:leg_slices(b)[leg][1] + 1])
                                for r, b in zip(runs, bounds_all)]))
    stats = [leg_stats(R) for R in legs_R]

    # 5. scalar metrics per run, regrouped as metric -> array over runs
    per_run = [run_metrics(r, b, obstacles) for r, b in zip(runs, bounds_all)]
    metrics = {name: np.array([m[name] for m in per_run]) for name in per_run[0]}

    return {'path': path, 'world': world, 'policy': policy, 'keys': keys, 'runs': runs,
            'bounds': bounds_all, 'flagged': flagged, 'goals': goals, 'tol': tol,
            'obstacles': obstacles, 'stats': stats, 'metrics': metrics}


# ===================== Tables =====================
def summary_rows(metrics, stats):
    rows = []
    # run-level metrics: mean, SD, CV, range
    for name, v in metrics.items():
        mean, sd = v.mean(), v.std(ddof=1)
        # CV is meaningless for a signed quantity that can sit near zero (clearance)
        cv = np.nan if name.startswith('Min clearance') or not mean else 100 * sd / mean
        rows.append((name, mean, sd, cv, v.min(), v.max()))
    # band width per leg (start and max); no spread statistics since it is one value per leg
    for k, st in enumerate(stats):
        rows.append((f'Leg {k + 1} start band width (m)', st['width'][0], np.nan, np.nan, np.nan, np.nan))
        rows.append((f'Leg {k + 1} max band width (m)', st['width'].max(), np.nan, np.nan, np.nan, np.nan))
    return rows


def _f(v, nd=3):
    return '     –' if not np.isfinite(v) else f'{v:{6}.{nd}f}'


def print_table(title, rows):
    print(f'\n{title}')
    print(f'{"Metric":<32}{"Mean":>8}{"SD":>8}{"CV %":>8}{"Min":>8}{"Max":>8}')
    for name, mean, sd, cv, vmin, vmax in rows:
        print(f'{name:<32}{_f(mean):>8}{_f(sd):>8}{_f(cv, 1):>8}{_f(vmin):>8}{_f(vmax):>8}')


def write_csvs(res, tag):
    os.makedirs(FIG_DIR, exist_ok=True)
    # one row per run
    with open(os.path.join(FIG_DIR, f'repeat_{tag}_runs.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        names = list(res['metrics'])
        w.writerow(['run'] + names)
        for i, k in enumerate(res['keys']):
            w.writerow([k] + [f"{res['metrics'][n][i]:.4f}" for n in names])
    # one row per metric (summary statistics)
    with open(os.path.join(FIG_DIR, f'repeat_{tag}_summary.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['metric', 'mean', 'sd', 'cv_percent', 'min', 'max'])
        for row in summary_rows(res['metrics'], res['stats']):
            w.writerow([row[0]] + [f'{v:.4f}' if np.isfinite(v) else '' for v in row[1:]])


def mann_whitney(res_a, res_b):
    """Unpaired comparison (the two sets of runs are not matched pairs). Uncorrected p-values."""
    try:
        from scipy.stats import mannwhitneyu
    except ImportError:
        print('scipy not installed; skipping Mann-Whitney U')
        return
    a, b = res_a['policy'], res_b['policy']
    print(f'\nMann-Whitney U: {a} vs {b}  (r_rb > 0: {a} tends to be larger; p uncorrected)')
    print(f'{"Metric":<32}{"U":>8}{"p":>9}{"r_rb":>8}')
    rows = []
    # test every metric; r_rb is the rank-biserial effect size derived from U
    for name in res_a['metrics']:
        A, B = res_a['metrics'][name], res_b['metrics'][name]
        U, p = mannwhitneyu(A, B, alternative='two-sided')
        r_rb = 2 * U / (len(A) * len(B)) - 1
        rows.append((name, U, p, r_rb))
        print(f'{name:<32}{U:8.1f}{p:9.4f}{r_rb:8.2f}')
    os.makedirs(FIG_DIR, exist_ok=True)
    out = os.path.join(FIG_DIR, f"mwu_{res_a['path']}_{res_a['world']}_{a}_vs_{b}.csv")
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['metric', 'U', 'p_uncorrected', 'rank_biserial'])
        for name, U, p, r in rows:
            w.writerow([name, f'{U:.1f}', f'{p:.5f}', f'{r:.3f}'])


# ===================== Plots =====================
def save_fig(fig, name):
    os.makedirs(FIG_DIR, exist_ok=True)
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(FIG_DIR, f'{name}.{ext}'), dpi=SAVE_DPI, bbox_inches='tight')


def plot_ensemble(res, tag):
    fig, ax = make_arena_fig(res['world'])
    n = len(res['runs'])

    # individual runs (lower opacity)
    for r in res['runs']:
        ax.plot(r['x'], r['y'], color='0.45', lw=1.0, alpha=0.00, zorder=1)

    # median path and percentile band, one polygon per leg
    colors = [plt.cm.tab10(k % 10) if COLOR_BY_LEG else plt.cm.tab10(0) for k in range(len(res['stats']))]
    for k, st in enumerate(res['stats']):
        m, nrm = st['median'], st['normal']
        # band edges = median shifted along the normal by the upper / lower offset percentile
        up = m + st['hi'][:, None] * nrm
        dn = m + st['lo'][:, None] * nrm
        poly = np.vstack([up, dn[::-1]])
        ax.fill(poly[:, 0], poly[:, 1], color=colors[k], alpha=0.30, lw=0, zorder=2)
        ax.plot(m[:, 0], m[:, 1], color=colors[k], lw=2.2, zorder=3)

    # environment overlay and axis styling
    draw_obstacles(ax, res['obstacles'])
    draw_goals(ax, res['goals'], res['tol'])
    apply_arena_style(ax, res['world'])

    # legend
    handles = [Line2D([0], [0], color='0.45', lw=1.2, alpha=0.6, label=f'Individual runs (n={n})')]
    if COLOR_BY_LEG:
        handles += [Line2D([0], [0], color=colors[k], lw=2.2, label=f'Median, leg {leg_label(k)}')
                    for k in range(len(colors))]
    else:
        handles.append(Line2D([0], [0], color=colors[0], lw=2.2, label='Median path'))
    # handles.append(Patch(facecolor='0.5', alpha=0.3, label=f'{BAND_PCT[0]}–{BAND_PCT[1]}th pct. band'))
    ax.legend(handles=handles, loc='lower right', fontsize=14, framealpha=0.9)

    save_fig(fig, f'repeat_{tag}')
    return fig


def plot_ensemble_compare(results: list, name):
    """Policies overlaid on one map: faint runs, median path and percentile band, coloured by policy."""
    fig, ax = make_arena_fig(results[0]['world'])
    handles = []
    # one colour per policy: individual runs, then band and median per leg
    for res in results:
        color = POLICY_COLORS.get(res['policy'], 'C0')
        for r in res['runs']:
            ax.plot(r['x'], r['y'], color=color, lw=0.8, alpha=0.00, zorder=1)
        for st in res['stats']:
            m, nrm = st['median'], st['normal']
            poly = np.vstack([m + st['hi'][:, None] * nrm, (m + st['lo'][:, None] * nrm)[::-1]])
            ax.fill(poly[:, 0], poly[:, 1], color=color, alpha=0.25, lw=0, zorder=2)
            ax.plot(m[:, 0], m[:, 1], '-', color=color, lw=2.2, zorder=3)
        handles.append(Line2D([0], [0], color=color, lw=2.2,
                              label=f"{res['policy']} median (n={len(res['runs'])})"))

    # obstacles, goals and arena are shared, so draw them once from the first result
    ref = results[0]
    draw_obstacles(ax, ref['obstacles'])
    draw_goals(ax, ref['goals'], ref['tol'])
    apply_arena_style(ax, ref['world'])

    # handles.append(Patch(facecolor='0.5', alpha=0.3, label=f'{BAND_PCT[0]}–{BAND_PCT[1]}th pct. band'))
    ax.legend(handles=handles, loc='lower right', fontsize=14, framealpha=0.9)
    save_fig(fig, f'repeat_{name}_compare')
    return fig


def plot_spread(results: dict, name):
    """
    results: policy -> stats list. One wide axis: legs are laid end to end (progress 0 -> 1 within
    each leg) and separated by vertical dashed lines. Several policies are overlaid.
    """
    n_legs = len(next(iter(results.values())))
    fig, ax = plt.subplots(figsize=(max(6.0, 2.6 * n_legs), 3.8))
    # band width vs normalized progress; leg k occupies x in [k, k+1]
    g = np.linspace(0, 1, N_RESAMPLE)
    for pol, stats in results.items():
        color = POLICY_COLORS.get(pol, 'C0')
        for k, st in enumerate(stats):
            ax.plot(k + g, st['width'], lw=2, color=color, label=pol if k == 0 else None)
    # leg separators
    for k in range(1, n_legs):
        ax.axvline(k, color='0.4', lw=1.0, ls='--', zorder=0)

    # axis formatting: one centered tick label per leg
    ax.set_xlim(0, n_legs)
    ax.set_ylim(bottom=0)
    ax.set_xticks(np.arange(n_legs) + 0.5)
    ax.set_xticklabels([f'Leg {leg_label(k)}' for k in range(n_legs)], fontsize=12)
    ax.tick_params(axis='x', which='both', length=0)
    ax.yaxis.set_minor_locator(AutoMinorLocator())
    ax.tick_params(axis='y', which='both', direction='in')
    ax.set_ylabel(f'Band width, p{BAND_PCT[0]}–p{BAND_PCT[1]} (m)', fontsize=12)
    ax.set_xlabel('Progress along each leg (0 → 1)', fontsize=12)
    if len(results) > 1:
        ax.legend(fontsize=11, loc='upper right')
    fig.tight_layout()
    save_fig(fig, f'spread_{name}')
    return fig


def plot_boundaries(res, tag):
    """Diagnostic: detected leg boundaries for every run (checks the leg splitting)."""
    fig, ax = make_arena_fig(res['world'])
    # each run in grey, with its detected leg-end poses as coloured dots (one colour per leg)
    for r, b in zip(res['runs'], res['bounds']):
        ax.plot(r['x'], r['y'], color='0.5', lw=0.8, alpha=0.4, zorder=1)
        ax.scatter(r['x'][b], r['y'][b], s=22, c=[plt.cm.tab10(i % 10) for i in range(len(b))], zorder=5)
    draw_obstacles(ax, res['obstacles'])
    draw_goals(ax, res['goals'], res['tol'])
    apply_arena_style(ax, res['world'])
    save_fig(fig, f'boundaries_{tag}')
    return fig


# ===================== Driver =====================
def report(res, plot=True, debug=False):
    tag = f"{res['path']}_{res['world']}_{res['policy']}"
    n = len(res['runs'])
    print(f"\n=== {res['policy']} | {res['path']} | {res['world']} | n={n} | tol={res['tol']:.2f} m ===")
    # runs where a goal was never reached within tolerance
    for k, legs in res['flagged']:
        print(f'  WARNING {k}: no pose within tolerance+margin of goal(s) {legs}; closest approach used')

    # console table + CSVs always; single-policy figures only when not comparing
    print_table('Summary (mean, SD, CV, range)', summary_rows(res['metrics'], res['stats']))
    write_csvs(res, tag)
    if plot:
        plot_ensemble(res, tag)
        plot_spread({res['policy']: res['stats']}, tag)
    if debug:
        plot_boundaries(res, tag)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--all', action='store_true', help='every batch in REPEAT_BATCHES')
    ap.add_argument('--compare', action='store_true', help='overlay policies (map + spread) + Mann-Whitney U')
    ap.add_argument('--debug', action='store_true', help='save leg-boundary diagnostic plot')
    ap.add_argument('--no-show', action='store_true')
    args = ap.parse_args()

    # batches to process: all registered ones or the single selection at the top of the file
    configs = list(REPEAT_BATCHES) if args.all else [(PATH, WORLD, POLICY)]
    # analysis results are cached so --compare does not redo them
    cache = {}

    def get(cfg):
        if cfg not in cache:
            cache[cfg] = analyze(*cfg)
        return cache[cfg]

    # per-batch tables/CSVs; single-policy figures are skipped with --compare (comparison figures replace them)
    for cfg in configs:
        report(get(cfg), plot=not args.compare, debug=args.debug)

    # cross-policy comparison for each (path, world) that has more than one policy
    if args.compare:
        for path, world in sorted({(p, w) for p, w, _ in configs}):
            pols = sorted(p for (pa, w, p) in REPEAT_BATCHES if pa == path and w == world)
            if len(pols) < 2:
                continue
            res = [get((path, world, p)) for p in pols]
            plot_spread({r['policy']: r['stats'] for r in res}, f'{path}_{world}_compare')
            plot_ensemble_compare(res, f'{path}_{world}')
            mann_whitney(res[0], res[1])

    if args.no_show or args.all:
        plt.close('all')
    else:
        plt.show()


if __name__ == '__main__':
    main()