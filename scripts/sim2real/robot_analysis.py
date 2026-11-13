#!/usr/bin/env python3
"""Paper tables, statistics and figures of the confirmatory real-robot runs (P7), from robot/manifest_robot.csv and the run JSONs.

Builds on robot_results.py (kept runs, online goal count, failure type, path length, time) and adds, per run:
  - real-robot SPL with the shortest collision-free path of the lab world (scripts/metrics/shortest_path.py, world_lab.sdf, clearance
    0.21 m), defined as in the Gazebo tables: SPL per leg = L / max(L, P), L = shortest path from the pose where the leg started to the
    goal, P = travelled length of the leg; failed and not-attempted legs score 0; run SPL = mean over all planned legs ('all legs'),
    'reached legs' = mean over the legs the robot completed;
  - minimum distance of the robot centre to any obstacle surface (scripts/viz, as the repeatability figures);
  - failure forensics for the failed runs: failing goal, message, last pose, nearest obstacle, speed before the stop;
  - RMS acceleration of the commanded and the executed velocity (scripts/viz/visualize_vel.compute_accel).
Statistics (the policy is the unit of replication; there are 8 policies and 20 runs each): Wilson 95 % interval of each policy's
full-sequence success, cluster bootstrap over policies then runs for the groups (planner, checkpoint, tier), Fisher exact test of path 1
against path 2 within a policy, Spearman correlation of the Gazebo and the robot values over the 8 policies (exploratory).
Score = online goal count (notes/scoring_rule_decision_20261028.md): goals reached according to the robot's own odometry.

Outputs (--out, default robot/): robot_run_metrics.csv, robot_failures.csv, robot_policy_stats.csv, robot_group_stats.csv,
robot_stats.json, table_robot_results.tex, table_robot_by_path.tex, table_robot_groups.tex, table_robot_failures.tex,
table_robot_smoothness.tex, robot_path_spread.{csv,json} (run-to-run band width of the paths, see path_spread),
fig_robot_sim_vs_real.{pdf,png}, fig_robot_paths.{pdf,png}, panels/repeat_<path>_world_lab_<variant>_compare.{pdf,png}.

Usage: robot_analysis.py [--out DIR] [--n-boot N] [--seed S] [--no-figs | --paths-only | --spread-only | --simreal-only]
"""
import argparse
import collections
import csv
import datetime
import hashlib
import json
import math
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for sub in ('', '../viz', '../metrics'):
    sys.path.insert(0, os.path.normpath(os.path.join(HERE, sub)))
import robot_results as rr  # noqa: E402
import shortest_path as sps  # noqa: E402
import viz_common as vc  # noqa: E402

WORLD = 'world_lab'
GOAL_SLACK = 0.01                      # m, the unlogged decision-time pose (scoring rule note, section 1)
SIM_UNITS = os.path.join(HERE, '..', 'metrics', 'output', 'multigoal_units.csv')
# trajectory figure style of scripts/viz/visualize_comparison_figures.py (COLORS, GRID_STYLE), so the figures agree in the paper
PLANNER_COLOR = {'direct': 'tab:orange', 'delta': 'tab:green'}
# B and M are both solid and differ by shade of the planner colour (B darker, M the planner colour of those figures); B is drawn on top of
# the slightly thicker M, so both stay visible where the medians coincide. Failed runs are thin and faint below the medians, and an x marks
# the pose where each ended, so they cannot be mistaken for a median and stay countable where many overlap.
PATH_STYLE = {'B': dict(ls='-', lw=1.1, zorder=4), 'M': dict(ls='-', lw=1.6, zorder=3)}
B_DARKEN = 0.6                                                # factor on the planner colour (RGB) for policy B
FAILED_STYLE = dict(ls='-', lw=0.6, alpha=0.6, zorder=2.5)
FAILED_END = dict(marker='x', ms=6, mew=1.0, ls='', zorder=5)
TICK_STEP = 0.5                                               # m, major ticks of both axes
PANEL_IN = 8.35                                               # width of one panel in the composite figure (in)
PLANNER_STYLE = {'direct': ('#ff7f0e', 's'), 'delta': ('#2ca02c', '^')}       # as make_tables.py
INK, MUTED, GRID = '#222222', '#666666', '#d9d9d9'
LABEL = rr.LABEL


# ----------------------------------------------------------------------------- statistics (pure, unit tested)
def wilson(k, n, z=1.96):
    """Wilson score interval of a proportion k/n."""
    if n == 0:
        return float('nan'), float('nan')
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return c - h, c + h


def fisher_two_sided(a, b, c, d):
    """Two-sided Fisher exact p of the 2x2 table [[a, b], [c, d]] (sum of the tables no more likely than the observed one)."""
    r1, r2, c1 = a + b, c + d, a + c
    n = r1 + r2
    p_of = lambda x: math.comb(r1, x) * math.comb(r2, c1 - x) / math.comb(n, c1)
    lo, hi = max(0, c1 - r2), min(r1, c1)
    p_obs = p_of(a)
    return float(min(1.0, sum(p_of(x) for x in range(lo, hi + 1) if p_of(x) <= p_obs * (1 + 1e-9))))


def ranks(v):
    """Average ranks (ties share the mean rank)."""
    v = np.asarray(v, float)
    order = np.argsort(v, kind='mergesort')
    r = np.empty(len(v))
    i = 0
    while i < len(v):
        j = i
        while j + 1 < len(v) and v[order[j + 1]] == v[order[i]]:
            j += 1
        r[order[i:j + 1]] = (i + j) / 2 + 1
        i = j + 1
    return r


def spearman(x, y):
    """Spearman rank correlation (NaN when one variable is constant)."""
    rx, ry = ranks(x), ranks(y)
    if rx.std() == 0 or ry.std() == 0:
        return float('nan')
    return float(np.corrcoef(rx, ry)[0, 1])


def cluster_bootstrap(units, n_boot, rng, level=95):
    """Cluster bootstrap of the mean over `units` (list of per-run value arrays, one per policy): resample the policies with
    replacement, then the runs of each drawn policy. Returns the replicate array (n_boot,)."""
    reps = np.empty(n_boot)
    m = len(units)
    for b in range(n_boot):
        drawn = rng.integers(0, m, m)
        reps[b] = np.mean([rng.choice(units[i], len(units[i])).mean() for i in drawn])
    return reps


def pct_ci(reps, level=95):
    lo, hi = np.percentile(reps, [(100 - level) / 2, 100 - (100 - level) / 2])
    return float(lo), float(hi)


def quart(v):
    v = np.asarray([x for x in v if not (x is None or (isinstance(x, float) and math.isnan(x)))], float)
    if not len(v):
        return float('nan'), float('nan'), float('nan')
    return tuple(float(x) for x in np.percentile(v, [50, 25, 75]))


# ----------------------------------------------------------------------------- per-run metrics
def load_walls(sdf):
    """Arena walls (models named wall_*) of an SDF world as box obstacles in the plotting frame."""
    out = []
    for m in ET.parse(sdf).getroot().iter('model'):
        if m.get('name', '').startswith('wall'):
            v = [float(t) for t in m.find('pose').text.split()]
            sx, sy, _ = [float(t) for t in m.find('.//collision/geometry/box/size').text.split()]
            out.append(dict(name=m.get('name'), type='box', x=v[0], y=v[1], sx=sx, sy=sy, yaw=v[5] if len(v) > 5 else 0.0))
    return out


class Lab:
    """World constants of the lab: goals in the odom frame, obstacles in the plotting frame, shortest-path planner."""

    def __init__(self):
        self.odom_init = vc.world_config(WORLD)['odom_init']                       # odom origin in the plotting frame
        self.sp_shift = (self.odom_init[0] - sps.SPAWN_XY[0], self.odom_init[1] - sps.SPAWN_XY[1])   # odom -> shortest_path frame
        with open(vc.paths_json_path(WORLD)) as f:
            self.goals = {k: np.array(v, float) for k, v in json.load(f).items()}
        self.obstacles = vc.load_obstacles(WORLD)
        self.stop_obstacles = self.obstacles + load_walls(vc.sdf_path(WORLD))      # failure forensics also see the arena walls
        self.sp = sps.ShortestPath(WORLD)

    def to_sp(self, p):
        return (p[0] + self.sp_shift[0], p[1] + self.sp_shift[1])


def find_boundaries(x, y, goals, tol, margin):
    """Index of the first pose within tol + margin of each goal, searched after the previous one (closest approach otherwise)."""
    bounds, start = [], 0
    for gx, gy in goals:
        d = np.hypot(x[start:] - gx, y[start:] - gy)
        hit = np.flatnonzero(d <= tol + margin)
        i = start + (int(hit[0]) if hit.size else int(np.argmin(d)))
        bounds.append(i)
        start = i
    return bounds


def leg_spl(x, y, goals, k, tol, lab):
    """SPL of each of the k completed legs (list of k floats)."""
    if k == 0:
        return []
    bounds = find_boundaries(x, y, goals[:k], tol, GOAL_SLACK)
    out, start = [], 0
    for i, e in enumerate(bounds):
        P = float(np.hypot(np.diff(x[start:e + 1]), np.diff(y[start:e + 1])).sum())
        L, _ = lab.sp.length(lab.to_sp((x[start], y[start])), lab.to_sp(goals[i]))
        out.append(L / max(L, P, 1e-9))
        start = e
    return out


def min_obstacle_clearance(xp, yp, obstacles):
    """Minimum signed distance (m) of the robot centre to any obstacle surface, plotting-frame arrays; also the nearest obstacle name."""
    best, name = float('inf'), ''
    for o in obstacles:
        c, s = math.cos(o.get('yaw', 0.0)), math.sin(o.get('yaw', 0.0))
        if o['type'] == 'cylinder':
            d = np.hypot(xp - o['x'], yp - o['y']) - o['radius']
        else:
            dx, dy = xp - o['x'], yp - o['y']
            qx, qy = np.abs(c * dx + s * dy) - o['sx'] / 2, np.abs(-s * dx + c * dy) - o['sy'] / 2
            d = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0)
        m = float(np.min(d))
        if m < best:
            best, name = m, o['name']
    return best, name


def rms_accel(samples):
    """(RMS linear, RMS angular) acceleration of a [{t, vx, vyaw}] log by finite differences; NaN when too short."""
    if not samples or len(samples) < 3:
        return float('nan'), float('nan')
    t = np.array([s['t'] for s in samples], float)
    dt = np.diff(t)
    ok = dt > 0
    out = []
    for key in ('vx', 'vyaw'):
        v = np.array([s[key] for s in samples], float)
        a = np.diff(v)[ok] / dt[ok]
        out.append(float(np.sqrt(np.mean(a ** 2))))
    return out[0], out[1]


SMOOTH_S = 0.5            # boxcar window (s) of the smoothed acceleration; the logged wheel velocity is quantised (0.024 rad/s, 0.007 m/s at 50 Hz)
SMOOTH_DT = 0.02          # uniform grid (s) the logs are resampled to before smoothing (the log timestamps jitter)


def smooth_rms_accel(samples, window=SMOOTH_S, dt=SMOOTH_DT):
    """(RMS linear, RMS angular) acceleration of a [{t, vx, vyaw}] log after resampling to a uniform grid and averaging the velocity over
    `window` seconds, so that the quantisation of the logged velocity does not count as acceleration; NaN when the log is shorter than the window."""
    if not samples or len(samples) < 3:
        return float('nan'), float('nan')
    t = np.array([s['t'] for s in samples], float)
    order = np.argsort(t)
    t = t[order]
    grid = np.arange(t[0], t[-1], dt)
    k = int(round(window / dt))
    if len(grid) < k + 2:
        return float('nan'), float('nan')
    out = []
    for key in ('vx', 'vyaw'):
        v = np.interp(grid, t, np.array([s[key] for s in samples], float)[order])
        a = np.diff(np.convolve(v, np.ones(k) / k, mode='valid')) / dt
        out.append(float(np.sqrt(np.mean(a ** 2))))
    return out[0], out[1]


def tail_speed(samples, seconds=3.0):
    """Mean |vx| over the last `seconds` of a velocity log (NaN when empty)."""
    if not samples:
        return float('nan')
    t = np.array([s['t'] for s in samples], float)
    vx = np.array([s['vx'] for s in samples], float)
    return float(np.mean(np.abs(vx[t >= t[-1] - seconds])))


def run_metrics(row, root, lab):
    """All per-run metrics of one kept manifest row."""
    rec = rr.run_record(row, root)
    d = json.load(open(os.path.join(root, row['saved_file'])))
    poses = [p for p in d['poses'] if np.isfinite(p.get('x', np.nan)) and np.isfinite(p.get('y', np.nan))]
    x, y = np.array([p['x'] for p in poses]), np.array([p['y'] for p in poses])
    tol = d.get('goal_tolerance') or 0.3
    goals = lab.goals[rec['path']]
    legs = leg_spl(x, y, goals, rec['goals'], tol, lab)
    clr, clr_name = min_obstacle_clearance(x + lab.odom_init[0], y + lab.odom_init[1], lab.obstacles)
    cmd, act = d.get('velocities') or [], d.get('actual_velocities') or []
    a_cmd, a_act = rms_accel(cmd), rms_accel(act)
    s_cmd, s_act = smooth_rms_accel(cmd), smooth_rms_accel(act)
    rec.update(spl_all=sum(legs) / rec['total'], spl_reached=float(np.mean(legs)) if legs else float('nan'),
               min_clearance=clr, cmd_rms_a_lin=a_cmd[0], cmd_rms_a_ang=a_cmd[1], act_rms_a_lin=a_act[0], act_rms_a_ang=a_act[1],
               cmd_smooth_a_lin=s_cmd[0], cmd_smooth_a_ang=s_cmd[1], act_smooth_a_lin=s_act[0], act_smooth_a_ang=s_act[1],
               planner=row['variant'].split('_')[0], checkpoint=row['variant'].split('_')[1])
    msg = (d.get('result') or {}).get('message') or ''
    m = re.search(r'Obstacle hit with distance\s+([0-9.]+),\s*index\s+(\d+)', msg)
    nxt = goals[rec['goals']] if rec['goals'] < len(goals) else None
    rec.update(fail_goal=rec['goals'] + 1 if not rec['success'] else '', message=msg if not rec['success'] else '',
               hit_distance=float(m.group(1)) if m and not rec['success'] else '',
               hit_index=int(m.group(2)) if m and not rec['success'] else '',
               last_x=float(x[-1] + lab.odom_init[0]), last_y=float(y[-1] + lab.odom_init[1]),
               last_yaw=float(poses[-1].get('yaw', float('nan'))), t_end=float(poses[-1]['t']),
               dist_to_next_goal=float(np.hypot(x[-1] - nxt[0], y[-1] - nxt[1])) if nxt is not None else '',
               nearest_obstacle=min_obstacle_clearance(np.array([x[-1] + lab.odom_init[0]]), np.array([y[-1] + lab.odom_init[1]]),
                                                       lab.stop_obstacles)[1],
               end_cmd_speed=tail_speed(cmd), end_exec_speed=tail_speed(act))
    return rec


# ----------------------------------------------------------------------------- aggregation
def policy_order(recs):
    seen = {}
    for r in recs:
        seen.setdefault((r['variant'], r['tier']), r['model'])
    return [(v, t, seen[(v, t)]) for v in rr.VARIANTS for t in ('B', 'M') if (v, t) in seen]


def policy_stats(recs, sim):
    """One row per policy: runs, success with Wilson interval, per-path success, goals, failures, SPL, path length and time."""
    rows = []
    for v, t, model in policy_order(recs):
        rs = [r for r in recs if r['variant'] == v and r['tier'] == t]
        ok = [r for r in rs if r['success']]
        k, n = len(ok), len(rs)
        lo, hi = wilson(k, n)
        s = sim.get(model, {})
        per = {p: [r for r in rs if r['path'] == p] for p in rr.PATHS}
        pk = {p: sum(r['success'] for r in per[p]) for p in rr.PATHS}
        row = dict(variant=v, tier=t, model=model, n_runs=n, n_success=k, success_lo=lo, success_hi=hi,
                   p1_success=pk[rr.PATHS[0]], p1_n=len(per[rr.PATHS[0]]), p2_success=pk[rr.PATHS[1]], p2_n=len(per[rr.PATHS[1]]),
                   fisher_p_paths=fisher_two_sided(pk[rr.PATHS[0]], len(per[rr.PATHS[0]]) - pk[rr.PATHS[0]],
                                                   pk[rr.PATHS[1]], len(per[rr.PATHS[1]]) - pk[rr.PATHS[1]]),
                   goals_pct=100 * sum(r['goals'] for r in rs) / sum(r['total'] for r in rs),
                   collisions=sum(r['failure'] == 'collision' for r in rs), timeouts=sum(r['failure'] == 'timeout' for r in rs),
                   other=sum(r['failure'] == 'other' for r in rs), spl_all=float(np.mean([r['spl_all'] for r in rs])),
                   spl_reached=float(np.nanmean([r['spl_reached'] for r in rs])),
                   min_clearance_median=float(np.median([r['min_clearance'] for r in rs])),
                   sim_goals_pct=float(s['goal_completion']) if s else float('nan'),
                   sim_spl=float(s['spl_geo']) if s else float('nan'),
                   sim_runs_success=float(s['run_success']) if s else float('nan'))
        for path in rr.PATHS:
            for key, col in (('path_m', 'path'), ('time_s', 'time')):
                m, q1, q3 = quart([r[key] for r in per[path] if r['success']])
                row.update({f'{col}_{path[:5]}_median': m, f'{col}_{path[:5]}_q1': q1, f'{col}_{path[:5]}_q3': q3})
        rows.append(row)
    return rows


GROUPS = [('planner', 'delta'), ('planner', 'direct'), ('checkpoint', 'best'), ('checkpoint', 'final'), ('tier', 'B'), ('tier', 'M')]


def group_stats(recs, n_boot, rng):
    """Pooled success, goals and SPL per group of policies with cluster-bootstrap intervals (policies, then runs)."""
    def sel(r, kind, val):
        return (r['planner'] if kind == 'planner' else r['checkpoint'] if kind == 'checkpoint' else r['tier']) == val
    rows, reps_by = [], {}
    for kind, val in GROUPS:
        rs = [r for r in recs if sel(r, kind, val)]
        models = sorted({r['model'] for r in rs})
        units = [np.array([float(r['success']) for r in rs if r['model'] == m]) for m in models]
        reps = cluster_bootstrap(units, n_boot, rng)
        reps_by[(kind, val)] = reps
        lo, hi = pct_ci(reps)
        per_pol = [u.mean() * 100 for u in units]
        spl = [np.array([r['spl_all'] for r in rs if r['model'] == m]) for m in models]
        slo, shi = pct_ci(cluster_bootstrap(spl, n_boot, rng))
        rows.append(dict(group=kind, value=val, n_policies=len(models), n_runs=len(rs), success_pct=100 * np.mean([r['success'] for r in rs]),
                         success_lo=100 * lo, success_hi=100 * hi, policy_min=min(per_pol), policy_max=max(per_pol),
                         goals_pct=100 * sum(r['goals'] for r in rs) / sum(r['total'] for r in rs),
                         spl_all=float(np.mean([r['spl_all'] for r in rs])), spl_lo=slo, spl_hi=shi))
    diffs = {}
    for kind, a, b in (('planner', 'delta', 'direct'), ('checkpoint', 'best', 'final'), ('tier', 'B', 'M')):
        lo, hi = pct_ci(reps_by[(kind, a)] - reps_by[(kind, b)])
        diffs[f'{kind}: {a} - {b}'] = dict(diff_pct=100 * (np.mean([r['success'] for r in recs if sel(r, kind, a)]) -
                                                         np.mean([r['success'] for r in recs if sel(r, kind, b)])),
                                           lo=100 * lo, hi=100 * hi)
    return rows, diffs


def sim_correlations(pstats):
    out = {}
    for name, sim_key, real_key in (('goals', 'sim_goals_pct', 'goals_pct'), ('spl', 'sim_spl', 'spl_all'),
                                    ('success', 'sim_runs_success', 'n_success')):
        out[name] = spearman([p[sim_key] for p in pstats], [p[real_key] for p in pstats])
    return out


def failure_cells(recs):
    """Failed runs grouped by policy and path: count, type, failing goal, median last pose, nearest obstacle, speeds at the stop."""
    cells = collections.OrderedDict()
    for r in recs:
        if not r['success']:
            cells.setdefault((r['variant'], r['tier'], r['model'], r['path']), []).append(r)
    rows = []
    for (v, t, model, path), rs in cells.items():
        n_all = sum(1 for r in recs if r['model'] == model and r['path'] == path)
        mode = lambda key: collections.Counter(r[key] for r in rs).most_common(1)[0][0]
        rows.append(dict(variant=v, tier=t, model=model, path=path, n_failed=len(rs), n_runs=n_all,
                         collisions=sum(r['failure'] == 'collision' for r in rs), timeouts=sum(r['failure'] == 'timeout' for r in rs),
                         fail_goal=mode('fail_goal'), last_x=float(np.median([r['last_x'] for r in rs])),
                         last_y=float(np.median([r['last_y'] for r in rs])), nearest_obstacle=mode('nearest_obstacle'),
                         dist_to_next_goal=float(np.median([r['dist_to_next_goal'] for r in rs])),
                         end_cmd_speed=float(np.median([r['end_cmd_speed'] for r in rs])),
                         end_exec_speed=float(np.median([r['end_exec_speed'] for r in rs])),
                         t_end_median=float(np.median([r['t_end'] for r in rs]))))
    order = {k: i for i, k in enumerate((v, t) for v in rr.VARIANTS for t in ('B', 'M'))}
    rows.sort(key=lambda f: (order[(f['variant'], f['tier'])], f['path']))
    return rows


def failures_by_obstacle(recs):
    """Failed runs counted by the obstacle nearest to the last pose, by failing goal and by type."""
    bad = [r for r in recs if not r['success']]
    return dict(n_failed=len(bad), by_obstacle=dict(collections.Counter(r['nearest_obstacle'] for r in bad).most_common()),
                by_failing_goal=dict(sorted(collections.Counter(int(r['fail_goal']) for r in bad).items())),
                by_hit_index=dict(sorted(collections.Counter(int(r['hit_index']) for r in bad if r.get('hit_index', '') != '').items())),
                by_type=dict(collections.Counter(r['failure'] for r in bad)))


def bm_tests(ps):
    """Fisher exact test of B against M within each variant (full-sequence success, 20 runs each)."""
    out = {}
    for v in rr.VARIANTS:
        b = next(p for p in ps if p['variant'] == v and p['tier'] == 'B')
        m = next(p for p in ps if p['variant'] == v and p['tier'] == 'M')
        out[v] = dict(B=f"{b['n_success']}/{b['n_runs']}", M=f"{m['n_success']}/{m['n_runs']}",
                      p=fisher_two_sided(b['n_success'], b['n_runs'] - b['n_success'], m['n_success'], m['n_runs'] - m['n_success']))
    return out


def order_checks(recs, manifest):
    """Drift checks: success against stretch and the attempt/fault bookkeeping."""
    by_stretch = collections.OrderedDict()
    for r in recs:
        by_stretch.setdefault(int(r['stretch']), []).append(r['success'])
    batt = [float(r['battery_v']) for r in recs if r['battery_v'] not in ('', None) and float(r['battery_v']) > 5]   # 0 = voltage not read
    return dict(attempts=len(manifest), kept=len(recs), faults=sum(1 for m in manifest if m['fault']),
                success_by_stretch={s: f'{sum(v)}/{len(v)}' for s, v in by_stretch.items()},
                battery_v=[min(batt), max(batt)] if batt else None,
                codes=dict(collections.Counter(m['code'] for m in manifest)),
                dirty=sum(m['dirty'] == '1' for m in manifest), config_ok=sum(m['config_ok'] == '1' for m in manifest),
                start_ok=sum(m['start_ok'] == '1' for m in manifest))


# ----------------------------------------------------------------------------- LaTeX tables
def esc(s):
    return str(s).replace('_', r'\_')


def iqr(m, a, b, nd):
    return '--' if math.isnan(m) else f'{m:.{nd}f} [{a:.{nd}f}, {b:.{nd}f}]'


def tab(spec, head, rows, breaks=True):
    """rows: list of (variant, line); a \\midrule is put between variants."""
    out = [rf'\begin{{tabular}}{{{spec}}}', r'\midrule[1.5pt]', *head, r'\midrule[1.5pt]']
    prev = None
    for v, line in rows:
        if breaks and prev is not None and v != prev:
            out.append(r'\midrule')
        out.append(line)
        prev = v
    return '\n'.join(out + [r'\midrule[1.5pt]', r'\end{tabular}', ''])


def table_results(ps):
    """Variant and role rows (no policy id); Gazebo goals and SPL, then the robot goals and SPL next to them, then success and failures."""
    head = [r' & & \multicolumn{2}{c}{\textbf{Gazebo}} & \multicolumn{5}{c}{\textbf{Robot (20 runs per policy)}} \\',
            r'\cmidrule(lr){3-4}\cmidrule(lr){5-9}',
            r'\textbf{Variant} & \textbf{Role} & Goals (\%) & SPL & Goals (\%) & SPL & Success & 95\% CI & Coll./Timeouts \\']
    rows = [(p['variant'], f"{LABEL[p['variant']]} & {p['tier']} & {p['sim_goals_pct']:.1f} & {p['sim_spl']:.3f} & "
             f"{p['goals_pct']:.1f} & {p['spl_all']:.3f} & {p['n_success']}/{p['n_runs']} & "
             f"[{p['success_lo']:.2f}, {p['success_hi']:.2f}] & {p['collisions']}/{p['timeouts']} \\\\") for p in ps]
    return tab('llccccccc', head, rows)


def table_by_path(ps):
    head = [r'\textbf{Variant} & \textbf{Role} & \textbf{Path} & \textbf{Success} & \textbf{Path length (m)} & \textbf{Time (s)} \\']
    rows = []
    for p in ps:
        for i, path in enumerate(rr.PATHS, 1):
            key = path[:5]
            rows.append((p['variant'], f"{LABEL[p['variant']]} & {p['tier']} & {i} & {p[f'p{i}_success']}/{p[f'p{i}_n']} & "
                         f"{iqr(p[f'path_{key}_median'], p[f'path_{key}_q1'], p[f'path_{key}_q3'], 2)} & "
                         f"{iqr(p[f'time_{key}_median'], p[f'time_{key}_q1'], p[f'time_{key}_q3'], 1)} \\\\"))
    return tab('lllccll', head, rows)


def table_groups(gs, diffs):
    names = {'delta': 'TD3\\_delta', 'direct': 'TD3\\_direct', 'best': 'Best checkpoint', 'final': 'Final checkpoint',
             'B': 'Role B', 'M': 'Role M'}
    head = [r'\textbf{Group} & \textbf{Policies} & \textbf{Runs} & \textbf{Success (\%)} & \textbf{95\% cluster CI} & '
            r'\textbf{Per-policy range (\%)} & \textbf{SPL} \\']
    rows = [(g['group'], f"{names[g['value']]} & {g['n_policies']} & {g['n_runs']} & {g['success_pct']:.1f} & "
             f"[{g['success_lo']:.1f}, {g['success_hi']:.1f}] & {g['policy_min']:.0f} to {g['policy_max']:.0f} & {g['spl_all']:.3f} \\\\")
            for g in gs]
    return tab('lcccccc', head, rows)


def table_failures(fs):
    head = [r'\textbf{Variant} & \textbf{Role} & \textbf{Path} & \textbf{Failed} & \textbf{Coll./Timeouts} & \textbf{Stopped at goal} & '
            r'\textbf{Last pose (m)} & \textbf{Nearest obstacle} \\']
    rows = [(f['variant'], f"{LABEL[f['variant']]} & {f['tier']} & {f['path'][4]} & {f['n_failed']}/{f['n_runs']} & "
             f"{f['collisions']}/{f['timeouts']} & {f['fail_goal']} & ({f['last_x']:.2f}, {f['last_y']:.2f}) & {esc(f['nearest_obstacle'])} \\\\")
            for f in fs]
    return tab('lllcccll', head, rows)


def table_smoothness(recs):
    head = [r'\textbf{Planner} & \textbf{Runs} & \textbf{Commanded $a_{lin}$} & \textbf{Executed $a_{lin}$} & '
            r'\textbf{Commanded $a_{ang}$} & \textbf{Executed $a_{ang}$} \\']
    rows = []
    for pl in ('direct', 'delta'):
        rs = [r for r in recs if r['planner'] == pl and r['success']]
        rows.append((pl, f"TD3\\_{pl} & {len(rs)} & " + ' & '.join(
            iqr(*quart([r[k] for r in rs]), 2) for k in ('cmd_smooth_a_lin', 'act_smooth_a_lin', 'cmd_smooth_a_ang', 'act_smooth_a_ang')) + r' \\'))
    return tab('lccccc', head, rows)


# ----------------------------------------------------------------------------- figures
def fig_sim_vs_real(ps, path):
    """Gazebo against robot goal completion (a) and SPL (b) of the eight policies, drawn at the reference scale of the trajectory figures
    (17.9 in wide, scaled to the 455 pt text width in the paper) with the default matplotlib style and the same font sizes."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    with matplotlib.rc_context():
        matplotlib.rcdefaults()
        plt.rcParams.update({'pdf.fonttype': 42, 'ps.fonttype': 42})
        fig, axes = plt.subplots(1, 2, figsize=(17.9, 7.4), sharex=True)
        x = np.arange(len(ps)) + (np.arange(len(ps)) // 2) * 0.5
        panels = (('sim_goals_pct', 'goals_pct', 'Goal completion (%)', (58, 102), np.arange(60, 101, 10)),
                  ('sim_spl', 'spl_all', 'SPL, shortest path, all legs', (0.58, 1.02), np.arange(0.6, 1.01, 0.1)))
        for ax, (sk, rk, ylabel, ylim, yticks), letter in zip(axes, panels, ('(a)', '(b)')):
            for g in range(2, len(ps), 2):                          # faint dotted separators between the variants
                ax.axvline((x[g - 1] + x[g]) / 2, color='0.7', ls=':', lw=1.5, zorder=0)
            for xi, p in zip(x, ps):
                color, marker = PLANNER_STYLE[p['variant'].split('_')[0]]
                ax.plot([xi, xi], [p[sk], p[rk]], color='0.75', lw=3, zorder=1)
                ax.scatter(xi, p[sk], marker='o', facecolor='white', edgecolor='0.4', s=170, linewidths=1.8, zorder=2)
                ax.scatter(xi, p[rk], marker=marker, color=color, s=190, zorder=3)
            ax.set_xticks(x, [p['tier'] for p in ps])
            for g in range(0, len(ps), 2):                          # checkpoint under each B/M pair; the planner is the marker colour
                ax.text((x[g] + x[g + 1]) / 2, -0.075, ps[g]['variant'].split('_')[1].capitalize(), transform=ax.get_xaxis_transform(),
                        ha='center', va='top', fontsize=14)
            ax.set_ylabel(ylabel, fontsize=12)
            ax.set_ylim(*ylim)
            ax.set_yticks(yticks)
            ax.minorticks_on()
            ax.tick_params(axis='x', which='minor', bottom=False)
            ax.tick_params(axis='both', which='both', direction='in', labelsize=14)
            ax.set_title(letter, loc='left', fontsize=14)
        fig.legend(handles=[Line2D([0], [0], marker='o', ls='', mfc='white', mec='0.4', mew=1.8, ms=13, label='Gazebo, 6 scenarios'),
                            Line2D([0], [0], marker='s', ls='', color=PLANNER_STYLE['direct'][0], ms=13, label='Robot, TD3_direct'),
                            Line2D([0], [0], marker='^', ls='', color=PLANNER_STYLE['delta'][0], ms=13, label='Robot, TD3_delta')],
                   loc='lower center', ncol=3, fontsize=18.7, frameon=False)
        fig.tight_layout(rect=(0, 0.07, 1, 1))
        for ext in ('pdf', 'png'):
            fig.savefig(f'{path}.{ext}', dpi=150)
        plt.close(fig)


def run_key(r):
    """Run name as visualize_path_variance loads it (saved file relative to scripts/recorded_paths, without .json)."""
    return r['saved_file'].replace('scripts/recorded_paths/', '')[:-5]


SPREAD_MIN_RUNS = 3                    # fewer successful runs give no meaningful 10th to 90th percentile band


def path_spread(recs):
    """Run-to-run spread of the paths, per policy and path: the width of the 10th to 90th percentile band of the cross-track offset from
    the median path (the shaded band of fig_robot_paths, visualize_path_variance.leg_stats), over the successful runs, every leg resampled
    to the same number of points. mean_width is the average over all points of all legs, max_width the widest point. Policies with fewer
    than SPREAD_MIN_RUNS successful runs on a path are left out. Paths are odometry estimates, so the widths include odometry error."""
    import visualize_path_variance as vp
    rows = []
    for v in rr.VARIANTS:
        for t in ('B', 'M'):
            for path in rr.PATHS:
                ok = [run_key(r) for r in recs if r['variant'] == v and r['tier'] == t and r['path'] == path and r['success']]
                if len(ok) < SPREAD_MIN_RUNS:
                    continue
                res = vp.analyze(path[:5].replace('path', 'path_'), WORLD, t, keys=ok)
                w = np.concatenate([s['width'] for s in res['stats']])
                rows.append(dict(variant=v, planner=v.split('_')[0], tier=t, path=path, n_success=len(ok),
                                 mean_width=round(float(w.mean()), 4), max_width=round(float(w.max()), 4)))
    return rows


def spread_summary(rows):
    """Per planner over its policy-path rows: number of rows, median, minimum and maximum of mean_width and of max_width."""
    out = {}
    for planner in ('delta', 'direct'):
        sub = [r for r in rows if r['planner'] == planner]
        if not sub:
            continue
        s = dict(n_rows=len(sub))
        for c in ('mean_width', 'max_width'):
            v = [r[c] for r in sub]
            s.update({f'{c}_median': round(float(np.median(v)), 4), f'{c}_min': min(v), f'{c}_max': max(v)})
        out[planner] = s
    return out


def write_spread(recs, out_dir):
    rows = path_spread(recs)
    write_csv(os.path.join(out_dir, 'robot_path_spread.csv'), rows)
    summary = dict(min_runs=SPREAD_MIN_RUNS, script_commit=git_head(), by_planner=spread_summary(rows))
    json.dump(summary, open(os.path.join(out_dir, 'robot_path_spread.json'), 'w'), indent=1)
    return summary


def draw_panel(ax, lab, recs, path, variant, fs=None):
    """B and M of one variant on one path: median and band of the successful runs (solid; B a darker shade of the planner colour than M),
    failed runs thin with an x at their last pose.
    With `fs` (single panels) the fonts are set to that size and the label is the axes title; without it (composite figure) the fonts are
    those of apply_arena_style and the label sits in the upper left corner, as in the other trajectory figures."""
    import matplotlib.colors as mcolors
    from matplotlib.ticker import AutoMinorLocator, MultipleLocator
    import visualize_path_variance as vp
    cells = {t: [r for r in recs if r['variant'] == variant and r['tier'] == t and r['path'] == path] for t in ('B', 'M')}
    key = run_key
    planner, ckpt = variant.split('_')
    base = mcolors.to_rgb(PLANNER_COLOR[planner])
    color = {'B': tuple(c * B_DARKEN for c in base), 'M': base}
    style = {t: dict(color=color[t], band_alpha=0.2, **PATH_STYLE[t]) for t in ('B', 'M')}
    results = []
    for t in ('B', 'M'):
        ok = [key(r) for r in cells[t] if r['success']]
        if ok:
            results.append(vp.analyze(path[:5].replace('path', 'path_'), WORLD, t, keys=ok))
    if results:
        vp.plot_ensemble_compare(results, '', ax=ax, style=style)
    else:
        vc.draw_obstacles(ax, lab.obstacles)
        vc.draw_goals(ax, lab.goals[path], 0.3)
        vc.apply_arena_style(ax, WORLD)
    for t in ('B', 'M'):
        for r in cells[t]:
            if not r['success']:
                run = vp.clean(vc.load_run(key(r), WORLD))
                ax.plot(run['x'], run['y'], color=color[t], **FAILED_STYLE)
                ax.plot(run['x'][-1:], run['y'][-1:], color=color[t], **FAILED_END)
    ks = {t: f"{sum(r['success'] for r in cells[t])}/{len(cells[t])}" for t in ('B', 'M')}
    name = f"TD3_{planner} {ckpt.capitalize()}"
    for axis, minor in ((ax.xaxis, 5), (ax.yaxis, 5)):               # the same ticks on both axes
        axis.set_major_locator(MultipleLocator(TICK_STEP))
        axis.set_minor_locator(AutoMinorLocator(minor))
    if fs is None:
        box = dict(facecolor='white', edgecolor='none', alpha=0.8, pad=2)
        head = ax.text(0.025, 0.975, f"{name}, Path {path[4]}", transform=ax.transAxes, ha='left', va='top', fontsize=14, zorder=20, bbox=box)
        # the B and M counts in the colours of their lines, so the panel needs no key for the two shades
        b_txt = ax.annotate(f"B {ks['B']}", xy=(0, 0), xycoords=head, xytext=(0, -3), textcoords='offset points', ha='left', va='top',
                            fontsize=14, color=color['B'], zorder=20, bbox=box)
        ax.annotate(f"M {ks['M']}", xy=(1, 0.5), xycoords=b_txt, xytext=(10, 0), textcoords='offset points', ha='left', va='center',
                    fontsize=14, color=color['M'], zorder=20, bbox=box)
    else:
        ax.tick_params(axis='both', which='both', labelsize=fs)
        for a in ax.texts:
            a.set_fontsize(fs)
        ax.xaxis.label.set_size(fs)
        ax.yaxis.label.set_size(fs)
        ax.set_title(f"{name}, Path {path[4]}:  B {ks['B']}  M {ks['M']}", fontsize=fs + 1)
    return ks


def fig_paths(recs, lab, out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import visualize_path_variance as vp
    panels = os.path.join(out_dir, 'panels')
    os.makedirs(panels, exist_ok=True)
    vp.FIG_DIR = panels
    cfg = vc.world_config(WORLD)
    nrow, ncol = len(rr.VARIANTS), len(rr.PATHS)
    with matplotlib.rc_context():                                   # default style, as the other trajectory figures (fig_sim_vs_real changes the globals)
        matplotlib.rcdefaults()
        # drawn at the scale of the other trajectory figures (fonts 14 pt, goal circles, line widths), which the paper shrinks by a common
        # factor; one shared x axis per column and one shared y axis per row, so only the outer panels carry tick labels and axis labels
        pw, gap = PANEL_IN, 0.12
        ph = pw * cfg['arena_w'] / cfg['arena_l']
        left, right, top, bottom = 0.95, 0.1, 0.1, 0.7             # margins (in): y labels at the left, x labels below (no legend, the caption explains the lines)
        W, H = left + right + ncol * pw + (ncol - 1) * gap, top + bottom + nrow * ph + (nrow - 1) * gap
        fig = plt.figure(figsize=(W, H))
        gs = fig.add_gridspec(nrow, ncol, left=left / W, right=1 - right / W, top=1 - top / H, bottom=bottom / H,
                              wspace=gap / pw, hspace=gap / ph)
        axes = [[None] * ncol for _ in range(nrow)]
        for i in range(nrow):
            for j in range(ncol):
                axes[i][j] = fig.add_subplot(gs[i, j], sharex=axes[0][0] if (i or j) else None, sharey=axes[0][0] if (i or j) else None)
        for i, v in enumerate(rr.VARIANTS):
            for j, path in enumerate(rr.PATHS):
                draw_panel(axes[i][j], lab, recs, path, v)
                axes[i][j].label_outer()
        for ext in ('pdf', 'png'):
            fig.savefig(os.path.join(out_dir, f'fig_robot_paths.{ext}'), dpi=200)
        print(f'fig_robot_paths: {W:.2f} x {H:.2f} in; width {W / 17.9:.3f} of the text width at the scale of the other trajectory figures')
        plt.close(fig)
        for v in rr.VARIANTS:                                       # single panels, large
            for path in rr.PATHS:
                f, ax = vc.make_arena_fig(WORLD, long_side=8.0)
                draw_panel(ax, lab, recs, path, v, 12)
                vp.save_fig(f, f'repeat_{path[:5].replace("path", "path_")}_{WORLD}_{v}_compare')
                plt.close(f)


# ----------------------------------------------------------------------------- main
def write_csv(path, rows):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def load_sim(path=SIM_UNITS):
    return {r['unit']: r for r in csv.DictReader(open(path))} if os.path.exists(path) else {}


def git_head():
    try:
        return subprocess.check_output(['git', '-C', HERE, 'rev-parse', '--short', 'HEAD'], text=True).strip()
    except Exception:
        return 'unknown'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=rr.rrs.OUT_DIR)
    ap.add_argument('--manifest', default=rr.rrs.MANIFEST)
    ap.add_argument('--root', default=rr.rrs.WS)
    ap.add_argument('--n-boot', type=int, default=10000)
    ap.add_argument('--seed', type=int, default=20261103)
    ap.add_argument('--no-figs', action='store_true')
    ap.add_argument('--paths-only', action='store_true', help='only redraw fig_robot_paths and panels/, leave every other output untouched')
    ap.add_argument('--simreal-only', action='store_true', help='only redraw fig_robot_sim_vs_real, leave every other output untouched')
    ap.add_argument('--spread-only', action='store_true', help='only write robot_path_spread.csv and .json, leave every other output untouched')
    a = ap.parse_args()
    manifest = rr.rrs.load_manifest(a.manifest)
    kept = rr.kept_runs(manifest)
    schedule = rr.rrs.load_schedule()
    rr.progress(schedule, manifest, kept)
    lab = Lab()
    recs = [run_metrics(r, a.root, lab) for _, r in sorted(kept.items())]
    if a.paths_only:
        os.makedirs(a.out, exist_ok=True)
        fig_paths(recs, lab, a.out)
        return
    if a.simreal_only:
        os.makedirs(a.out, exist_ok=True)
        fig_sim_vs_real(policy_stats(recs, load_sim()), os.path.join(a.out, 'fig_robot_sim_vs_real'))
        return
    if a.spread_only:
        os.makedirs(a.out, exist_ok=True)
        print(json.dumps(write_spread(recs, a.out), indent=1))
        return
    rng = np.random.default_rng(a.seed)
    ps = policy_stats(recs, load_sim())
    gs, diffs = group_stats(recs, a.n_boot, rng)
    fs = failure_cells(recs)
    os.makedirs(a.out, exist_ok=True)
    write_csv(os.path.join(a.out, 'robot_run_metrics.csv'), recs)
    write_csv(os.path.join(a.out, 'robot_policy_stats.csv'), ps)
    write_csv(os.path.join(a.out, 'robot_group_stats.csv'), gs)
    if fs:
        write_csv(os.path.join(a.out, 'robot_failures.csv'), fs)
    for name, text in (('table_robot_results.tex', table_results(ps)), ('table_robot_by_path.tex', table_by_path(ps)),
                       ('table_robot_groups.tex', table_groups(gs, diffs)), ('table_robot_failures.tex', table_failures(fs)),
                       ('table_robot_smoothness.tex', table_smoothness(recs))):
        open(os.path.join(a.out, name), 'w').write(text)
    write_spread(recs, a.out)
    stats = dict(date=datetime.date.today().isoformat(), script_commit=git_head(), n_kept=len(kept), n_scheduled=len(schedule),
                 complete=len(kept) == len(schedule), manifest_sha1=hashlib.sha1(open(a.manifest, 'rb').read()).hexdigest(),
                 n_boot=a.n_boot, seed=a.seed, checks=order_checks(recs, manifest), group_differences=diffs,
                 spearman_sim_vs_robot=sim_correlations(ps), failures=failures_by_obstacle(recs), b_vs_m=bm_tests(ps),
                 fisher_paths={f"{p['variant']} {p['tier']}": p['fisher_p_paths'] for p in ps})
    json.dump(stats, open(os.path.join(a.out, 'robot_stats.json'), 'w'), indent=1)
    if not a.no_figs:
        fig_sim_vs_real(ps, os.path.join(a.out, 'fig_robot_sim_vs_real'))
        fig_paths(recs, lab, a.out)
    print(json.dumps(stats, indent=1))
    print('wrote', a.out)


if __name__ == '__main__':
    main()
