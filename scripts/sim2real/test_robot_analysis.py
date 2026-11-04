"""Unit tests of robot_analysis.py: `cd scripts/sim2real && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test_robot_analysis.py`."""
import math

import numpy as np
import pytest

import robot_analysis as ra


def test_wilson_known_values():
    lo, hi = ra.wilson(0, 10)
    assert lo == pytest.approx(0.0, abs=1e-9) and hi == pytest.approx(0.2775, abs=1e-3)
    lo, hi = ra.wilson(10, 10)
    assert lo == pytest.approx(0.7225, abs=1e-3) and hi == pytest.approx(1.0, abs=1e-9)
    assert all(math.isnan(v) for v in ra.wilson(0, 0))


def test_fisher_two_sided():
    assert ra.fisher_two_sided(3, 0, 0, 3) == pytest.approx(0.1)
    assert ra.fisher_two_sided(1, 9, 11, 3) == pytest.approx(0.0027, abs=1e-4)      # textbook table
    assert ra.fisher_two_sided(5, 5, 5, 5) == pytest.approx(1.0)


def test_ranks_and_spearman():
    assert list(ra.ranks([10, 20, 20, 30])) == [1.0, 2.5, 2.5, 4.0]
    assert ra.spearman([1, 2, 3, 4], [10, 20, 30, 40]) == pytest.approx(1.0)
    assert ra.spearman([1, 2, 3, 4], [4, 3, 2, 1]) == pytest.approx(-1.0)
    assert math.isnan(ra.spearman([1, 1, 1], [1, 2, 3]))


def test_cluster_bootstrap_is_deterministic_and_bounded():
    units = [np.ones(5), np.zeros(5), np.array([1., 0., 1., 1., 0.])]
    a = ra.cluster_bootstrap(units, 200, np.random.default_rng(1))
    b = ra.cluster_bootstrap(units, 200, np.random.default_rng(1))
    assert np.array_equal(a, b) and 0.0 <= a.min() and a.max() <= 1.0
    lo, hi = ra.pct_ci(a)
    assert lo <= np.mean(a) <= hi
    assert set(ra.cluster_bootstrap([np.ones(4)], 20, np.random.default_rng(0))) == {1.0}


def test_quart_ignores_nan():
    assert ra.quart([1.0, 2.0, 3.0, float('nan')])[0] == 2.0
    assert all(math.isnan(v) for v in ra.quart([]))


def test_rms_accel_and_tail_speed():
    s = [{'t': 0.1 * i, 'vx': 0.01 * i, 'vyaw': 0.0} for i in range(20)]      # constant 0.1 m/s^2
    lin, ang = ra.rms_accel(s)
    assert lin == pytest.approx(0.1) and ang == pytest.approx(0.0)
    assert all(math.isnan(v) for v in ra.rms_accel(s[:2]))
    assert ra.tail_speed(s, 0.35) == pytest.approx(np.mean([0.16, 0.17, 0.18, 0.19]))
    assert math.isnan(ra.tail_speed([]))


def test_min_obstacle_clearance():
    obs = [dict(name='cylinder_1', type='cylinder', x=0.0, y=0.0, radius=0.5, yaw=0.0),
           dict(name='box_1', type='box', x=5.0, y=0.0, sx=2.0, sy=2.0, yaw=0.0)]
    d, name = ra.min_obstacle_clearance(np.array([2.0]), np.array([0.0]), obs)
    assert d == pytest.approx(1.5) and name == 'cylinder_1'
    d, name = ra.min_obstacle_clearance(np.array([3.0, 3.5]), np.array([0.0, 0.0]), obs)
    assert d == pytest.approx(0.5) and name == 'box_1'                           # box face at x = 4, nearest pose x = 3.5
    assert ra.min_obstacle_clearance(np.array([4.0]), np.array([0.0]), obs)[0] == pytest.approx(0.0)


class FakeSP:
    def length(self, a, b):
        return float(np.hypot(b[0] - a[0], b[1] - a[1])), []


class FakeLab:
    sp = FakeSP()

    def to_sp(self, p):
        return p


def test_find_boundaries_and_leg_spl():
    x = np.linspace(0, 2, 201)
    y = np.zeros(201)
    goals = np.array([[1.0, 0.0], [2.0, 0.0]])
    b = ra.find_boundaries(x, y, goals, 0.3, 0.01)
    assert x[b[0]] == pytest.approx(0.7, abs=0.02) and x[b[1]] == pytest.approx(1.7, abs=0.02)      # first pose inside the circle
    spl = ra.leg_spl(x, y, goals, 2, 0.3, FakeLab())
    assert len(spl) == 2 and all(0.9 < v <= 1.0 for v in spl)
    assert ra.leg_spl(x, y, goals, 0, 0.3, FakeLab()) == []


def test_leg_spl_detour_scores_below_one():
    t = np.linspace(0, math.pi, 200)
    x, y = 1.0 - np.cos(t), np.sin(t)                    # half circle from (0,0) to (2,0), length pi, straight line 2
    spl = ra.leg_spl(x, y, np.array([[2.0, 0.0]]), 1, 0.05, FakeLab())
    assert spl[0] == pytest.approx(2.0 / math.pi, abs=0.05)


def _rec(model, path, success, failure='none', fail_goal='', obstacle='', variant='delta_best', tier='B'):
    return dict(model=model, path=path, success=success, failure=failure, fail_goal=fail_goal, nearest_obstacle=obstacle,
                variant=variant, tier=tier, last_x=1.0, last_y=2.0, dist_to_next_goal=0.5, end_cmd_speed=0.1, end_exec_speed=0.1,
                t_end=30.0)


def test_failure_cells_and_counts():
    recs = [_rec('m1', 'path1_world_lab', 1), _rec('m1', 'path1_world_lab', 0, 'collision', 4, 'box_2'),
            _rec('m1', 'path1_world_lab', 0, 'timeout', 4, 'box_2'), _rec('m1', 'path2_world_lab', 1)]
    cells = ra.failure_cells(recs)
    assert len(cells) == 1 and cells[0]['n_failed'] == 2 and cells[0]['n_runs'] == 3
    assert (cells[0]['collisions'], cells[0]['timeouts'], cells[0]['fail_goal'], cells[0]['nearest_obstacle']) == (1, 1, 4, 'box_2')
    f = ra.failures_by_obstacle(recs)
    assert f['n_failed'] == 2 and f['by_obstacle'] == {'box_2': 2} and f['by_failing_goal'] == {4: 2}


def test_bm_tests_and_tables_render():
    ps = []
    for v in ra.rr.VARIANTS:
        for tier, k in (('B', 20), ('M', 10)):
            ps.append(dict(variant=v, tier=tier, model=f'TD3_{v}_{tier}', n_runs=20, n_success=k, success_lo=0.1, success_hi=0.9,
                           goals_pct=90.0, collisions=1, timeouts=2, spl_all=0.9, sim_goals_pct=100.0, sim_spl=0.9,
                           p1_success=5, p1_n=10, p2_success=5, p2_n=10,
                           **{f'{c}_{p}_{s}': 1.0 for c in ('path', 'time') for p in ('path1', 'path2') for s in ('median', 'q1', 'q3')}))
    t = ra.bm_tests(ps)
    assert t['delta_best']['B'] == '20/20' and t['delta_best']['p'] < 0.01
    text = ra.table_results(ps)
    assert text.count(r'\\') >= 10 and text.count('&') > 70 and r'\end{tabular}' in text
    assert r'\end{tabular}' in ra.table_by_path(ps)
