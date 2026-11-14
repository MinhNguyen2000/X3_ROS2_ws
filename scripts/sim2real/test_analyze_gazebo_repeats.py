"""Unit tests of analyze_gazebo_repeats.py (pure logic, no ROS):
cd scripts/sim2real && python3 -m pytest -q -p no:anyio test_analyze_gazebo_repeats.py"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze_gazebo_repeats as ag  # noqa: E402


def test_failure_type():
    assert ag.failure_type({'success': True}) == 'none'
    assert ag.failure_type({'success': False, 'message': 'Obstacle hit with distance 0.2, index 3'}) == 'collision'
    assert ag.failure_type({'success': False, 'message': 'Goal timeout'}) == 'timeout'
    assert ag.failure_type({'success': False, 'message': 'x'}) == 'other'


def test_leg_spl_straight_and_detour():
    # straight 1 m leg, then a detour back to the origin
    x = np.array([0, 0.5, 1.0, 1.0, 1.0, 0.0])
    y = np.array([0, 0.0, 0.0, 1.0, 2.0, 0.0])
    goals = np.array([[1.0, 0.0], [1.0, 0.0]])
    straight = lambda p, q: float(np.hypot(p[0] - q[0], p[1] - q[1]))
    legs = ag.leg_spl(x, y, goals[:1], 1, 0.3, straight)
    assert legs == [1.0]
    goals2 = np.array([[1.0, 0.0], [0.0, 0.0]])
    legs = ag.leg_spl(x, y, goals2, 2, 0.3, straight)
    assert abs(legs[0] - 1.0) < 1e-9 and abs(legs[1] - 1 / (2 + np.hypot(1, 2))) < 1e-9   # L = 1, P = 1 + 1 + sqrt(5)


def test_policy_table_and_rank():
    recs = [dict(model='A', success=1, goals=2, total=2, legs_attempted=2, failure='none', spl_all_sum=1.8, spl_reached_sum=1.8),
            dict(model='A', success=0, goals=1, total=2, legs_attempted=2, failure='collision', spl_all_sum=0.9, spl_reached_sum=0.9),
            dict(model='B', success=1, goals=2, total=2, legs_attempted=2, failure='none', spl_all_sum=2.0, spl_reached_sum=2.0)]
    rows = ag.policy_table(recs, {'A': ('delta_final', 'M'), 'B': ('direct_best', 'B')})
    a = rows[0]
    assert a['n_runs'] == 2 and a['n_success'] == 1 and abs(a['goals_pct'] - 75.0) < 1e-9
    assert abs(a['collisions_per_100_legs'] - 25.0) < 1e-9
    assert abs(a['spl_all'] - (0.9 + 0.45) / 2) < 1e-9 and abs(a['spl_reached'] - 2.7 / 3) < 1e-9
    assert ag.rank_by_goals(rows) == {'A': 2, 'B': 1}
