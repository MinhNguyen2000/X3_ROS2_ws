"""Unit tests of the pure functions of odom_scale_check.py.
Run: cd scripts/sim2real && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test_odom_scale_check.py"""
import math

import odom_scale_check as O


def test_yaw_from_quaternion():
    assert abs(O.yaw_from_quaternion(0.0, 1.0)) < 1e-12
    assert abs(O.yaw_from_quaternion(math.sin(0.25), math.cos(0.25)) - 0.5) < 1e-12
    assert abs(O.yaw_from_quaternion(1.0, 0.0) - math.pi) < 1e-12


def test_distance():
    assert O.distance((0, 0), (3, 4)) == 5
    assert O.distance((1, 1), (1, 1)) == 0


def test_turned_angle_wraps_and_adds_whole_turns():
    assert abs(O.turned_angle(0.1, 0.1 + 0.5) - 0.5) < 1e-12
    assert abs(O.turned_angle(3.0, -3.0) - (2 * math.pi - 6.0)) < 1e-12      # across the +-pi cut
    assert abs(O.turned_angle(0.0, 0.1, turns_hint=1) - (0.1 + 2 * math.pi)) < 1e-12


def test_ratio():
    assert abs(O.ratio(1.9, 2.0) - 0.95) < 1e-12
    assert math.isnan(O.ratio(1.0, 0.0))
