"""Unit tests of the pure logic in run_robot_schedule.py.
Run: cd scripts/sim2real && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test_run_robot_schedule.py"""
import copy
import os

import run_robot_schedule as R

SCHED = R.load_schedule()


def row(run, **kw):
    base = dict(run=str(run), attempt='1', status='ok', config_ok='1', fault='')
    base.update(kw)
    return base


def test_schedule_is_the_committed_160_runs():
    assert len(SCHED) == 160
    assert [r['run'] for r in SCHED] == list(range(1, 161))
    assert {r['stretch'] for r in SCHED} == {1, 2, 3, 4}
    assert len({r['model'] for r in SCHED}) == 8
    assert all(r['path'] in ('path1_world_lab', 'path2_world_lab') for r in SCHED)


def test_everything_pending_without_a_manifest():
    assert [r['run'] for r in R.pending_runs(SCHED, [])][:3] == [1, 2, 3]
    assert len(R.pending_runs(SCHED, [], stretch=2)) == 40


def test_kept_runs_are_skipped_and_faulted_runs_repeat_last():
    m = [row(1), row(2, fault='person'), row(3, config_ok='0'), row(4, status='no_output'), row(5)]
    p = [r['run'] for r in R.pending_runs(SCHED, m)]
    assert 1 not in p and 5 not in p
    assert p[:3] == [6, 7, 8]                         # untried runs first
    assert p[-4:] == [2, 3, 4] or set(p[-3:]) == {2, 3, 4}   # failed attempts come back at the end
    assert set([2, 3, 4]) <= set(p)


def test_a_repeat_that_succeeds_is_kept():
    m = [row(2, fault='battery'), row(2, attempt='2')]
    assert 2 not in [r['run'] for r in R.pending_runs(SCHED, m)]


def test_config_check():
    good = dict(R.FROZEN, model_name='TD3_00390_1000', action_mode='delta')
    assert R.config_mismatches(good) == []
    bad = copy.deepcopy(good)
    bad['max_angular_vel'] = 0.5
    bad['rear_mask_fill'] = 'range_max'
    assert R.config_mismatches(bad) == ['max_angular_vel', 'rear_mask_fill']
    assert R.config_mismatches(None) == sorted(R.FROZEN)
    assert R.config_mismatches({'max_lin_vel': 0.2}) != []


def test_parse_client_result():
    ok = '[INFO] ... [SUCCESS] All waypoints completed | Completed 5 waypoint(s) | Total distance:  1.2m'
    fail = '[FAILED] Stopped at waypoint 3: Obstacle hit with distance 0.2 | Completed 2 waypoint(s)'
    assert R.parse_client_result(ok) == (True, 5)
    assert R.parse_client_result(fail) == (False, 2)
    assert R.parse_client_result('nothing here') is None


def test_frozen_matches_the_launch_file():
    """FROZEN must equal the parameters of drl_policy.launch.py."""
    src = open(os.path.join(R.WS, 'src', 'x3_bringup', 'launch', 'drl_policy.launch.py')).read()
    for k, v in R.FROZEN.items():
        assert f"'{k}'" in src, k
        needle = f"'{v}'" if isinstance(v, str) else repr(v) if not isinstance(v, bool) else str(v)
        line = [l for l in src.splitlines() if f"'{k}'" in l][0]
        assert needle in line.replace('11.10', '11.1') or needle in line, (k, line)


def test_start_offset_and_kept_rule():
    assert R.start_offset([]) is None
    d, yaw = R.start_offset([{'x': 0.03, 'y': -0.04, 'yaw': 0.02}])
    assert abs(d - 0.05) < 1e-9 and yaw == 0.02
    far = R.start_offset([{'x': 1.49, 'y': 1.41, 'yaw': 0.1}])
    assert far[0] > R.START_TOL[0]
    assert R.is_kept(row(1, start_ok='1')) and not R.is_kept(row(1, start_ok='0'))


def test_manifest_columns_include_the_dirty_marker():
    assert 'dirty' in R.COLS and R.COLS.index('code') < R.COLS.index('dirty')
