"""Unit tests of robot_results.py: `cd scripts/sim2real && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test_robot_results.py`."""
import csv
import json
import math

import robot_results as rr

COLS = ['run', 'attempt', 'stretch', 'block', 'variant', 'tier', 'model', 'path', 'status', 'waypoints_completed', 'total_waypoints',
        'success', 'elapsed_s', 'battery_v', 'saved_file', 'code', 'dirty', 'config_ok', 'start_ok', 'fault', 'timestamp']


def test_failure_type():
    assert rr.failure_type({'success': True, 'message': ''}) == 'none'
    assert rr.failure_type({'success': False, 'message': 'Stopped at waypoint 4: Obstacle hit with distance 0.187'}) == 'collision'
    assert rr.failure_type({'success': False, 'message': 'Stopped at waypoint 4: Goal timeout after 90.0s'}) == 'timeout'
    assert rr.failure_type({'success': False, 'message': 'x'}) == 'other'


def test_path_length():
    assert rr.path_length([{'x': 0, 'y': 0}, {'x': 3, 'y': 4}, {'x': 3, 'y': 5}]) == 6.0
    assert rr.path_length([]) == 0.0


def _row(run, fault='', config_ok='1', model='TD3_1', path='path1_world_lab', f='a.json'):
    return dict(run=run, attempt=1, stretch=1, block=1, variant='delta_final', tier='B', model=model, path=path, status='ok',
                waypoints_completed=3, total_waypoints=5, success=0, elapsed_s=10, battery_v=11.9, saved_file=f, code='abc', dirty=0,
                config_ok=config_ok, start_ok='1', fault=fault, timestamp='t')


def test_only_kept_runs_and_last_attempt(tmp_path):
    m = [_row(1, fault='person'), _row(1), _row(2, config_ok='0')]
    assert list(rr.kept_runs(m)) == [1] and rr.kept_runs(m)[1]['fault'] == ''


def test_record_and_units(tmp_path):
    for name, ok, msg in (('a.json', True, ''), ('b.json', False, 'Stopped at waypoint 2: Obstacle hit with distance 0.1')):
        (tmp_path / name).write_text(json.dumps({'elapsed_time': 20.0, 'poses': [{'x': 0, 'y': 0}, {'x': 0, 'y': 2}],
                                                 'result': {'waypoints_completed': 5 if ok else 1, 'total_waypoints': 5, 'success': ok,
                                                            'message': msg}}))
    recs = [rr.run_record(_row(1, f='a.json'), tmp_path), rr.run_record(_row(2, f='b.json'), tmp_path)]
    assert [r['failure'] for r in recs] == ['none', 'collision']
    u = rr.unit_row(recs, path='both')
    assert (u['n_runs'], u['n_success'], u['collisions'], u['timeouts']) == (2, 1, 1, 0)
    assert math.isclose(u['goals_pct'], 60.0) and u['path_median'] == 2.0
    t1, t2 = rr.tex_tables(rr.build_units(recs and [dict(r, variant='delta_final', tier='B') for r in recs]))
    assert '1/2' in t1 and r'\end{tabular}' in t2
