"""Unit tests of robot_dropped.py: `cd scripts/sim2real && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test_robot_dropped.py`."""
import io
import json
import tarfile

import pytest

import robot_dropped as rd


def test_classify():
    assert rd.classify({'success': True, 'message': ''}) == 'success'
    assert rd.classify({'success': False, 'message': 'Stopped at waypoint 3: Obstacle hit with distance  0.187, index 6. Mission aborted.'}) == 'phantom_index_6'
    assert rd.classify({'success': False, 'message': 'Stopped at waypoint 3: Obstacle hit with distance  0.2, index 14. Mission aborted.'}) == 'collision_other_index'
    assert rd.classify({'success': False, 'message': 'Stopped at waypoint 4: Goal timeout after 90.0s'}) == 'other_failure'


def _tar(path, runs):
    with tarfile.open(path, 'w:gz') as t:
        for name, result in runs.items():
            data = json.dumps({'result': result, 'elapsed_time': 12.0, 'code': 'abc'}).encode()
            info = tarfile.TarInfo(f'scripts/recorded_paths/{name}')
            info.size = len(data)
            t.addfile(info, io.BytesIO(data))
        info = tarfile.TarInfo('scripts/recorded_paths/README.txt')          # non-JSON members are ignored
        info.size = 2
        t.addfile(info, io.BytesIO(b'hi'))


def test_read_attempts_from_tar_without_extracting(tmp_path):
    p = tmp_path / 'a.tar.gz'
    _tar(p, {'TD3_1/path1_world_lab_1.json': {'success': False, 'waypoints_completed': 2, 'total_waypoints': 5,
                                              'message': 'Stopped at waypoint 3: Obstacle hit with distance  0.16, index 6. Mission aborted.'},
             'TD3_1/path2_world_lab_2.json': {'success': True, 'waypoints_completed': 4, 'total_waypoints': 4, 'message': ''}})
    rows = rd.read_attempts([str(p)])
    assert [r['kind'] for r in rows] == ['phantom_index_6', 'success'] and rows[0]['hit_index'] == 6 and rows[0]['path'] == 'path1'
    assert not list(tmp_path.glob('scripts'))                                 # nothing was extracted


def test_per_policy_sensitivity():
    schedule = [{'model': 'TD3_1', 'variant': 'delta_best', 'tier': 'B'}]
    kept = [{'model': 'TD3_1', 'success': '1'}] * 9 + [{'model': 'TD3_1', 'success': '0'}]
    dropped = [{'model': 'TD3_1', 'kind': 'phantom_index_6'}] * 2 + [{'model': 'TD3_1', 'kind': 'collision_other_index'},
                                                                      {'model': 'TD3_1', 'kind': 'success'}]
    r = rd.per_policy(schedule, kept, dropped)[0]
    assert (r['kept_n'], r['kept_success'], r['dropped_n'], r['dropped_phantom_index_6']) == (10, 9, 4, 2)
    assert r['success_primary'] == pytest.approx(0.9)
    assert r['success_all_attempts'] == pytest.approx(10 / 14)              # 9 + 1 completed dropped, 14 attempts
    assert r['success_without_phantom'] == pytest.approx(10 / 12)           # the two index-6 stops are left out
    assert r"\end{tabular}" in rd.table_dropped([r])
