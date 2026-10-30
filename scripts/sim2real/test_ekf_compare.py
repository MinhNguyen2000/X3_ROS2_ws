import numpy as np

import ekf_compare as ec


def track(n=200, yaw_rate=0.0):
    t = np.linspace(0.0, 20.0, n)
    th = yaw_rate * t
    return t, np.c_[0.2 * t * np.cos(th), 0.2 * t * np.sin(th), th]


def test_yaw_from_quaternion():
    for yaw in (-3.0, -0.5, 0.0, 1.2, 3.0):
        assert abs(ec.yaw_from_quaternion((0, 0, np.sin(yaw / 2), np.cos(yaw / 2))) - yaw) < 1e-12


def test_rigid_fit_recovers_rotation_and_translation():
    rng = np.random.default_rng(0)
    src = rng.normal(size=(50, 2))
    th = 0.7
    r = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    dst = src @ r.T + [1.0, -2.0]
    got, tr = ec.rigid_fit(src, dst)
    assert abs(got - th) < 1e-9 and np.allclose(tr, [1.0, -2.0])


def test_identical_track_in_a_rotated_frame_scores_zero():
    t, odom = track(yaw_rate=0.05)
    th = 1.0
    c, s = np.cos(th), np.sin(th)
    cam = np.c_[odom[:, 0] * c - odom[:, 1] * s + 3.0, odom[:, 0] * s + odom[:, 1] * c - 1.0, odom[:, 2] + th]
    s_ = ec.score(t, odom, t, cam)
    assert s_['fit_rms_m'] < 1e-9 and abs(s_['len_ratio'] - 1) < 1e-9
    assert abs(s_['yaw_drift_deg']) < 1e-6 and s_['yaw_max_deg'] < 1e-6


def test_scale_and_yaw_drift_are_reported():
    t, cam = track(yaw_rate=0.02)
    odom = cam.copy()
    odom[:, :2] *= 0.9
    odom[:, 2] += np.radians(5.0) * t / t[-1]          # 5 degrees of yaw drift over the drive
    s = ec.score(t, odom, t, cam)
    assert abs(s['len_ratio'] - 0.9) < 1e-6
    assert abs(s['yaw_drift_deg'] - 5.0) < 1e-6 and abs(s['yaw_max_deg'] - 5.0) < 1e-6


def test_no_overlap_gives_none():
    t, odom = track()
    assert ec.score(t, odom, t + 100.0, odom) is None


def test_wrapped_yaw_is_unwrapped_before_resampling():
    t = np.linspace(0, 10, 100)
    yaw = wrapped = ec.wrap(0.5 * t)                      # crosses +-pi
    odom = np.c_[np.zeros_like(t), np.zeros_like(t), wrapped]
    s = ec.score(t, odom, t, odom)
    assert s['yaw_max_deg'] < 1e-6


def test_warning_pattern():
    assert ec.WARN.search('Failed to meet update rate! Took 0.05')
    assert ec.WARN.search('Could not transform measurement into agent0_odom')
    assert not ec.WARN.search('Covariance filter node is running')


def test_empty_track_gives_none():
    t, odom = track()
    assert ec.score(np.array([]), np.zeros((0, 3)), t, odom) is None
