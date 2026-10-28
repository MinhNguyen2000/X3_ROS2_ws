import numpy as np
import pytest

from x3_drl_policy.lidar_processing import preprocess_scan, pool_groups

N = 1600
ANGLE_MIN = -np.pi
INC = 2 * np.pi / N
RANGE_MIN, RANGE_MAX = 0.13, 12.0


def legacy(ranges, antennae_mask_dist=0.20, rear_mask_deg=0.0):
    """The processing of policy_node.py at af91cae (callback plus pooling), copied for comparison."""
    ranges = np.asarray(ranges, dtype=np.float32)
    zero_idx = round(-ANGLE_MIN / INC)
    raw = np.roll(ranges, -zero_idx)
    raw = np.where(np.isfinite(raw), raw, RANGE_MAX)
    raw[raw < RANGE_MIN] = RANGE_MAX
    if antennae_mask_dist > 0.0:
        raw[raw <= antennae_mask_dist] = RANGE_MAX
    if rear_mask_deg > 0.0:
        ang = np.arange(raw.size) * INC
        raw[np.abs(ang - np.pi) <= np.deg2rad(rear_mask_deg)] = RANGE_MAX
    raw = np.clip(raw, 0.0, RANGE_MAX)
    raw = np.flip(raw).copy()
    return np.array([g.min() for g in np.array_split(raw, 18)], dtype=np.float32)


def run(scan, fill_means=None, **kw):
    r, m = preprocess_scan(scan, ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX, **kw)
    return pool_groups(r, m, 18, np.full(18, 2.8, dtype=np.float32) if fill_means is None else fill_means)


@pytest.fixture
def scan():
    rng = np.random.default_rng(0)
    s = rng.uniform(0.05, 12.0, N).astype(np.float32)
    s[rng.random(N) < 0.05] = np.inf
    s[rng.random(N) < 0.02] = np.nan
    return s


@pytest.mark.parametrize('antennae,rear', [(0.20, 0.0), (0.0, 0.0), (0.20, 40.0), (0.0, 45.8)])
def test_defaults_reproduce_legacy(scan, antennae, rear):
    got = run(scan, antennae_mask_dist=antennae, rear_mask_deg=rear)
    np.testing.assert_array_equal(got, legacy(scan, antennae, rear))


def test_obstacle_in_front_is_not_hidden_without_antennae_mask():
    s = np.full(N, 3.0, dtype=np.float32)
    front = N // 2                       # index of angle 0 in a scan that starts at -pi
    s[front - 5:front + 5] = 0.18
    assert run(s, antennae_mask_dist=0.0).min() == pytest.approx(0.18)
    assert run(s, antennae_mask_dist=0.20).min() == pytest.approx(3.0)   # the global mask hides it


def test_train_mean_fills_fully_masked_groups_and_pools_partial_ones():
    s = np.full(N, 3.0, dtype=np.float32)
    means = np.arange(18, dtype=np.float32) + 100.0
    # a return of 0.5 m directly behind, inside the masked wedge, must not reach the policy
    s[0:5] = 0.5                          # index 0 is angle -pi, i.e. behind
    r, m = preprocess_scan(s, ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX, antennae_mask_dist=0.0,
                           rear_mask_deg=45.8, rear_fill='train_mean')
    out = pool_groups(r, m, 18, means)
    covered = [g for g, mk in enumerate(np.array_split(m, 18)) if mk.all()]
    partial = [g for g, mk in enumerate(np.array_split(m, 18)) if mk.any() and not mk.all()]
    assert covered and partial
    for g in covered:
        assert out[g] == means[g]
    for g in set(range(18)) - set(covered):
        assert out[g] == pytest.approx(3.0)   # partly masked and open groups pool the unmasked beams only
    assert 0.5 not in out


def test_range_clip_bounds_valid_and_invalid_returns():
    s = np.full(N, 12.0, dtype=np.float32)
    s[100] = np.inf
    s[200] = np.nan
    s[300] = 0.05                         # below range_min
    r, _ = preprocess_scan(s, ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX, antennae_mask_dist=0.0, range_clip=11.10)
    assert np.allclose(r, 11.10)
    assert run(s, antennae_mask_dist=0.0, range_clip=11.10).max() == pytest.approx(11.10)


def test_range_max_fill_keeps_wedge_beams_out_of_the_mask():
    s = np.full(N, 3.0, dtype=np.float32)
    s[0:5] = 0.5
    r, m = preprocess_scan(s, ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX, antennae_mask_dist=0.0,
                           rear_mask_deg=45.8, rear_fill='range_max')
    assert not m.any()
    assert r.max() == pytest.approx(RANGE_MAX)
