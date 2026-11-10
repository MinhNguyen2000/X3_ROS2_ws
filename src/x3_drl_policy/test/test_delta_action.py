import numpy as np

from x3_drl_policy.delta_action import asym_delta


def test_asym_delta_matches_the_training_rule():
    """The sequence of ROS2_DRL_Navigation tests/test_nav2d_p14.py test_delta_asymmetric (up 0.1, down 0.2), with the
    node's clip to [0, 1] x [-1, 1] applied after each step."""
    low, high = np.array([0.0, -1.0]), np.array([1.0, 1.0])
    last = np.zeros(2)
    steps = [([1, 1], [0.1, 0.1]), ([1, 1], [0.2, 0.2]), ([0.5, 1], [0.25, 0.3]), ([-1, -1], [0.05, 0.1]),
             ([-1, -1], [0.0, 0.0]), ([-1, -1], [0.0, -0.1]), ([-0.5, 1], [0.0, 0.0])]
    for frac, expected in steps:
        last = np.clip(asym_delta(last, np.array(frac, float), np.array([0.1, 0.1]), 0.2), low, high)
        np.testing.assert_allclose(last, expected, atol=1e-9)
