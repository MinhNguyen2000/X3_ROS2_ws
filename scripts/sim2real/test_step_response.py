"""step_response.py's delivery measure on a synthetic drive (needs ROS 2 for the module's rosbag2_py import)."""
import os
import sys

import numpy as np
import pytest

pytest.importorskip('rosbag2_py')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step_response as R  # noqa: E402


def test_delivery_matches_each_change_to_its_own_echo():
    t = np.arange(10) * 0.05
    b = np.column_stack([t, [0, .1, .1, .2, .2, .5, .5, 0, 0, 0], np.zeros(10)])
    echo = np.column_stack([t + 0.5, b[:, 1], b[:, 2]])     # every command reaches the driver 0.5 s late
    echo = echo[echo[:, 1] != 0.2]                           # and 0.2 never does
    r = R.delivery(b, echo)
    assert [x['v'] for x in r] == [0.1, 0.2, 0.5, 0.0]
    assert np.isnan(r[1]['rx_delay_s'])
    # the return to 0 must not match the late echo of the lead-in zero
    assert np.allclose([r[0]['rx_delay_s'], r[2]['rx_delay_s'], r[3]['rx_delay_s']], 0.5)
