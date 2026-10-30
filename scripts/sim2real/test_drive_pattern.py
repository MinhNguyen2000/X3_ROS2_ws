"""drive_pattern.sh against a dummy cmd_vel subscriber (needs ROS 2; skipped without rclpy).

The driver keeps the last command, so the one thing that matters is that the last message is all zeros, also when the
script is interrupted.
"""
import os
import signal
import subprocess
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import TwistStamped  # noqa: E402

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drive_pattern.sh')
TOPIC = '/test_drive/cmd_vel'


def run(pattern, interrupt_after=None, timeout=120):
    rclpy.init()
    node = rclpy.create_node('drive_pattern_test')
    got = []
    node.create_subscription(TwistStamped, TOPIC, lambda m: got.append((m.twist.linear.x, m.twist.angular.z)), 100)
    for _ in range(20):                      # let the subscription settle
        rclpy.spin_once(node, timeout_sec=0.05)
    proc = subprocess.Popen(['bash', SCRIPT, pattern], env=dict(os.environ, CMD_VEL_TOPIC=TOPIC),
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    t0 = time.time()
    interrupted = False
    while proc.poll() is None and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        if interrupt_after and not interrupted and sum(1 for g in got if g != (0.0, 0.0)) >= interrupt_after:
            os.killpg(proc.pid, signal.SIGINT)      # what Ctrl-C does: the whole foreground process group
            interrupted = True
    for _ in range(40):                       # messages still in flight
        rclpy.spin_once(node, timeout_sec=0.05)
    proc.wait(timeout=10)
    node.destroy_node()
    rclpy.shutdown()
    return got


def moving(got):
    return [g for g in got if g != (0.0, 0.0)]


def test_straight_sends_the_pattern_and_ends_with_zeros():
    got = run('straight')
    assert abs(len(moving(got)) - 300) <= 10
    assert all(g == (0.2, 0.0) for g in moving(got))
    assert got[-1] == (0.0, 0.0) and got[-5:] == [(0.0, 0.0)] * 5


def test_turn_sends_the_pattern_and_ends_with_zeros():
    got = run('turn')
    assert abs(len(moving(got)) - 420) <= 10
    assert all(g == (0.0, 0.3) for g in moving(got))
    assert got[-1] == (0.0, 0.0)


def test_interrupt_ends_with_zeros():
    got = run('straight', interrupt_after=40)
    assert 40 <= len(moving(got)) < 250
    assert got[-5:] == [(0.0, 0.0)] * 5


def test_unknown_pattern_is_refused_without_moving():
    got = run('spin')
    assert not moving(got)


def test_square_is_eight_sides_and_eight_turns_and_ends_with_zeros():
    got = run('square', timeout=300)
    assert abs(len([g for g in got if g == (0.2, 0.0)]) - 800) <= 40
    assert abs(len([g for g in got if g == (0.0, 0.3)]) - 840) <= 40
    assert got[-1] == (0.0, 0.0)
