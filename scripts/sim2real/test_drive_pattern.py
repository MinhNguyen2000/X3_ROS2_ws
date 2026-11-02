"""drive_pattern.sh (drive_pattern.py) against a dummy cmd_vel subscriber (needs ROS 2; skipped without rclpy).

The driver keeps the last command, so the one thing that matters is that the last message is all zeros, also when the
script is interrupted, and that the script waits for the subscriber and only claims a stop it has checked. The dummy
node plays the driver (DRIVER_NODE) and, if asked, its wheel velocity on vel_raw.
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
VEL_TOPIC = '/test_drive/vel_raw'


def run(pattern, interrupt_after=None, timeout=120, subscribe_after=0.0, vel=None, args=()):
    """Run the script; returns (commands received, exit code, stdout).

    subscribe_after: seconds before the dummy subscription exists; None for never.
    vel: None for no vel_raw publisher, 'echo' for wheels that follow the last command, 'stuck' for wheels that
    never stop.
    """
    rclpy.init()
    node = rclpy.create_node('drive_pattern_test')
    got = []
    last = [(0.0, 0.0)]

    def on_cmd(m):
        got.append((m.twist.linear.x, m.twist.angular.z))
        last[0] = got[-1]

    vel_pub = node.create_publisher(TwistStamped, VEL_TOPIC, 10) if vel else None
    proc = subprocess.Popen(['bash', SCRIPT, pattern, *args],
                            env=dict(os.environ, CMD_VEL_TOPIC=TOPIC, DRIVER_NODE='drive_pattern_test'),
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, start_new_session=True)
    t0 = time.time()
    sub = None
    interrupted = False
    while proc.poll() is None and time.time() - t0 < timeout:
        if sub is None and subscribe_after is not None and time.time() - t0 >= subscribe_after:
            sub = node.create_subscription(TwistStamped, TOPIC, on_cmd, 100)
        rclpy.spin_once(node, timeout_sec=0.05)
        if vel_pub:
            m = TwistStamped()
            m.twist.linear.x, m.twist.angular.z = (0.1, 0.0) if vel == 'stuck' else last[0]
            vel_pub.publish(m)
        if interrupt_after and not interrupted and sum(1 for g in got if g != (0.0, 0.0)) >= interrupt_after:
            os.killpg(proc.pid, signal.SIGINT)      # what Ctrl-C does: the whole foreground process group
            interrupted = True
    for _ in range(40):                       # messages still in flight
        rclpy.spin_once(node, timeout_sec=0.05)
    out = proc.communicate(timeout=10)[0]
    node.destroy_node()
    rclpy.shutdown()
    return got, proc.returncode, out


def moving(got):
    return [g for g in got if g != (0.0, 0.0)]


def test_straight_sends_the_pattern_and_ends_with_zeros():
    got, _, _ = run('straight')
    assert abs(len(moving(got)) - 300) <= 10
    assert all(g == (0.2, 0.0) for g in moving(got))
    assert got[-1] == (0.0, 0.0) and got[-5:] == [(0.0, 0.0)] * 5


def test_turn_sends_the_pattern_and_ends_with_zeros():
    got, _, _ = run('turn')
    assert abs(len(moving(got)) - 420) <= 10
    assert all(g == (0.0, 0.3) for g in moving(got))
    assert got[-1] == (0.0, 0.0)


def test_interrupt_ends_with_zeros():
    got, _, _ = run('straight', interrupt_after=40)
    assert 40 <= len(moving(got)) < 250
    assert got[-5:] == [(0.0, 0.0)] * 5


def test_unknown_pattern_is_refused_without_moving():
    got, _, _ = run('spin')
    assert not moving(got)


def test_square_is_eight_sides_and_eight_turns_and_ends_with_zeros():
    got, _, _ = run('square', timeout=300)
    assert abs(len([g for g in got if g == (0.2, 0.0)]) - 800) <= 40
    assert abs(len([g for g in got if g == (0.0, 0.3)]) - 840) <= 40
    assert got[-1] == (0.0, 0.0)


def test_waits_for_a_late_subscriber_and_loses_nothing():
    got, code, out = run('straight', subscribe_after=3.0)
    assert code == 0
    assert abs(len(moving(got)) - 300) <= 3
    assert got[0] == (0.0, 0.0) and got[-5:] == [(0.0, 0.0)] * 5
    assert 'stop not checked' in out


def test_no_subscriber_gives_up_without_moving_or_claiming_a_stop():
    got, code, out = run('straight', subscribe_after=None, args=('--wait', '2'))
    assert code != 0
    assert not moving(got)
    assert 'stopped' not in out


def test_stop_is_confirmed_on_vel_raw():
    got, code, out = run('straight', interrupt_after=20, vel='echo')
    assert 'done, robot stopped' in out
    assert got[-5:] == [(0.0, 0.0)] * 5


def test_wheels_that_keep_turning_are_reported():
    got, code, out = run('straight', interrupt_after=20, vel='stuck')
    assert code == 1
    assert 'STOP NOT CONFIRMED' in out and 'done' not in out
    assert got[-5:] == [(0.0, 0.0)] * 5
