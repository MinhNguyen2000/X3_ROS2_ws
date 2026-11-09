#!/usr/bin/env python3
"""Fixed drive patterns for the odometry recordings (docs/real_robot_run_protocol.md, ekf_compare.py) and scale check.

  drive_pattern.py straight   3 m at 0.2 m/s (300 messages at 20 Hz)
  drive_pattern.py turn       one full turn at 0.3 rad/s, counter-clockwise (420 messages)
  drive_pattern.py square     two laps of a 1 m square: 8 x (1 m straight, 90 degree turn), 1 s stops in between
  drive_pattern.py step       0.5 m/s for 2 s, then 0 for 2 s (about 1.0 m)
  drive_pattern.py ramp10     0 -> 0.5 m/s -> 0 in steps of 0.05 m/s per 0.1 s (delta bound 0.1), 1 s at 0.5 m/s (1.0 m)
  drive_pattern.py ramp20     the same in steps of 0.1 m/s per 0.1 s (delta bound 0.2) (about 0.75 m)
  drive_pattern.py turnsteps  in place, 0 -> +1 -> -1 -> 0 rad/s three times, 2 s at each level: as steps, then ramps of
                              0.1 and of 0.2 rad/s per 0.1 s (delta bounds 0.1 and 0.2)

The driver (mcnamu_driver.py) keeps the last cmd_vel until a new one arrives, so the robot keeps moving when the
publishing ends, and everything depends on the zero command reaching the driver. The first version of this tool
(drive_pattern.sh, a `ros2 topic pub` per segment) lost it: each new publisher has to be discovered again, through the
lab's discovery server since 2026-11-01, and a short burst of zeros could end before the driver had matched it
(2026-11-02: the robot kept turning after 'done, robot stopped'). So this tool
  1. uses one publisher for the whole pattern,
  2. sends nothing until the driver's subscription (node DRIVER_NODE, default driver_node) is matched and acknowledges
     zero commands, i.e. both sides have discovered each other; it gives up after --wait seconds without moving,
  3. ends with zero commands, also on Ctrl-C, SIGTERM or an error, for at least 1 s, then until the driver's wheel
     velocity (vel_raw next to the cmd_vel topic) reads zero, then until every subscriber has acknowledged them.
It prints 'done, robot stopped' only when vel_raw shows the wheels stopped, and 'STOP NOT CONFIRMED' (exit 1) when the
zeros were not acknowledged or the wheels still turn. The e-stop is still the operator's.

CMD_VEL_TOPIC overrides the topic (default /agent0/cmd_vel), VEL_RAW_TOPIC the wheel velocity topic and DRIVER_NODE
the node whose subscription must be matched (empty: any subscriber).
"""
import argparse
import os
import signal
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TwistStamped

RATE = 20.0                 # Hz of the commands
FORWARD = (0.2, 0.0)        # (linear x m/s, angular z rad/s)
TURN = (0.0, 0.3)
STOP = (0.0, 0.0)
SETTLE_S = 1.0              # zeros sent before the first motion command, after the driver has acknowledged
STOP_MIN_S = 1.0            # zeros sent at the end before the stop is checked
STOP_MAX_S = 5.0            # longest wait for vel_raw to read zero
ACK_TIMEOUT_S = 3.0
STILL_LINEAR = 0.01         # m/s, wheel velocity counted as stopped
STILL_ANGULAR = 0.02        # rad/s

PATTERNS = {
    'straight': [(300, FORWARD)],
    'turn': [(420, TURN)],
    'square': [seg for _ in range(8) for seg in ((100, FORWARD), (20, STOP), (105, TURN), (20, STOP))],
}

# Step-response test of the delta bounds (P14): A2's top speeds, and ramps shaped as policy_node sends them in delta
# mode, a change of bound x top speed per 10 Hz decision.
TOP_V, TOP_W = 0.5, 1.0
DEC = int(RATE / 10)        # ticks per 10 Hz decision


def _twist(axis, v):
    return (v, 0.0) if axis == 0 else (0.0, v)


def _hold(axis, v, s):
    return [(int(s * RATE), _twist(axis, v))]


def _ramp(axis, a, b, bound):
    top = TOP_V if axis == 0 else TOP_W
    n = round(abs(b - a) / (bound * top))
    return [(DEC, _twist(axis, round(a + (b - a) * k / n, 6))) for k in range(1, n + 1)]


def _turn(bound):
    """0 -> +TOP_W -> -TOP_W -> 0, 2 s at each level; a step when bound is None."""
    to = (lambda a, b: []) if bound is None else (lambda a, b: _ramp(1, a, b, bound))
    return (to(0, TOP_W) + _hold(1, TOP_W, 2) + to(TOP_W, -TOP_W) + _hold(1, -TOP_W, 2)
            + to(-TOP_W, 0) + _hold(1, 0.0, 2))


PATTERNS.update({
    'step': _hold(0, TOP_V, 2) + _hold(0, 0.0, 2),                                              # about 1.0 m
    'ramp10': _ramp(0, 0, TOP_V, 0.1) + _hold(0, TOP_V, 1) + _ramp(0, TOP_V, 0, 0.1) + _hold(0, 0.0, 1),  # 1.0 m
    'ramp20': _ramp(0, 0, TOP_V, 0.2) + _hold(0, TOP_V, 1) + _ramp(0, TOP_V, 0, 0.2) + _hold(0, 0.0, 1),  # 0.75 m
    'turnsteps': _turn(None) + _turn(0.1) + _turn(0.2),                                        # in place
})


class Interrupted(Exception):
    pass


class Drive:
    def __init__(self, topic, vel_topic, driver_node):
        self.node = rclpy.create_node('drive_pattern')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.topic, self.vel_topic, self.driver_node = topic, vel_topic, driver_node
        self.pub = self.node.create_publisher(TwistStamped, topic, qos)
        self.vel = None             # (arrival time, vx, vy, wz) of the last vel_raw message
        self.node.create_subscription(TwistStamped, vel_topic, self._on_vel, 10)
        self.next_tick = time.monotonic()

    def _on_vel(self, msg):
        self.vel = (time.monotonic(), msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)

    def send(self, twist):
        """Publish one command, then wait for the next 20 Hz tick while handling vel_raw."""
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.angular.z = twist
        self.pub.publish(msg)
        self.next_tick += 1.0 / RATE
        while True:
            left = self.next_tick - time.monotonic()
            if left <= 0:
                break
            rclpy.spin_once(self.node, timeout_sec=left)
        if self.next_tick < time.monotonic() - 1.0:     # fell far behind (e.g. a stall): do not burst to catch up
            self.next_tick = time.monotonic()

    def driver_matched(self):
        subs = self.node.get_subscriptions_info_by_topic(self.topic)
        if not subs:
            return False
        if self.driver_node and self.driver_node not in [s.node_name for s in subs]:
            return False
        return self.pub.get_subscription_count() >= len(subs)

    def wait_for_driver(self, wait_s, check):
        """True once the driver's subscription is matched from both sides; sends only zeros meanwhile."""
        deadline = time.monotonic() + wait_s
        while not self.driver_matched():
            check()
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.next_tick = time.monotonic()
        for _ in range(int(SETTLE_S * RATE)):
            check()
            self.send(STOP)
        # an acknowledgement means the driver has matched this publisher too
        while not self.pub.wait_for_all_acked(Duration(seconds=0.5)):
            check()
            if time.monotonic() > deadline:
                return False
        return True

    def stop(self):
        """Send zeros until the stop is confirmed or time is up; returns (acked, wheels), wheels None if no vel_raw."""
        t0 = time.monotonic()
        self.next_tick = t0
        has_vel = self.node.count_publishers(self.vel_topic) > 0
        while True:
            self.send(STOP)
            elapsed = time.monotonic() - t0
            if elapsed < STOP_MIN_S:
                continue
            wheels = None
            if has_vel:
                v = self.vel
                fresh = v is not None and v[0] > t0 + 0.3
                wheels = fresh and abs(v[1]) < STILL_LINEAR and abs(v[2]) < STILL_LINEAR and abs(v[3]) < STILL_ANGULAR
            if wheels is not False or elapsed > STOP_MAX_S:
                break
        acked = self.pub.wait_for_all_acked(Duration(seconds=ACK_TIMEOUT_S))
        acked = acked and self.pub.get_subscription_count() > 0
        return acked, wheels


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('pattern', choices=sorted(PATTERNS))
    ap.add_argument('--wait', type=float, default=15.0, help='seconds to wait for the driver before giving up')
    args = ap.parse_args()
    topic = os.environ.get('CMD_VEL_TOPIC', '/agent0/cmd_vel')
    vel_topic = os.environ.get('VEL_RAW_TOPIC', topic.rsplit('/', 1)[0] + '/vel_raw')
    driver_node = os.environ.get('DRIVER_NODE', 'driver_node')

    # rclpy's own handlers would shut the context down on Ctrl-C, and then no zero could be sent
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    interrupted = []

    def on_signal(signum, _frame):
        interrupted.append(signum)

    def check():
        if interrupted:
            raise Interrupted()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    drive = Drive(topic, vel_topic, driver_node)
    moved = False
    code = 0
    try:
        if not drive.wait_for_driver(args.wait, check):
            print(f'no matched subscriber {driver_node or ""} on {topic} after {args.wait:.0f} s '
                  f'(subscribers: {[s.node_name for s in drive.node.get_subscriptions_info_by_topic(topic)]}); '
                  'nothing sent but zeros', file=sys.stderr)
            code = 3
        else:
            moved = True
            segments = PATTERNS[args.pattern]
            for i, (count, twist) in enumerate(segments):
                if args.pattern == 'square' and i % 4 == 0:
                    print(f'square: side {i // 4 + 1} of 8', flush=True)
                elif i == 0:
                    print({'straight': 'straight: 3 m', 'turn': 'turn: one full turn'}.get(args.pattern, args.pattern),
                          flush=True)
                for _ in range(count):
                    check()
                    drive.send(twist)
    except Interrupted:
        print('interrupted, stopping', flush=True)
        code = 130
    finally:
        # Further Ctrl-C is recorded but ignored until the zeros are out (Ctrl-\ still kills the process)
        acked, wheels = drive.stop()
        drive.node.destroy_node()
        rclpy.shutdown()
    if not moved:
        return code
    if not acked or wheels is False:
        why = 'zero commands not acknowledged by every subscriber' if not acked else 'vel_raw still shows wheel motion'
        print(f'STOP NOT CONFIRMED ({why}) - use the e-stop', flush=True)
        return 1
    if wheels:
        print('done, robot stopped (wheel velocity 0 on vel_raw)', flush=True)
    else:
        print(f'done, zero commands acknowledged (stop not checked: no publisher on {vel_topic})', flush=True)
    return code


if __name__ == '__main__':
    sys.exit(main())
