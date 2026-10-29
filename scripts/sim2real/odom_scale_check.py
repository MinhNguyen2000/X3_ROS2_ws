#!/usr/bin/env python3
"""Odometry scale check on the robot: compare what odometry says with a tape measure.

The runs are scored by the robot's own odometry, so an odometry scale error changes what 'goal reached' means physically.
rf2o on the raw scan underestimates linear motion (the robot's own parts behind the LiDAR move with it); this tool puts a
number on the remaining error of the setup that will be used for the runs.

Procedure (clear 3 m runway, e-stop in hand, the frozen bringup running: robot.launch.py start_policy:=false enable_face_emotion:=false):
  1. python3 scripts/sim2real/odom_scale_check.py --kind straight --note "rf2o on scan_filtered"
  2. place the robot at the start of a taped line, press Enter (odometry start pose is read)
  3. drive it straight for about 3 m at 0.2 m/s, e.g.
       ros2 topic pub -r 20 -t 300 /agent0/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.2}}}"
     (check the driver's cmd_vel topic and message type first), stop, press Enter (end pose is read)
  4. type the tape-measured distance in metres; for --kind turn drive about one full turn and type the measured angle in degrees
One row per topic (/agent0/odom, the EKF output the policy uses, and /agent0/odom_rf2o) is appended to scripts/sim2real/odom_scale_check.csv
with ratio = odometry / measured (1.0 = exact, below 1.0 = odometry underestimates).
"""
import argparse
import csv
import math
import os
import subprocess
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'odom_scale_check.csv')
COLS = ['timestamp', 'kind', 'topic', 'odom_value', 'measured', 'unit', 'ratio', 'note', 'code']


# ----------------------------------------------------------------------------- pure functions (unit tested)
def yaw_from_quaternion(z, w):
    """Yaw of a planar pose from the quaternion z and w components."""
    return 2.0 * math.atan2(z, w)


def distance(p0, p1):
    return math.hypot(p1[0] - p0[0], p1[1] - p0[1])


def turned_angle(yaw0, yaw1, turns_hint=0):
    """Signed angle [rad] from yaw0 to yaw1 wrapped to (-pi, pi], plus whole turns_hint turns (odometry alone cannot count turns)."""
    d = (yaw1 - yaw0 + math.pi) % (2 * math.pi) - math.pi
    return d + turns_hint * 2 * math.pi


def ratio(odom_value, measured):
    return odom_value / measured if measured else float('nan')


# ----------------------------------------------------------------------------- robot access
def read_pose(topic):
    """(x, y, yaw) from one message of an Odometry topic."""
    def field(name):
        r = subprocess.run(['ros2', 'topic', 'echo', '--once', '--field', name, topic], capture_output=True, text=True, timeout=15)
        return float(r.stdout.strip().splitlines()[0])
    return (field('pose.pose.position.x'), field('pose.pose.position.y'),
            yaw_from_quaternion(field('pose.pose.orientation.z'), field('pose.pose.orientation.w')))


def code_marker():
    try:
        r = subprocess.run(['git', '-C', HERE, 'rev-parse', '--short', 'HEAD'], capture_output=True, text=True, timeout=5)
        dirty = subprocess.run(['git', '-C', HERE, 'status', '--porcelain', '--untracked-files=no', '--', '../../src'],
                               capture_output=True, text=True, timeout=10).stdout.strip()
        return r.stdout.strip() + ('+dirty' if dirty else '')
    except Exception:
        return ''


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--kind', choices=['straight', 'turn'], default='straight')
    ap.add_argument('--agent', default='agent0')
    ap.add_argument('--topics', nargs='+', default=None, help='odometry topics (default: /<agent>/odom and /<agent>/odom_rf2o)')
    ap.add_argument('--turns', type=int, default=1, help='whole turns driven, for --kind turn')
    ap.add_argument('--note', default='', help='what is being tested, e.g. "rf2o on scan_filtered"')
    ap.add_argument('--out', default=OUT)
    args = ap.parse_args()
    topics = args.topics or [f'/{args.agent}/odom', f'/{args.agent}/odom_rf2o']

    input('Robot at the start mark. [Enter] to read the start pose: ')
    start = {t: read_pose(t) for t in topics}
    input('Drive it, let it stop. [Enter] to read the end pose: ')
    end = {t: read_pose(t) for t in topics}
    unit = 'm' if args.kind == 'straight' else 'deg'
    measured = float(input(f'Measured {"distance [m]" if args.kind == "straight" else "angle turned [deg]"}: '))

    new = not os.path.exists(args.out)
    with open(args.out, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLS)
        if new:
            w.writeheader()
        for t in topics:
            if args.kind == 'straight':
                val = distance(start[t], end[t])
            else:
                val = abs(math.degrees(turned_angle(start[t][2], end[t][2], args.turns)))
            row = dict(timestamp=datetime.now().isoformat(timespec='seconds'), kind=args.kind, topic=t, odom_value=round(val, 4),
                       measured=measured, unit=unit, ratio=round(ratio(val, measured), 4), note=args.note, code=code_marker())
            w.writerow(row)
            print(f'{t}: odometry {val:.3f} {unit}, measured {measured} {unit}, ratio {row["ratio"]}')


if __name__ == '__main__':
    main()
