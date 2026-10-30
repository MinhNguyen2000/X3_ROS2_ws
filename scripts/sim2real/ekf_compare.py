#!/usr/bin/env python3
"""Compare odometry parameter sets (EKF + covariance files, odom_profile of x3_bringup) on the same recorded drives.

The odometry is the scoring sensor of the real-robot runs, so the setup is chosen from recorded data before the first
stretch, not by driving once per setup. Raw sensor topics of a drive are recorded once on the robot; each profile is
replayed through lidar_rear_mask, rf2o, the covariance filter and the EKF on the bag's clock; the replayed /agent0/odom
is scored against the overhead-camera track (/agent0/odom_camera) recorded in the same bag.

  1. on the robot, per drive:       ekf_compare.py record-cmd --name straight3m     (prints the ros2 bag record command)
  2. replay one profile of a bag:   ekf_compare.py replay --bag BAG --profile ''  --out OUT_main
                                    ekf_compare.py replay --bag BAG --profile _robot_20261029 --out OUT_robot
  2b. optional: ekf_compare.py decimate --bag BAG --factor 5 --out BAG_10hz   (keeps every 5th IMU, mag and wheel-velocity
                                    message: a 50 Hz drive gives the paired 10 Hz data; replay and score it the same way)
  3. score:                         ekf_compare.py score --bag BAG --replay main=OUT_main --replay robot=OUT_robot

Scores (one row per bag and candidate; 'live' is the /agent0/odom recorded during the drive):
  fit_rms_m     position RMS after a rigid 2D fit (rotation + translation, no scale) of the odometry path onto the camera path
  end_err_m     distance between the fitted last odometry position and the camera's last position
  len_ratio     odometry path length / camera path length (the camera's x-y is poor when the robot is off to one side of
                the scene, so read this with the scale check against the tape measure)
  yaw_drift_deg turning of the odometry minus turning of the camera between first and last sample (the camera yaw is the
                stable signal of the overhead tracker); yaw_max_deg is the largest deviation along the drive
  odom_hz       median rate of the odometry (the EKF output follows its inputs, so it is the driver rate when that is the
                slowest fast input)
  warn          lines in the replay log about update rate, transforms or dropped messages
"""
import argparse
import glob
import os
import re
import signal
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
AGENT = 'agent0'
RAW_TOPICS = [f'/{AGENT}/scan', f'/{AGENT}/imu/data_raw', f'/{AGENT}/imu/mag', f'/{AGENT}/vel_raw']
REF_TOPICS = [f'/{AGENT}/odom_camera', f'/{AGENT}/odom', f'/{AGENT}/odom_rf2o', f'/{AGENT}/cmd_vel']
DECIMATED = [f'/{AGENT}/imu/data_raw', f'/{AGENT}/imu/mag', f'/{AGENT}/vel_raw']
WARN = re.compile(r'update rate|could not transform|extrapolat|dropp|timed out|exceed', re.I)


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def yaw_from_quaternion(q):
    """Yaw of a (x, y, z, w) quaternion."""
    x, y, z, w = q
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rigid_fit(src, dst):
    """Rotation angle and translation mapping src (N, 2) onto dst (N, 2), least squares, no scale (Kabsch)."""
    sc, dc = src.mean(0), dst.mean(0)
    h = (src - sc).T @ (dst - dc)
    th = np.arctan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    c, s = np.cos(th), np.sin(th)
    r = np.array([[c, -s], [s, c]])
    return th, dc - r @ sc


def resample(t, xyyaw, t_new):
    """Linear resample of (N, 3) x, y, yaw onto t_new (yaw unwrapped first); t_new outside t is clipped to the ends."""
    out = np.c_[[np.interp(t_new, t, xyyaw[:, k]) for k in (0, 1)]].T
    yaw = np.interp(t_new, t, np.unwrap(xyyaw[:, 2]))
    return np.c_[out, yaw]


def path_length(xy):
    return float(np.hypot(*np.diff(xy, axis=0).T).sum())


def score(t_odom, odom, t_cam, cam):
    """Scores of one odometry track (N, 3) against the camera track (M, 3) over the time both cover."""
    if len(t_odom) < 2 or len(t_cam) < 2:
        return None
    lo, hi = max(t_odom[0], t_cam[0]), min(t_odom[-1], t_cam[-1])
    if hi <= lo:
        return None
    t = t_cam[(t_cam >= lo) & (t_cam <= hi)]
    if len(t) < 3:
        return None
    o = resample(t_odom, odom, t)
    c = resample(t_cam, cam, t)
    th, tr = rigid_fit(o[:, :2], c[:, :2])
    rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    fitted = o[:, :2] @ rot.T + tr
    err = np.hypot(*(fitted - c[:, :2]).T)
    dyaw = wrap((o[:, 2] - o[0, 2]) - (c[:, 2] - c[0, 2]))
    return {
        'fit_rms_m': float(np.sqrt((err ** 2).mean())),
        'end_err_m': float(err[-1]),
        'len_ratio': path_length(o[:, :2]) / path_length(c[:, :2]) if path_length(c[:, :2]) > 0.2 else float('nan'),
        'yaw_drift_deg': float(np.degrees(dyaw[-1])),
        'yaw_max_deg': float(np.degrees(np.abs(dyaw).max())),
        'odom_hz': float(1.0 / np.median(np.diff(t_odom))),
    }


def read_odometry(bag_dir, topic):
    """(t [s], (N, 3) x, y, yaw) of a nav_msgs/Odometry topic, by header stamp."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    t, v = [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, Odometry)
        q = m.pose.pose.orientation
        t.append(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
        v.append([m.pose.pose.position.x, m.pose.pose.position.y, yaw_from_quaternion((q.x, q.y, q.z, q.w))])
    return np.array(t), np.array(v)


def keep_every(index, factor):
    """True for every factor-th message (0, factor, 2*factor, ...) of a topic."""
    return index % factor == 0


def decimate(bag, factor, out):
    """Copy a bag keeping every factor-th IMU, magnetometer and wheel-velocity message (all other topics whole), so a
    drive recorded at a high driver rate gives a paired low-rate version of the same data."""
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=out, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    for meta in reader.get_all_topics_and_types():
        writer.create_topic(meta)
    seen = {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        i = seen.get(topic, 0)
        seen[topic] = i + 1
        if topic not in DECIMATED or keep_every(i, factor):
            writer.write(topic, data, t)
    return {topic: seen[topic] for topic in DECIMATED if topic in seen}


def record_cmd(name):
    return ['ros2', 'bag', 'record', '-o', name] + RAW_TOPICS + REF_TOPICS


def replay(bag, profile, out, rate):
    launch = os.path.join(HERE, 'replay_odom.launch.py')
    log = out + '.launch.log'
    with open(log, 'w') as lf:
        profile_arg = [f'odom_profile:={profile}'] if profile else []     # an empty value is not a valid launch argument
        lp = subprocess.Popen(['ros2', 'launch', launch] + profile_arg, stdout=lf, stderr=subprocess.STDOUT,
                              start_new_session=True)
        time.sleep(6)
        rec = subprocess.Popen(['ros2', 'bag', 'record', '-o', out, f'/{AGENT}/odom'], stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(2)
        subprocess.run(['ros2', 'bag', 'play', bag, '--clock', '--rate', str(rate), '--topics'] + RAW_TOPICS, check=True)
        time.sleep(3)
        for p in (rec, lp):
            os.killpg(p.pid, signal.SIGINT)
        for p in (rec, lp):
            p.wait(timeout=30)
    return log


def warn_count(log):
    try:
        return sum(1 for line in open(log) if WARN.search(line))
    except OSError:
        return -1


def cmd_score(args):
    t_cam, cam = read_odometry(args.bag, f'/{AGENT}/odom_camera')
    rows = []
    t, v = read_odometry(args.bag, f'/{AGENT}/odom')
    rows.append(('live', score(t, v, t_cam, cam), None))
    for item in args.replay:
        name, path = item.split('=', 1)
        t, v = read_odometry(path, f'/{AGENT}/odom')
        rows.append((name, score(t, v, t_cam, cam), warn_count(path + '.launch.log')))
    print(f"{'candidate':<10}{'fit_rms_m':>10}{'end_err_m':>10}{'len_ratio':>10}{'yaw_drift_deg':>14}{'yaw_max_deg':>12}{'odom_hz':>9}{'warn':>6}")
    for name, s, w in rows:
        if s is None:
            print(f'{name:<10} no overlap with the camera track')
            continue
        print(f"{name:<10}{s['fit_rms_m']:>10.3f}{s['end_err_m']:>10.3f}{s['len_ratio']:>10.3f}"
              f"{s['yaw_drift_deg']:>14.1f}{s['yaw_max_deg']:>12.1f}{s['odom_hz']:>9.1f}{'' if w is None else w:>6}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('record-cmd')
    p.add_argument('--name', required=True)
    p = sub.add_parser('replay')
    p.add_argument('--bag', required=True)
    p.add_argument('--profile', default='', help="odom_profile: '' (main) or _robot_20261029")
    p.add_argument('--out', required=True)
    p.add_argument('--rate', type=float, default=1.0)
    p = sub.add_parser('decimate')
    p.add_argument('--bag', required=True)
    p.add_argument('--factor', type=int, default=5, help='keep every factor-th IMU, mag and wheel-velocity message')
    p.add_argument('--out', required=True)
    p = sub.add_parser('score')
    p.add_argument('--bag', required=True)
    p.add_argument('--replay', action='append', default=[], metavar='NAME=OUT_BAG')
    args = ap.parse_args()
    if args.cmd == 'record-cmd':
        print(' '.join(record_cmd(args.name)))
    elif args.cmd == 'decimate':
        print('kept every', args.factor, 'th message; input counts', decimate(args.bag, args.factor, args.out))
    elif args.cmd == 'replay':
        print('launch log:', replay(args.bag, args.profile, args.out, args.rate))
    else:
        cmd_score(args)


if __name__ == '__main__':
    main()
