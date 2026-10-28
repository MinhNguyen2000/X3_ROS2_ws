#!/usr/bin/env python3
"""Find the sector of the LiDAR scan that is the robot's own structure (antennae etc.), from a static scan.

Place the robot in open space (nothing within ~1 m except the robot itself), then either
  - on the robot:  antenna_sector.py --topic /agent0/scan --scans 30
  - from a file:   antenna_sector.py --json scan.json    ({"angle_min":..., "angle_increment":..., "ranges": [[...], ...]})

A beam counts as self-return when its range is below --near [m] in at least --frac of the scans. The result is
given in degrees relative to the robot's forward axis (0 = forward, +-180 = behind), the half-width around the rear
to pass as `rear_mask_deg`, and the observation groups (18 groups of 20 degrees as in policy_node) it touches.
Returns that are not behind the sensor are reported separately: they would NOT be covered by a rear wedge.
"""
import argparse
import json

import numpy as np


def analyse(scans, angle_min, angle_increment, near=0.3, frac=0.5, n_groups=18):
    scans = np.asarray(scans, dtype=np.float64)
    valid = np.isfinite(scans) & (scans > 0.0)
    close = valid & (scans < near)
    hit = close.mean(axis=0) >= frac
    ang = np.degrees(angle_min + angle_increment * np.arange(scans.shape[1]))
    ang = (ang + 180.0) % 360.0 - 180.0                           # (-180, 180], 0 = forward
    rear_off = 180.0 - np.abs(ang)                                # angular distance from straight behind
    hit_ang = ang[hit]
    out = dict(n_scans=int(scans.shape[0]), n_beams=int(scans.shape[1]), n_self_beams=int(hit.sum()))
    rear = hit & (rear_off <= 90.0)                               # rear half-plane
    front = hit & (rear_off > 90.0)                               # front half-plane: a rear wedge would not cover these
    if not hit.any():
        out.update(rear_half_width_deg=0.0, rear_sector_deg=None, front_beams=0, groups=[])
        return out
    rear_signed = ((ang[rear] + 360.0) % 360.0) - 180.0           # 0 = straight behind, + towards the robot's right
    # observation groups: index 0 = forward, running clockwise as in policy_node (angle -> index = (-ang) mod 360)
    idx = np.floor(((-hit_ang) % 360.0) / (360.0 / n_groups)).astype(int) % n_groups
    out.update(rear_half_width_deg=round(float(np.abs(rear_signed).max()), 1) if rear.any() else 0.0,
               rear_sector_deg=[round(float(rear_signed.min()), 1), round(float(rear_signed.max()), 1)] if rear.any() else None,
               front_beams=int(front.sum()), groups=sorted({int(i) for i in idx}))
    return out


def capture(topic, n):
    import rclpy
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node('antenna_sector')
    got = []
    meta = {}

    def cb(msg):
        meta.update(angle_min=msg.angle_min, angle_increment=msg.angle_increment)
        got.append(list(msg.ranges))

    node.create_subscription(LaserScan, topic, cb, QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
    while len(got) < n:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()
    return got, meta['angle_min'], meta['angle_increment']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default=None)
    ap.add_argument('--scans', type=int, default=30)
    ap.add_argument('--json', default=None)
    ap.add_argument('--near', type=float, default=0.3)
    ap.add_argument('--frac', type=float, default=0.5)
    ap.add_argument('--save', default=None, help='write the captured scans to this JSON file')
    args = ap.parse_args()

    if args.json:
        d = json.load(open(args.json))
        scans, amin, inc = d['ranges'], d['angle_min'], d['angle_increment']
    elif args.topic:
        scans, amin, inc = capture(args.topic, args.scans)
        if args.save:
            json.dump(dict(angle_min=amin, angle_increment=inc, ranges=scans), open(args.save, 'w'))
    else:
        ap.error('give --topic or --json')
    res = analyse(scans, amin, inc, args.near, args.frac)
    print(json.dumps(res, indent=2))
    if res['front_beams']:
        print('WARNING: self-returns in the front half-plane are not covered by a rear wedge; '
              'check the sensor mounting before using rear_mask_deg')


if __name__ == '__main__':
    main()
