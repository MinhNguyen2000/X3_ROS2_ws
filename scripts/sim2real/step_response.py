#!/usr/bin/env python3
"""Step-response test of the delta bounds (P14): measures from the bags of drive_pattern.py step, ramp10, ramp20 and
turnsteps, as fixed in the personal_hub note delta_bound_step_test_protocol_20261109.md.

Per drive part and signal:
  lag       delay of the command (0 to 0.5 s, 0.02 s grid) that minimises the RMS of measured - commanded speed
  rms, max  error after shifting by the lag, while the command is non-zero or within 1 s after it returns to 0;
            0.3 s after each step change is left out in step parts
Per step change: delay to 10 %, 10 to 90 % rise time, peak acceleration (largest slope of the measured speed after a
0.1 s moving mean, within 1 s of the change).
Signals: linear vel_raw linear.x (judged) and EKF odom linear.x; angular IMU gyro z (judged) and vel_raw angular.z.
slip_max: largest |odom - vel_raw| linear speed in the window. Times are the recorder's receive stamps.
Drives are found as bursts of cmd_vel with a non-zero command; --exclude HH:MM:SS (the drive's first non-zero command)
leaves a drive out of the pass summary.

  step_response.py BAG [BAG ...] [--exclude HH:MM:SS ...] [--out DIR]   (needs ROS 2 for rosbag2_py)
"""
import argparse
import datetime
import os

import numpy as np
import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

AGENT = '/agent0'
TOP = {'linear': 0.5, 'angular': 1.0}
JUDGED = {'linear': 'vel_raw', 'angular': 'gyro'}
TURN_PARTS = [('step', None, 120), ('ramp10', 0.1, 200), ('ramp20', 0.2, 160)]   # turnsteps parts in 20 Hz ticks
# the protocol fixed 0 to 0.5 s; the 0.2 ramps fitted at that edge, so the search runs to 1.0 s (pass rule unchanged)
LAGS = np.arange(0.0, 1.0001, 0.02)


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    s = {k: [] for k in ('cmd', 'vel', 'gyro', 'odom', 'volt')}
    while r.has_next():
        topic, data, t = r.read_next()
        m = deserialize_message(data, types[topic])
        t = t / 1e9
        if topic == f'{AGENT}/cmd_vel':
            s['cmd'].append((t, m.twist.linear.x, m.twist.angular.z))
        elif topic == f'{AGENT}/vel_raw':
            s['vel'].append((t, m.twist.linear.x, m.twist.angular.z))
        elif topic == f'{AGENT}/imu/data_raw':
            s['gyro'].append((t, -m.angular_velocity.z))    # the board's gyro z points down (reads -1 at +1 rad/s)
        elif topic == f'{AGENT}/odom':
            s['odom'].append((t, m.twist.twist.linear.x))
        elif topic == f'{AGENT}/voltage':
            s['volt'].append((t, m.data))
    return {k: np.array(v, float) for k, v in s.items()}


def held(tc, c, t):
    """Zero-order hold of the command c (stamps tc) at times t; 0 before the first command."""
    i = np.searchsorted(tc, t, side='right') - 1
    return np.where(i >= 0, c[np.clip(i, 0, None)], 0.0)


def drives(cmd):
    """Bursts of cmd_vel (gap > 2 s splits) that contain a non-zero command, each led by one zero command (the lead-in
    zeros of drive_pattern.py come before its acknowledgement wait, so the gap cuts them off)."""
    cut = np.flatnonzero(np.diff(cmd[:, 0]) > 2.0) + 1
    return [np.vstack([[b[0, 0] - 0.05, 0.0, 0.0], b]) for b in np.split(cmd, cut) if np.any(b[:, 1:] != 0)]


def kind_of(b):
    if np.any(b[:, 2] != 0):
        return 'turnsteps'
    d = np.abs(np.diff(b[:, 1])).max()
    return 'step' if d > 0.3 else 'ramp20' if d > 0.075 else 'ramp10'


def parts_of(b, kind):
    """(part, bound, axis, commands of the part) for one drive."""
    i0 = int(np.flatnonzero(np.any(b[:, 1:] != 0, axis=1))[0])
    if kind != 'turnsteps':
        return [(kind, {'step': None, 'ramp10': 0.1, 'ramp20': 0.2}[kind], 'linear', b[:, [0, 1]])]
    out, i = [], i0
    for name, bound, n in TURN_PARTS:
        seg = b[i:i + n]
        # the part starts with its first command: +1 rad/s for the step, one bound step for the ramps
        assert np.isclose(seg[0, 2], 1.0 if bound is None else bound), (name, seg[0])
        lo = b[i - 1, 0] if i > 0 else seg[0, 0] - 0.05
        out.append((name, bound, 'angular', np.vstack([[lo, 0.0], seg[:, [0, 2]]])))
        i += n
    return out


def track(tc, c, t, y, is_step):
    """lag, rms, max error of y(t) against the command (tc, c), in the protocol's window."""
    nz = tc[c != 0]
    j = np.searchsorted(nz, t, side='right') - 1
    win = (j >= 0) & (t - nz[np.clip(j, 0, None)] <= 1.0) & (t <= nz[-1] + 1.0)
    if is_step:
        chg = tc[1:][np.diff(c) != 0]
        for tg in chg:
            win &= ~((t >= tg) & (t < tg + 0.3))
    errs = [y[win] - held(tc, c, t[win] - L) for L in LAGS]
    k = int(np.argmin([np.sqrt(np.mean(e ** 2)) for e in errs]))
    return LAGS[k], float(np.sqrt(np.mean(errs[k] ** 2))), float(np.abs(errs[k]).max()), win


def step_changes(tc, c, t, y, top):
    rows = []
    for k in np.flatnonzero(np.diff(c) != 0) + 1:
        a, b, tg = c[k - 1], c[k], tc[k]
        if abs(b - a) < 0.3 * top:
            continue
        sel = (t >= tg) & (t <= tg + 1.5)
        tt, yy = t[sel], y[sel]
        sm = np.convolve(yy, np.ones(5) / 5, mode='same')       # 0.1 s at 50 Hz
        frac = (yy - a) / (b - a)
        t10 = tt[np.argmax(frac >= 0.1)] if np.any(frac >= 0.1) else np.nan
        t90 = tt[np.argmax(frac >= 0.9)] if np.any(frac >= 0.9) else np.nan
        slope = np.gradient(sm, tt) * np.sign(b - a)
        inner = (tt >= tg) & (tt <= tg + 1.0)
        inner[:2], inner[-2:] = False, False      # moving-mean edges
        rows.append(dict(change_from=a, change_to=b, direction='speed up' if abs(b) > abs(a) else 'slow down'
                         if abs(b) < abs(a) else 'reverse', delay10_s=t10 - tg, rise_10_90_s=t90 - t10,
                         peak_acc=float(slope[inner].max()) if inner.any() else np.nan))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--exclude', nargs='*', default=[])
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'step_test_output'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    trk, stp = [], []
    for bag in args.bags:
        s = read(bag)
        sig = {'vel_raw': s['vel'][:, [0, 1]], 'odom': s['odom'], 'gyro': s['gyro'], 'vel_raw_w': s['vel'][:, [0, 2]]}
        for b in drives(s['cmd']):
            kind = kind_of(b)
            first = b[np.flatnonzero(np.any(b[:, 1:] != 0, axis=1))[0], 0]
            drive = datetime.datetime.fromtimestamp(first).strftime('%H:%M:%S')
            sel = (s['volt'][:, 0] >= b[0, 0]) & (s['volt'][:, 0] <= b[-1, 0])
            volt = float(s['volt'][sel, 1].mean())
            for part, bound, axis, pc in parts_of(b, kind):
                tc, c = pc[:, 0], pc[:, 1]
                is_step = bound is None
                names = ['vel_raw', 'odom'] if axis == 'linear' else ['gyro', 'vel_raw_w']
                for name in names:
                    m = sig[name]
                    inr = (m[:, 0] >= tc[0] - 0.5) & (m[:, 0] <= tc[-1] + 1.5)
                    t, y = m[inr, 0], m[inr, 1]
                    lag, rms, mx, win = track(tc, c, t, y, is_step)
                    row = dict(bag=os.path.basename(os.path.normpath(bag)), drive=drive, kind=kind, part=part,
                               bound=bound, axis=axis, signal=name.replace('_w', ''), judged=name == JUDGED[axis],
                               lag_s=lag, rms=rms, max_err=mx, volt=volt, excluded=drive in args.exclude)
                    if axis == 'linear' and name == 'vel_raw':
                        od = np.interp(t[win], s['odom'][:, 0], s['odom'][:, 1])
                        row['slip_max'] = float(np.abs(od - y[win]).max())
                    trk.append(row)
                    if is_step:
                        stp += [dict(row, **r) for r in step_changes(tc, c, t, y, TOP[axis])]
    trk, stp = pd.DataFrame(trk), pd.DataFrame(stp)
    trk.to_csv(os.path.join(args.out, 'tracking.csv'), index=False, float_format='%.4g')
    stp.to_csv(os.path.join(args.out, 'steps.csv'), index=False, float_format='%.4g')

    pd.set_option('display.width', 200)
    print('drives:')
    print(trk.drop_duplicates('drive')[['bag', 'drive', 'kind', 'volt', 'excluded']].to_string(index=False))
    use = trk[trk.judged & ~trk.excluded]
    print('\npass rule (ramps at bound B: lag <= 0.2 s and RMS <= 10 % of top on every repeat):')
    for (axis, bound), d in use[use.bound.notna()].groupby(['axis', 'bound']):
        ok = (d.lag_s <= 0.2) & (d.rms <= 0.1 * TOP[axis])
        print(f'  {axis:7s} B={bound}: repeats {len(d)}, lag {list(d.lag_s.round(2))}, '
              f'rms {list(d.rms.round(3))}, pass {bool(ok.all()) and len(d) == 3}')
    s2 = stp[stp.judged & ~stp.excluded]
    print('\nstep changes (judged signal): peak acceleration against 1.5 x bound acceleration')
    for (axis, direction), d in s2.groupby(['axis', 'direction']):
        acc = TOP[axis] * 10      # acceleration per unit of bound: top speed x 10 decisions per second
        print(f'  {axis:7s} {direction:9s} n={len(d)} peak acc median {d.peak_acc.median():.2f} '
              f'[{d.peak_acc.min():.2f}, {d.peak_acc.max():.2f}]; rise 10-90 median {d.rise_10_90_s.median():.2f} s; '
              f'delay10 median {d.delay10_s.median():.2f} s; 1.5 x B: B=0.1 {1.5 * 0.1 * acc:.2f}, '
              f'B=0.2 {1.5 * 0.2 * acc:.2f}')
    if 'slip_max' in use:
        print('\nslip_max (linear, odom - vel_raw):', list(use.slip_max.dropna().round(3)))


if __name__ == '__main__':
    main()
