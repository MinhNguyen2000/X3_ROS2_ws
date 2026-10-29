#!/usr/bin/env python3
"""Robot-side runner for the confirmatory real-robot runs (P7): follows schedule.csv, one run at a time.

Prerequisites (operator): a shell with ROS 2 and this workspace sourced. The odometry origin is where the bringup was started and
the goals are relative to it, so before EVERY run the robot is put on the start marks and the whole bringup is relaunched (what the
2026-10 scouting runs did). The script therefore, per run:
  1. stops client, server, policy node and bringup, then asks the operator to place the robot on the start marks (Enter);
  2. starts the bringup (ros2 launch x3_bringup robot.launch.py start_policy:=false enable_face_emotion:=false), waits until
     /<agent>/odom publishes and sends the battery light cue (Int32 6 on /<agent>/RGBLight); the battery voltage is logged per run;
  3. starts the policy node (ros2 launch x3_bringup drl_policy.launch.py model_name:=<model>) and checks that its `Config:` line
     is the frozen hardware config (FROZEN below), then a goal_sequence_server for that model;
  4. starts goal_sequence_client (world_lab, path1|path2, tolerance 0.3, stop on failure);
  5. reads the outcome from the JSON the server saves (`result`, `planner_config`, `code`), checks `planner_config` and that the
     first logged pose is at the odometry origin (start_ok), then asks the operator for a fault (person entered, LiDAR/odometry
     dropout, battery, driver, other, or none).
With --no-bringup (functional tests in simulation) steps 1 and 2 are skipped and the policy and server are reused per model.
A faulted run (or a config mismatch) stays pending and is run again; everything else is kept. The score is the server's
online goal count (notes/scoring_rule_decision_20261028.md). One row per attempt goes to the manifest; the script resumes
from it. Ctrl-C stops the client, server and policy node (the policy node sends a zero command on SIGINT).

Usage: run_robot_schedule.py [--stretch N] [--max-runs K] [--dry-run] [--no-bringup --auto] [--world world_lab] [--agent agent0]
"""
import argparse
import csv
import glob
import json
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..'))
RECORDED = os.path.join(WS, 'scripts', 'recorded_paths')
SCHEDULE = os.path.join(HERE, 'schedule.csv')
OUT_DIR = os.path.join(HERE, 'robot')
MANIFEST = os.path.join(OUT_DIR, 'manifest_robot.csv')
LOG_DIR = os.path.join(OUT_DIR, 'logs')

# frozen hardware config (drl_policy.launch.py); compared with the `planner_config` the policy node publishes
FROZEN = dict(normalized_actions=True, max_lin_vel=0.2, max_angular_vel=0.3, antennae_mask_dist=0.0, rear_mask_deg=40.0,
              rear_mask_fill='train_mean', range_clip=11.1, goal_timeout=90.0, obstacle_tolerance=0.205)

COLS = ['run', 'attempt', 'stretch', 'block', 'variant', 'tier', 'model', 'path', 'status', 'waypoints_completed',
        'total_waypoints', 'success', 'elapsed_s', 'battery_v', 'saved_file', 'code', 'config_ok', 'start_ok', 'fault', 'timestamp']


# ----------------------------------------------------------------------------- pure logic (unit tested)
def load_schedule(path=SCHEDULE):
    return [dict(r, run=int(r['run']), stretch=int(r['stretch']), block=int(r['block'])) for r in csv.DictReader(open(path))]


def load_manifest(path=MANIFEST):
    return list(csv.DictReader(open(path))) if os.path.exists(path) else []


def is_kept(row):
    """An attempt counts when it produced a result, matched the frozen config, started at the odometry origin and the operator
    flagged no fault."""
    return row['status'] == 'ok' and row['config_ok'] == '1' and row.get('start_ok', '1') == '1' and row['fault'] == ''


def pending_runs(schedule, manifest, stretch=None):
    kept = {int(r['run']) for r in manifest if is_kept(r)}
    todo = [r for r in schedule if r['run'] not in kept and (stretch is None or r['stretch'] == stretch)]
    # a faulted run is repeated after the runs that have no attempt yet
    tried = {int(r['run']) for r in manifest}
    return [r for r in todo if r['run'] not in tried] + [r for r in todo if r['run'] in tried]


def config_mismatches(planner_config, expected=FROZEN):
    """Keys of the frozen config that the saved planner_config does not match (all keys when it is missing)."""
    if not planner_config:
        return sorted(expected)
    return sorted(k for k, v in expected.items() if k not in planner_config or planner_config[k] != v)


def start_offset(poses):
    """(distance [m], |yaw| [rad]) of the first logged odometry pose from the origin; None without poses.
    The odometry origin must be the start marker, so a run that does not start near (0, 0, 0) had no fresh bringup."""
    if not poses:
        return None
    p = poses[0]
    return (p['x'] ** 2 + p['y'] ** 2) ** 0.5, abs(p['yaw'])


START_TOL = (0.15, 0.15)       # m, rad: the first pose is logged a few control ticks after the start


def parse_client_result(text):
    """(success, waypoints_completed) from the goal_sequence_client log, or None."""
    m = re.search(r'\[(SUCCESS|FAILED)\].*?Completed\s+(\d+)\s+waypoint', text)
    return (m.group(1) == 'SUCCESS', int(m.group(2))) if m else None


# ----------------------------------------------------------------------------- processes
procs = []


def spawn(name, cmd, log_path):
    log = open(log_path, 'w')
    p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, cwd=WS)
    procs.append((name, p))
    return p


def stop(p, grace=8.0):
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 3.0), (signal.SIGKILL, 3.0)):
        if p.poll() is not None:
            return
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            return
        t0 = time.time()
        while time.time() - t0 < wait:
            if p.poll() is not None:
                return
            time.sleep(0.2)


def stop_named(*names):
    for name, p in list(procs):
        if name in names:
            stop(p)
            procs.remove((name, p))


def wait_log(path, needle, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            if needle in open(path, errors='ignore').read():
                return True
        except FileNotFoundError:
            pass
        time.sleep(1.0)
    return False


def wait_odom(args, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            r = subprocess.run(['ros2', 'topic', 'echo', '--once', f'/{args.agent}/odom'], capture_output=True, text=True, timeout=10)
            if r.stdout.strip():
                return True
        except subprocess.TimeoutExpired:
            pass
        time.sleep(2.0)
    return False


def start_bringup(args, tag):
    """Relaunch the robot bringup (new odometry origin) without a policy node; True when odometry is publishing."""
    log = os.path.join(LOG_DIR, f'{tag}_bringup.log')
    # no face detection / emotion recognition: they are not needed here and drain the battery faster
    spawn('bringup', ['ros2', 'launch', 'x3_bringup', 'robot.launch.py', 'start_policy:=false', 'enable_face_emotion:=false'], log)
    if not wait_odom(args, args.bringup_timeout):
        print('    bringup: no odometry within', args.bringup_timeout, 's (see', log + ')')
        return False
    time.sleep(args.settle)
    if not args.no_battery_light:       # the driver's LED effect 6 is the battery indicator the operator watches
        subprocess.run(['ros2', 'topic', 'pub', '--once', f'/{args.agent}/RGBLight', 'std_msgs/msg/Int32', '{data: 6}'],
                       capture_output=True, timeout=20)
    return True


def read_voltage(args):
    """Battery voltage from the driver topic /<agent>/voltage (one message), or '' when unavailable."""
    try:
        r = subprocess.run(['ros2', 'topic', 'echo', '--once', '--field', 'data', f'/{args.agent}/voltage'],
                           capture_output=True, text=True, timeout=8)
        return str(round(float(r.stdout.strip().splitlines()[0]), 2)) if r.stdout.strip() else ''
    except (subprocess.TimeoutExpired, ValueError, IndexError):
        return ''


def start_model(model, args, tag):
    """Policy node + sequence server for `model`; True when both are up."""
    stop_named('server', 'policy')
    pol = os.path.join(LOG_DIR, f'{tag}_policy.log')
    srv = os.path.join(LOG_DIR, f'{tag}_server.log')
    spawn('policy', ['ros2', 'launch', 'x3_bringup', 'drl_policy.launch.py', f'agent_name:={args.agent}', f'model_name:={model}'], pol)
    if not wait_log(pol, f'Running DRL policy - {model}', 90):
        print('    policy node did not start (see', pol + ')')
        return False
    spawn('server', ['ros2', 'run', 'x3_nav_bringup', 'goal_sequence_server', '--ros-args', '-p', 'planner_choice:=drl',
                     '--remap', f'__ns:=/{args.agent}'], srv)
    if not wait_log(srv, f'Detected active planner: {model}', 60):
        print('    sequence server did not detect the planner (see', srv + ')')
        return False
    return True


def run_one(run, args, tag):
    row = dict(run=run['run'], attempt='', stretch=run['stretch'], block=run['block'], variant=run['variant'], tier=run['tier'],
               model=run['model'], path=run['path'], status='launch_fail', waypoints_completed='', total_waypoints='',
               success='', elapsed_s='', battery_v='', saved_file='', code='', config_ok='0', start_ok='0', fault='',
               timestamp=datetime.now().isoformat(timespec='seconds'))
    path = run['path'].split('_')[0]                    # path1_world_lab -> path1
    client_log = os.path.join(LOG_DIR, f'{tag}_client.log')
    planner_dir = os.path.join(RECORDED, run['model'])
    os.makedirs(planner_dir, exist_ok=True)
    row['battery_v'] = read_voltage(args)
    t_start = time.time()
    client = spawn('client', ['ros2', 'run', 'x3_nav_bringup', 'goal_sequence_client', '--ros-args', '-p', f'world:={args.world}',
                              '-p', f'path:={path}', '-p', 'goal_tolerance:=0.3', '-p', 'stop_on_failure:=true',
                              '--remap', f'__ns:=/{args.agent}'], client_log)
    try:
        client.wait(timeout=args.seq_timeout)
    except subprocess.TimeoutExpired:
        stop(client)
        row['status'] = 'timeout'
        return row
    row['elapsed_s'] = round(time.time() - t_start, 1)
    res = parse_client_result(open(client_log, errors='ignore').read())
    time.sleep(1.0)
    saved = sorted((p for p in glob.glob(os.path.join(planner_dir, f'{path}_{args.world}_*.json')) if os.path.getmtime(p) >= t_start - 1),
                   key=os.path.getmtime)
    if not saved or res is None:
        row['status'] = 'no_output'
        return row
    d = json.load(open(saved[-1]))
    r = d.get('result') or {}
    row.update(status='ok' if d.get('poses') and d.get('velocities') and r else 'no_output', saved_file=os.path.relpath(saved[-1], WS),
               waypoints_completed=r.get('waypoints_completed', res[1]), total_waypoints=r.get('total_waypoints', ''),
               success=int(r.get('success', res[0])), code=d.get('code') or '')
    bad = config_mismatches(d.get('planner_config'))
    row['config_ok'] = '0' if bad else '1'
    off = start_offset(d.get('poses'))
    row['start_ok'] = '1' if off and off[0] <= START_TOL[0] and off[1] <= START_TOL[1] else '0'
    if row['start_ok'] == '0':
        print('    START NOT AT THE ODOMETRY ORIGIN (offset %s): the bringup was not fresh' % (None if off is None else '%.2f m, %.2f rad' % off))
    if bad:
        print('    CONFIG MISMATCH with the frozen hardware config:', ', '.join(bad))
    return row


def append_manifest(row):
    os.makedirs(OUT_DIR, exist_ok=True)
    new = not os.path.exists(MANIFEST)
    with open(MANIFEST, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLS)
        if new:
            w.writeheader()
        w.writerow(row)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--stretch', type=int, help='only the pending runs of this stretch')
    ap.add_argument('--max-runs', type=int, default=0)
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--auto', action='store_true', help='no operator prompts (functional tests in simulation only)')
    ap.add_argument('--world', default='world_lab')
    ap.add_argument('--agent', default='agent0')
    ap.add_argument('--seq-timeout', type=float, default=300)
    ap.add_argument('--no-bringup', action='store_true', help='do not manage the bringup (simulation tests): policy and server are reused per model')
    ap.add_argument('--bringup-timeout', type=float, default=180)
    ap.add_argument('--no-battery-light', action='store_true', help='do not send the battery light cue (RGBLight 6) after the bringup')
    ap.add_argument('--settle', type=float, default=5, help='wait after the odometry is up [s]')
    ap.add_argument('--schedule', default=SCHEDULE)
    args = ap.parse_args()

    schedule, manifest = load_schedule(args.schedule), load_manifest()
    todo = pending_runs(schedule, manifest, args.stretch)
    print(f'{len(schedule)} scheduled runs, {len({int(r["run"]) for r in manifest if is_kept(r)})} kept, {len(todo)} pending'
          + (f' in stretch {args.stretch}' if args.stretch else ''))
    if args.dry_run:
        for r in todo[:12]:
            print(f'  run {r["run"]:3d} stretch {r["stretch"]} block {r["block"]:2d}  {r["variant"]:13s} {r["tier"]}  {r["model"]:15s} {r["path"]}')
        return
    os.makedirs(LOG_DIR, exist_ok=True)

    def on_signal(signum, frame):
        print('\nInterrupted: stopping client, server and policy node...')
        for _, p in reversed(procs):
            stop(p)
        sys.exit(130)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    current, done = None, 0
    for run in todo:
        if args.max_runs and done >= args.max_runs:
            break
        tag = f'{datetime.now():%Y%m%d_%H%M%S}_run{run["run"]:03d}'
        attempt = 1 + sum(int(m['run']) == run['run'] for m in manifest)
        print(f'\n=== run {run["run"]}/{len(schedule)} (stretch {run["stretch"]}, block {run["block"]}): '
              f'{run["variant"]} {run["tier"]} {run["model"]}, {run["path"]}')
        fail = dict(run=run['run'], attempt=attempt, stretch=run['stretch'], block=run['block'], variant=run['variant'], tier=run['tier'],
                    model=run['model'], path=run['path'], status='launch_fail', config_ok='0', start_ok='0', fault='',
                    timestamp=datetime.now().isoformat(timespec='seconds'))
        if not args.no_bringup:
            stop_named('client', 'server', 'policy', 'bringup')
            current = None
            if not args.auto:
                if input('Place the robot on the start marks (this pose becomes the odometry origin). [Enter] relaunch and run, [q] quit: ').strip().lower() == 'q':
                    break
            if not start_bringup(args, tag):
                append_manifest(fail)
                print('    stopping: fix the bringup and start the script again (it resumes)')
                break
        elif not args.auto:
            if input('[Enter] run, [q] quit: ').strip().lower() == 'q':
                break
        if current != run['model']:
            current = run['model'] if start_model(run['model'], args, tag) else None
            if current is None:
                append_manifest(fail)
                print('    stopping: fix the launch problem and start the script again (it resumes)')
                break
        row = run_one(run, args, tag)
        row['attempt'] = attempt
        print(f'    -> {row["status"]}: {row["waypoints_completed"]}/{row["total_waypoints"]} goals in {row["elapsed_s"]} s')
        if not args.auto:
            row['fault'] = input('Fault? [Enter = none, or a reason: person / lidar / odom / battery / driver / other]: ').strip()
        append_manifest(row)
        manifest.append({k: str(v) for k, v in row.items()})
        done += 1
    for _, p in reversed(procs):
        stop(p)


if __name__ == '__main__':
    main()
