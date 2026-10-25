#!/usr/bin/env python3
'''
Batch runner for multi-goal Gazebo evaluation of the TD3 (delta/direct) and APF planners.

For every job (planner/checkpoint x world x path x repeat) this launches a fresh Gazebo, the
planner node, the goal sequence server and the goal sequence client, waits for the sequence to
finish, tears everything down, and appends one row to a manifest CSV.

The navigation results themselves (poses, velocities, ...) are still written by the existing
goal_sequence_server to scripts/recorded_paths/<planner>/<path>_<world>_<ddmmyy>_<HHMM>.json.
This script only orchestrates and records the outcome (the saved JSON has no success flag).

Usage (from any shell; ROS 2 and the workspace are sourced internally):
    python3 scripts/batch_multigoal.py --dry-run
    python3 scripts/batch_multigoal.py --only-model TD3_00386_1000 --only-scenario world_1/path1
    python3 scripts/batch_multigoal.py            # full batch, resumable
'''
import argparse
import csv
import hashlib
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

WS          = Path(__file__).resolve().parents[1]
POLICY_DIR  = WS / 'src' / 'x3_drl_policy' / 'policies'
PATHS_DIR   = WS / 'src' / 'x3_nav_bringup' / 'paths'
RECORDED    = WS / 'scripts' / 'recorded_paths'
OUT_DIR     = WS / 'scripts' / 'batch_runs'
LOG_DIR     = OUT_DIR / 'logs'
MANIFEST    = OUT_DIR / 'manifest.csv'

NS          = '/agent0'
ROS_DOMAIN  = '66'      # dedicated domain so the batch never talks to interactive sessions (domain 6)
# Mirrors .devcontainer/ros_entrypoint.sh (which `docker exec` / non-interactive shells skip):
# IGN_GAZEBO_RESOURCE_PATH lets the Gazebo GUI resolve model://x3_description/meshes/*.STL.
ROS_SETUP   = ('source /opt/ros/humble/setup.bash && source {ws}/install/setup.bash && '
               'export ROS_DOMAIN_ID={{domain}} ROS_LOCALHOST_ONLY=0 LIDAR_TYPE=x3 && '
               'export IGN_GAZEBO_RESOURCE_PATH={ws}/install/x3_description/share:'
               '$IGN_GAZEBO_RESOURCE_PATH').format(ws=WS)

# (world, path) pairs: 3 paths in world_1, one path in each of world_2..4
SCENARIOS   = [('world_1', 'path1'), ('world_1', 'path2'), ('world_1', 'path3'),
               ('world_2', 'path1'), ('world_3', 'path1'), ('world_4', 'path1')]
# TD3_delta: ids 386-395, TD3_direct: ids 396-405 (id 386-395 -> policy_node infers delta mode)
DELTA_IDS   = range(386, 396)
DIRECT_IDS  = range(396, 406)

MANIFEST_COLS = ['model', 'kind', 'world', 'path', 'rep', 'status', 'success',
                 'waypoints_completed', 'total_waypoints', 'elapsed_s', 'saved_file', 'log_file',
                 'timestamp', 'code_state']

procs = []          # (name, Popen) currently alive, for cleanup
CODE_STATE = ''     # git HEAD + policy_node.py hash + planner params, set in main()


def code_state(planner_params):
    '''Identify the code a batch ran with, since policy_node.py edits may be uncommitted.'''
    try:
        head = subprocess.run(['git', '-C', str(WS), 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True, timeout=10).stdout.strip() or 'nogit'
    except Exception:
        head = 'nogit'
    try:
        node = (WS / 'src' / 'x3_drl_policy' / 'x3_drl_policy' / 'policy_node.py').read_bytes()
        sha = hashlib.sha1(node).hexdigest()[:10]
    except OSError:
        sha = 'unknown'
    return f'{head}+policy_node:{sha}' + (f' [{planner_params}]' if planner_params else '')


# ----------------------------------------------------------------------------- job list
def build_jobs(apf_reps):
    jobs = []
    for world, path in SCENARIOS:
        for rep in range(1, apf_reps + 1):
            jobs.append(dict(model='APF', kind='apf', world=world, path=path, rep=rep))
    for ids, family in ((DELTA_IDS, 'delta'), (DIRECT_IDS, 'direct')):
        for i in ids:
            folders = sorted(p.name for p in POLICY_DIR.glob(f'TD3_{i:05d}_*') if p.is_dir())
            for name in folders:
                suffix = name.rsplit('_', 1)[1]
                kind = 'final' if suffix == '1000' else 'best'
                for world, path in SCENARIOS:
                    jobs.append(dict(model=name, kind=f'{family}_{kind}', world=world,
                                     path=path, rep=1))
    return jobs


def job_key(j):
    return (j['model'], j['world'], j['path'], str(j['rep']))


def load_done():
    done = set()
    if MANIFEST.exists():
        with open(MANIFEST) as f:
            for row in csv.DictReader(f):
                # a job only counts as done while its recorded JSON still exists
                if row['status'] == 'ok' and row['saved_file'] and (WS / row['saved_file']).is_file():
                    done.add((row['model'], row['world'], row['path'], row['rep']))
    return done


def append_manifest(row):
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    new = not MANIFEST.exists()
    with open(MANIFEST, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=MANIFEST_COLS)
        if new:
            w.writeheader()
        w.writerow(row)


# ----------------------------------------------------------------------------- processes
def bash(cmd):
    return ['bash', '-c', f'{ROS_SETUP.format(domain=ROS_DOMAIN)} && exec {cmd}']


def spawn(name, cmd, log_path):
    log = open(log_path, 'w')
    p = subprocess.Popen(bash(cmd), stdout=log, stderr=subprocess.STDOUT,
                         start_new_session=True, cwd=WS)
    procs.append((name, p))
    return p


def stop(p, grace=8.0):
    if p.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 3.0), (signal.SIGKILL, 3.0)):
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            return
        t0 = time.time()
        while time.time() - t0 < wait:
            if p.poll() is not None:
                return
            time.sleep(0.2)


def teardown():
    for name, p in reversed(procs):
        stop(p)
    procs.clear()
    # Gazebo can leave server/gui children outside the process group
    subprocess.run(['pkill', '-f', '[g]z sim'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-f', '[i]gn gazebo'], stderr=subprocess.DEVNULL)
    time.sleep(2.0)


def gz_running():
    return subprocess.run(['pgrep', '-f', '[g]z sim|[i]gn gazebo'],
                          stdout=subprocess.DEVNULL).returncode == 0


def ros_out(cmd, timeout=15):
    try:
        r = subprocess.run(bash(cmd), capture_output=True, text=True, timeout=timeout, cwd=WS)
        return r.stdout
    except subprocess.TimeoutExpired:
        return ''


def wait_sim_ready(timeout):
    '''Gazebo is up when the robot topics exist and odom is actually publishing.'''
    t0 = time.time()
    while time.time() - t0 < timeout:
        topics = ros_out('ros2 topic list').split()
        if f'{NS}/odom' in topics and f'{NS}/scan' in topics:
            if ros_out(f'timeout 10 ros2 topic echo --once {NS}/odom', timeout=15).strip():
                return True
        time.sleep(2.0)
    return False


def wait_log(path, needle, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            if needle in Path(path).read_text(errors='ignore'):
                return True
        except FileNotFoundError:
            pass
        time.sleep(1.0)
    return False


# ----------------------------------------------------------------------------- one job
def total_waypoints(world, path):
    d = json.load(open(PATHS_DIR / f'paths_{world}.json'))
    return len(d[f'{path}_{world}'])


def run_job(job, args, tag):
    model, world, path = job['model'], job['world'], job['path']
    planner_dir = RECORDED / model          # server saves under the active_planner string
    log = {n: LOG_DIR / f'{tag}_{n}.log' for n in ('gazebo', 'planner', 'server', 'client')}
    row = dict(model=model, kind=job['kind'], world=world, path=path, rep=job['rep'],
               status='launch_fail', success='', waypoints_completed='',
               total_waypoints=total_waypoints(world, path), elapsed_s='', saved_file='',
               log_file=str(log['client']), timestamp=datetime.now().isoformat(timespec='seconds'),
               code_state=CODE_STATE)
    try:
        if gz_running():
            teardown()
        spawn('gazebo', f'ros2 launch x3_description gazebo.launch.py world:={world}.sdf',
              log['gazebo'])
        if not wait_sim_ready(args.sim_timeout):
            print('    sim not ready'); return row
        time.sleep(args.settle)

        if job['kind'] == 'apf':
            spawn('planner', f'ros2 run x3_apf_planner apf_node --ros-args --remap __ns:={NS}',
                  log['planner'])
            server_choice = 'apf'
        else:
            spawn('planner', 'ros2 run x3_drl_policy policy_node --ros-args '
                  f'-p model_name:={model} {args.planner_params} --remap __ns:={NS}',
                  log['planner'])
            server_choice = 'drl'
        time.sleep(3.0)
        spawn('server', 'ros2 run x3_nav_bringup goal_sequence_server --ros-args '
              f'-p planner_choice:={server_choice} --remap __ns:={NS}', log['server'])
        if not wait_log(log['server'], 'Detected active planner', args.node_timeout):
            print('    server did not detect planner'); return row
        if f'Detected active planner: {model}' not in log['server'].read_text(errors='ignore'):
            print('    server detected an unexpected planner'); return row

        # filenames have minute resolution; never run two saves for one planner in the same minute
        planner_dir.mkdir(parents=True, exist_ok=True)
        t_start = time.time()
        client = spawn('client', 'ros2 run x3_nav_bringup goal_sequence_client --ros-args '
                       f'-p world:={world} -p path:={path} --remap __ns:={NS}', log['client'])
        try:
            client.wait(timeout=args.seq_timeout)
        except subprocess.TimeoutExpired:
            row['status'] = 'timeout'; return row
        row['elapsed_s'] = round(time.time() - t_start, 1)

        text = log['client'].read_text(errors='ignore')
        m = re.search(r'\[(SUCCESS|FAILED)\].*?Completed\s+(\d+)\s+waypoint', text)
        if not m:
            row['status'] = 'no_output'; return row
        row['success'] = int(m.group(1) == 'SUCCESS')
        row['waypoints_completed'] = int(m.group(2))

        time.sleep(1.0)
        saved = sorted((p for p in planner_dir.glob(f'{path}_{world}_*.json')
                        if p.stat().st_mtime >= t_start - 1), key=lambda p: p.stat().st_mtime)
        if not saved:
            row['status'] = 'no_output'; return row
        data = json.load(open(saved[-1]))
        row['saved_file'] = str(saved[-1].relative_to(WS))
        row['status'] = 'ok' if data.get('poses') and data.get('velocities') else 'no_output'
        return row
    finally:
        teardown()


# ----------------------------------------------------------------------------- main
def wait_next_minute_if_needed(job, last_save):
    '''Server saves as <path>_<world>_<ddmmyy>_<HHMM>.json, so two saves of the same
    planner/scenario in one minute would overwrite each other.'''
    key = (job['model'], job['world'], job['path'])
    now = datetime.now().strftime('%d%m%y_%H%M')
    if last_save.get(key) == now:
        time.sleep(61 - datetime.now().second)


def main():
    global ROS_DOMAIN, MANIFEST, CODE_STATE, SCENARIOS
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--dry-run', action='store_true', help='print the job list and exit')
    ap.add_argument('--only-model', action='append', help='model folder name or APF (repeatable)')
    ap.add_argument('--only-scenario', action='append', help='e.g. world_1/path1 (repeatable)')
    ap.add_argument('--apf-reps', type=int, default=5)
    ap.add_argument('--scenarios', nargs='+', metavar='WORLD/PATH',
                    help='replace the default six scenarios, e.g. world_5/path1')
    ap.add_argument('--skip-apf', action='store_true', help='run the DRL policies only')
    ap.add_argument('--start-index', type=int, default=0, help='skip the first N jobs of the list')
    ap.add_argument('--limit', type=int, default=0, help='run at most N jobs (0 = all)')
    ap.add_argument('--seq-timeout', type=float, default=600, help='per-sequence timeout [s]')
    ap.add_argument('--sim-timeout', type=float, default=90, help='Gazebo readiness timeout [s]')
    ap.add_argument('--node-timeout', type=float, default=45, help='planner/server timeout [s]')
    ap.add_argument('--settle', type=float, default=5, help='extra wait after sim is ready [s]')
    ap.add_argument('--ros-domain-id', default=ROS_DOMAIN, help='ROS_DOMAIN_ID for the batch')
    ap.add_argument('--planner-param', action='append', default=[], metavar='KEY:=VALUE',
                    help='extra policy_node parameter, e.g. normalized_actions:=false (repeatable)')
    ap.add_argument('--manifest', help='manifest CSV path (default: scripts/batch_runs/manifest.csv); '
                                       'use a separate one for A/B experiments')
    ap.add_argument('--rerun-failed', action='store_true',
                    help='ignore resume: rerun jobs even if already ok in the manifest')
    args = ap.parse_args()

    ROS_DOMAIN = str(args.ros_domain_id)
    args.planner_params = ' '.join(f'-p {p}' for p in args.planner_param)
    if args.manifest:
        MANIFEST = Path(args.manifest).resolve()
    CODE_STATE = code_state(args.planner_params)
    if args.scenarios:
        SCENARIOS = [tuple(x.split('/')) for x in args.scenarios]
    jobs = build_jobs(args.apf_reps)
    if args.skip_apf:
        jobs = [j for j in jobs if j['kind'] != 'apf']
    if args.only_model:
        jobs = [j for j in jobs if j['model'] in args.only_model]
    if args.only_scenario:
        wanted = {tuple(s.split('/')) for s in args.only_scenario}
        jobs = [j for j in jobs if (j['world'], j['path']) in wanted]
    jobs = jobs[args.start_index:]
    done = set() if args.rerun_failed else load_done()
    todo = [j for j in jobs if job_key(j) not in done]
    if args.limit:
        todo = todo[:args.limit]

    n_models = len({j['model'] for j in jobs if j['kind'] != 'apf'})
    print(f'{len(jobs)} jobs selected ({n_models} DRL policies + APF), '
          f'{len(jobs) - len([j for j in jobs if job_key(j) not in done])} already ok, '
          f'{len(todo)} to run')
    if args.dry_run:
        for k, j in enumerate(todo):
            print(f'{k:4d}  {j["kind"]:13s} {j["model"]:16s} {j["world"]}/{j["path"]} rep{j["rep"]}')
        return

    if gz_running():
        sys.exit('A Gazebo instance is already running. Close it first (this script kills '
                 'every "gz sim" it finds between jobs).')

    LOG_DIR.mkdir(parents=True, exist_ok=True)

    def on_signal(signum, frame):
        print('\nInterrupted: cleaning up...')
        teardown()
        sys.exit(130)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    last_save = {}
    t_batch = time.time()
    for k, job in enumerate(todo, 1):
        tag = f'{datetime.now():%Y%m%d_%H%M%S}_{job["model"]}_{job["world"]}_{job["path"]}_r{job["rep"]}'
        print(f'[{k}/{len(todo)}] {job["model"]} {job["world"]}/{job["path"]} rep{job["rep"]}', flush=True)
        wait_next_minute_if_needed(job, last_save)
        try:
            row = run_job(job, args, tag)
        except Exception as e:      # keep the batch alive on unexpected errors
            print(f'    error: {e!r}')
            row = dict(model=job['model'], kind=job['kind'], world=job['world'], path=job['path'],
                       rep=job['rep'], status='launch_fail', log_file=str(LOG_DIR / tag),
                       timestamp=datetime.now().isoformat(timespec='seconds'),
                       code_state=CODE_STATE)
            teardown()
        if row['status'] == 'launch_fail':
            print('    launch failed, retrying once')
            row = run_job(job, args, tag + '_retry')
        append_manifest(row)
        last_save[(job['model'], job['world'], job['path'])] = datetime.now().strftime('%d%m%y_%H%M')
        print(f'    -> {row["status"]} success={row.get("success")} '
              f'wp={row.get("waypoints_completed")}/{row.get("total_waypoints")} '
              f'({(time.time() - t_batch) / 60:.1f} min elapsed)', flush=True)


if __name__ == '__main__':
    main()
