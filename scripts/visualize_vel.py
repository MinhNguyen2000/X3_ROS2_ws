import os, json
import matplotlib.pyplot as plt
import numpy as np

CURRENT_DIR         = os.path.dirname(os.path.abspath(__file__))
SAVED_PATHS_DIR     = os.path.join(CURRENT_DIR, '..', 'scripts', 'recorded_paths')

# --- Path 1 World 1
path_list = [
    'APF/path1_world1_300726_0018',
    'TD3_00380_1000/path1_world1_300726_1029',
    # 'TD3_00384_500/path1_world1_300726_0031',
    'TD3_00384_1000/path1_world1_300726_1101',
]

# --- Path 2 World 1
path_list = [ 
    'APF/path2_world1_300726_1054', 
    'TD3_00380_1000/path2_world1_300726_1033',
    # 'TD3_00384_500/path2_world1_300726_1025',
    'TD3_00384_1000/path2_world1_300726_1107',
]

# --- Path 3 World 1
path_list = [
    'APF/path3_world1_300726_1055', 
    'TD3_00380_1000/path3_world1_300726_1035',
    # 'TD3_00384_500/path3_world1_300726_1050',
    'TD3_00384_1000/path3_world1_300726_1108',
]

# --- Path 4 World 1
path_list = [
    'APF/path4_world1_300726_1056', 
    'TD3_00380_1000/path4_world1_300726_1039',
    # 'TD3_00384_500/path4_world1_300726_1051',
    'TD3_00384_1000/path4_world1_300726_1111',
]

# TODO - change these according to the runs
MAX_LIN_VEL = 0.5
MAX_ANGULAR_VEL = 1.0

def load_run(path: str) -> dict:
    '''
    Loads a single recorded run's JSON (poses + velocities + metadata)
    '''
    run_path = os.path.join(SAVED_PATHS_DIR, f'{path}.json')
    with open(run_path, 'r') as f:
        data = json.load(f)

    return data

def compute_accel(velocities: list) -> dict:
    '''
    Derive acceleration via finite differencing from the raw {t, vx, vyaw} samples
    '''

    t    = np.array([v['t'] for v in velocities], dtype=np.float64)
    vx   = np.array([v['vx'] for v in velocities], dtype = np.float64)
    vyaw = np.array([v['vyaw'] for v in velocities], dtype = np.float64)

    dt = np.diff(t)
    dt = np.where(dt <= 0, np.nan, dt)

    a_lin = np.diff(vx) / dt
    a_ang = np.diff(vyaw) / dt

    # shift the accel timestamps for the derivative arrays to sit at the midpoint of each interval
    t_accel = t[:-1] + dt / 2.0

    # ===== STATS =====
    oscillations = np.sum(np.abs(np.diff(np.sign(vyaw)))) / 2   # total number of vyaw diff sign flips
    duration = t[-1] - t[0] if len(t) > 1 else np.nan

    stats = {
        "mean_abs_a_lin":   float(np.nanmean(np.abs(a_lin))),
        "rms_a_lin":        float(np.sqrt(np.nanmean(a_lin**2))),
        "mean_abs_a_ang":   float(np.nanmean(np.abs(a_ang))),
        "rms_a_ang":        float(np.sqrt(np.nanmean(a_ang**2))),
        "mean_vx":          float(np.mean(vx)),
        "vx_utilization":   float(np.mean(vx) / MAX_LIN_VEL),
        "mean_signed_vyaw":  float(np.mean(vyaw)),
        "oscillations_per_s": float(oscillations / duration) if duration else np.nan,
        "n_samples":        len(velocities),
        "sample_rate_hz":   float(1.0 / np.nanmean(dt)) if len(dt) else np.nan,
    }

    return {
        "t": t,
        "vx": vx,
        "vyaw": vyaw,
        "t_accel": t_accel,
        "a_lin": a_lin, 
        "a_ang": a_ang,
        "stats": stats,
    }

def align_to_grid(t_a: np.ndarray, v_a: np.ndarray, t_b: np.ndarray, v_b: np.ndarray, dt: float = 0.02):
    '''
    Interpolates two irregularly-sampled, independently-timestamped series
    (e.g. commanded cmd_vel and executed wheel_odom velocity) onto a shared
    uniform time grid so they can be directly compared/differenced.

    Bounds the grid to the overlap of both series' time ranges - np.interp
    clamps (holds flat) outside its input range rather than extrapolating,
    so anything outside the overlap would otherwise show up as a spurious
    flat segment rather than real data.

    Requires t_a/t_b to be sorted ascending (true by construction, since
    samples are appended in arrival order) - re-sorts defensively in case a
    QoS/threading edge case ever delivers a message out of order.
    '''
    order_a = np.argsort(t_a)
    order_b = np.argsort(t_b)
    t_a, v_a = t_a[order_a], v_a[order_a]
    t_b, v_b = t_b[order_b], v_b[order_b]

    t_start = max(t_a[0], t_b[0])
    t_end   = min(t_a[-1], t_b[-1])

    if t_end <= t_start:
        return np.array([]), np.array([]), np.array([])

    t_grid = np.arange(t_start, t_end, dt)
    v_a_i  = np.interp(t_grid, t_a, v_a)
    v_b_i  = np.interp(t_grid, t_b, v_b)

    return t_grid, v_a_i, v_b_i

def compare_cmd_vs_actual(velocities: list, actual_velocities: list, dt: float = 0.02) -> dict:
    '''
    Aligns commanded (cmd_vel) and executed (wheel_odom) velocity samples
    onto a shared time grid and computes tracking-error stats per channel.
    '''
    t_cmd    = np.array([v['t'] for v in velocities], dtype=np.float64)
    vx_cmd   = np.array([v['vx'] for v in velocities], dtype=np.float64)
    vyaw_cmd = np.array([v['vyaw'] for v in velocities], dtype=np.float64)

    t_act    = np.array([v['t'] for v in actual_velocities], dtype=np.float64)
    vx_act   = np.array([v['vx'] for v in actual_velocities], dtype=np.float64)
    vyaw_act = np.array([v['vyaw'] for v in actual_velocities], dtype=np.float64)

    t_grid, vx_cmd_i, vx_act_i     = align_to_grid(t_cmd, vx_cmd, t_act, vx_act, dt)
    _,      vyaw_cmd_i, vyaw_act_i = align_to_grid(t_cmd, vyaw_cmd, t_act, vyaw_act, dt)

    if len(t_grid) == 0:
        return None

    err_vx   = vx_act_i - vx_cmd_i
    err_vyaw = vyaw_act_i - vyaw_cmd_i

    stats = {
        "rms_err_vx":   float(np.sqrt(np.mean(err_vx**2))),
        "rms_err_vyaw": float(np.sqrt(np.mean(err_vyaw**2))),
        "mean_abs_err_vx":   float(np.mean(np.abs(err_vx))),
        "mean_abs_err_vyaw": float(np.mean(np.abs(err_vyaw))),
    }

    return {
        "t_grid": t_grid,
        "vx_cmd": vx_cmd_i, "vx_act": vx_act_i,
        "vyaw_cmd": vyaw_cmd_i, "vyaw_act": vyaw_act_i,
        "err_vx": err_vx, "err_vyaw": err_vyaw,
        "stats": stats,
    }


def main():
    fig, axes = plt.subplots(4,1, figsize=(10,12), sharex=True)
    ax_vx, ax_vyaw, ax_alin, ax_ayaw = axes

    print(f"{'planner':<16}{'mean|a_lin|':>12}{'rms a_lin':>12}{'mean|a_ang|':>12}"
          f"{'rms a_ang':>12}{'vx util':>10}{'mean vyaw':>12}{'osc/s':>10}{'rate(Hz)':>10}")

    tracking_results = {}

    for path in path_list:
        data = load_run(path)
        planner_name = path.split('/')[0].split('_')[:2]
        if len(planner_name) > 1:
            if int(planner_name[1]) >= 383:
                planner_name[1] = 'delta'
            else:
                planner_name[1] = 'direct'
        planner_name = '_'.join(planner_name)

        velocities = data.get("velocities")
        if not velocities:
            print(f"No velocity data found for {path}")
            continue

        result = compute_accel(velocities)
        s = result["stats"]

        print(f"{planner_name:<16}{s['mean_abs_a_lin']:>12.3f}{s['rms_a_lin']:>12.3f}"
              f"{s['mean_abs_a_ang']:>12.3f}{s['rms_a_ang']:>12.3f}{s['vx_utilization']:>10.2f}"
              f"{s['mean_signed_vyaw']:>12.3f}{s['oscillations_per_s']:>10.2f}{s['sample_rate_hz']:>10.1f}")

        label = f"{planner_name}" # | {s['n_samples']} samples @ {s['sample_rate_hz']:.0f}Hz"

        ax_vx.plot(result["t"], result["vx"], lw=1.5, alpha=0.85, label=label)
        ax_vyaw.plot(result["t"], result["vyaw"], lw=1.5, alpha=0.85, label=label)
        ax_alin.plot(result["t_accel"], result["a_lin"], lw=1.0, alpha=0.75, label=label)
        ax_ayaw.plot(result["t_accel"], result["a_ang"], lw=1.0, alpha=0.75, label=label)

        # --- Commanded vs actual tracking comparison ---
        actual_velocities = data.get("actual_velocities")
        if actual_velocities:
            cmp = compare_cmd_vs_actual(velocities, actual_velocities)
            if cmp is not None:
                cs = cmp["stats"]
                print(f"  -> tracking error vs wheel_odom: "
                      f"rms_err_vx={cs['rms_err_vx']:.3f}  mean|err_vx|={cs['mean_abs_err_vx']:.3f}  "
                      f"rms_err_vyaw={cs['rms_err_vyaw']:.3f}  mean|err_vyaw|={cs['mean_abs_err_vyaw']:.3f}")
                tracking_results[planner_name] = cmp
            else:
                print(f"  -> no overlapping time range between cmd_vel and wheel_odom for {path}")
        else:
            print(f"  -> no actual_velocities found for {path} (re-run with updated goal_sequence_server)")

    ax_vx.set_ylabel("vx (m/s)")
    ax_vx.axhline(0, color='k', lw=0.5, alpha=0.3)
    ax_vx.legend(loc='upper right', fontsize=12)
    ax_vx.set_title("Commanded linear velocity", fontsize=12)
 
    ax_vyaw.set_ylabel("vyaw (rad/s)")
    ax_vyaw.axhline(0, color='k', lw=0.5, alpha=0.3)
    ax_vyaw.set_title("Commanded angular velocity", fontsize=12)
 
    ax_alin.set_ylabel("a_lin (m/s²)")
    ax_alin.axhline(0, color='k', lw=0.5, alpha=0.3)
    ax_alin.set_title("Linear acceleration", fontsize=12)
 
    ax_ayaw.set_ylabel("a_ang (rad/s²)")
    ax_ayaw.axhline(0, color='k', lw=0.5, alpha=0.3)
    ax_ayaw.set_title("Angular acceleration", fontsize=12)
    ax_ayaw.set_xlabel("time (s)")
 
    for ax in axes:
        ax.minorticks_on()
        ax.tick_params(axis='both', which='minor', direction='in')
        ax.tick_params(axis='both', which='major', direction='in')
        ax.grid(False)
 
    fig.tight_layout()

    # --- Second figure: commanded vs actual, one column per planner ---
    if tracking_results:
        n_planners = len(tracking_results)
        fig2, axes2 = plt.subplots(3, n_planners, figsize=(6 * n_planners, 9), sharex='col', squeeze=False)

        for col, (planner_name, cmp) in enumerate(tracking_results.items()):
            ax_vx_cmp, ax_vyaw_cmp, ax_err = axes2[0, col], axes2[1, col], axes2[2, col]

            ax_vx_cmp.plot(cmp["t_grid"], cmp["vx_cmd"], lw=1.3, alpha=0.75, label="commanded")
            ax_vx_cmp.plot(cmp["t_grid"], cmp["vx_act"], lw=1.3, alpha=0.75, label="actual")
            ax_vx_cmp.set_title(f"{planner_name}: commanded vs actual vx", fontsize=14)
            ax_vx_cmp.set_ylabel("vx (m/s)")
            ax_vx_cmp.legend(loc='upper right', fontsize=8)

            ax_vyaw_cmp.plot(cmp["t_grid"], cmp["vyaw_cmd"], lw=1.3, alpha=0.75, label="commanded")
            ax_vyaw_cmp.plot(cmp["t_grid"], cmp["vyaw_act"], lw=1.3, alpha=0.75, label="actual")
            ax_vyaw_cmp.set_title(fr"{planner_name}: commanded vs actual $\omega$", fontsize=14)
            ax_vyaw_cmp.set_ylabel("vyaw (rad/s)")

            ax_err.plot(cmp["t_grid"], cmp["err_vx"], lw=1.0, alpha=0.8, label="err vx")
            ax_err.plot(cmp["t_grid"], cmp["err_vyaw"], lw=1.0, alpha=0.8, label=f"err $\omega$")
            ax_err.axhline(0, color='k', lw=0.5, alpha=0.3)
            ax_err.set_title(f"{planner_name} - tracking error (actual - commanded)", fontsize=14)
            ax_err.set_ylabel("error")
            ax_err.set_xlabel("time (s)")
            ax_err.legend(loc='upper right', fontsize=8)

            for ax in (ax_vx_cmp, ax_vyaw_cmp, ax_err):
                ax.minorticks_on()
                ax.tick_params(axis='both', which='minor', direction='in')
                ax.tick_params(axis='both', which='major', direction='in')
                ax.grid(False)

        fig2.tight_layout()

    plt.show()

if __name__ == "__main__":
    main()