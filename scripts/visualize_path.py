import os, json
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

CURRENT_DIR         = os.path.dirname(os.path.abspath(__file__))
SAVED_PATHS_DIR     = os.path.join(CURRENT_DIR, '..', 'scripts', 'recorded_paths')
DEFAULT_PATHS_PATH   = os.path.join(CURRENT_DIR, '..', 'src', 'x3_nav_bringup', 'paths', 'paths_world_1.json')

WORLD = 'world_1'
SDF_PATH = os.path.join(CURRENT_DIR, '..', 'src', 'x3_description', 'worlds', f'{WORLD}.sdf')

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

odom_init = {'x': -3.0, 'y': -3.0}

def main():
    fig, ax = plt.subplots(figsize=(10,10))

    # --- Visualize the path ---
    for path in path_list:
        PATHS_PATH = os.path.join(SAVED_PATHS_DIR, f'{path}.json')
        planner_name = path.split('/')[0].split('_')[:2]
        if len(planner_name) > 1:
            if int(planner_name[1]) >= 383:
                planner_name[1] = 'delta'
            else:
                planner_name[1] = 'direct'
        planner_name = '_'.join(planner_name)

        with open(PATHS_PATH, 'r') as f:
            data = json.load(f)
            
        poses = data["poses"]
        x   = [pose.get('x', float('nan')) + odom_init['x'] for pose in poses]
        y   = [pose.get('y', float('nan')) + odom_init['y'] for pose in poses]
        yaw = [pose.get('yaw', float('nan')) for pose in poses]

        elapsed_time = data["elapsed_time"]
        total_distance = data["total_distance"]

        line = ax.plot(x, y, '--', alpha=0.7, lw=2, label=f"{planner_name} | {elapsed_time: 6.2f}s | {total_distance: 6.2f}m")

        ARROW_EVERY_N = 250
        indices = range(0, len(x), ARROW_EVERY_N)

        ax.quiver(
            [x[i] for i in indices], 
            [y[i] for i in indices], 
            [np.cos(yaw[i]) for i in indices],
            [np.sin(yaw[i]) for i in indices],
            color=line[0].get_color(),
            scale=50,
            width=0.004,
            headwidth=4, headlength=5,
            alpha=0.8
        )

    # --- TODO: Visualize the obstacles ---
    obstacles = parse_sdf_obstacles(SDF_PATH)

    for obs in obstacles:
        if obs["type"] == 'box':
            rect = patches.Rectangle(
                xy=(obs['x'] - obs['sx']/2, obs['y'] - obs['sy']/2),
                width=obs['sx'], height=obs['sy'],
                color='red', alpha=0.5, linewidth=2
            )
            ax.add_patch(rect)

        elif obs["type"] == 'cylinder':
            circle = patches.Circle(
                xy = (obs['x'], obs['y']),
                radius = obs['radius'],
                color='red', alpha=0.5, linewidth=2
            )
            ax.add_patch(circle)

    # --- Visualize the goals ---       
    path_name = path_list[0].split('/')[1].split('_')[:2]
    path_name = "_".join(path_name)
    print(f'Comparing the policy performance on {path_name}')
    with open(DEFAULT_PATHS_PATH, 'r') as f:
        AVAILABLE_PATHS = json.load(f)
    goal_x = [goal[0] + odom_init['x'] for goal in AVAILABLE_PATHS[path_name]]
    goal_y = [goal[1] + odom_init['y'] for goal in AVAILABLE_PATHS[path_name]]
    goal_tolerance = 0.30        # radius (m)
    goal_scatter_size = goal_tolerance * 2 * 100 / (2.54 / 72)     # one scatter size = 1/72 of an inch
    ax.scatter(goal_x, goal_y, linewidths=1, edgecolors="#1C7826", s=goal_scatter_size, c="#3EA047", alpha=0.5)

    axis_limits = (-4.0, 4.0)
    ax.set_xlim(axis_limits); ax.set_ylim(axis_limits)
    ax.minorticks_on() 
    ax.tick_params(axis='both', which='minor', direction='in', labelsize=14)
    ax.tick_params(axis='both', which='major', direction='in', labelsize=14)
    ax.grid(False)
    ax.legend(loc="upper left", fontsize=14)
    plt.show()
        
import xml.etree.ElementTree as ET
def parse_sdf_obstacles(sdf_path: str) -> list:
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    obstacles = []

    for elem in root.iter('model'):
        # print(f"Tag: {elem.tag}, Attributes: {elem.attrib}")   # debug print to show the main tags under <sdf>
        
        name = elem.get('name','')
        is_box = name.startswith('box')
        is_cylinder = name.startswith('cylinder')

        if not (is_box or is_cylinder):
            continue

        pose_elem = elem.find('pose')
        pose_vals = [float(v) for v in pose_elem.text.strip().split()]
        x, y = pose_vals[0], pose_vals[1]

        if is_box:
            size_elem = elem.find('.//collision/geometry/box/size')
            size_x, size_y, _ = [float(v) for v in size_elem.text.strip().split()]
            obstacles.append({
                "name": name,
                "type": "box",
                "x": x,
                "y": y,
                "sx": size_x,
                "sy": size_y
            })
        elif is_cylinder:
            radius_elem = elem.find('.//collision/geometry/cylinder/radius')
            obstacles.append({
                "name": name,
                "type": "cylinder",
                "x": x,
                "y": y,
                "radius": float(radius_elem.text.strip())
            })

    return obstacles


if __name__ == "__main__":
    main()