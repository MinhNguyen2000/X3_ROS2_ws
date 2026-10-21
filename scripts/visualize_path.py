import os, json
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

CURRENT_DIR         = os.path.dirname(os.path.abspath(__file__))

PATH = 'path_2'
WORLD = 'world_lab'
SAVED_PATHS_DIR     = os.path.join(CURRENT_DIR, '..', 'scripts', 'recorded_paths')
DEFAULT_PATHS_PATH  = os.path.join(CURRENT_DIR, '..', 'src', 'x3_nav_bringup', 'paths', f'paths_{WORLD}.json')
SDF_PATH            = os.path.join(CURRENT_DIR, '..', 'src', 'x3_description', 'worlds', f'{WORLD}.sdf')

PATH_LISTS = {
    # ===== WORLD 1 =====
    ('path_1', 'world_1'): [
        'APF/path1_world_1_300726_0018',
        # 'TD3_00380_1000/path1_world_1_300726_1029',
        # 'TD3_00384_500/path1_world_1_300726_0031',
        # 'TD3_00384_1000/path1_world_1_300726_1101',
        # 'TD3_00385_1000/path1_world_1_260826_2308'

        # # TD3_delta (5 seeds)
        # 'TD3_00386_1000/path1_world_1_300826_1152',
        # 'TD3_00387_1000/path1_world_1_300826_1218',
        # 'TD3_00388_1000/path1_world_1_300826_1246',
        # 'TD3_00389_1000/path1_world_1_300826_1328',
        # 'TD3_00390_1000/path1_world_1_300826_1336',

        'TD3_00400_700/path1_world_1_110926_1830',
        'TD3_00390_800/path1_world_1_110926_1820',
    ],
    ('path_2', 'world_1'): [
        'APF/path2_world_1_300726_1054',
        # 'TD3_00380_1000/path2_world_1_300726_1033',
        # 'TD3_00384_500/path2_world_1_300726_1025',
        # 'TD3_00384_1000/path2_world_1_300726_1107',

        # # TD3_delta (5 seeds)
        # 'TD3_00386_1000/path2_world_1_300826_1201',
        # 'TD3_00387_1000/path2_world_1_300826_1222',
        # 'TD3_00388_1000/path2_world_1_300826_1247',
        # 'TD3_00389_1000/path2_world_1_300826_1329',
        # 'TD3_00390_1000/path2_world_1_300826_1338',

        'TD3_00400_700/path2_world_1_110926_1832',
        'TD3_00390_800/path2_world_1_110926_1822',
    ],
    ('path_3', 'world_1'): [
        'APF/path3_world_1_300726_1055',
        # 'TD3_00380_1000/path3_world_1_300726_1035',
        # 'TD3_00384_500/path3_world_1_300726_1050',
        # 'TD3_00384_1000/path3_world_1_300726_1108',

        # # TD3_delta (5 seeds)
        # 'TD3_00386_1000/path3_world_1_300826_1202',
        # 'TD3_00387_1000/path3_world_1_300826_1226',
        # 'TD3_00388_1000/path3_world_1_300826_1253',
        # 'TD3_00389_1000/path3_world_1_300826_1329',
        # 'TD3_00390_1000/path3_world_1_300826_1339',

        'TD3_00400_700/path3_world_1_110926_1836',
        'TD3_00390_800/path3_world_1_110926_1824',
    ],
    ('path_4', 'world_1'): [
        'APF/path4_world_1_300726_1056',
        # 'TD3_00380_1000/path4_world_1_300726_1039',
        # 'TD3_00384_500/path4_world_1_300726_1051',
        # 'TD3_00384_1000/path4_world_1_300726_1111',

        # # TD3_delta (5 seeds)
        # 'TD3_00386_1000/path4_world_1_300826_1203',
        # 'TD3_00387_1000/path4_world_1_300826_1228',
        # 'TD3_00388_1000/path4_world_1_300826_1257',
        # 'TD3_00389_1000/path4_world_1_300826_1330',
        # 'TD3_00390_1000/path4_world_1_300826_1340',

        'TD3_00400_700/path4_world_1_110926_1837',
        'TD3_00390_800/path4_world_1_110926_1824',
    ],
    # ===== WORLD 2 =====
    ('path_1', 'world_2'): [
        'APF/path1_world_2_130826_1201',
        # 'TD3_00380_1000/path1_world_2_120826_1655',
        # 'TD3_00384_1000/path1_world_2_120826_1650',

        # # TD3_delta (5 seeds)
        # 'TD3_00386_1000/path1_world_2_300826_1204',
        # 'TD3_00387_1000/path1_world_2_300826_1229',
        # 'TD3_00388_1000/path1_world_2_300826_1323',
        # 'TD3_00389_1000/path1_world_2_300826_1332',
        # 'TD3_00390_1000/path1_world_2_300826_1341',

        'TD3_00400_700/path1_world_2_110926_1840',
        'TD3_00390_800/path1_world_2_110926_1842',
    ],
    # ===== WORLD 3 =====
    ('path_1', 'world_3'): [
        'APF/path1_world_3_130826_1204',
        # 'TD3_00380_1000/path1_world_3_120826_1659',
        # 'TD3_00384_1000/path1_world_3_120826_1700',

        # # TD3_delta (5 seeds)
        # 'TD3_00386_1000/path1_world_3_300826_1207',
        # 'TD3_00387_1000/path1_world_3_300826_1230',
        # 'TD3_00388_1000/path1_world_3_300826_1324',
        # 'TD3_00389_1000/path1_world_3_300826_1333',
        # 'TD3_00390_1000/path1_world_3_300826_1342',

        'TD3_00400_700/path1_world_3_110926_1849',
        'TD3_00390_800/path1_world_3_110926_1845',
    ],
    # ===== WORLD 4 =====
    ('path_1', 'world_4'): [
        'APF/path1_world_4_130826_1209',
        # 'TD3_00380_1000/path1_world_4_130826_1141',
        # 'TD3_00384_1000/path1_world_4_130826_1137',

        # # TD3_delta (5 seeds)
        # 'TD3_00386_1000/path1_world_4_300826_1210',
        # 'TD3_00387_1000/path1_world_4_300826_1233',
        # 'TD3_00388_1000/path1_world_4_300826_1326',
        # 'TD3_00389_1000/path1_world_4_300826_1334',
        # 'TD3_00390_1000/path1_world_4_300826_1344',
    ],
    # ===== WORLD 5 =====
    ('path_1', 'world_5'): [
        'APF/path1_world_5_130826_1211',
        # 'TD3_00380_1000/path1_world_5_130826_1215',
        # 'TD3_00384_1000/path1_world_5_130826_1155',

        # # TD3_delta (5 seeds)
        # 'TD3_00386_1000/path1_world_5_300826_1213',
        # 'TD3_00387_1000/path1_world_5_300826_1243',
        # 'TD3_00388_1000/path1_world_5_300826_1327',
        # 'TD3_00389_1000/path1_world_5_300826_1334',
        # 'TD3_00390_1000/path1_world_5_300826_1345',

        'TD3_00400_700/path1_world_5_110926_1850',
        'TD3_00390_800/path1_world_5_110926_1846',
    ],

    # ===== WORLD LAB =====
    ('path_1', 'world_lab'): [
        # # vyaw = 0.3
        # # 'TD3_00386_1000/path1_world_lab_131026_2242',
        # # path1_world_lab_131026_2306',
        # 'TD3_00395_1000/path1_world_lab_131026_2311',
        # # 'TD3_00395_900/path1_world_lab_131026_2312',
        # 'TD3_00404_500/path1_world_lab_141026_0951',

        # # vyaw = 0.4
        # # 'TD3_00386_1000/path1_world_lab_131026_2242',
        # # 'TD3_00386_1000/path1_world_lab_131026_2314',
        # # 'TD3_00387_500/path1_world_lab_141026_0806',
        # # 'TD3_00394_1000/path1_world_lab_141026_0913',
        # 'TD3_00395_1000/path1_world_lab_141026_0916',
        # # 'TD3_00395_900/path1_world_lab_141026_0918',

        # 'TD3_00400_700/path1_world_lab_141026_1621',
        # # 'TD3_00404_500/path1_world_lab_141026_1719',

        # Batch of 10 TD3_delta runs (2026-10-15)
        'TD3_00395_1000/path1_world_lab_151026_2036',
        'TD3_00395_1000/path1_world_lab_151026_2037',
        'TD3_00395_1000/path1_world_lab_151026_2039',
        'TD3_00395_1000/path1_world_lab_151026_2040',
        'TD3_00395_1000/path1_world_lab_151026_2041',
        'TD3_00395_1000/path1_world_lab_151026_2042',
        'TD3_00395_1000/path1_world_lab_151026_2044',
        'TD3_00395_1000/path1_world_lab_151026_2045',
        'TD3_00395_1000/path1_world_lab_151026_2046',
        'TD3_00395_1000/path1_world_lab_151026_2048',

        # Batch of 10 TD3_direct runs (2026-10-15)
        'TD3_00404_500/path1_world_lab_151026_2050',
        'TD3_00404_500/path1_world_lab_151026_2052',
        'TD3_00404_500/path1_world_lab_151026_2054',
        'TD3_00404_500/path1_world_lab_151026_2055',
        'TD3_00404_500/path1_world_lab_151026_2057',
        'TD3_00404_500/path1_world_lab_151026_2058',
        'TD3_00404_500/path1_world_lab_151026_2100',
        'TD3_00404_500/path1_world_lab_151026_2102',
        'TD3_00404_500/path1_world_lab_151026_2110',
        'TD3_00404_500/path1_world_lab_151026_2111',
    ],

    ('path_2', 'world_lab'): [
        # # vyaw = 0.3
        # # 'TD3_00386_1000/path2_world_lab_151026_1149',
        # 'TD3_00388_1000/path2_world_lab_151026_1214',
        # # 'TD3_00400_1000/path2_world_lab_151026_1506',
        # 'TD3_00405_1000/path2_world_lab_151026_1527',

        # # # vyaw = 0.4
        # # 'TD3_00387_500/path2_world_lab_151026_1426',
        # # 'TD3_00389_1000/path2_world_lab_151026_1431',
        # # 'TD3_00390_800/path2_world_lab_151026_1434',
        # 'TD3_00393_1000/path2_world_lab_151026_1440',
        # # 'TD3_00394_1000/path2_world_lab_151026_1443',
        # # 'TD3_00395_1000/path2_world_lab_151026_1446',
        # 'TD3_00400_1000/path2_world_lab_151026_1543',

        # Batch of 10 TD3_delta runs (2026-10-15)
        'TD3_00388_1000/path2_world_lab_151026_2118',
        'TD3_00388_1000/path2_world_lab_151026_2119',
        'TD3_00388_1000/path2_world_lab_151026_2120',
        'TD3_00388_1000/path2_world_lab_151026_2121',
        'TD3_00388_1000/path2_world_lab_151026_2122',
        'TD3_00388_1000/path2_world_lab_151026_2124',
        'TD3_00388_1000/path2_world_lab_151026_2125',
        'TD3_00388_1000/path2_world_lab_151026_2128',
        'TD3_00388_1000/path2_world_lab_151026_2129',
        'TD3_00388_1000/path2_world_lab_151026_2131',

        # Batch of 10 TD3_delta runs (2026-10-15)
        'TD3_00405_1000/path2_world_lab_151026_2144',
        'TD3_00405_1000/path2_world_lab_151026_2145',
        'TD3_00405_1000/path2_world_lab_151026_2146',
        'TD3_00405_1000/path2_world_lab_151026_2147',
        'TD3_00405_1000/path2_world_lab_151026_2148',
        'TD3_00405_1000/path2_world_lab_151026_2149',
        'TD3_00405_1000/path2_world_lab_151026_2154',
        'TD3_00405_1000/path2_world_lab_151026_2156',
        'TD3_00405_1000/path2_world_lab_151026_2157',
        'TD3_00405_1000/path2_world_lab_151026_2158',
        
    ]
}

path_list = PATH_LISTS.get((PATH, WORLD), [])

# odom_init = {'x': -3.0, 'y': -3.0}
odom_init = {'x': -1.6775, 'y': -0.7625}


def rotated_box_corners(cx: float, cy: float, sx: float, sy: float, yaw: float) -> np.ndarray:
    '''
    Compute the 4 corners of a box centered at (cx, cy) with size (sx, sy),
    rotated by yaw (radians, CCW positive) about its own center.
    Similar to the function used in visualize_batch_trials.py script
    '''
    hx, hy = sx / 2.0, sy / 2.0
    local_corners = np.array([
        [-hx, -hy],
        [ hx, -hy],
        [ hx,  hy],
        [-hx,  hy],
    ])
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s],
                  [s,  c]])
    world_corners = local_corners @ R.T + np.array([cx, cy])
    return world_corners

def main():
    fig, ax = plt.subplots(figsize=(12,12))
 
    # --- Visualize the path ---
    for path in path_list:
        PATHS_PATH = os.path.join(SAVED_PATHS_DIR, f'{path}.json')
        planner_name = path.split('/')[0].split('_')[:2]
        if len(planner_name) > 1:
            if int(planner_name[1]) <= 395:
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
 
        line = ax.plot(x, y, '--', alpha=0.7, lw=1.5, label=f"{planner_name} | {elapsed_time: 6.2f}s | {total_distance: 6.2f}m", zorder=1)
 
        ARROW_EVERY_N = 200
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
 
    obstacles = parse_sdf_obstacles(SDF_PATH)
 
    for obs in obstacles:
        if obs["type"] == 'box':
            corners = rotated_box_corners(
                obs['x'], obs['y'], obs['sx'], obs['sy'], obs.get('yaw', 0.0)
            )
            poly = patches.Polygon(
                corners, closed=True,
                color='red', alpha=0.5, linewidth=2
            )
            ax.add_patch(poly)
 
        elif obs["type"] == 'cylinder':
            circle = patches.Circle(
                xy = (obs['x'], obs['y']),
                radius = obs['radius'],
                color='red', alpha=0.5, linewidth=2
            )
            ax.add_patch(circle)
 
    # --- Visualize the goals ---       
    path_name = path_list[0].split('/')[1].split('_')[:3]
    path_name = "_".join(path_name)
    print(f'Comparing the policy performance on {path_name}')
    with open(DEFAULT_PATHS_PATH, 'r') as f:
        AVAILABLE_PATHS = json.load(f)
    goal_x = [goal[0] + odom_init['x'] for goal in AVAILABLE_PATHS[path_name]]
    goal_y = [goal[1] + odom_init['y'] for goal in AVAILABLE_PATHS[path_name]]
    goal_tolerance = 0.30        # radius (m)
    goal_scatter_size = goal_tolerance * 8.0 * 100 / (2.54 / 72)     # one scatter size = 1/72 of an inch
    ax.scatter(goal_x, goal_y, linewidths=1, linestyle='--', edgecolors="#1C7826", s=goal_scatter_size, c="#3EA047", alpha=0.25, zorder=10)
 
    # --- Number each goal so order stays unambiguous even where paths cross a repeated goal ---
    for i, (gx, gy) in enumerate(zip(goal_x, goal_y)):
        ax.annotate(
            str(i + 1),
            xy=(gx, gy), xycoords='data',
            ha='center', va='center',
            fontsize=14, fontweight='bold', color='black',
            zorder=3
        )

    arena_w = 9 * 0.305
    arena_l = 15 * 0.305
    x_axis_lim = (-arena_l/2, arena_l/2)
    y_axis_lim = (-arena_w/2, arena_w/2)
    # x_axis_lim = (-4.0, 4.0)
    # y_axis_lim = (-4.0, 4.0)
    ax.set_xlim(x_axis_lim); 
    ax.set_ylim(y_axis_lim)
    ax.minorticks_on() 
    ax.tick_params(axis='both', which='minor', direction='in', labelsize=14)
    ax.tick_params(axis='both', which='major', direction='in', labelsize=14)
    ax.grid(False)
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_aspect('equal')
    ax.legend(loc='upper right', bbox_to_anchor=(0.2, 1.0), fontsize=14)
    plt.show()
        
import xml.etree.ElementTree as ET
def parse_sdf_obstacles(sdf_path: str) -> list:
    """
    Parse obstacles from SDF file.
    Returns list of dicts with 'type', 'x', 'y', and geometry info.
    Similar to the function used in visualize_path.py script
    """
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
        # pose is (x, y, z, roll, pitch, yaw) in SDF; fall back to 0 if a
        # shorter pose string is ever encountered
        yaw = pose_vals[5] if len(pose_vals) >= 6 else 0.0
 
        if is_box:
            size_elem = elem.find('.//collision/geometry/box/size')
            size_x, size_y, _ = [float(v) for v in size_elem.text.strip().split()]
            obstacles.append({
                "name": name,
                "type": "box",
                "x": x,
                "y": y,
                "sx": size_x,
                "sy": size_y,
                "yaw": yaw
            })
        elif is_cylinder:
            radius_elem = elem.find('.//collision/geometry/cylinder/radius')
            obstacles.append({
                "name": name,
                "type": "cylinder",
                "x": x,
                "y": y,
                "radius": float(radius_elem.text.strip()),
                "yaw": yaw   # unused for rendering (circle is rotationally symmetric)
            })
 
    return obstacles
 
 
if __name__ == "__main__":
    main()