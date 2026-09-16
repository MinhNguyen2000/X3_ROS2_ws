#!/usr/bin/env python3
""" 
Visualize a batch of goal-goal navigation results

Usage:
    python3 visualize_batch_trials.py \
        --csv <csv_file_name> \
        --timeseries-dir <path_to_time_series_dir>

Example:
    python3 visualize_batch_trials.py \
        --csv world_1_APF_20260825_194324
"""

import os
import re
import json
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as mlines

from ament_index_python.packages import get_package_share_directory

# TODO - a mechanism to store the initial odom offset of the agent
odom_init = {'x': -3.0, 'y': -3.0}

def parse_arguments():
    parser = argparse.ArgumentParser(description="Visualize batch of navigation trial results")
    parser.add_argument('--csv', required=True, help='Name of experiment-level CSV file inside trial_results directory')
    # parser.add_argument('--timeseries-dir', required=True, help='Path to timeseries data directory')
    return parser.parse_args()

def rotated_box_corners(cx: float, cy: float, sx: float, sy: float, yaw: float) -> np.ndarray:
    """
    Compute the 4 corners of a box centered at (cx, cy) with size (sx, sy),
    rotated by yaw (radians, CCW positive) about its own center.
    Similar to the function used in visualize_path.py script
    """
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

def parse_csv_filename(csv_filename):
    """Extract world name and planner type from CSV filename"""
    # Parse world SDF file path and planner name using regex
    # Following the naming schema of "world_<world_number>_<planner>_YYYYMMDD_HHMMSS.csv"
    csv_name_pattern = r'(world_\d+)_(.+)_(\d{8})_(\d{6})$'
    match = re.match(csv_name_pattern, csv_filename)

    if match:
        world_name      = match.group(1)    # world_<world_number>
        planner_type    = match.group(2)    # APF or TD3_00380_1000   
        date_str        = match.group(3)    # YYYYMMDD
        time_str        = match.group(4)    # HHMMSS
    else:
        raise ValueError(f"CSV filename {csv_filename} does not match expected pattern")

    return world_name, planner_type

def load_trial_json(json_path: str):
    with open(json_path, 'r') as f:
        return json.load(f)

def main():
    args = parse_arguments()

    # Extract arguments
    csv_path = os.path.join("trial_results", f"{args.csv}.csv")
    description_pkg_path = get_package_share_directory("x3_description")

    world_name, planner_type = parse_csv_filename(args.csv)
    world_sdf_path = os.path.join(description_pkg_path, "worlds", f"{world_name}.sdf")
    timeseries_dir = os.path.join("trial_results", "timeseries")

    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"Error: CSV file not found: {csv_path}")
    if not os.path.exists(timeseries_dir):
        raise FileNotFoundError(f"Error: Timeseries directory not found: {timeseries_dir}")
    if not os.path.exists(world_sdf_path):
        raise FileNotFoundError(f"Error: SDF file not found: {world_sdf_path}")

    # Load CSV
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} trials from {args.csv}")

    fig, ax = plt.subplots(figsize=(12,12))
    # ===== Layer 1 - Render Environment w/ Obstacles =====
    obstacles = parse_sdf_obstacles(world_sdf_path)
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

    # ===== Layer 2 - Render Trajectories =====
    collision_count = 0
    for idx, row in df.iterrows():
        json_path = row['timeseries_log_path']

        if not os.path.exists(json_path):
            print(f"[WARNING]: JSON not found: {json_path}")
            continue

        try:
            trial_data = load_trial_json(json_path)
        except Exception as e:
            print(f"[WARNING]: Failed to load {json_path}: {e}")
            continue

        # Extract gt_odom list for trajectory plotting
        if 'gt_odom' not in trial_data or len(trial_data['gt_odom']) == 0:
            print(f"[WARNING]: No trajectory data for trial {idx}")
            continue

        x = np.array([pt['x']+odom_init['x'] for pt in trial_data['gt_odom']])
        y = np.array([pt['y']+odom_init['y'] for pt in trial_data['gt_odom']])

        # Plot trajectory
        if row['success']:
            ax.plot(x, y, alpha=0.25, linewidth=1.0, linestyle='-', color='#228B22')
        else:
            ax.plot(x, y, alpha=0.50, linewidth=1.0, linestyle='--', color='#CD5C5C')

            if "Obstacle" in row['failure_reason']:
                end_x, end_y = x[-1], y[-1]
                ax.plot(end_x, end_y, marker='x', markersize=10, 
                    color='red', markeredgewidth=2, zorder=10)
                collision_count += 1

            if "timeout" in row['failure_reason']:
                end_x, end_y = x[-1], y[-1]
                ax.plot(end_x, end_y, marker='o', markersize=3, 
                    color='red', markeredgewidth=2, zorder=10)

            goal_x = row['goal_x']+odom_init['x']
            goal_y = row['goal_y']+odom_init['y']
            ax.plot([goal_x, end_x], [goal_y, end_y], 
                color='red', alpha=0.25, linewidth=1.0, linestyle=':', zorder=5)

    # ===== Layer 3: Render Goals =====
    for idx, row in df.iterrows():
        goal_x = row['goal_x']+odom_init['x']
        goal_y = row['goal_y']+odom_init['y']
        
        if row['success']:
        #     # Green filled circle for success
            goal_color = '#228B22'
            goal_edgecolor = '#228B22'
        #     ax.scatter(
        #         goal_x, goal_y, s=40, c=goal_color, edgecolors=goal_edgecolor,
        #         alpha=0.6, zorder=5, 
        #     )
        
        else:
        #     # Red open circle for failure
            goal_color = '#CD5C5C'
            goal_edgecolor = '#CD5C5C'
        #     ax.scatter(
        #         goal_x, goal_y, s=40, facecolors='none', edgecolors=goal_edgecolor, 
        #         linewidths=1.5, zorder=5
        #     )

        # goal_color = "#8B8B8B"

        goal_tolerance = 0.20        # radius (m)
        goal_scatter_size = goal_tolerance * 100 / (2.54 / 72)     # one scatter size = 1/72 in
        ax.scatter(goal_x, goal_y, c=goal_color, edgecolors=goal_color, 
                           s=10, alpha=0.3, zorder=2)
        ax.scatter(goal_x, goal_y, linewidths=1, linestyle='--', c=goal_color, edgecolors=goal_color, 
                   s=goal_scatter_size, alpha=0.3, zorder=2)

    # ===== Layer 4: Render Start Pose =====
    ax.quiver(
        -3, -3,
        1, 0,
        color='green',
        scale=50,
        width=0.004,
        headwidth=4, headlength=5,
        alpha=0.8
    )

    # ===== Styling =====
    axis_limits = (-4.0, 4.0)
    ax.set_xlim(axis_limits)
    ax.set_ylim(axis_limits)
    ax.set_aspect('equal')
    ax.minorticks_on() 
    ax.tick_params(axis='both', which='minor', direction='in', labelsize=12)
    ax.tick_params(axis='both', which='major', direction='in', labelsize=12)
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.grid(False)

    # Legend
    from matplotlib.lines import Line2D
    legends = [
        Line2D([0], [0], color='#228B22', alpha= 0.25, linewidth=2.0, linestyle='-', label='Successful trajectory'),
        Line2D([0], [0], color='#CD5C5C', alpha= 0.50, linewidth=2.0, linestyle='--', label='Failed trajectory (collision)'),
        Line2D([0], [0], marker='o', color='#228B22', markerfacecolor='#228B22', markersize=8, alpha=0.5,
               linestyle='None', label='Goal reached'),
        Line2D([0], [0], marker='o', color='#CD5C5C', markerfacecolor='#CD5C5C', markersize=8, alpha=0.5,
               linestyle='None', markeredgewidth=1.5, label='Goal not reached'),
        Line2D([0], [0], marker='x', color='red', markersize=10, 
               linestyle='None', markeredgewidth=2, label='Collision point'),
        Line2D([0], [0], marker='o', color='red', markersize=3, 
                        linestyle='None', markeredgewidth=2, label='Goal timeout'),
        patches.Patch(facecolor='red', alpha=0.5, edgecolor='red', linewidth=1.5, 
                label='Obstacles (box & cylinder)'),
    ]
    ax.legend(handles=legends, loc=(-0.05, 1.02), fontsize=10, framealpha=0.95, ncol=4, handlelength=1.5)

    plt.show()

    

if __name__ == "__main__":
    main()