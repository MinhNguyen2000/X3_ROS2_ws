#!/usr/bin/env python3
"""Shortest collision-free path length between two points of a Gazebo world, for SPL.

Obstacles (static box and cylinder collisions, walls included) are read from
src/x3_description/worlds/<world>.sdf and inflated by a clearance radius, by default the
planners' obstacle_tolerance (0.21 m, policy_node.py / apf_node.py). The robot centre is
treated as a point in the inflated map. The lidar sits 4 cm ahead of base_link, which is ignored.

Each inflated obstacle is a convex polygon that contains the exact Minkowski sum (a polygon
circumscribed around every rounded corner), so lengths are upper bounds by at most a few mm.
The shortest path is found on the visibility graph of the polygon vertices.

Coordinates are in the odom frame used by the planners and the path files: the robot spawns
at world (-3, -3) with yaw 0 (gazebo.launch.py x_offset / y_offset), so odom = world + 3.

Usage as a module:
    sp = ShortestPath('world_1')               # radius 0.21 m
    length, flags = sp.length((0, 0), (2, 1))  # flags: 'start_in_obstacle', 'goal_in_obstacle'
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WORLDS_DIR = os.path.join(HERE, '..', '..', 'src', 'x3_description', 'worlds')
SPAWN_XY = (-3.0, -3.0)      # gazebo.launch.py x_offset / y_offset defaults
OBSTACLE_TOLERANCE = 0.21    # policy_node.py / apf_node.py obstacle_tolerance
N_ARC = 32                   # sides of the polygon circumscribed around each rounded corner
EPS = 1e-9


def _pose2d(elem):
    '''(x, y, yaw) of the <pose> child of elem, identity if absent.'''
    p = elem.find('pose') if elem is not None else None
    if p is None or not (p.text or '').strip():
        return 0.0, 0.0, 0.0
    v = [float(s) for s in p.text.split()]
    return v[0], v[1], v[5]


def _compose(a, b):
    '''Pose b expressed in the frame of pose a.'''
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return ax + c * bx - s * by, ay + s * bx + c * by, ath + bth


def load_obstacles(world, worlds_dir=WORLDS_DIR):
    '''Static box / cylinder collisions of a world SDF, in the odom frame.

    Returns a list of dicts: kind 'box' (cx, cy, yaw, hx, hy) or 'cylinder' (cx, cy, r).
    '''
    root = ET.parse(os.path.join(worlds_dir, f'{world}.sdf')).getroot()
    obstacles = []
    for model in root.iter('model'):
        m_pose = _pose2d(model)
        for link in model.findall('link'):
            l_pose = _compose(m_pose, _pose2d(link))
            for col in link.findall('collision'):
                x, y, yaw = _compose(l_pose, _pose2d(col))
                x, y = x - SPAWN_XY[0], y - SPAWN_XY[1]
                geom = col.find('geometry')
                box, cyl = geom.find('box'), geom.find('cylinder')
                if box is not None:
                    sx, sy, _ = (float(s) for s in box.find('size').text.split())
                    obstacles.append(dict(name=model.get('name'), kind='box', cx=x, cy=y,
                                          yaw=yaw, hx=sx / 2, hy=sy / 2))
                elif cyl is not None:
                    obstacles.append(dict(name=model.get('name'), kind='cylinder', cx=x, cy=y,
                                          r=float(cyl.find('radius').text)))
    return obstacles


def _convex_hull(pts):
    '''Counter-clockwise convex hull (Andrew's monotone chain), no repeated end point.'''
    pts = sorted(set(map(tuple, np.round(pts, 12))))
    if len(pts) <= 2:
        return np.array(pts)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def inflate(ob, radius, n_arc=N_ARC):
    '''Convex polygon (CCW, (k, 2)) that contains the obstacle grown by radius.'''
    if ob['kind'] == 'box':
        c, s = math.cos(ob['yaw']), math.sin(ob['yaw'])
        corners = [(ob['cx'] + c * dx - s * dy, ob['cy'] + s * dx + c * dy)
                   for dx in (-ob['hx'], ob['hx']) for dy in (-ob['hy'], ob['hy'])]
        rho = radius
    else:
        corners = [(ob['cx'], ob['cy'])]
        rho = ob['r'] + radius
    if rho <= 0:
        return _convex_hull(np.array(corners))
    phi = 2 * np.pi * (np.arange(n_arc) + 0.5) / n_arc
    ring = (rho / math.cos(math.pi / n_arc)) * np.stack([np.cos(phi), np.sin(phi)], axis=1)
    return _convex_hull(np.concatenate([np.array(p) + ring for p in corners]))


def _blocked(a, b, polys, skip=None):
    '''Boolean (m,): segment a[i] -> b[i] passes through the interior of any polygon.

    Cyrus-Beck clipping against each convex polygon; a segment that only touches the boundary
    (grazes a vertex or runs along an edge) is not blocked. skip: (m, n_polys) bool mask of
    polygons to ignore per segment (used for an end point inside an obstacle).
    '''
    d = b - a
    seg_len = np.linalg.norm(d, axis=1)
    out = np.zeros(len(a), dtype=bool)
    for j, (verts, normals) in enumerate(polys):
        num = np.einsum('mk,ek->me', a, normals) - np.einsum('ek,ek->e', verts, normals)[None]
        den = d @ normals.T                          # (m, e)
        with np.errstate(divide='ignore', invalid='ignore'):
            t = -num / den
        parallel = np.abs(den) < EPS
        t_enter = np.where(den < -EPS, t, -np.inf).max(axis=1)
        t_exit = np.where(den > EPS, t, np.inf).min(axis=1)
        t_enter = np.maximum(t_enter, 0.0)
        t_exit = np.minimum(t_exit, 1.0)
        outside_parallel = (parallel & (num >= -1e-9)).any(axis=1)
        hit = ~outside_parallel & ((t_exit - t_enter) * seg_len > 1e-6)
        if skip is not None:
            hit &= ~skip[:, j]
        out |= hit
    return out


def _inside(p, polys):
    '''Boolean (n_polys,): point p strictly inside each polygon.'''
    return np.array([np.all(normals @ p - np.einsum('ek,ek->e', verts, normals) < -1e-9)
                     for verts, normals in polys])


class ShortestPath:
    def __init__(self, world, radius=OBSTACLE_TOLERANCE, worlds_dir=WORLDS_DIR):
        self.world, self.radius = world, radius
        self.obstacles = load_obstacles(world, worlds_dir)
        self.polys = []
        for ob in self.obstacles:
            v = inflate(ob, radius)
            e = np.roll(v, -1, axis=0) - v
            normals = np.stack([e[:, 1], -e[:, 0]], axis=1)          # outward for CCW
            normals /= np.linalg.norm(normals, axis=1, keepdims=True)
            self.polys.append((v, normals))

        nodes = np.concatenate([v for v, _ in self.polys])
        keep = np.array([not _inside(p, self.polys).any() for p in nodes])
        self.nodes = nodes[keep]
        n = len(self.nodes)
        ii, jj = np.triu_indices(n, k=1)
        blocked = _blocked(self.nodes[ii], self.nodes[jj], self.polys)
        dist = np.full((n, n), np.inf)
        np.fill_diagonal(dist, 0.0)
        w = np.linalg.norm(self.nodes[jj] - self.nodes[ii], axis=1)
        dist[ii[~blocked], jj[~blocked]] = w[~blocked]
        dist[jj[~blocked], ii[~blocked]] = w[~blocked]
        for k in range(n):                                            # Floyd-Warshall
            np.minimum(dist, dist[:, k, None] + dist[None, k, :], out=dist)
        self.dist = dist

    def _visible(self, p, skip_mask):
        a = np.repeat(np.asarray(p, float)[None], len(self.nodes), axis=0)
        skip = np.repeat(skip_mask[None], len(self.nodes), axis=0)
        vis = ~_blocked(a, self.nodes, self.polys, skip)
        return np.where(vis, np.linalg.norm(self.nodes - p, axis=1), np.inf)

    def length(self, start, goal):
        '''Shortest path length (m) from start to goal, and a list of flags.

        An end point inside an inflated obstacle (the robot stood, or the goal lies, closer to
        an obstacle than the clearance radius) is let out of that obstacle and flagged.
        '''
        s, g = np.asarray(start, float), np.asarray(goal, float)
        in_s, in_g = _inside(s, self.polys), _inside(g, self.polys)
        flags = (['start_in_obstacle'] if in_s.any() else []) + \
                (['goal_in_obstacle'] if in_g.any() else [])
        skip = (in_s | in_g)[None]
        if not _blocked(s[None], g[None], self.polys, skip)[0]:
            return float(np.linalg.norm(g - s)), flags
        ds, dg = self._visible(s, in_s), self._visible(g, in_g)
        best = np.min((ds[:, None] + self.dist).min(axis=0) + dg)
        if not np.isfinite(best):
            flags.append('unreachable')
        return float(best), flags


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('world')
    ap.add_argument('--radius', type=float, default=OBSTACLE_TOLERANCE)
    ap.add_argument('points', nargs=4, type=float, metavar=('X0', 'Y0', 'X1', 'Y1'))
    args = ap.parse_args()
    sp = ShortestPath(args.world, args.radius)
    for ob in sp.obstacles:
        print(ob)
    print(f'{len(sp.nodes)} graph nodes')
    print(sp.length(args.points[:2], args.points[2:]))
