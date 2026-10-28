"""LiDAR scan -> policy observation (18 min-pooled groups), kept free of ROS so it can be unit tested.

With the default arguments this reproduces the processing that policy_node.py had before the module existed.
"""
import numpy as np


def preprocess_scan(ranges, angle_min, angle_increment, range_min, range_max,
                    antennae_mask_dist=0.20, rear_mask_deg=0.0, rear_fill='range_max', range_clip=0.0):
    """Rotate the scan so index 0 is straight ahead, clean it, and flip it to the training beam order.

    :param antennae_mask_dist: returns at or below this distance [m] become free space; 0 disables
    :param rear_mask_deg: half-width [deg] of the wedge behind the LiDAR; 0 disables
    :param rear_fill: 'range_max' sets the wedge beams to the maximum range here; 'train_mean' leaves them
        untouched and returns them as a mask, so pool_groups can use the policy's training mean for groups
        that are fully masked and ignore masked beams in partly masked groups
    :param range_clip: maximum range [m] the policy sees (training observation bound); 0 uses range_max.
        Invalid (inf, NaN), below-range_min and masked returns are set to this value.
    :return: (ranges, rear_mask), both in the flipped training order
    """
    top = range_clip if range_clip > 0.0 else range_max
    ranges = np.asarray(ranges, dtype=np.float32)

    # (-pi, pi) -> (0, 2pi), index 0 = forward
    zero_idx = round(-angle_min / angle_increment)
    raw = np.roll(ranges, -zero_idx)

    raw = np.where(np.isfinite(raw), raw, top)
    raw[raw < range_min] = top
    if antennae_mask_dist > 0.0:
        raw[raw <= antennae_mask_dist] = top

    rear = np.zeros(raw.size, dtype=bool)
    if rear_mask_deg > 0.0:
        ang = np.arange(raw.size) * angle_increment
        wedge = np.abs(ang - np.pi) <= np.deg2rad(rear_mask_deg)
        if rear_fill == 'train_mean':
            rear = wedge
        else:
            raw[wedge] = top

    raw = np.clip(raw, 0.0, top)
    return np.flip(raw).copy(), np.flip(rear).copy()


def pool_groups(ranges, rear_mask, n_groups, fill_values):
    """Min-pool into n_groups. Masked beams are ignored; a fully masked group takes fill_values[group]."""
    groups = np.array_split(ranges, n_groups)
    masks = np.array_split(rear_mask, n_groups)
    out = np.empty(n_groups, dtype=np.float32)
    for g, (r, m) in enumerate(zip(groups, masks)):
        keep = ~m
        out[g] = r[keep].min() if keep.any() else fill_values[g]
    return out
