"""Asymmetric delta step of the P14 policies (TD3_014xx), identical to asym_delta() in ROS2_DRL_Navigation
python/environments/nav2d.py, which the policies were trained with; keep the two identical."""
import numpy as np


def asym_delta(last, frac, up, down):
    """Each command component moves by frac * up when it moves away from zero (same sign as the command, or from zero)
    and by frac * down when it moves toward zero, stopping at zero instead of crossing it. frac is the actor output in
    [-1, 1]; last, up and the result are in normalized command units."""
    last, frac = np.asarray(last, np.float64), np.asarray(frac, np.float64)
    away = (frac * last > 0) | (last == 0)
    new = last + frac * np.where(away, up, down)
    return np.where(~away & (new * last < 0), 0.0, new)
