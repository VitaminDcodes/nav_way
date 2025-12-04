# path_smoothing.py
import numpy as np
from scipy.interpolate import CubicSpline

def smooth_path(waypoints, samples=300):
    """
    Cubic spline interpolation of waypoints.
    waypoints: list of (x,y)
    returns: list of (x,y) densely sampled
    """
    if len(waypoints) < 2:
        return waypoints[:]

    wp = np.array(waypoints, dtype=float)
    t = np.arange(len(wp))
    # If points are collinear or duplicate, fall back to linear spacing
    try:
        spline_x = CubicSpline(t, wp[:, 0])
        spline_y = CubicSpline(t, wp[:, 1])
    except Exception:
        # fallback linear interpolation
        xs = np.interp(np.linspace(0, len(wp)-1, samples), t, wp[:,0])
        ys = np.interp(np.linspace(0, len(wp)-1, samples), t, wp[:,1])
        return list(zip(xs.tolist(), ys.tolist()))

    t_new = np.linspace(0, len(wp)-1, samples)
    xs = spline_x(t_new)
    ys = spline_y(t_new)
    return [(float(px), float(py)) for px, py in zip(xs, ys)]

