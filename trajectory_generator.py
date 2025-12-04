# trajectory_generator.py
import math

def generate_time_parameterized_trajectory(smooth_path, v_max=0.22, accel=0.5):
    """
    smooth_path: list of (x,y)
    v_max: max linear speed (m/s)
    accel: linear acceleration (m/s^2)
    returns: list of (x,y,t, desired_speed)
    This uses a simple piecewise approach: compute segment distances,
    accelerate / clip speed to limit (approx trapezoidal).
    """
    if len(smooth_path) == 0:
        return []

    traj = []
    t = 0.0
    prev_v = 0.0
    traj.append((smooth_path[0][0], smooth_path[0][1], t, 0.0))

    for i in range(1, len(smooth_path)):
        x0, y0 = smooth_path[i-1]
        x1, y1 = smooth_path[i]
        dist = math.hypot(x1 - x0, y1 - y0)
        if dist < 1e-6:
            traj.append((x1, y1, t, prev_v))
            continue

        # target (cruise) speed is v_max (can plan ramp up)
        # compute how long to accelerate from prev_v to v_max
        time_to_vmax = max(0.0, (v_max - prev_v) / accel)
        dist_accel = (prev_v + v_max) / 2.0 * time_to_vmax

        if dist_accel >= dist:
            # cannot reach vmax on this short segment: use kinematics to compute time
            # s = (v1+v0)/2 * dt -> dt = 2*s / (v0+v1); choose v1 = v0 + accel * dt -> solve quadratic approx
            # fallback: assume constant speed = prev_v (safe)
            dt = dist / max(1e-3, prev_v)
            if prev_v < 1e-3:
                # if prev_v ~ 0, assume small step at v_max but slow
                dt = dist / max(1e-3, 0.05)
            t += dt
            prev_v = min(v_max, prev_v + accel * dt)
            traj.append((x1, y1, t, prev_v))
        else:
            # accelerate then cruise
            # accelerate part
            t += time_to_vmax
            prev_v = v_max
            # cruise time
            dist_remaining = dist - dist_accel
            dt_cruise = dist_remaining / v_max
            t += dt_cruise
            traj.append((x1, y1, t, v_max))
    return traj

