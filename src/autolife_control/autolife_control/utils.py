"""Common utilities for joint controllers."""


def cubic_hermite(t, t0, t1, p0, p1, v0, v1):
    """
    Cubic Hermite spline interpolation between two waypoints.
    Returns (position, velocity) at time t.
    """
    h = t1 - t0
    if h <= 0:
        return p1, v1

    s = (t - t0) / h
    s2 = s * s
    s3 = s2 * s

    pos = ((2 * s3 - 3 * s2 + 1) * p0 +
           (s3 - 2 * s2 + s) * h * v0 +
           (-2 * s3 + 3 * s2) * p1 +
           (s3 - s2) * h * v1)

    vel = ((6 * s2 - 6 * s) / h * p0 +
           (3 * s2 - 4 * s + 1) * v0 +
           (-6 * s2 + 6 * s) / h * p1 +
           (3 * s2 - 2 * s) * v1)

    return pos, vel


def auto_compute_velocities(positions_list, times):
    """
    Auto-compute velocities at each waypoint using central differences.
    Used when the trajectory message doesn't include velocities.
    """
    n = len(times)
    velocities = [0.0] * n

    for i in range(n):
        if i == 0:
            velocities[i] = 0.0
        elif i == n - 1:
            velocities[i] = 0.0
        else:
            dt = times[i + 1] - times[i - 1]
            if dt > 0:
                velocities[i] = (positions_list[i + 1] - positions_list[i - 1]) / dt
            else:
                velocities[i] = 0.0

    return velocities
