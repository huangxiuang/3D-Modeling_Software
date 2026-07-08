"""
Flight math utilities — extracted from main_window.py for modularity.

Stateless pure functions for flight path computation, spline interpolation,
speed modeling, and angle interpolation.
"""

import math
import numpy as np


# ═══════════════════════════════════════════════════════════════
#  Speed model
# ═══════════════════════════════════════════════════════════════

def compute_flight_speed(pitch_deg, yaw_rate_deg_s, cruise=5.0):
    """Systematic speed model: V = V0 * k_pitch * k_turn.

    k_pitch = max(0.30, 1.0 - 0.35 * sin(pitch_rad))
    k_turn  = max(0.30, 1.0 - 0.006 * abs(yaw_rate))
    """
    pitch_rad = math.radians(pitch_deg)
    k_pitch = max(0.30, 1.0 - 0.35 * math.sin(pitch_rad))
    k_turn = max(0.30, 1.0 - 0.006 * abs(yaw_rate_deg_s))
    return cruise * k_pitch * k_turn


# ═══════════════════════════════════════════════════════════════
#  Interpolation
# ═══════════════════════════════════════════════════════════════

def lerp_angle(a, b, t):
    """Linearly interpolate between two angles in degrees, handling 0/360 wrap.
    
    Returns the shortest-path interpolation.
    """
    a = a % 360.0
    b = b % 360.0
    diff = b - a
    if diff > 180.0:
        diff -= 360.0
    elif diff < -180.0:
        diff += 360.0
    return (a + diff * t) % 360.0


# ═══════════════════════════════════════════════════════════════
#  Catmull-Rom spline
# ═══════════════════════════════════════════════════════════════

def catmull_rom_position(control_points, seg_idx, t):
    """Evaluate Catmull-Rom spline position at (segment, t).

    Parameters
    ----------
    control_points : list of np.array (3,)
        Waypoints the spline passes through.
    seg_idx : int
        Which segment (0..N-2).
    t : float
        Parameter in [0, 1] within the segment.

    Returns
    -------
    np.array of shape (3,) — interpolated 3D position.
    """
    n = len(control_points)
    if n < 2:
        return control_points[0] if n == 1 else np.zeros(3)

    seg_idx = max(0, min(seg_idx, n - 2))
    t = max(0.0, min(t, 1.0))

    p0 = _get_point(control_points, seg_idx - 1)
    p1 = control_points[seg_idx]
    p2 = control_points[min(seg_idx + 1, n - 1)]
    p3 = _get_point(control_points, seg_idx + 2)

    # Catmull-Rom matrix
    tt = t * t
    ttt = tt * t
    result = (
        0.5 * ((2 * p1) +
               (-p0 + p2) * t +
               (2 * p0 - 5 * p1 + 4 * p2 - p3) * tt +
               (-p0 + 3 * p1 - 3 * p2 + p3) * ttt)
    )
    return result


def _get_point(points, idx):
    """Get control point at *idx*, with mirror-padding for boundaries."""
    n = len(points)
    if idx < 0:
        return 2 * points[0] - points[1] if n > 1 else points[0]
    if idx >= n:
        return 2 * points[-1] - points[-2] if n > 1 else points[-1]
    return points[idx]


# ═══════════════════════════════════════════════════════════════
#  Flight state computation
# ═══════════════════════════════════════════════════════════════

def compute_flight_state(path, segments, seg_idx, step, flight_speed_fn=None):
    """Compute aircraft position + attitude at a given (segment, step).

    Returns dict with keys: pos, yaw, pitch, roll — or None if seg_idx out of range.

    Parameters
    ----------
    path : list of np.array
        All waypoints.
    segments : list of dict
        Each with keys: steps, yaw, pitch, from, to.
    seg_idx : int
    step : int
    flight_speed_fn : callable(pitch_deg, yaw_rate_deg_s) -> float, optional

    Returns
    -------
    dict or None
    """
    if seg_idx >= len(segments):
        return None

    seg = segments[seg_idx]
    steps = seg["steps"]
    t_val = step / float(steps) if steps > 0 else 1.0
    t_val = min(t_val, 1.0)

    pos = catmull_rom_position(path, seg_idx, t_val)
    entry_yaw = float(seg["yaw"])
    exit_yaw = float(segments[seg_idx + 1]["yaw"]) if seg_idx + 1 < len(segments) else entry_yaw
    yaw = lerp_angle(entry_yaw, exit_yaw, t_val)

    entry_pitch = float(seg["pitch"])
    exit_pitch = float(segments[seg_idx + 1]["pitch"]) if seg_idx + 1 < len(segments) else entry_pitch
    pitch = max(-90.0, min(90.0, entry_pitch + (exit_pitch - entry_pitch) * t_val))

    return {"pos": pos, "yaw": yaw, "pitch": pitch, "roll": 0.0}
