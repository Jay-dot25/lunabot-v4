"""Pure LaserScan evaluation helpers used by the ROS obstacle monitor."""
from __future__ import annotations

import math
from typing import Iterable, Optional, Tuple


def evaluate_forward_scan(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    threshold: float,
    half_angle: float,
) -> Tuple[str, Optional[float]]:
    """Return (OBSTACLE|CLEAR|INVALID, nearest finite forward range).

    Positive infinity is a valid no-return observation. NaN, negative
    infinity, zero, and finite values outside the sensor's declared range are
    discarded. An empty/entirely invalid forward sector is INVALID, never CLEAR.
    The scan's own angular convention defines forward as angle zero.
    """
    if (not all(math.isfinite(v) for v in (angle_min, angle_increment,
                                             range_min, range_max, threshold,
                                             half_angle))
            or angle_increment == 0 or range_min < 0 or range_max <= range_min
            or threshold <= 0 or half_angle <= 0):
        return "INVALID", None

    observed = False
    nearest: Optional[float] = None
    for index, raw_range in enumerate(ranges):
        angle = angle_min + index * angle_increment
        if abs(math.atan2(math.sin(angle), math.cos(angle))) > half_angle:
            continue
        try:
            value = float(raw_range)
        except (TypeError, ValueError):
            continue
        if math.isinf(value) and value > 0:
            observed = True
            continue
        if not math.isfinite(value) or value < range_min or value > range_max:
            continue
        observed = True
        nearest = value if nearest is None else min(nearest, value)

    if not observed:
        return "INVALID", None
    if nearest is not None and nearest <= threshold:
        return "OBSTACLE", nearest
    return "CLEAR", nearest


def clamp_velocity(linear_x: float, angular_z: float,
                   max_linear: float, max_angular: float) -> Tuple[float, float]:
    """Clamp planar command speeds and reject non-finite values as zero."""
    if not math.isfinite(linear_x):
        linear_x = 0.0
    if not math.isfinite(angular_z):
        angular_z = 0.0
    return (max(-max_linear, min(max_linear, linear_x)),
            max(-max_angular, min(max_angular, angular_z)))
