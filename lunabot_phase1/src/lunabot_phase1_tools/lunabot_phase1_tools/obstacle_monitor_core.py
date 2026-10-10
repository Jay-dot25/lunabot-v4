"""ROS-independent forward-sector LaserScan reduction and status logic."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Optional

CLEAR = "CLEAR"
OBSTACLE_DETECTED = "OBSTACLE_DETECTED"
NO_VALID_MEASUREMENTS = "NO_VALID_MEASUREMENTS"


@dataclass(frozen=True)
class ObstacleObservation:
    status: str
    closest_range_m: Optional[float]


def closest_forward_range(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    forward_half_angle_rad: float,
) -> Optional[float]:
    """Return the nearest finite in-range return around rover-forward angle 0.

    Angles are wrapped to [-pi, pi], so this also works for scans whose start
    angle is represented outside the conventional interval. NaN, infinity,
    values below range_min, and values above range_max are ignored.
    """
    values = (angle_min, angle_increment, range_min, range_max, forward_half_angle_rad)
    if not all(math.isfinite(float(value)) for value in values):
        return None
    if range_min < 0.0 or range_max < range_min:
        return None
    if forward_half_angle_rad < 0.0 or forward_half_angle_rad > math.pi:
        return None

    nearest: Optional[float] = None
    for index, raw_range in enumerate(ranges):
        angle = float(angle_min) + index * float(angle_increment)
        wrapped_angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(wrapped_angle) > forward_half_angle_rad:
            continue
        try:
            measured_range = float(raw_range)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(measured_range):
            continue
        if measured_range < range_min or measured_range > range_max:
            continue
        if nearest is None or measured_range < nearest:
            nearest = measured_range
    return nearest


def scan_is_stale(
    last_received_monotonic: Optional[float],
    now_monotonic: float,
    timeout_sec: float,
) -> bool:
    """Return true when no usable scan has arrived within the wall timeout."""
    if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
        raise ValueError("timeout_sec must be finite and positive")
    if last_received_monotonic is None or not math.isfinite(last_received_monotonic):
        return True
    if not math.isfinite(now_monotonic) or now_monotonic < last_received_monotonic:
        return True
    return now_monotonic - last_received_monotonic > timeout_sec


def classify_obstacle(
    closest_range_m: Optional[float], warning_distance_m: float
) -> ObstacleObservation:
    """Classify a valid forward return, preserving an explicit unknown state."""
    if not math.isfinite(warning_distance_m) or warning_distance_m <= 0.0:
        raise ValueError("warning_distance_m must be finite and positive")
    if closest_range_m is None or not math.isfinite(closest_range_m):
        return ObstacleObservation(NO_VALID_MEASUREMENTS, None)
    if closest_range_m <= 0.0:
        return ObstacleObservation(NO_VALID_MEASUREMENTS, None)
    status = OBSTACLE_DETECTED if closest_range_m <= warning_distance_m else CLEAR
    return ObstacleObservation(status, float(closest_range_m))
