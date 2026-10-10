"""ROS-independent velocity limiting and deadman-watchdog policy."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class GuardDecision:
    """The command to forward and whether the input has gone stale."""

    linear_x: float
    angular_z: float
    stale: bool


def _finite_or_zero(value: float) -> float:
    value = float(value)
    return value if math.isfinite(value) else 0.0


def clamp_command(
    linear_x: float,
    angular_z: float,
    linear_limit_mps: float,
    angular_limit_radps: float,
) -> tuple[float, float]:
    """Clamp planar velocity and discard NaN / infinite commands safely."""
    if not math.isfinite(linear_limit_mps) or linear_limit_mps <= 0.0:
        raise ValueError("linear_limit_mps must be finite and positive")
    if not math.isfinite(angular_limit_radps) or angular_limit_radps <= 0.0:
        raise ValueError("angular_limit_radps must be finite and positive")

    linear = _finite_or_zero(linear_x)
    angular = _finite_or_zero(angular_z)
    return (
        max(-linear_limit_mps, min(linear_limit_mps, linear)),
        max(-angular_limit_radps, min(angular_limit_radps, angular)),
    )


def resolve_command(
    linear_x: float,
    angular_z: float,
    received_at_monotonic: Optional[float],
    now_monotonic: float,
    timeout_sec: float,
    linear_limit_mps: float,
    angular_limit_radps: float,
) -> GuardDecision:
    """Apply the wall-clock deadman timeout, then velocity limits.

    A missing command, invalid timestamp, or expired command resolves to zero.
    Monotonic time is used by the ROS node so a paused simulation clock cannot
    disable this command-safety mechanism.
    """
    if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
        raise ValueError("timeout_sec must be finite and positive")
    if not math.isfinite(now_monotonic):
        return GuardDecision(0.0, 0.0, True)

    stale = (
        received_at_monotonic is None
        or not math.isfinite(received_at_monotonic)
        or now_monotonic - received_at_monotonic > timeout_sec
    )
    if stale:
        return GuardDecision(0.0, 0.0, True)

    linear, angular = clamp_command(
        linear_x,
        angular_z,
        linear_limit_mps,
        angular_limit_radps,
    )
    return GuardDecision(linear, angular, False)
