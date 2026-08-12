#!/usr/bin/env python3
"""Deterministic local close-approach screening under constant relative velocity.

This module is a portfolio reference model. It does not ingest TLE/CDM catalogs,
implement SGP4/SDP4, compute calibrated probability of collision, plan avoidance
maneuvers, contact spacecraft, or provide operational orbital-safety authority.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

EVIDENCE_STATE = "LOCAL_CLOSE_APPROACH_GEOMETRY_NOT_COLLISION_AVOIDANCE_AUTHORITY"
DEFAULT_SCREENING_DISTANCE_KM = 5.0
DEFAULT_HORIZON_S = 3600.0


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _positive(value: float, name: str) -> float:
    value = _finite(value, name)
    if value <= 0:
        raise ValueError(f"{name} must be positive")
    return value


@dataclass(frozen=True)
class State:
    """Relative Cartesian state: position in km and velocity in km/s."""

    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float

    def validated(self) -> "State":
        return State(
            _finite(self.x, "x"),
            _finite(self.y, "y"),
            _finite(self.z, "z"),
            _finite(self.vx, "vx"),
            _finite(self.vy, "vy"),
            _finite(self.vz, "vz"),
        )


def miss_distance_km(state: State) -> float:
    state = state.validated()
    return math.sqrt(state.x**2 + state.y**2 + state.z**2)


def rel_speed_kms(state: State) -> float:
    state = state.validated()
    return math.sqrt(state.vx**2 + state.vy**2 + state.vz**2)


def time_to_cpa_s(state: State, horizon_s: float = DEFAULT_HORIZON_S) -> float:
    """Analytic closest-approach time for constant relative velocity.

    The result is clamped to ``[0, horizon_s]``. This is not orbit propagation.
    """

    state = state.validated()
    horizon_s = _positive(horizon_s, "horizon_s")
    speed_sq = state.vx**2 + state.vy**2 + state.vz**2
    if speed_sq <= 1e-18:
        return 0.0
    dot = state.x * state.vx + state.y * state.vy + state.z * state.vz
    unconstrained = -dot / speed_sq
    return min(horizon_s, max(0.0, unconstrained))


def closest_approach_km(
    state: State, horizon_s: float = DEFAULT_HORIZON_S
) -> tuple[float, float]:
    """Return ``(time_s, miss_distance_km)`` for the local linear model."""

    state = state.validated()
    time_s = time_to_cpa_s(state, horizon_s=horizon_s)
    x = state.x + state.vx * time_s
    y = state.y + state.vy * time_s
    z = state.z + state.vz * time_s
    return time_s, math.sqrt(x * x + y * y + z * z)


def screening_score(miss_km: float, threshold_km: float) -> float:
    """Bounded geometric proximity score; explicitly not a probability."""

    miss_km = _finite(miss_km, "miss_km")
    threshold_km = _positive(threshold_km, "threshold_km")
    if miss_km < 0:
        raise ValueError("miss_km must be non-negative")
    return max(0.0, min(1.0, 1.0 - miss_km / threshold_km))


def screen_close_approach(
    state: State,
    threshold_km: float = DEFAULT_SCREENING_DISTANCE_KM,
    horizon_s: float = DEFAULT_HORIZON_S,
) -> dict[str, float | str | bool]:
    """Evaluate one caller-supplied relative state with local geometry only."""

    state = state.validated()
    threshold_km = _positive(threshold_km, "threshold_km")
    horizon_s = _positive(horizon_s, "horizon_s")
    initial = miss_distance_km(state)
    relative_speed = rel_speed_kms(state)
    tca, closest = closest_approach_km(state, horizon_s=horizon_s)
    score = screening_score(closest, threshold_km)

    if closest <= threshold_km * 0.2:
        status = "ELEVATED_GEOMETRY"
    elif closest < threshold_km:
        status = "WATCH_GEOMETRY"
    else:
        status = "CLEAR_GEOMETRY"

    return {
        "initial_separation_km": round(initial, 6),
        "closest_approach_km": round(closest, 6),
        "relative_speed_kms": round(relative_speed, 6),
        "time_to_closest_approach_s": round(tca, 6),
        "screening_score": round(score, 6),
        "status": status,
        "collision_probability": False,
        "maneuver_authority": False,
        "evidence_state": EVIDENCE_STATE,
    }


def risk_index(
    state: State,
    thresh_km: float = DEFAULT_SCREENING_DISTANCE_KM,
) -> dict[str, float | str | bool]:
    """Compatibility alias for :func:`screen_close_approach`.

    The historical name does not imply a calibrated risk or collision
    probability. New callers should use ``screen_close_approach``.
    """

    return screen_close_approach(state, threshold_km=thresh_km)


if __name__ == "__main__":
    print(screen_close_approach(State(2.0, 0.5, 0.1, -0.02, 0.01, 0.0)))
