#!/usr/bin/env python3
"""Deterministic local conjunction-assessment simulation toolkit.

Portfolio software only: no flight authority, no operational maneuver execution,
no mission-certified orbit determination, and no SpaceX affiliation.
"""
from __future__ import annotations

import argparse
import json
import math
import random
from dataclasses import asdict, dataclass
from itertools import combinations
from pathlib import Path
from typing import Mapping, Sequence

MU_EARTH_KM3_S2 = 398600.4418
SECONDS_PER_DAY = 86400.0
TWO_PI = 2.0 * math.pi


def _finite(name: str, value: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a real number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _positive(name: str, value: float) -> float:
    value = _finite(name, value)
    if value <= 0.0:
        raise ValueError(f"{name} must be > 0")
    return value


@dataclass(frozen=True)
class State:
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float

    def __post_init__(self) -> None:
        for name in ("x", "y", "z", "vx", "vy", "vz"):
            object.__setattr__(self, name, _finite(name, getattr(self, name)))

    @property
    def position(self) -> tuple[float, float, float]:
        return self.x, self.y, self.z

    @property
    def velocity(self) -> tuple[float, float, float]:
        return self.vx, self.vy, self.vz


@dataclass(frozen=True)
class Covariance2D:
    xx: float
    xy: float
    yy: float

    def __post_init__(self) -> None:
        xx, yy, xy = _positive("xx", self.xx), _positive("yy", self.yy), _finite("xy", self.xy)
        if xx * yy - xy * xy <= 0.0:
            raise ValueError("covariance must be positive definite")
        object.__setattr__(self, "xx", xx)
        object.__setattr__(self, "xy", xy)
        object.__setattr__(self, "yy", yy)


@dataclass(frozen=True)
class TLE:
    norad_id: int
    epoch_year: int
    epoch_day: float
    inclination_deg: float
    raan_deg: float
    eccentricity: float
    arg_perigee_deg: float
    mean_anomaly_deg: float
    mean_motion_rev_day: float

    def __post_init__(self) -> None:
        if isinstance(self.norad_id, bool) or int(self.norad_id) <= 0:
            raise ValueError("norad_id must be a positive integer")
        if not 1957 <= int(self.epoch_year) <= 2200:
            raise ValueError("epoch_year outside supported range")
        if not 0.0 < _finite("epoch_day", self.epoch_day) < 367.0:
            raise ValueError("epoch_day outside supported range")
        if not 0.0 <= _finite("inclination_deg", self.inclination_deg) <= 180.0:
            raise ValueError("inclination_deg must be in [0, 180]")
        if not 0.0 <= _finite("eccentricity", self.eccentricity) < 1.0:
            raise ValueError("eccentricity must be in [0, 1)")
        _positive("mean_motion_rev_day", self.mean_motion_rev_day)
        for name in ("raan_deg", "arg_perigee_deg", "mean_anomaly_deg"):
            _finite(name, getattr(self, name))


@dataclass(frozen=True)
class ConjunctionAssessment:
    tca_s: float
    miss_distance_km: float
    relative_speed_kms: float
    encounter_x_km: float
    encounter_y_km: float
    probability_of_collision: float
    risk_level: str
    monte_carlo_samples: int
    seed: int


@dataclass(frozen=True)
class ManeuverPlan:
    feasible: bool
    delta_v_mps: float
    delta_v_vector_mps: tuple[float, float, float]
    burn_time_s: float
    predicted_miss_km: float
    target_miss_km: float
    model: str = "linearized-impulse-review-only"


def _sub(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def _scale(a: Sequence[float], k: float) -> tuple[float, float, float]:
    return a[0] * k, a[1] * k, a[2] * k


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]


def _norm(a: Sequence[float]) -> float:
    return math.sqrt(_dot(a, a))


def _unit(a: Sequence[float]) -> tuple[float, float, float]:
    n = _norm(a)
    if n <= 0.0:
        raise ValueError("cannot normalize a zero vector")
    return _scale(a, 1.0 / n)


def state_at(s: State, t_s: float) -> State:
    t_s = _finite("t_s", t_s)
    return State(s.x + s.vx * t_s, s.y + s.vy * t_s, s.z + s.vz * t_s, s.vx, s.vy, s.vz)


def miss_distance_km(s: State) -> float:
    return _norm(s.position)


def rel_speed_kms(s: State) -> float:
    return _norm(s.velocity)


def relative_state(primary: State, secondary: State) -> State:
    return State(*_sub(primary.position, secondary.position), *_sub(primary.velocity, secondary.velocity))


def time_to_cpa_s(s: State, dt: float = 1.0, horizon_s: float = 3600.0) -> float:
    """Analytic CPA under constant relative velocity; ``dt`` is compatibility-only."""
    _positive("dt", dt)
    horizon_s = _positive("horizon_s", horizon_s)
    vv = _dot(s.velocity, s.velocity)
    if vv == 0.0:
        return 0.0
    return min(horizon_s, max(0.0, -_dot(s.position, s.velocity) / vv))


def _encounter_plane(relative_velocity: Sequence[float]) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    speed = _norm(relative_velocity)
    if speed < 1e-12:
        return (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)
    normal = _scale(relative_velocity, 1.0 / speed)
    reference = (0.0, 0.0, 1.0) if abs(normal[2]) < 0.9 else (0.0, 1.0, 0.0)
    e1 = _unit(_cross(normal, reference))
    return e1, _unit(_cross(normal, e1))


def monte_carlo_collision_probability(
    miss_x_km: float,
    miss_y_km: float,
    covariance: Covariance2D,
    hard_body_radius_km: float,
    *,
    samples: int = 20_000,
    seed: int = 0,
) -> float:
    miss_x_km, miss_y_km = _finite("miss_x_km", miss_x_km), _finite("miss_y_km", miss_y_km)
    hard_body_radius_km = _positive("hard_body_radius_km", hard_body_radius_km)
    if isinstance(samples, bool) or not isinstance(samples, int) or samples < 1000:
        raise ValueError("samples must be an integer >= 1000")
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise TypeError("seed must be an integer")
    l11 = math.sqrt(covariance.xx)
    l21 = covariance.xy / l11
    l22 = math.sqrt(covariance.yy - l21 * l21)
    hbr2, collisions, rng = hard_body_radius_km**2, 0, random.Random(seed)
    for _ in range(samples):
        z1, z2 = rng.gauss(0.0, 1.0), rng.gauss(0.0, 1.0)
        x, y = miss_x_km + l11 * z1, miss_y_km + l21 * z1 + l22 * z2
        collisions += x * x + y * y <= hbr2
    return collisions / samples


def _risk_level(pc: float) -> str:
    return "CRITICAL_REVIEW" if pc >= 1e-3 else "MANEUVER_REVIEW" if pc >= 1e-4 else "WATCH" if pc >= 1e-5 else "CLEAR"


def assess_conjunction(
    primary: State,
    secondary: State,
    covariance: Covariance2D,
    *,
    hard_body_radius_km: float = 0.02,
    horizon_s: float = 72 * 3600.0,
    samples: int = 20_000,
    seed: int = 0,
) -> ConjunctionAssessment:
    rel = relative_state(primary, secondary)
    tca = time_to_cpa_s(rel, horizon_s=horizon_s)
    at_tca = state_at(rel, tca)
    e1, e2 = _encounter_plane(at_tca.velocity)
    x, y = _dot(at_tca.position, e1), _dot(at_tca.position, e2)
    pc = monte_carlo_collision_probability(x, y, covariance, hard_body_radius_km, samples=samples, seed=seed)
    return ConjunctionAssessment(tca, miss_distance_km(at_tca), rel_speed_kms(at_tca), x, y, pc, _risk_level(pc), samples, seed)


def plan_avoidance_maneuver(
    primary: State,
    secondary: State,
    *,
    target_miss_km: float = 5.0,
    lead_time_s: float = 1800.0,
    max_delta_v_mps: float = 25.0,
    horizon_s: float = 72 * 3600.0,
) -> ManeuverPlan:
    target_miss_km, lead_time_s, max_delta_v_mps = (
        _positive("target_miss_km", target_miss_km),
        _positive("lead_time_s", lead_time_s),
        _positive("max_delta_v_mps", max_delta_v_mps),
    )
    rel = relative_state(primary, secondary)
    tca = time_to_cpa_s(rel, horizon_s=horizon_s)
    at_tca, current = state_at(rel, tca), miss_distance_km(state_at(rel, tca))
    if current >= target_miss_km:
        return ManeuverPlan(True, 0.0, (0.0, 0.0, 0.0), max(0.0, tca - lead_time_s), current, target_miss_km)
    burn_time, coast = max(0.0, tca - lead_time_s), min(lead_time_s, tca)
    if coast <= 0.0:
        return ManeuverPlan(False, 0.0, (0.0, 0.0, 0.0), burn_time, current, target_miss_km)
    e1, e2 = _encounter_plane(at_tca.velocity)
    x, y = _dot(at_tca.position, e1), _dot(at_tca.position, e2)
    direction = _unit((e1[0] * x + e2[0] * y, e1[1] * x + e2[1] * y, e1[2] * x + e2[2] * y)) if math.hypot(x, y) > 1e-12 else e1
    required = (target_miss_km - current) / coast * 1000.0
    applied = min(required, max_delta_v_mps)
    return ManeuverPlan(required <= max_delta_v_mps, applied, _scale(direction, applied), burn_time, current + applied / 1000.0 * coast, target_miss_km)


def risk_index(s: State, thresh_km: float = 5.0) -> dict[str, float | str]:
    """Compatibility geometric index, explicitly not probability of collision."""
    thresh_km = _positive("thresh_km", thresh_km)
    tca = time_to_cpa_s(s)
    at_tca, v = state_at(s, tca), rel_speed_kms(s)
    d = miss_distance_km(at_tca)
    risk = min(1.0, math.exp(-0.5 * (d / thresh_km) ** 2) * (0.75 + 0.25 * min(1.0, v)))
    status = "CRITICAL" if d < 1.0 else "WATCH" if d < thresh_km else "CLEAR"
    return {"miss_km": round(d, 6), "rel_speed_kms": round(v, 6), "tca_s": round(tca, 3), "risk": round(risk, 6), "status": status}


def screen_catalog(objects: Mapping[str, State], *, horizon_s: float = 72 * 3600.0, screening_distance_km: float = 10.0) -> list[dict[str, float | str]]:
    """Deterministic O(n²) local screening for small catalogs."""
    _positive("horizon_s", horizon_s)
    screening_distance_km = _positive("screening_distance_km", screening_distance_km)
    results: list[dict[str, float | str]] = []
    for (id_a, a), (id_b, b) in combinations(objects.items(), 2):
        rel = relative_state(a, b)
        tca = time_to_cpa_s(rel, horizon_s=horizon_s)
        miss = miss_distance_km(state_at(rel, tca))
        if miss <= screening_distance_km:
            results.append({"primary": id_a, "secondary": id_b, "tca_s": round(tca, 3), "miss_distance_km": round(miss, 6), "relative_speed_kms": round(rel_speed_kms(rel), 6)})
    return sorted(results, key=lambda item: (float(item["miss_distance_km"]), float(item["tca_s"]), str(item["primary"]), str(item["secondary"])))


def parse_tle(line1: str, line2: str) -> TLE:
    if not isinstance(line1, str) or not isinstance(line2, str) or len(line1) < 32 or len(line2) < 63 or not line1.startswith("1 ") or not line2.startswith("2 "):
        raise ValueError("invalid TLE line shape")
    norad1, norad2 = int(line1[2:7]), int(line2[2:7])
    if norad1 != norad2:
        raise ValueError("TLE NORAD identifiers do not match")
    yy = int(line1[18:20])
    return TLE(norad1, 2000 + yy if yy < 57 else 1900 + yy, float(line1[20:32]), float(line2[8:16]), float(line2[17:25]), float(f"0.{line2[26:33].strip()}"), float(line2[34:42]), float(line2[43:51]), float(line2[52:63]))


def propagate_two_body(tle: TLE, minutes_since_epoch: float) -> State:
    """Two-body Kepler propagation from TLE mean elements. Not SGP4/SDP4."""
    minutes_since_epoch = _finite("minutes_since_epoch", minutes_since_epoch)
    n = tle.mean_motion_rev_day * TWO_PI / SECONDS_PER_DAY
    a, e = (MU_EARTH_KM3_S2 / n**2) ** (1.0 / 3.0), tle.eccentricity
    m = (math.radians(tle.mean_anomaly_deg) + n * minutes_since_epoch * 60.0) % TWO_PI
    eccentric = m
    for _ in range(30):
        delta = (eccentric - e * math.sin(eccentric) - m) / (1.0 - e * math.cos(eccentric))
        eccentric -= delta
        if abs(delta) < 1e-13:
            break
    ce, se = math.cos(eccentric), math.sin(eccentric)
    denom = 1.0 - e * ce
    r, cos_nu, sin_nu = a * denom, (ce - e) / denom, math.sqrt(1.0 - e * e) * se / denom
    scale = math.sqrt(MU_EARTH_KM3_S2 / (a * (1.0 - e * e)))
    x_pf, y_pf, vx_pf, vy_pf = r * cos_nu, r * sin_nu, -scale * sin_nu, scale * (e + cos_nu)
    raan, inc, argp = map(math.radians, (tle.raan_deg, tle.inclination_deg, tle.arg_perigee_deg))
    co, so, ci, si, cw, sw = math.cos(raan), math.sin(raan), math.cos(inc), math.sin(inc), math.cos(argp), math.sin(argp)
    r11, r12 = co * cw - so * sw * ci, -co * sw - so * cw * ci
    r21, r22 = so * cw + co * sw * ci, -so * sw + co * cw * ci
    r31, r32 = sw * si, cw * si
    return State(r11*x_pf+r12*y_pf, r21*x_pf+r22*y_pf, r31*x_pf+r32*y_pf, r11*vx_pf+r12*vy_pf, r21*vx_pf+r22*vy_pf, r31*vx_pf+r32*vy_pf)


class AlertEngine:
    def __init__(self, watch_pc: float = 1e-5, maneuver_pc: float = 1e-4, urgent_pc: float = 1e-3) -> None:
        self.watch_pc, self.maneuver_pc, self.urgent_pc = _positive("watch_pc", watch_pc), _positive("maneuver_pc", maneuver_pc), _positive("urgent_pc", urgent_pc)
        if not self.watch_pc < self.maneuver_pc < self.urgent_pc:
            raise ValueError("thresholds must satisfy watch < maneuver < urgent")

    def classify(self, probability_of_collision: float) -> str:
        pc = _finite("probability_of_collision", probability_of_collision)
        if not 0.0 <= pc <= 1.0:
            raise ValueError("probability_of_collision must be in [0, 1]")
        return "URGENT_REVIEW" if pc >= self.urgent_pc else "MANEUVER_REVIEW" if pc >= self.maneuver_pc else "WATCH" if pc >= self.watch_pc else "CLEAR"


def evaluate_payload(payload: Mapping[str, object]) -> dict[str, object]:
    p, s, c = payload["primary"], payload["secondary"], payload["covariance"]
    if not isinstance(p, Mapping) or not isinstance(s, Mapping) or not isinstance(c, Mapping):
        raise ValueError("primary, secondary and covariance must be objects")
    primary = State(*(p[name] for name in ("x", "y", "z", "vx", "vy", "vz")))
    secondary = State(*(s[name] for name in ("x", "y", "z", "vx", "vy", "vz")))
    covariance = Covariance2D(c["xx"], c.get("xy", 0.0), c["yy"])
    horizon = float(payload.get("horizon_s", 72 * 3600.0))
    assessment = assess_conjunction(primary, secondary, covariance, hard_body_radius_km=float(payload.get("hard_body_radius_km", 0.02)), horizon_s=horizon, samples=int(payload.get("samples", 20_000)), seed=int(payload.get("seed", 0)))
    plan = plan_avoidance_maneuver(primary, secondary, target_miss_km=float(payload.get("target_miss_km", 5.0)), lead_time_s=float(payload.get("lead_time_s", 1800.0)), max_delta_v_mps=float(payload.get("max_delta_v_mps", 25.0)), horizon_s=horizon)
    return {"schema": "glaciereq.conjunction-assessment.v1", "assessment": asdict(assessment), "maneuver_review": asdict(plan), "operational_authority": False}


def _demo_payload() -> dict[str, object]:
    return {"primary": {"x": 0, "y": 0, "z": 0, "vx": 0, "vy": 0, "vz": 0}, "secondary": {"x": 1, "y": 0.04, "z": 0, "vx": -0.001, "vy": 0, "vz": 0}, "covariance": {"xx": 0.0004, "xy": 0, "yy": 0.0004}, "horizon_s": 2000, "samples": 5000, "seed": 7, "lead_time_s": 600.0}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Deterministic local conjunction assessment simulator")
    sub = parser.add_subparsers(dest="command")
    sub.add_parser("demo")
    assess = sub.add_parser("assess")
    assess.add_argument("input")
    args = parser.parse_args(argv)
    if args.command in (None, "demo"):
        result = evaluate_payload(_demo_payload())
    else:
        import sys
        payload = json.load(sys.stdin) if args.input == "-" else json.loads(Path(args.input).read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("scenario must be a JSON object")
        result = evaluate_payload(payload)
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
