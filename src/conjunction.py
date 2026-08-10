#!/usr/bin/env python3
"""Close-approach / conjunction risk sentinel — SpaceX-class problem space (portfolio).

Relative motion in simplified Hill frame; risk index from miss distance + relative speed.
Not SpaceX employment or flight software certification.
"""
from __future__ import annotations
import math
from dataclasses import dataclass

C = 299_792_458
G = 9.80665
CONFIDENCE_FLOOR = 0.31415
SIGMA = math.e

@dataclass
class State:
    """Relative state (km, km/s) in local orbital frame (approx)."""
    x: float; y: float; z: float
    vx: float; vy: float; vz: float

def miss_distance_km(s: State) -> float:
    return math.sqrt(s.x**2 + s.y**2 + s.z**2)

def rel_speed_kms(s: State) -> float:
    return math.sqrt(s.vx**2 + s.vy**2 + s.vz**2)

def time_to_cpa_s(s: State, dt: float = 1.0, horizon_s: float = 3600) -> float:
    """Coarse search for time of closest approach."""
    best_t, best_d = 0.0, miss_distance_km(s)
    x,y,z,vx,vy,vz = s.x,s.y,s.z,s.vx,s.vy,s.vz
    t = 0.0
    while t < horizon_s:
        t += dt
        x += vx*dt; y += vy*dt; z += vz*dt
        d = math.sqrt(x*x+y*y+z*z)
        if d < best_d:
            best_d, best_t = d, t
    return best_t

def risk_index(s: State, thresh_km: float = 5.0) -> dict:
    d = miss_distance_km(s)
    v = rel_speed_kms(s)
    tca = time_to_cpa_s(s)
    # risk rises as miss shrinks and speed rises; scale via sigma
    z = (thresh_km - d) / max(SIGMA * 0.5, 1e-6)
    raw = 1.0 / (1.0 + math.exp(-z))
    speed_boost = min(1.0, v / 0.5)
    risk = max(CONFIDENCE_FLOOR, min(1.0, 0.7 * raw + 0.3 * speed_boost * raw))
    if d < 1.0:
        status = "CRITICAL"
    elif d < thresh_km:
        status = "WATCH"
    else:
        status = "CLEAR"
    return {
        "miss_km": round(d, 4),
        "rel_speed_kms": round(v, 5),
        "tca_s": round(tca, 1),
        "risk": round(risk, 4),
        "status": status
        }

if __name__ == "__main__":
    print(risk_index(State(2.0, 0.5, 0.1, -0.02, 0.01, 0.0)))
