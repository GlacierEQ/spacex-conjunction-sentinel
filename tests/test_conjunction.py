import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from conjunction import (
    AlertEngine,
    Covariance2D,
    State,
    assess_conjunction,
    evaluate_payload,
    miss_distance_km,
    monte_carlo_collision_probability,
    parse_tle,
    plan_avoidance_maneuver,
    propagate_two_body,
    risk_index,
    screen_catalog,
    time_to_cpa_s,
)


def test_far_clear_and_legacy_surface():
    r = risk_index(State(50, 0, 0, 0, 0, 0), thresh_km=5)
    assert r["status"] == "CLEAR"
    assert miss_distance_km(State(3, 4, 0, 0, 0, 0)) == 5.0


def test_cpa_is_analytic_not_step_quantized():
    s = State(10, 1, 0, -2, 0, 0)
    assert time_to_cpa_s(s, dt=10, horizon_s=20) == pytest.approx(5.0)


def test_no_artificial_risk_floor():
    r = risk_index(State(1000, 0, 0, 0, 0, 0), thresh_km=5)
    assert r["risk"] < 1e-6


def test_covariance_rejects_non_positive_definite():
    with pytest.raises(ValueError):
        Covariance2D(1, 1, 1)


def test_monte_carlo_is_deterministic():
    cov = Covariance2D(0.0004, 0, 0.0004)
    p1 = monte_carlo_collision_probability(0, 0, cov, 0.02, samples=5000, seed=7)
    p2 = monte_carlo_collision_probability(0, 0, cov, 0.02, samples=5000, seed=7)
    assert p1 == p2
    assert 0.2 < p1 < 0.6


def test_assessment_detects_crossing_conjunction():
    primary = State(0, 0, 0, 0, 0, 0)
    secondary = State(1, 0.01, 0, -0.001, 0, 0)
    assessment = assess_conjunction(
        primary,
        secondary,
        Covariance2D(0.0004, 0, 0.0004),
        horizon_s=2000,
        samples=5000,
        seed=1,
    )
    assert 900 < assessment.tca_s < 1100
    assert assessment.miss_distance_km == pytest.approx(0.01, abs=1e-8)
    assert assessment.probability_of_collision > 0


def test_maneuver_planner_returns_minimum_linearized_review_vector():
    primary = State(0, 0, 0, 0, 0, 0)
    secondary = State(1, 0.01, 0, -0.001, 0, 0)
    plan = plan_avoidance_maneuver(
        primary,
        secondary,
        target_miss_km=1.0,
        lead_time_s=500,
        max_delta_v_mps=10,
        horizon_s=2000,
    )
    assert plan.feasible
    assert 0 < plan.delta_v_mps < 10
    assert plan.predicted_miss_km == pytest.approx(1.0, rel=1e-8)


def test_maneuver_planner_fails_closed_when_cap_insufficient():
    primary = State(0, 0, 0, 0, 0, 0)
    secondary = State(1, 0, 0, -0.001, 0, 0)
    plan = plan_avoidance_maneuver(
        primary,
        secondary,
        target_miss_km=10,
        lead_time_s=10,
        max_delta_v_mps=0.1,
        horizon_s=2000,
    )
    assert not plan.feasible
    assert plan.delta_v_mps == 0.1
    assert plan.predicted_miss_km < 10


def test_pairwise_catalog_screening_sorted():
    objects = {
        "A": State(0, 0, 0, 0, 0, 0),
        "B": State(1, 0, 0, -0.001, 0, 0),
        "C": State(100, 0, 0, 0, 0, 0),
    }
    results = screen_catalog(objects, horizon_s=2000, screening_distance_km=2)
    assert len(results) == 1
    assert results[0]["primary"] == "A"
    assert results[0]["secondary"] == "B"


def test_tle_parse_and_two_body_propagation_are_finite():
    line1 = "1 25544U 98067A   26224.50000000  .00000000  00000-0  00000-0 0  9999"
    line2 = "2 25544  51.6434 247.4627 0006703 130.5360 325.0288 15.49000000123456"
    tle = parse_tle(line1, line2)
    assert tle.norad_id == 25544
    state = propagate_two_body(tle, 0)
    radius = math.sqrt(state.x**2 + state.y**2 + state.z**2)
    speed = math.sqrt(state.vx**2 + state.vy**2 + state.vz**2)
    assert 6500 < radius < 7000
    assert 7 < speed < 8.5


def test_alert_policy_monotonic():
    engine = AlertEngine()
    assert engine.classify(0) == "CLEAR"
    assert engine.classify(1e-5) == "WATCH"
    assert engine.classify(1e-4) == "MANEUVER_REVIEW"
    assert engine.classify(1e-3) == "URGENT_REVIEW"


def test_payload_emits_truth_boundary():
    payload = {
        "primary": {"x": 0, "y": 0, "z": 0, "vx": 0, "vy": 0, "vz": 0},
        "secondary": {"x": 1, "y": 0.01, "z": 0, "vx": -0.001, "vy": 0, "vz": 0},
        "covariance": {"xx": 0.0004, "xy": 0, "yy": 0.0004},
        "horizon_s": 2000,
        "samples": 1000,
    }
    out = evaluate_payload(payload)
    assert out["schema"] == "glaciereq.conjunction-assessment.v1"
    assert out["operational_authority"] is False
