from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from conjunction import (  # noqa: E402
    EVIDENCE_STATE,
    State,
    closest_approach_km,
    miss_distance_km,
    risk_index,
    screen_close_approach,
    screening_score,
    time_to_cpa_s,
)


def test_far_geometry_is_clear() -> None:
    result = screen_close_approach(State(50, 0, 0, 0, 0, 0), threshold_km=5)
    assert result["status"] == "CLEAR_GEOMETRY"
    assert result["screening_score"] == 0.0
    assert result["collision_probability"] is False
    assert result["maneuver_authority"] is False
    assert result["evidence_state"] == EVIDENCE_STATE


def test_approaching_state_uses_analytic_closest_approach() -> None:
    state = State(10, 0, 0, -1, 0, 0)
    assert time_to_cpa_s(state, horizon_s=20) == pytest.approx(10.0)
    time_s, miss_km = closest_approach_km(state, horizon_s=20)
    assert time_s == pytest.approx(10.0)
    assert miss_km == pytest.approx(0.0)
    result = screen_close_approach(state, threshold_km=5, horizon_s=20)
    assert result["status"] == "ELEVATED_GEOMETRY"
    assert result["screening_score"] == pytest.approx(1.0)


def test_receding_state_clamps_closest_approach_to_now() -> None:
    state = State(2, 0, 0, 1, 0, 0)
    assert time_to_cpa_s(state, horizon_s=20) == 0.0
    assert closest_approach_km(state, horizon_s=20) == pytest.approx((0.0, 2.0))


def test_geometry_helpers_and_compatibility_alias() -> None:
    assert miss_distance_km(State(3, 4, 0, 0, 0, 0)) == 5.0
    assert screening_score(2.5, 5.0) == pytest.approx(0.5)
    result = risk_index(State(2, 0, 0, 0.1, 0, 0), thresh_km=5)
    assert "risk" not in result
    assert "screening_score" in result
    assert result["collision_probability"] is False


def test_invalid_inputs_fail_closed() -> None:
    with pytest.raises(ValueError, match="finite"):
        screen_close_approach(State(math.nan, 0, 0, 0, 0, 0))
    with pytest.raises(ValueError, match="positive"):
        screen_close_approach(State(1, 0, 0, 0, 0, 0), threshold_km=0)
    with pytest.raises(ValueError, match="positive"):
        time_to_cpa_s(State(1, 0, 0, 0, 0, 0), horizon_s=-1)
