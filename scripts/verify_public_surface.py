from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PY_TOKEN = "LOCAL_CLOSE_APPROACH_GEOMETRY_NOT_COLLISION_AVOIDANCE_AUTHORITY"
RUST_TOKEN = "LOCAL_J2_KEPLER_REFERENCE_NOT_SGP4_OR_COLLISION_AVOIDANCE_AUTHORITY"
APPROVED_CAPABILITIES = [
    "deterministic-local-relative-motion-close-approach-screening",
    "bounded-geometric-proximity-scoring-not-collision-probability",
    "native-rust-simplified-j2-kepler-reference",
]


def text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def main() -> int:
    readme = text("README.md")
    python_source = text("src/conjunction.py")
    rust_source = text("src/orbit_propagator.rs")
    capabilities = json.loads(text("machine/capabilities.json"))
    excellence = json.loads(text("machine/excellence-state.json"))
    contract = json.loads(text("machine/target-contract.json"))

    assert "Not affiliated with, endorsed by, or connected to SpaceX" in readme
    assert PY_TOKEN in readme and PY_TOKEN in python_source
    assert RUST_TOKEN in readme and RUST_TOKEN in rust_source
    assert "not a probability of collision" in readme
    assert "not SGP4/SDP4" in readme

    for forbidden in (
        "Autonomous conjunction assessment and collision avoidance maneuver planning",
        "TLE/ephemeris processing for tracking 30,000+ cataloged objects",
        "Probability of Collision",
        "MCP Tool Exposure",
        "ABORT_TRAJECTORY",
    ):
        assert forbidden not in readme
        assert forbidden not in python_source

    assert "probability_of_collision" not in rust_source
    assert "pub struct SGP4Propagator" not in rust_source
    assert "SGP4/SDP4 Orbit Propagator" not in rust_source

    assert capabilities["evidence_state"] == PY_TOKEN
    assert capabilities["capabilities"] == APPROVED_CAPABILITIES
    assert excellence["state"] == "TESTED"
    assert excellence["principal_state"] == "TESTED"
    assert excellence["evidence_state"] == PY_TOKEN
    assert excellence["gates"]["COLLISION_PROBABILITY_AUTHORITY"] == "NOT_CLAIMED"
    assert excellence["gates"]["MANEUVER_COMMAND_AUTHORITY"] == "NOT_CLAIMED"
    assert excellence["gates"]["SGP4_SDP4_VALIDATION"] == "NOT_PROVEN"
    assert excellence["proof_receipt"]["state"] == "EXTERNAL_EXACT_HEAD_RECEIPT_REQUIRED"
    assert contract["current"]["state"] == "TESTED"
    assert contract["evidence_state"] == PY_TOKEN
    assert contract["proof_contract"]["rust_native_test_required"] is True
    assert contract["next_gate"] == "exact-current-head Python plus native Rust proof receipts"

    print(
        json.dumps(
            {
                "status": "PASS",
                "python_evidence_state": PY_TOKEN,
                "rust_evidence_state": RUST_TOKEN,
                "capabilities": APPROVED_CAPABILITIES,
                "collision_probability_proven": False,
                "maneuver_authority": False,
                "sgp4_sdp4_proven": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
