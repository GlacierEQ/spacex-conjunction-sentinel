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


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_public_front_door_is_independent_and_bounded() -> None:
    readme = read("README.md")
    assert "Not affiliated with, endorsed by, or connected to SpaceX" in readme
    assert PY_TOKEN in readme
    assert RUST_TOKEN in readme
    assert "not a probability of collision" in readme
    assert "not SGP4/SDP4" in readme


def test_python_and_rust_source_remove_false_authority() -> None:
    python_source = read("src/conjunction.py")
    rust_source = read("src/orbit_propagator.rs")
    assert PY_TOKEN in python_source
    assert RUST_TOKEN in rust_source
    assert "probability_of_collision" not in rust_source
    assert "pub struct SGP4Propagator" not in rust_source
    assert "SGP4/SDP4 Orbit Propagator" not in rust_source
    assert "collision_probability\": False" in python_source
    assert "maneuver_authority\": False" in python_source


def test_machine_capabilities_are_exact_allowlist() -> None:
    payload = json.loads(read("machine/capabilities.json"))
    assert payload["evidence_state"] == PY_TOKEN
    assert payload["capabilities"] == APPROVED_CAPABILITIES


def test_machine_state_requires_external_current_head_receipts() -> None:
    state = json.loads(read("machine/excellence-state.json"))
    assert state["state"] == "TESTED"
    assert state["principal_state"] == "TESTED"
    assert state["evidence_state"] == PY_TOKEN
    assert state["gates"]["PYTHON_GEOMETRY_PROOF"] == "REQUIRES_CURRENT_HEAD_RECEIPT"
    assert state["gates"]["RUST_NATIVE_REFERENCE_PROOF"] == "REQUIRES_CURRENT_HEAD_RECEIPT"
    assert state["gates"]["COLLISION_PROBABILITY_AUTHORITY"] == "NOT_CLAIMED"
    assert state["gates"]["MANEUVER_COMMAND_AUTHORITY"] == "NOT_CLAIMED"
    assert state["gates"]["SGP4_SDP4_VALIDATION"] == "NOT_PROVEN"
    assert state["proof_receipt"]["state"] == "EXTERNAL_EXACT_HEAD_RECEIPT_REQUIRED"


def test_target_contract_is_tested_and_requires_rust_native_proof() -> None:
    contract = json.loads(read("machine/target-contract.json"))
    assert contract["current"] == {
        "state": "TESTED",
        "implemented": True,
        "tested": True,
        "deployed": False,
    }
    assert contract["evidence_state"] == PY_TOKEN
    assert contract["proof_contract"]["python_versions"] == ["3.11", "3.13"]
    assert contract["proof_contract"]["rust_native_test_required"] is True
    assert contract["proof_contract"]["exact_canonical_head_required"] is True
