# Close-Approach Geometry Sentinel

> **Independent portfolio project for deterministic local relative-motion screening. Not affiliated with, endorsed by, or connected to SpaceX.**

This repository demonstrates a bounded **close-approach geometry screen** using caller-supplied relative position/velocity plus a separate Rust J2/Kepler reference model. It is an engineering exhibit, not an operational conjunction-assessment or collision-avoidance system.

## Recruiter surface

The proven capability is deliberately narrower and more defensible than the historical README:

- analytic closest-point-of-approach time under constant relative velocity;
- deterministic closest-separation and relative-speed calculation;
- bounded geometric proximity scoring that is **not a probability of collision**;
- fail-closed handling of non-finite and invalid thresholds/horizons;
- a typed Rust two-body/J2 reference with native tests;
- explicit separation between local screening output and maneuver/control authority.

## Engineering surface

| Surface | What is actually established |
|---|---|
| `src/conjunction.py` | Executable local constant-relative-velocity close-approach geometry and bounded screening score |
| `src/orbit_propagator.rs` | Simplified Rust J2/Kepler orbit-reference model; **not SGP4/SDP4** |
| `tests/test_conjunction.py` | Python geometry, fail-closed input, and no-authority regression tests |
| `tests/test_adversarial.py` | Existing generic adversarial/import boundary tests |
| `scripts/verify_public_surface.py` | Fail-closed public/machine truth verifier |

### Evidence states

Python:

`LOCAL_CLOSE_APPROACH_GEOMETRY_NOT_COLLISION_AVOIDANCE_AUTHORITY`

Rust:

`LOCAL_J2_KEPLER_REFERENCE_NOT_SGP4_OR_COLLISION_AVOIDANCE_AUTHORITY`

The historical Python function name `risk_index()` remains only as a compatibility alias. Its result no longer exposes a pseudo-probability or generic `risk` field; it returns the same bounded geometric screening receipt as `screen_close_approach()`.

## Machine proof

Run:

```bash
python -m pytest -q
python scripts/verify_public_surface.py
rustc --edition=2021 --test src/orbit_propagator.rs -o /tmp/orbit-reference-tests
/tmp/orbit-reference-tests
```

Repository CI binds Python 3.11/3.13 and native Rust test execution to the exact reviewed/current source SHA.

## Explicit nonclaims

This repository establishes **none** of the following:

- SpaceX affiliation, endorsement, employment, proprietary data, or internal systems access;
- live NORAD/TLE/CDM catalog ingestion, 30,000-object screening, or real-time alerting;
- SGP4/SDP4 implementation or validated orbit determination/propagation accuracy;
- Alfano, Monte Carlo, Gaussian covariance, Mahalanobis covariance-realism, or calibrated probability-of-collision calculation;
- collision diagnosis, operational conjunction assessment, autonomous avoidance-burn planning, delta-V optimization, or maneuver authority;
- real spacecraft, telemetry, command, flight computer, or safety-critical operation;
- MCP tool exposure, APEX event-bus publication, live agent/provider/mesh integration, or production deployment;
- RL maneuver optimization, neural covariance calibration, or GNN debris-cloud prediction. Those were historical extension ideas, not implemented proof.

## Next proof gate

A legitimate next expansion would require a separately validated propagator/catalog/covariance pipeline with known reference vectors and independent numerical error bounds. Until then, the public capability remains **local deterministic close-approach geometry plus a simplified Rust J2/Kepler reference**.
