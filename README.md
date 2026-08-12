# Conjunction Sentinel — close-approach risk reference

> Independent GlacierEQ portfolio implementation for bounded conjunction-assessment mechanics. This is not SpaceX software, employment work, flight-certified software, or evidence of SpaceX adoption.

## What is implemented

The repository currently contains three concrete mechanism surfaces:

- `src/conjunction.py` — a deterministic Python reference model for relative-state miss distance, relative speed, coarse time-to-closest-approach, and threshold-based `CLEAR` / `WATCH` / `CRITICAL` risk classification.
- `src/orbit_propagator.rs` — a standalone Rust orbital-state reference with a TLE data model, J2 secular-rate adjustments, Kepler iteration, ECI state output, and miss-distance / relative-velocity helpers.
- `src/promotion_authority.py` — a local proof-bound promotion-authority mechanism that binds a grant to repository/proof identifiers and expiration semantics.

The Python mechanism intentionally uses a simplified linear relative-motion approximation. The Rust propagator is an educational/reference implementation; despite historical naming in the file, this repository does **not** claim validated or standards-conformant SGP4/SDP4 behavior.

## Current proof state

Canonical source observed before this repair: `ec3a8d5e088d0dcc454cfb37fd4216e29c8642a9`.

Repository-native CI run `31362141879` executed against that exact revision and **failed at the static-quality gate before pytest ran**. Ruff reported 63 findings. Therefore historical local test/operate receipts do not certify that current revision, and this repository is not currently presented as `TESTED`, `ADVERSARIAL_VERIFIED`, or `PROMOTED`.

The current machine-readable truth is recorded in:

- `machine/current-excellence-receipt.json`
- `machine/excellence-state.json`

The next gate is to repair the current static-quality failure without weakening verification, then execute deterministic and adversarial tests against the exact repair SHA.

## Direct architecture

```text
relative state --------------------------> Python bounded risk classifier
  x/y/z + vx/vy/vz                         miss distance
                                           relative speed
                                           coarse TCA
                                           threshold status

TLE-like orbital elements --------------> Rust reference propagator
                                           J2 secular adjustments
                                           Kepler iteration
                                           ECI state vector
                                           pairwise miss distance / relative velocity

proof receipt + requested promotion ----> local promotion authority
                                           exact receipt binding
                                           expiry / integrity checks
                                           allow or refuse
```

## Verification targets

The repository contains deterministic and adversarial test surfaces under `tests/`, plus Rust unit tests embedded in `src/orbit_propagator.rs`. They become current proof only when a repository-native run passes against the exact source revision being claimed.

```bash
python -m pytest -q
rustc --test src/orbit_propagator.rs -o /tmp/conjunction-orbit-tests
/tmp/conjunction-orbit-tests
```

## Explicit nonclaims

The following are **not** currently implemented or proven by this repository and must not be inferred from earlier README language:

- Monte Carlo probability-of-collision analysis
- Alfano probability-of-collision implementation
- covariance-calibrated collision probability
- automated collision-avoidance maneuver optimization
- production TLE/catalog ingestion for tens of thousands of objects
- an MCP server or deployed `conjunction_screen` tool
- reinforcement-learning maneuver optimization
- production alert/event-bus integration
- flight certification, production deployment, operational scale, or external adoption

Those items may be future evolution candidates only after implementation and exact-SHA proof.

## Role in the estate

Current role: **active specialist component pending canonical-family position**.

Its consequential implemented value is a compact conjunction-assessment reference combining a simple Python close-approach classifier, a Rust orbital propagation surface, and proof-bound local promotion semantics. Whether it remains a distinct canonical SpaceX-family specialist must be established by direct comparison with adjacent family repositories; repository naming alone does not decide that question.
