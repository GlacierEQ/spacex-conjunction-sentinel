# Conjunction Sentinel

**Deterministic collision-risk assessment and maneuver-review simulation for orbital conjunctions.**

This repository is an independent GlacierEQ portfolio project. It is not affiliated with SpaceX and does not provide flight authority, operational maneuver commands, mission-certified orbit determination, or a production collision-avoidance service.

## What it actually does

The repository now implements the mechanisms its earlier README merely advertised:

- analytic time of closest approach under a constant-relative-velocity encounter model;
- deterministic 2D Monte Carlo probability-of-collision estimation from a positive-definite encounter-plane covariance;
- configurable review-level alert classification;
- a minimum linearized impulse maneuver-review planner with explicit delta-v limits;
- deterministic pairwise small-catalog screening;
- NORAD TLE field parsing plus a clearly labeled **two-body** Kepler propagator in Python;
- an independent no-dependency Rust two-body orbit kernel with native tests;
- a JSON CLI surface suitable for reproducible local automation.

The previous artificial confidence floor and placeholder `hyper-scaling` capability are not part of the functional system.

## Run it

```bash
python -m pip install -e .
conjunction-sentinel demo
pytest -q
cargo test
```

The CLI emits `glaciereq.conjunction-assessment.v1` JSON and always records `operational_authority: false`.

## Architecture

```text
Relative Cartesian states
        |
        +--> analytic CPA --> encounter-plane miss vector
        |                           |
        |                           +--> covariance + HBR
        |                                      |
        |                                      v
        |                           deterministic Monte Carlo Pc
        |                                      |
        |                                      v
        |                               review-level alert
        |
        +--> linearized impulse review --> bounded delta-v proposal

TLE fields --> Python two-body propagator
           --> Rust two-body orbit kernel
```

## Functional surfaces

| Surface | Implemented behavior |
|---|---|
| `src/conjunction.py` | CPA, covariance validation, Monte Carlo Pc, maneuver review, catalog screening, TLE parsing, two-body propagation, alerts, JSON CLI |
| `src/orbit_propagator.rs` | Validated no-dependency Rust two-body propagation and relative-state utilities |
| `tests/test_conjunction.py` | Determinism, validation, CPA, probability, maneuver, catalog, TLE, propagation, alert and truth-boundary tests |
| `Cargo.toml` | Native Rust crate boundary |
| `pyproject.toml` | Installable Python module and CLI |
| `.github/workflows/ci.yml` | Native Python 3.11-3.13 and Rust verification |

## Models and limits

### Conjunction model

The Python assessment computes closest approach from linear relative motion. At TCA it constructs an encounter plane normal to relative velocity and estimates collision probability by seeded Gaussian Monte Carlo sampling against a hard-body radius.

That makes the result reproducible and testable. It does **not** make it equivalent to an operational conjunction-data-message pipeline.

### Maneuver review

`plan_avoidance_maneuver` computes a minimum linearized impulse needed to increase miss distance to a requested target over a specified coast interval. It applies a hard maximum delta-v and marks the plan infeasible if that limit cannot meet the requested miss distance.

It is a review artifact, not a spacecraft command.

### Orbit propagation

Both language boundaries implement a two-body Kepler model derived from TLE mean elements. They are deliberately not labeled SGP4/SDP4. Full perturbation propagation, covariance transport, maneuver execution, catalog ingestion and operational ephemeris services remain outside the verified scope.

### Catalog screening

`screen_catalog` is deterministic O(n^2) pairwise screening for small local catalogs. The repository makes no claim of real-time screening of the full public catalog or any particular object count.

## Example Python API

```python
from conjunction import Covariance2D, State, assess_conjunction

primary = State(0, 0, 0, 0, 0, 0)
secondary = State(1, 0.01, 0, -0.001, 0, 0)

assessment = assess_conjunction(
    primary,
    secondary,
    Covariance2D(xx=0.0004, xy=0.0, yy=0.0004),
    horizon_s=2000,
    samples=5000,
    seed=7,
)
print(assessment)
```

## Machine contract

```yaml
schema: glaciereq.readme.v1
repository: GlacierEQ/spacex-conjunction-sentinel
purpose: deterministic local orbital-conjunction assessment and maneuver-review simulation
state: FUNCTIONAL_CANDIDATE
languages:
  python:
    role: assessment orchestration, probability, screening, CLI
  rust:
    role: validated two-body orbital propagation kernel
verified_capabilities_after_ci:
  - analytic constant-relative-velocity CPA
  - deterministic encounter-plane Monte Carlo collision probability
  - bounded linearized maneuver review
  - small-catalog pairwise screening
  - TLE mean-element parsing
  - Python two-body propagation
  - Rust two-body propagation
  - deterministic JSON assessment CLI
nonclaims:
  - no SpaceX affiliation
  - no operational spacecraft authority
  - no maneuver execution
  - no flight certification
  - no SGP4/SDP4 claim
  - no production catalog-scale throughput claim
  - no live MCP or APEX event-bus integration claim
```

## Verification standard

A promotion is earned only when the exact reviewed head passes:

- Python 3.11, 3.12 and 3.13 installation, compile, test and CLI smoke;
- Rust formatting and native unit tests;
- the aggregate required quality job.

Green metadata without executable behavior is not accepted as proof.
