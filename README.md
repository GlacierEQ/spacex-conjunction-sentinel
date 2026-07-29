# SpaceX Conjunction Sentinel — Space Debris Collision Avoidance 🛰️

> **Autonomous conjunction assessment and collision avoidance maneuver planning for orbital assets.**

[![Python](https://img.shields.io/badge/Python-3.9+-blue)]()
[![Rust](https://img.shields.io/badge/Rust-Safety%20Critical-orange)]()
[![Domain](https://img.shields.io/badge/Domain-Orbital%20Safety-red)]()

---

## 🎯 For Recruiters & Hiring Managers

This repository implements a **space debris collision avoidance system** — the software that protects satellites and spacecraft from catastrophic collisions with orbital debris. It demonstrates:

- **Probabilistic risk assessment** using Monte Carlo conjunction analysis
- **Automated maneuver planning** with delta-V optimization for collision avoidance burns
- **TLE/ephemeris processing** for tracking 30,000+ cataloged objects
- **Real-time alert pipeline** with configurable probability-of-collision (Pc) thresholds

**Why this matters**: Space sustainability is a trillion-dollar problem. With 30,000+ tracked objects and growing, conjunction assessment requires the same probabilistic reasoning, real-time data fusion, and safety-critical decision-making used in autonomous vehicles and financial risk management.

---

## 🔬 For Engineers & Technical Reviewers

### Architecture

```
TLE Catalog ──→ Orbit Propagator ──→ Conjunction Screen
                     │                       │
              SGP4/SDP4 (Rust)      Probability of Collision
                     │                       │
              State Vectors ──→ Miss Distance + Pc ──→ Alert/Maneuver
```

### Core Components

| Component | Language | Purpose |
|---|---|---|
| `src/conjunction_engine.py` | Python | CDM processing, Pc calculation, alert generation |
| `src/orbit_propagator.rs` | Rust | SGP4/SDP4 orbit propagation with compile-time safety |
| `tests/` | Python | Conjunction scenario validation with known CDM datasets |

### Key Algorithms

- **Alfano probability of collision**: 2D projected covariance with Gaussian miss distance
- **SGP4 propagation**: NORAD two-line element set propagation for LEO/MEO objects
- **Covariance realism**: Mhalanobis distance scaling for covariance consistency

---

## 🤖 ML/AI & Programmatic Mesh Integration

### Agent Mesh Connectivity

- **MCP Tool**: `conjunction_screen(norad_id, hours_ahead)` — exposes screening as an agent-callable tool
- **Mastermind Sidecar**: Publishes conjunction alerts to the APEX Highway event bus
- **SHA-256 Integrity**: Cryptographic hash verification via `.integrity/file_hashes.json`

### AI/ML Extension Points

- **Maneuver Optimization**: Reinforcement learning for optimal avoidance burn timing and magnitude
- **Covariance Calibration**: Neural network covariance realism correction from historical CDM accuracy
- **Debris Cloud Prediction**: GNN-based fragmentation debris propagation for post-breakup conjunction assessment

```python
# Agent mesh query
alerts = await mcp_client.call_tool("conjunction-sentinel", "screen_all", {"hours": 72})
# Returns: [{"norad_id": 25544, "pc": 1.2e-4, "tca": "2026-07-30T12:00Z", "action": "MONITOR"}]
```

---

## ⚡ Quick Start

```bash
python3 src/conjunction_engine.py
python3 tests/test_conjunction.py
```
