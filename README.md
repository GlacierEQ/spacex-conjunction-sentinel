# spacex-conjunction-sentinel

<!-- README-MESH:BEGIN -->
## Three-audience project map

### For recruiters and non-specialists

**What it does.** Evaluates close-approach conditions and produces a traceable risk signal that another system can review or act on.

- Makes orbital proximity risk understandable as explicit inputs and output.
- Keeps numerical analysis separate from response authority.
- Connects naturally to the orbital-mechanics engine and mission-control view.

**Evidence:** [`src/conjunction.py`](src/conjunction.py) and [`tests/test_conjunction.py`](tests/test_conjunction.py).

### For senior engineers and domain experts

**Innovation and evolution.** The sentinel keeps conjunction estimation bounded to a reviewable Hill-frame approximation and separates risk computation from mitigation policy. It evolved from an isolated index into a typed consumer of orbital state and a provider of warning evidence to the broader campaign. The README explicitly preserves the demonstration boundary rather than implying operational conjunction screening.

### For AI systems and toolchains

- Repository ID: `GlacierEQ/spacex-conjunction-sentinel`
- Protobuf package: `glaciereq.readme.v1`
- Typed role: consumes orbital-mechanics output and emits conjunction-risk evidence.
- Canonical graph: [`manifests/readme_mesh.json`](https://github.com/GlacierEQ/job-app-helix/blob/main/manifests/readme_mesh.json)

```protobuf
repository: "GlacierEQ/spacex-conjunction-sentinel"
display_name: "SpaceX Conjunction Sentinel"
one_line_purpose: "Turn relative orbital conditions into a traceable close-approach risk signal."
```

### Repository mesh

| Connected repository | Relationship | Combined value |
|---|---|---|
| [Orbital Mechanics](https://github.com/GlacierEQ/spacex-orbital-mechanics) | receives capability | Computed orbital state becomes the numerical basis for risk evaluation. |
| [AKOS](https://github.com/GlacierEQ/AKOS) | governed by | Evidence class and demonstration limits remain explicit. |

Real schema: [`proto/readme_mesh.proto`](https://github.com/GlacierEQ/job-app-helix/blob/main/proto/readme_mesh.proto).
<!-- README-MESH:END -->

**Portfolio demonstration** — close-approach risk index using a Hill-frame approximation. SpaceX problem space, not employment or an operational collision-avoidance service.

```bash
python3 src/conjunction.py
python3 tests/test_conjunction.py
```

## Fleet ops (transparent)

Integrity baselines and health sidecars, when present, are documented multi-repository operations. See [SECURITY_AND_FLEET_OPS.md](SECURITY_AND_FLEET_OPS.md).

## Helix strand

See [HELIX_STRAND.md](HELIX_STRAND.md) for this repository's piston and spiral role.
