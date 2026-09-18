<div id="top"></div>

<div align="center">

  <a href="images/adaptive_pipeline.png">
    <img src="images/adaptive_pipeline.png" alt="Adaptive Coverage Path Planning pipeline" width="1000">
  </a>

  <h1 align="center">OverFOMO &mdash; Active Perception Coverage Path Planning</h1>

  <p align="center">
    <b>AP-CPP</b>: an active-sensing extension of the OverFOMO adaptive coverage planner
    that decides <i>where to look</i>, not only <i>how fast to fly</i>.
    <br />
    <a href="#quickstart">Quickstart</a> ·
    <a href="#key-features">Features</a> ·
    <a href="#algorithm">Algorithm</a> ·
    <a href="#project-tree">Project Tree</a> ·
    <a href="#cite-as">Citation</a> ·
    <a href="https://github.com/Xiangyuetang91/OverFOMO/issues">Report Bug</a>
  </p>
</div>

---

## Overview

This repository extends the **OverFOMO** adaptive coverage path planning scheme
(Krestenitis et al., *Robotics and Autonomous Systems*, 2023) for precision
agriculture. The published method flies a pre-computed boustrophedon route
open-loop and modulates the UAV's ground speed from the semantic content of the
incoming imagery: confident, sparse detections let the vehicle speed up, dense
or ambiguous canopies make it slow down.

That controller answers *how fast should I fly over this lane*. It does not
answer *is this lane the right place to spend my battery*. **AP-CPP adds the
missing degree of freedom.**

> **The gap AP-CPP closes.** A lawnmower sweep treats every square metre as
> equally interesting. Real fields are not: last season's yield map, an NDVI
> anomaly from a coarse overflight, a grower's report of a problem patch, and
> the gaps left by a previous flight all concentrate uncertainty in a handful of
> places. A robot that cannot deviate spends its endurance imaging ground it
> already understands, and returns with an unresolved map of the one region that
> mattered.

AP-CPP maintains an explicit **belief** over the operational area &mdash;
occupancy plus per-cell information entropy &mdash; and re-plans the remainder of
the route over a **receding horizon**, scoring every candidate with a utility
that trades tour cost against expected information gain.

The result is a planner that is *safe by construction*: on a field where nothing
in particular is known, a lateral-deviation term pins it to the published sweep,
so it degrades exactly to the baseline behaviour. It only leaves the corridor
when a genuine information hotspot outbids the detour.

<br />

<div align="center">
  <img src="results/ap_cpp_demo/ap_cpp_active.png" alt="AP-CPP mission diagnostics" width="880">
  <br />
  <em>Belief entropy before/after, accumulated coverage, and mission convergence.</em>
</div>

---

## Key Features

| Feature | Description |
|---|---|
| **Information-entropy belief grid** | Per-cell Shannon entropy over semantic classes, fused Bayesian-style from every observation. `ap_cpp/grid_model.py` |
| **Exact forward model** | The planner scores candidate views by replaying the *same* fusion kernel the runtime uses &mdash; no surrogate, no train/serve skew. Covered by `test_prediction_matches_the_realised_update`. |
| **Composite utility** | Information gain + frontier density − travel − heading change − revisit − corridor deviation, geometrically discounted over the horizon. `ap_cpp/utility.py` |
| **Rolling-horizon planning** | Receding-horizon (MPC-style) replanning: commit a short prefix, re-optimise from the pose actually reached. Robust to wind, controller lag and misdetections by construction. |
| **Mixed candidate set** | Reference sweep, entropy-biased A\* excursions that *rejoin* the route, an angular fan, and coverage repair &mdash; all normalised to a common horizon length before comparison. |
| **Sensor-aware footprints** | Oriented ground rectangle with cosine off-nadir efficiency falloff, matching the RedEdge-M payload in `main.py`. Swath/along-track axes are handled explicitly (`test_swath_is_the_cross_track_extent`). |
| **Baseline speed law preserved** | The published `G(confidence, coverage_ratio)` controller is reused bit-for-bit and extended with a bounded active-perception term. `ap_cpp/control.py` |
| **Two perception back-ends** | A dependency-free `DemoPerceptionModel` for CI/demos, and a `SegmentationPerceptionModel` adapter for the real U-Net + GDAL orthomosaic path. |
| **Zero-hardware demo** | Full mission with diagnostics and figures using only NumPy + matplotlib. No AirSim, no TensorFlow, no GDAL. |
| **AirSim integration** | `ap_cpp/airsim_driver.py` drops into the existing `parameters.py` configuration unchanged. |

---

## Quickstart

### 1. Requirements

The **core planner and demo need only NumPy** (matplotlib optional, for figures):

```sh
python -m pip install numpy matplotlib
```

The **simulation pipeline** (AirSim + segmentation + orthomosaic) additionally
needs the full stack from `requirements.txt`, including GDAL and TensorFlow:

```sh
python -m pip install -r requirements.txt
```

> GDAL is not on PyPI as a source distribution. Install a pre-built wheel
> matching your Python version from
> [here](https://www.lfd.uci.edu/~gohlke/pythonlibs/#gdal), then
> `python -m pip install path-to-wheel-file.whl`. Anaconda users should install
> `gdal` through conda instead.

### 2. Run the demo (one command, no simulator)

```sh
python demos/run_ap_cpp_demo.py --compare
```

This runs a complete active-perception coverage mission on a synthetic
L-shaped field with a no-fly zone, prints per-step diagnostics, runs the
reference-only ablation alongside it, and writes a JSON report plus four-panel
diagnostic figures to `results/ap_cpp_demo/`.

```
=== AP-CPP (active perception) ===
  termination            : max_steps
  observation steps      : 300
  flight distance        : 1821.5 m
  mean belief entropy    : 0.2384 (was 1.0000)
  coverage fraction      : 0.2076
  cumulative information : 63.101

=== Active perception vs reference-only ablation ===
metric                           active    reference      delta
------------------------------------------------------------------
flight distance [m]              1821.5       1767.7        +53.8
mean entropy                     0.2384       0.2560      -0.0176
coverage fraction                0.2076       0.2359      -0.0283
information gain                 63.101       28.299      +34.802
```

The honest reading of those numbers: **AP-CPP more than doubles the information
gathered per unit of flight** (+123%), and spends a small amount of coverage
throughput (−0.028) to do it. That is the trade the user is buying. To recover
coverage, raise the `frontier`/`information` weights, or run a longer mission.
On an *uninformative* prior the two configurations are identical by design &mdash;
see `test_active_mode_does_not_degrade_a_uniform_prior_mission`.

### 3. Run against the real field shipped in this repo

Uses `CPP/002/Polygon002.geojson` and the pre-baked `TurnWPs.txt` route,
converted through the same WGS84→NED path as the flight code:

```sh
python demos/run_ap_cpp_demo.py --source geojson --field 002 --compare
```

### 4. Run the test suite

```sh
python -m unittest discover -s tests -v
```

43 tests covering rasterisation, belief fusion, sensor geometry, the utility's
forward model, A\* and route resampling, speed-law conformance, mission
invariants and the WGS84 round trip. No simulator or TensorFlow required.

### 5. Useful demo flags

```sh
--source {synthetic,geojson}   # field geometry source
--prior {none,anomaly,two_zones}
                               # non-uniform uncertainty prior to explore
--prior-weight FLOAT           # prior blend, default 0.85
--horizon INT                  # rolling-horizon length, default 6
--execute-steps INT            # poses committed before replanning, default 2
--step-length FLOAT            # observation spacing, m, default 6.0
--entropy-bias FLOAT           # uncertainty discount in A* edge cost
--coverage-target / --entropy-target
--no-plots                     # skip figure generation
--compare                      # run the reference-only ablation
```

### 6. Fly it in AirSim

```sh
python -m ap_cpp.airsim_driver              # live flight
python -m ap_cpp.airsim_driver --dry-run    # validate config without Unreal
```

`--dry-run` exercises the full belief loop against the real orthomosaic without
connecting to the simulator &mdash; the fastest way to check that a field's
configuration is sane before launching Unreal.

---

## Algorithm

### Data flow

```
                      ┌───────────────────────────────────────────┐
   parameters.py ───▶ │  GeoBridge                                │
   *.geojson     ───▶ │  WGS84 ──▶ NED (handleGeo.ConvCoords)     │──▶ CoverageGrid
   TurnWPs.txt   ───▶ │  rasterise polygon + obstacles            │    (occupancy,
                      └───────────────────────────────────────────┘     entropy, coverage)
                                        │
                                        ▼
   ┌────────────────────────────────────────────────────────────────────────┐
   │  RollingHorizonPlanner.plan(pose)             ◀── receding horizon     │
   │                                                                        │
   │   candidate generators                    UtilityModel.evaluate_path   │
   │   ┌──────────────────────┐                ┌──────────────────────────┐ │
   │   │ reference_sweep      │                │ + w_i · IG(pose)         │ │
   │   │ frontier_astar_{k}   │─── scored ───▶ │ + w_f · frontier(pose)   │ │
   │   │ fan_{±θ}             │   against      │ − w_d · travel           │ │
   │   │ coverage_repair      │   the belief   │ − w_t · |Δyaw|           │ │
   │   └──────────────────────┘                │ − w_r · revisit          │ │
   │            ▲                              │ − w_c · corridor deviation│ │
   │            │                              └──────────────────────────┘ │
   │            │  entropy-biased A*, 8-connected, corner-cut-safe          │
   │            └───────────────────────────────────────────────────────────│
   │                                                                        │
   │   commit first `execute_steps` poses  ──▶  next replanning epoch       │
   └────────────────────────────────────────────────────────────────────────┘
                                        │
                                        ▼
   ┌────────────────────────────────────────────────────────────────────────┐
   │  SensorModel.observe(pose, measurement_entropy)                        │
   │  Bayesian multiplicative fusion:  H ← H · (1 − w · η · (1 − h_meas))   │
   │  Coverage accum.:                 C ← C + η · g · (1 − C)              │
   └────────────────────────────────────────────────────────────────────────┘
                                        │
                    ┌───────────────────┴───────────────────┐
                    ▼                                       ▼
        APCPPSpeedController                     Updated belief ──▶ next epoch
        baseline G(conf, cr) + active term
```

### The pieces

**1. Belief grid** (`ap_cpp/grid_model.py`)

A raster in the local NED frame already used by `handleGeo`. Each traversable
cell carries:

- `entropy ∈ [0, 1]` — normalised Shannon entropy of the semantic belief.
  `1.0` is "we know nothing", `0.0` is "the segmentation model is certain".
  This is the spatially-resolved generalisation of the published method's
  scalar *confidence level*.
- `coverage ∈ [0, 1]` — accumulated observation quality. `1.0` means imaged
  often enough, at a high enough ground sampling distance, to call it done.
  Generalises the scalar *coverage ratio*.

Non-traversable cells are sentinel-set (`entropy = 0`, `coverage = 1`) so they
never attract information-driven motion while remaining valid for path search.

**2. Fusion** — planned and realised updates are the *same code path*:

```
H_posterior = H_prior · (1 − w · η · (1 − h_meas))
```

where `w` is the in-frame weight, `η` the observation efficiency and `h_meas`
the frame entropy reported by the network. A perfect measurement of a
deterministic cell (`h_meas → 0`) collapses the belief; an uninformative one
(`h_meas → 1`) leaves it untouched. Because
`SensorModel.predict_update` calls `CoverageGrid.fuse` rather than reimplementing
it, the planner optimises the true objective, and a regression test pins that
equivalence.

**3. Sensor model** (`ap_cpp/sensor.py`)

The footprint is the oriented ground rectangle of a nadir camera. The
**swath** (cross-track extent) is computed from the 46° HFOV and is what sets
the lane spacing; the **along-track** extent is the 35° VFOV and sets the
observation cadence. Efficiency falls off as `1/sqrt(1 + t²)` with the
normalised off-nadir coordinate `t`, which is why *where* the robot looks
matters and not only *whether* it looked.

**4. Utility** (`ap_cpp/utility.py`)

```
U(π) = Σ_k γ^k [ w_i·IG(pose_k) + w_f·Φ(pose_k)
               − w_d·d(pose_{k−1}, pose_k)/L
               − w_t·|Δyaw|/180
               − w_r·mean C(pose_k)
               − w_c·dev(pose_k, corridor)/L ]
```

The belief is rolled forward on a scratch copy, so a candidate that re-observes
ground already covered by an earlier step of *its own* plan correctly earns no
second information gain.

**5. Rolling horizon** (`ap_cpp/planner.py`)

Candidate generators, each normalised to a common horizon length before
scoring &mdash; a candidate that stops early would otherwise bank no travel cost
for the leg it never flies and win by being lazy:

- `reference_sweep` — the published route, resampled by **arc length** at exact
  `step_length`. (Snapping to route *vertices* instead livelocks whenever the
  route is sampled more coarsely than `step_length` &mdash; which the shipped
  `TurnWPs.txt` files are. The projection is onto the nearest segment, not the
  nearest vertex.)
- `frontier_astar_{k}` — entropy-biased A\* out to a ranked uncertainty
  frontier, then a plain A\* leg back onto the route. The excursion is a single
  connected path so the travel penalty charges the full round trip; a detour
  that does not rejoin is a coverage hole, not a plan. Frontier ranking uses a
  summed-area integral image of uncertain density, so a lone stray cell cannot
  outrank a genuinely unobserved region.
- `fan_{±θ}` — straight marches on an angular fan, for the common case where a
  small heading tweak beats a detour.
- `coverage_repair` — shortest path to the most overdue cell by
  `(1 − C) / distance`.

Only the first `execute_steps` poses are committed. The next replan starts from
**the pose actually reached**, which is where wind, controller lag and model
error enter the loop and get corrected.

**6. Speed** (`ap_cpp/control.py`)

The published law is preserved exactly:

```
v = v_nominal + (1 − 2·cr_norm) · Q_max
```

AP-CPP adds a bounded active-perception term so the vehicle also slows where the
*map* is unresolved, even if the current frame happens to look confident:

```
v = clip(v_nominal + (1 − 2·cr_norm)·Q_max − Q_max·(tanh(IG/IG₀) + ρ·H̄))
```

---

## Project Tree

```
OverFOMO/
├── ap_cpp/                      # ← Active Perception Coverage Path Planning
│   ├── __init__.py              # Public API surface
│   ├── grid_model.py            # Belief raster: occupancy, entropy, coverage, fusion
│   ├── pose.py                  # Pose primitives, heading / angle helpers
│   ├── sensor.py                # Camera footprint, efficiency falloff, forward model
│   ├── utility.py               # Composite objective, path evaluation, G(x,y) bridge
│   ├── planner.py               # Rolling-horizon planner, A*, candidate generators
│   ├── control.py               # Baseline speed law + active-perception term
│   ├── runtime.py               # Perception back-ends (demo + U-Net adapter)
│   ├── mission.py               # Mission driver and flight log
│   ├── geo_bridge.py            # WGS84 ⇄ NED mission I/O, grid construction
│   └── airsim_driver.py         # AirSim integration (drop-in for main.py)
│
├── demos/
│   └── run_ap_cpp_demo.py       # One-command runnable demo + ablation + figures
│
├── tests/
│   └── test_ap_cpp.py           # 43 regression tests, NumPy-only
│
├── handleGeo/                   # WGS84 / NED / ECEF conversions (upstream)
│   ├── ConvCoords.py            #   coordinate frame conversions
│   ├── InPolygon.py             #   vectorised point-in-polygon
│   ├── NodesInPoly.py           #   BCD node generation (upstream)
│   ├── Dist.py
│   └── coordinates/
│       ├── WGS84.py
│       ├── NED.py
│       └── ECEF.py
│
├── CPP/                         # Per-field mission data
│   └── 002/
│       ├── Polygon002.geojson   #   operational polygon (QGIS export)
│       ├── TurnWPs.txt          #   pre-computed boustrophedon route (WGS84)
│       └── viewpoints_map.jpg
│
├── main.py                      # Original OverFOMO AirSim mission (upstream)
├── get_new_speed.py             # U-Net speed inference (upstream)
├── keras_tools.py, speed_function.py, check_g_func.py   # upstream analysis
├── parameters.py                # Mission configuration (paths, payload, speeds)
├── inputVariables.json          # Polygon/obstacle spec when QGIS=False
├── requirements.txt             # Full simulation-pipeline dependencies
├── results/                     # Mission outputs (generated)
├── images/, gif/                # Documentation assets
└── weights0500.hdf5             # Trained segmentation weights
```

---

## Configuration

All mission parameters live in `parameters.py`, exactly as for the original
pipeline. AP-CPP adds its own tunables through dataclasses, all of which have
sane defaults:

| Dataclass | Module | Purpose |
|---|---|---|
| `GridConfig` | `grid_model.py` | Raster resolution and extent |
| `SensorConfig` | `sensor.py` | FOV, altitude, efficiency falloff |
| `UtilityWeights` | `utility.py` | Objective term weighting |
| `PlannerConfig` | `planner.py` | Horizon, cadence, A\* knobs |
| `MissionConfig` | `mission.py` | Termination criteria |

Two knobs matter most in practice:

- **`UtilityWeights.corridor`** (default `1.15`) — how strongly to hold the
  published route. Raise it for conservative operations, lower it to let the
  planner chase information harder.
- **`UtilityWeights.information`** (default `1.0`) — the value of a unit of
  uncertainty resolved, relative to a metre of flight.

---

## Controlling a Real Field: an Agronomic Prior

The planner is most useful when seeded with an uncertainty prior, because a
robot with no prior knowledge has nothing to be curious *about*. Use
`CoverageGrid.set_uncertainty_prior`:

```python
from ap_cpp.grid_model import CoverageGrid
from ap_cpp.geo_bridge import GeoBridge, load_qgis_polygon

polygon, obstacles, _ = load_qgis_polygon("CPP/002/Polygon002.geojson")
grid = GeoBridge(polygon, obstacles).build_grid(resolution=2.5)

# Any [0, 1] risk layer at the raster's shape: last season's yield map, an
# NDVI anomaly, a scout's report, gaps from the previous flight.
grid.set_uncertainty_prior(ndvi_anomaly_normalised, weight=0.85)
```

Then hand the grid to a mission exactly as the demo does.

---

## Contributing

Contributions are welcome. The workflow is standard:

1. Fork the project
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Run the suite before pushing (`python -m unittest discover -s tests`)
4. Commit using [Conventional Commits](https://www.conventionalcommits.org/)
   (`feat(planner): ...`, `fix(sensor): ...`, `docs: ...`)
5. Open a pull request

Please keep `ap_cpp/` free of simulator and TensorFlow imports at module scope
&mdash; the demo and the test suite must keep running on a bare NumPy install.

---

## License

Distributed under the MIT License. See [LICENSE](LICENSE) for more information.

---

## Cite As

If you use the AP-CPP module, please cite the underlying OverFOMO work it
extends:

*M. Krestenitis, E. K. Raptis, A. C. Kapoutsis, K. Ioannidis, E. B. Kosmatopoulos,
and S. Vrochidis, "Overcome the fear of missing out: Active sensing UAV scanning
for precision agriculture," Robotics and Autonomous Systems, p. 104581, 2023.*
[[Link]](https://www.sciencedirect.com/science/article/pii/S0921889023002208)

```bibtex
@article{krestenitis2023overcome,
  title={Overcome the fear of missing out: Active sensing UAV scanning for precision agriculture},
  author={Krestenitis, Marios and Raptis, Emmanuel K and Kapoutsis, Athanasios Ch and Ioannidis, Konstantinos and Kosmatopoulos, Elias B and Vrochidis, Stefanos},
  journal={Robotics and Autonomous Systems},
  pages={104581},
  year={2023},
  publisher={Elsevier}
}
```

---

## Acknowledgments

This research has been financed by the European Regional Development Fund of the
European Union and Greek national funds through the Operational Program
Competitiveness, Entrepreneurship and Innovation, under the call RESEARCH –
CREATE – INNOVATE (T1EDK-00636).

The AP-CPP extension builds on the upstream
[Adaptive_Coverage_Path_Planning](https://github.com/emmarapt/Adaptive_Coverage_Path_Planning)
repository, whose `handleGeo` coordinate machinery, RedEdge-M payload
specification and segmentation network are reused unchanged.

<p align="right">(<a href="#top">back to top</a>)</p>
