# Plan: Finish FYP B28 Implementation

## Context

Goal: finish Module 1 so the airspace graph is realistic, harden Tier 1 and Tier 2, rerun the evaluation, and produce saved outputs for the Review 2 report. Wind integration comes first: it is already designed, touches little code, and makes energy and risk visibly change with conditions.

**Base branch:** `origin/meghna` holds the latest implementation work; `main` does not have it. Step 0 merges `meghna` into the working branch, and every phase builds on that branch.

### Implementation status (on `meghna`)

Already in place:
- **Tier 1:** NAMOA*/MO-A* with label dominance, Pareto pruning, persistence and plots (`module2_tier1_moastar.py:120`). Plots are relabelled "Cumulative Risk".
- **Tier 2:** payload weight formula, P ≥ 4 amplification, min-max scoring, argmax selection (`module4_tier2_payload.py:90`), plus `selected_index()` and `classify_point()`.
- **Baselines:** `module3_baselines.py` has single-objective A* (distance, energy, risk) and a compact MOPSO (path particles, prefix + weighted-shortest-suffix repair).
- **Metrics:** `module5_evaluation.py` has exact 3D HV, IGD, spread Δ, a union reference front, `objective_ranks` and `tier2_evidence` (ranks, point type, latency, unique routes).
- **Runner:** `run_benchmarks.py:36` runs S1/S2/S3 synthetic scenarios and writes `outputs/benchmarks/*.csv|json|png`.
- **Tests:** `python -m unittest discover -s Implementation/tests` → 11 tests pass. Tests use **unittest**; keep that framework.

### Remaining work (verified in code)

- **Module 1 isn't realistic yet.**
  - Base `fetch_graph()` always builds the synthetic grid, and OSM support is marked deferred (`module1_graph_builder.py:81`). `_fetch_from_osm()` is dead and broken.
  - The OSM demo uses the road "drive" graph, so the UAV can only follow roads. Its energy comes only from `_energy_cost(d, data)`, which uses a random grade; there is no terrain pipeline (`run_demo_osm.py:138`).
  - OSM risk is two made-up obstacle circles plus hash noise. There is no wind, weather, buildings, no-fly zones or population/road exposure (`run_demo_osm.py:157`).
  - On `meghna`, synthetic energy is a hand-made field (`_synthetic_energy_cost`, "diagonal headwind band"), not real wind.
- **Wind plan has bugs**, and the copy on `meghna` still has them:
  1. The headwind sign is inverted: wind FROM 90° on an edge heading 90° comes out as tailwind.
  2. It uses the UTC hour to index an IST series, which is 5.5 h off.
  3. The coordinates in the endpoint example (12.9020, 80.1322) don't match the constants (12.9829, 80.2555).
  4. Edge bearing has no `cos(lat)` scaling.
  5. It uses 10 m wind, but the drone flies at 60–120 m.
  6. `ρ_wind` is the same on every edge, so it only penalises hop count.
- **Tier 1 gaps:**
  - The scalar heuristic `‖g‖+h` uses `×111` for longitude without `cos(lat)`, so it overestimates by about 2.6%.
  - Front pruning uses `g`, not `g+h`.
  - The parent dict is copied on every push.
  - A truncated search (hitting `max_explored`) is not reported. S3 probably hits the cap: Emergency Medicine picks distance rank 22 in `tier2_evidence.csv`.
  - There is no refresh metadata or cache invalidation.
- **Tier 2 gaps:** no F/W/U/P range validation, no JSON/CSV output of the selected route, no battery feasibility check for payload mass, no weather go/no-go gate.
- **Evaluation gaps:**
  - MOPSO defaults (24 particles / 30 iterations / 2 reps) don't match the report (100 / 200 / 30 reps).
  - The synthetic scenarios have no obstacle-density parameter (report: 5/20/40%).
  - No calm-vs-windy comparison, no risk-component table, no 225-combination sweep, no monotonicity check.
- **Other gaps:**
  - `requirements.txt` lacks `requests`.
  - No mission file export, dashboard, or SITL/MAVLink.
- **Docs:**
  - `AGENTS.md` still describes the Tier 2 tags as F/T/U ("temp sensitivity") and says "module numbering skips 3". `meghna` now has `module3_baselines` and `module5_evaluation`.
  - The report's module list (M3 = Tier 2, M4 = Testing) doesn't match the code (M3 = baselines, M4 = Tier 2, M5 = evaluation). Align the Review 2 report to the code.
  - Keep **payload weight class W** everywhere. The frozen abstract's "temperature sensitivity" gets a one-line footnote.

### Decisions already made

- Grid-lattice airspace graph, not the road network.
- Include the front store with refresh, and the unit tests.
- Tier 2 evaluation uses payload-alignment score, selected objective rank, non-extreme count and latency instead of "accuracy".
- Don't renumber files.

**Network:** this cloud sandbox blocks `api.open-meteo.com`, and osmnx isn't installed here. Every fetcher needs a disk cache under `cache/` plus offline fixtures in `tests/fixtures/`. Live-API runs happen on the team's machine.

### Risk components, in priority order

| # | Component | Data | Edge model (each term is in [0, 1]) |
|---|---|---|---|
| 1 | Wind speed + direction | Open-Meteo `/v1/forecast` (`wind_speed_80m`, `wind_direction_80m`; 10 m with power law `(h/10)^0.25` as fallback) | energy: headwind/crosswind penalty; risk: crosswind term |
| 2 | Terrain/elevation gradient | `ElevationProvider` (Phase 2) | energy: real `|grade|`; risk: terrain-clearance and sudden altitude change `clip(|grade|/grade_max)` |
| 3 | Building/obstacle proximity | OSM buildings: `height=*`, else `building:levels=* × 3 m`, else a per-type default | fraction of along-edge samples within 30 m of a footprint, scaled by vertical margin |
| 4 | Gust / crosswind instability | `wind_gusts_10m` | `γ·clip((CW + k_g·(gust − v))/V_MAX_SAFE)` |
| 5 | Visibility + precipitation | `visibility`, `precipitation`, `weather_code` | uniform weather term, plus a global NO-GO for thunderstorm codes 95–99, heavy rain, low visibility or gusts above the limit (NE monsoon, Oct–Dec) |
| 6 | Road/people exposure | OSM land use, major roads, schools/hospitals/markets | SORA-style class lookup, averaged along the edge |
| 7 | Restricted/no-fly zones | OSM `aeroway`, `military`, `landuse=military`, prisons, protected areas, government; manual list (Raj Bhavan, Guindy NP, Digital Sky airport buffers) | hard removal inside a zone; soft buffer `exp(−dist/λ)` outside it |
| 8 | Communication/GPS risk | building density/height (urban canyon) | density × mean height normalised |
| + | Over water (payload loss) | OSM `natural=water`, `waterway`, coast | fraction of edge over water (Adyar estuary, Buckingham Canal, sea) |
| + | Urban turbulence | building density × wind | density × `v/V_MAX_SAFE` |

---

## Phase 0: Sync + housekeeping

- **Done.** `meghna` was cleaned (`.DS_Store` files and the stale `Review 1/` folder removed, `.gitignore` fixed) and merged into `main` via PR #1. The 11 tests pass.
- Add `Implementation/config.py` for all tunables:
  - region centre (Thiruvanmiyur 12.9830, 80.2594), box size, lattice spacing
  - `H_CRUISE_AGL=60`, `CLEARANCE=15`, `H_MAX_AGL=120` (Drone Rules 2021)
  - wind constants: `V_CRUISE_KMH=36`, `BETA_HEAD=0.40`, `BETA_CROSS=0.15`, `GAMMA_WIND=0.30`, `V_MAX_SAFE_KMH=40`, `ALPHA_SHEAR=0.25`
  - risk weights, battery/mass params, drift thresholds
- `requirements.txt`: add `requests` and `shapely`, and pin `osmnx>=1.9`. Keep the in-house HV; don't add pymoo.

## Phase 1: Module 1 complete (shared pipeline + lattice + wind)

- **One shared cost pipeline:** `EdgeFeatures` (static per-edge features) → `apply_weather(G, snapshot)` → `energy_cost()` / `risk_components()`. The synthetic builder and the OSM lattice builder both call it.
  - Remove the dead `_fetch_from_osm`, the random grade, the hash noise and the hard-coded two-cluster model.
  - The hand-made `_synthetic_energy_cost` field gives way to real wind/grade terms. Synthetic scenarios get a seeded wind and a terrain field so they stay deterministic.
- **`LatticeAirspaceBuilder`:**
  - Local equirectangular grid around the centre. Default spacing 200 m over a 6×6 km box, about 900 nodes.
  - Node attributes: `x=lon`, `y=lat`, `x_m`, `y_m`.
  - 8-neighbour edges in both directions.
  - Edge attributes: haversine `d_h`, `bearing` (with `cos(lat)`), and `time_s = d / ground_speed` (informational only).
  - Keep the largest strongly connected component.
  - `run_demo_osm.py` switches to the lattice. Also fix the "Kalavakkam" titles.
- **New `env_data.py`, `fetch_weather()`:**
  - Open-Meteo `current=wind_speed_10m,wind_direction_10m,wind_gusts_10m,precipitation,weather_code,visibility` plus `hourly=wind_speed_80m,wind_direction_80m`, with the hourly row picked by matching the response's `time` string.
  - Returns a `WeatherSnapshot` dataclass.
  - Offline fallback: cached snapshot first, otherwise calm. Output is deterministic either way.
- **Wind cost (corrected):**
  - `HW = v·cos(θ_edge − θ_from)` (positive = headwind), `CW = v·|sin(θ_edge − θ_from)|`.
  - `ε = d·(1 + |grade| + β_head·max(0,HW)/V_cruise + β_cross·CW/V_cruise)`, so `ε ≥ d`.
  - The wind risk term is direction-dependent (row 1/4 of the table).
- Demo CLI: `--offline`, `--wind-override SPEED DIR`. Print the wind conditions in the console and in the Module 1 figure title.
- Update `wind_integration_plan.md` to v1.1 with the six fixes above and the new file locations.

## Phase 2: Terrain + height

- **`ElevationProvider` abstraction** (`env_data.py`), tried in this order:
  - `LocalRasterProvider`: SRTM/Copernicus GeoTIFF via `rasterio`/`osmnx.elevation`.
  - `OpenTopographyProvider`: global DEM REST API. Needs a free API key in env `OPENTOPO_API_KEY`.
  - `OpenMeteoElevationProvider`: Copernicus 90 m, no key, batches of 100.
  - `FlatProvider`: 0 m, with a warning.
  - All providers cache node elevations on disk, keyed by rounded coordinates.
- **Buildings:** `fetch_osm_layers()` via `ox.features_from_point` returns buildings, restricted zones, water and land use/POIs, cached as GeoPackage.
  - Height rule: `height=*`, else `building:levels × 3 m`, else a per-type default (residential 9, commercial 15, other 10 m). This is a documented assumption, because Chennai's height coverage is sparse.
- **Node altitude:**
  - `ground_elev_m` is the DEM value.
  - `bldg_h_m` is the max height in the node's cell (STRtree).
  - `alt_amsl_m = ground + max(H_CRUISE_AGL, bldg_h + CLEARANCE)`.
  - If the required AGL exceeds `H_MAX_AGL`, block the node.
- **Edges:** `grade = Δalt/d_h` from endpoint altitudes replaces the random grade. `distance = √(d_h² + Δalt²)`.
- **Terrain-clearance risk:** the minimum vertical margin over along-edge samples (25 m step) against terrain and buildings, so a low margin means high risk.
- **Building footprint risk:** row 3 of the table.
- Thiruvanmiyur is flat and coastal, so expect small terrain grades. Building clearance and wind will create most of the energy differences; say this in the report.

## Phase 3: Risk model upgrade

- Replace `base_risk + noise` with weighted components: `ρ = clip(Σ w_k·ρ_k, 0, 1)` over obstacle, building, wind, gust, weather, population, NFZ, terrain clearance, comm/GPS, water and turbulence. Weights live in config.
- Store each edge's breakdown as `risk_components={...}` alongside the final `risk`.
- Hard NFZ: drop the node or edge when its segment intersects a restricted polygon.
- Length-weighted exposure for path sums: `risk_cost = ρ·d_h/spacing`. Document it as "cumulative risk exposure", which matches meghna's "Cumulative Risk" labels.
- Global weather gate: a `NO-GO` flag is stored on the graph and respected by Tier 2.
- **Ablation plots:** fronts and edge-risk maps for all components vs without wind / terrain / buildings. Output: `outputs/ablation/`.

## Phase 4: Tier 1 hardening

Keep the algorithm mostly the same.
- **Heuristic fix:** haversine `h` with a vector form `[h, h, 0]`. It stays admissible because `distance ≥ d_h` and `ε ≥ d`. Prune on `g+h` against the front.
- **Memory:** a label table (node, g, parent_label_id) replaces the per-push dict copy and visited set. Cycles are dominated because distance > 0.
- **Duplicates:** treat equal vectors as dominated so the front has no repeats.
- **Diagnostics:** expose `truncated`, the expanded-label count and the runtime. Warn when the cap is hit (check S3).
- **New tests:**
  - a ~12-node graph where NAMOA* must match a brute-force front over all simple paths
  - no route in the final front dominates another
  - every path is valid: starts at s, ends at t, every edge exists, no repeated nodes
- **Refresh metadata** (new `pareto_store.py`): `{location, graph_hash, graph_timestamp, weather_timestamp, weather_snapshot, source, target, created_at, valid_until, truncated, front}`.
- **Cache invalidation:** `get_or_build()` rebuilds when the record is missing, expired, or drift crosses a threshold:
  - |Δv| > 10 km/h
  - Δdir > 45° when v > 10 km/h
  - a jump in gusts
  - a go/no-go flip

  A rebuild reruns only `apply_weather` plus Tier 1; the static layers stay cached. CLI flag: `--refresh`.
- Optional `run_region(depot, delivery_points)`: one front per delivery point snapped to the lattice.
- **Front thinning (optional):** cap the front at `front_max` routes (e.g. 100) using ε-dominance, which gives the "well-distributed" front the report promises. Off by default, so benchmarks still use the exact front.

## Phase 5: Tier 2 hardening

- Leave `derive_weights`, the amplification and the argmax untouched; they are the novelty claim.
- **Input validation:** F 1–5, W 1–3, U 1–3, P 1–5, integers. Bad input raises `ValueError`.
- **Feasibility:** W maps to a nominal payload mass for an F450 (0.25 / 0.5 / 1.0 kg, set in config). Route Wh = `k_cal·ε·((m_uav+m_p)/m_uav)^1.5`. A route is feasible if it needs ≤ capacity × (1 − 20% reserve). Infeasible routes are filtered out before the argmax; if none remain, or the weather gate says NO-GO, return a hold advisory.
- **Output:** a `SelectionResult` dataclass written to `outputs/tier2/selection_<preset|custom>.json` and `.csv`. It carries:
  - tags and weights
  - the selected index, cost and path with lat/lon/alt
  - normalised scores and per-objective ranks
  - point type, latency, feasible count
- CLI: `--payload F W U P`.
- **New tests:**
  - On a realistic front (the saved lattice front fixture), a mixed payload (3,2,2,3) picks a **non-extreme** point.
  - Monotonicity: raising F never increases the selected risk; same for U → distance and W → energy.
  - Validation errors.
  - Latency: median and p95 of a selection over a 100-route front stay below 1 s.

## Phase 6: Evaluation finish

- Rerun `run_benchmarks.py` after the new risk model.
- Add `obstacle_density` 5/20/40% to S1/S2/S3, plus a real Thiruvanmiyur lattice scenario.
- Add a **weighted-sum A\*** baseline to `module3_baselines.py`, using each preset's Tier 2 weights. This ablation shows the non-supported Pareto points that a single scalarised search can't reach.
- Raise MOPSO to the report's settings (swarm 100, 200 iterations, archive 100, 30 seeded reps, mean ± std). Keep the current small settings as `--smoke`.
- **Calm vs windy:** compare the fronts (observed snapshot vs `--wind-override` presets such as 30 km/h E and 30 km/h SW). Report the HV shift, the front shift, and how far each preset's pick moves.
- **Risk-component contribution table:** mean share of each component across front routes and across the selected routes.
- **Tier 2 evidence:** extend `module5_evaluation.tier2_evidence` with:
  - **payload-alignment score** `A = Σ_k w_k·(1 − (rank_k−1)/(K−1))`, rank-based so it is independent of the min-max utility. Compare it with the pickers for A* shortest, knee point, crowding distance (Ren) and fixed weights (Chokkalingam-style).
  - selected objective ranks, point type, and unique selected routes, which already exist
  - a sweep over all 225 tag combinations: non-extreme count, distinct points, monotonicity violations (target 0)
  - latency median and p95
- Save every output under `outputs/benchmarks/` and `outputs/ablation/` for the Review 2 Results chapter.

## Phase 7: Integration extras

- **Mission file:** export the selected route as QGroundControl/Mission Planner `QGC WPL 110` `.waypoints`, using lattice `alt_amsl_m` (or relative AGL) per waypoint. Collinear lattice waypoints are merged.
- **Minimal dashboard** (Tkinter or Streamlit): load the graph and front, enter tags, show the route on the map, export the mission. Satisfies Objective 3.
- **Later:** SITL validation with ArduPilot over MAVLink (pymavlink upload, measure route adherence and latency), then hardware flights.

## Docs alignment (alongside the phases)

- `AGENTS.md`: Tier 2 tags F/W/U/P with `derive_weights(F, W, U)`. Drop "numbering skips 3" and list module 3 (baselines) and module 5 (evaluation). Switch the test command to unittest.
- `FYP_B28_Project_Context.md` (caveman style): update Status, the graph source (lattice) and the risk components.
- Review 2 report: use W throughout, with a footnote on the renamed abstract tag. Align the module list to the code.

## Critical files

Modify:
- `Implementation/module1_graph_builder.py`
- `Implementation/run_demo_osm.py`
- `Implementation/run_demo_syn.py`
- `Implementation/module2_tier1_moastar.py`
- `Implementation/module4_tier2_payload.py`
- `Implementation/module3_baselines.py` (haversine heuristic)
- `Implementation/module5_evaluation.py`
- `Implementation/run_benchmarks.py`
- `Implementation/requirements.txt`
- `Implementation/wind_integration_plan.md`
- `AGENTS.md`, `FYP_B28_Project_Context.md`

New:
- `Implementation/config.py`, `env_data.py`, `pareto_store.py`
- `Implementation/mission_export.py` (Phase 7), `dashboard.py` (Phase 7)
- `tests/test_wind.py`, `test_lattice.py`, `test_tier1_front.py`, `test_store.py`
- `tests/fixtures/`

Reuse:
- `pareto_dominates` and `update_pareto_front`
- `TierTwoSelector.selected_index` and `classify_point`
- `route_cost` and `is_valid_route` (module3)
- `hypervolume`, `igd`, `spread_delta`, `objective_ranks` and `tier2_evidence` (module5)
- `save`, `load` and `visualise` (module1)
- `pick_distant_pair` (run_demo_osm)

## Verification

1. `python -m unittest discover -s Implementation/tests` passes offline: the old 11 plus the new wind-sign, lattice (no NFZ nodes, all altitudes ≤ 120 m AGL), brute-force front, store-drift, validation, monotonicity and non-extreme tests.
2. `python run_demo_syn.py`, then `python run_demo_osm.py --offline` (fixtures), both finish and write their plots.
3. On a networked machine, `python run_demo_osm.py` logs live wind, elevation provider and OSM layer counts. The lattice plot shows NFZ holes and altitude shading.
4. `--wind-override 30 90` vs calm:
   - east-bound edges cost more energy than west-bound edges of the same length
   - the front moves
   - Fragile Medical's pick moves more than Emergency Medicine's
5. A second run reuses the stored front. Changing the wind override triggers drift and a rebuild.
6. `python run_benchmarks.py --scenario all` writes metric CSVs, the risk-component table, ablation plots, the 225-combination Tier 2 sweep (monotonicity violations = 0, p95 latency < 1 s) and a truncation flag per scenario.
7. If Tier 1 is slow on the real lattice, tune spacing or box size until it runs in a few minutes on a laptop.
