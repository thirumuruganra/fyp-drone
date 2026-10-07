# FYP B28 — Project Context

**Payload-Aware Two-Tier UAV Route Planning for Last-Mile Delivery**

SSN College of Engineering, Dept. of CSE. Agent-facing canonical context: why, not just what.

## Team

| Name | Roll Number |
|---|---|
| Thirumurugan RA | 3122235001149 |
| Shrika SP Thota | 3122235001125 |
| Meghna Manimaran | 3122235001079 |

**Guide:** Dr. Sujaudeen N, Asst. Professor, CSE

## Status (2026-10-07)

- Review 0 done (FYP_B27.pdf). Review 1 done (Phase 1 Report, Review 1 Report, Review 1 PPT).
- Review 2 in prep. `Review_2/Report/r2_reviewreport.tex` + `Review_2/PPT/R2_UG_Project_Presentation_Template.tex` still blank templates. Report chapters needed: Introduction/Objectives, First Review Queries and Answers, Problem Statement, Methodology (architecture + modules), Implementation, Results and Discussion.
- All `Implementation/IMPLEMENTATION_PLAN.md` phases 0–7 merged to `main`. Merged ≠ validated: see Open Defects.
- `Implementation/` built:
  - M1: shared wind-aware cost pipeline. `LatticeAirspaceBuilder` (200 m lattice, 6 km box, Thiruvanmiyur) + DEM terrain + OSM buildings → node altitude. 11-component weighted risk, hard restricted zones, weather go/no-go gate.
  - M2: NAMOA* hardened (`truncated` flag, `g+h` pruning, no per-push copy) + `ParetoStore` (JSON fronts, 3 h expiry, weather-drift refresh).
  - M3 baselines (A* ×3, compact MOPSO). M4 Tier 2: validation, battery feasibility filter (`select_detailed`), weather HOLD, `SelectionResult`, JSON/CSV export. M5 metrics (HV, IGD, spread, Tier 2 evidence, 225-tag sweep). Ablation (`run_ablation.py`). `mission_export.py` (frame 3 = AMSL − home AMSL), Tkinter dashboard.
  - Env: `env_data.py` (Open-Meteo weather, DEM provider chain, OSM layers), `overpass_client.py` (multi-endpoint Overpass w/ retry; Kumi mirror + env extra endpoints; see `OVERPASS_NOTES.md`). API keys via `<repo>/.env` (`load_dotenv()` in entry scripts only).
- `Simulation/` built (2026-10-06): Gazebo Harmonic + ArduPilot SITL city sim. Frozen region bundle → world + clearance-checked lattice → Tier 1 cached / Tier 2 → `.plan`/`.waypoints` → pymavlink fly → eval (cross-track, clearance, landing offset). Read `Simulation/README.md`. Run in WSL2.
  - Verified: `sitl_mission_runner.py` flown vs real ArduCopter SITL 4.8.0-dev (built-in `quad`, fixture bundle): upload/read-back/AUTO/land OK, cross-track RMSE 0.11–0.13 m, landing offset ≤ 0.02 m.
  - NOT run: Gazebo itself, live OSM/Open-Meteo bundle build, world load/real-time-factor gate. Needs WSL machine.
- Tests: `Implementation/tests` 223, `Simulation/tests` 42 collected. Local Windows run: 2 Implementation errors (`test_ablation` smoke, `test_risk` gate) = cp1252 `UnicodeEncodeError` on print, not logic → set `PYTHONUTF8=1`. `test_world_builder` needs `trimesh`; `test_dashboard_helpers` needs Tk. Last full pass: audit 2026-10-06 (193 pass, Tk missing).
- Not built: calibrated risk weights, real battery model, live-OSM-verified Review 2 results, Mission Planner/QGC import check, Gazebo flights, hardware flights.

### Open Defects (from `Implementation/IMPLEMENTATION_AUDIT_2026-10-06.md`; re-verified 2026-10-07 by grep)

Still open:
1. `dashboard.py` builds `TierTwoSelector(front)` w/o `no_go_reasons` → bypasses weather HOLD.
2. Dashboard loads synthetic `airspace_graph.pkl`, OSM demo overwrites shared `pareto_front.pkl` → graph/front mismatch. No file picker.
3. `run_all_presets()` calls `select()` → ignores battery filter. Only `select_detailed()` filters. Use `--payload`/`--preset`.
4. `module5_evaluation.monotonicity_violations` keys rows by F/W/U/P only; `run_benchmarks.py` mixes scenarios → false violations. Group by scenario.
5. `run_benchmarks.py` omits `obstacle_density=density`; `_apply_density_risk()` scales final risk instead → S1/S2/S3 density labels not real obstacle coverage.
6. Phase 6 gaps: no real-lattice benchmark, no calm-vs-windy comparison, MOPSO repeats pooled (no per-repeat HV/IGD/spread mean ± std). Report-scale run not done.
7. Battery constants PLACEHOLDER (`config.py` ~l.153), `NO_GO_GUST_KMH` PLACEHOLDER. Risk weights uncalibrated. Default 6 km route → mixed-cargo battery HOLD expected.
8. Offline OSM fixtures hand-made, not real evidence. Live OSM fetch intermittent (resets/504). Failed fetch falls back to empty layers → exit 0 ≠ real OSM. Check console layer counts.

Fixed since audit: mission altitude frame (#3 in audit) — `mission_export.py` now `alt_amsl_m − home_alt_amsl`.

**Repo layout (no spaces in paths):** `Abstract/`, `Initial Screening PPT/`, `Review_0/`, `Review_1/` (`Report/`, `PPT/`, `diagrams/`), `Review_2/` (`Report/`, `PPT/`), `Phase_1/` (`Phase 1 Report/`, seniors' format reference), `Reference_Papers/`, `Implementation/` (planner), `Simulation/` (Gazebo + SITL, imports `Implementation/`), this file at root. `AGENTS.md` = agent guidance (`CLAUDE.md` points to it).

## Problem

UAV medical/emergency last-mile delivery. Route planning must trade off distance, battery, flight risk under dynamic field conditions, no on-site expert.

- Payloads need different trade-offs: fragile → low-vibration/low-risk; emergency medicine → min-time; disaster relief → energy-efficient.
- No current planner adapts priorities to payload. Fixed weights for every mission → operators re-calibrate manually between deployments. Impractical in emergencies.

### Problem Statement

Existing planners fold distance, energy, risk into one **hand-tuned weighted-sum scalar** before search → trade-off fixed regardless of payload. Payload changes → expert re-tunes weights, re-runs search. Infeasible in field. Fix: separate Pareto front generation from mission-context route selection. Payload metadata as sole input to autonomous Pareto point selection: absent from UAV delivery literature.

### Formal Statement (report chapter "Problem Statement")

Given:
(i) airspace graph `G = (V, E)`, edge cost `c(e) = [d(e), ε(e), ρ(e)]` — geodesic distance, energy, flight risk;
(ii) precomputed Pareto front `P` of non-dominated routes over `G`;
(iii) payload tuple `(F, W, U, P_lvl)` — fragility `F∈{1..5}`, weight class `W∈{1..3}`, urgency `U∈{1..3}`, priority `P_lvl∈{1..5}`.

Select `p* ∈ P` best matching payload requirements. No expert re-tuning, no search re-run.

Constraints: deterministic, reproducible, ≤1s after tag entry. Success: selected route's (distance, energy, risk) match payload requirements, measured by Tier 2 selection accuracy across payload presets.

Three conditions: `p*` depends on `(F,W,U,P_lvl)` not front geometry (Gap 1). `P` reused across missions between refreshes (Gap 2). Map `(F,W,U,P_lvl)→p*` explicit algebraic formula, not rule/lookup (Gap 3).

### Three Gaps (report: Related Work → Research Gap)

1. **Gap 1 — no payload-aware selection.** Existing: fixed geometric criterion (entropy/crowding) independent of payload, or no selection (Tsoutsouras folds mass into cost model). Tier 2 maps four tags → Pareto point via formula.
2. **Gap 2 — no front reuse.** Existing planners re-run full MO search when context changes. Two-tier split: Tier 1 refreshes periodically, payload-independent; Tier 2 selects per tag against current front.
3. **Gap 3 — no algebraic bridge.** Nothing formalises payload description → routing trade-off (fragility → min risk, urgency → min distance). Weight formula supplies it, deterministic.

### Real-World Evidence (report Background, added post-Review-0)

- **Swiss Post–Matternet M2 crashes (Zürich, Jan + May 2019):** drones with lab specimens flew fixed payload-invariant routes, crashed; same corridors regardless of cargo (`r12`).
- **Vibration/turbulence studies:** drone-transported medicine degrades — insulin fibrillation, biologic protein denaturation, elevated haemolysis in blood on high-vibration segments (`r13`, `r14`). UAV blood-transport study: more altitude variation/turbulence → higher lab rejection (`r15`).
- **Zipline Rwanda** (≤80 km/charge): heavier blood payloads draw more lift power, cut range, same fixed route anyway (`r16`).

Common root cause: planner blind to cargo.

## Literature (report table, `r1`, `r3`, `r10`)

Post-Review-0 set. Datsko (2024) + Ntakolia (2023) dropped; replaced by Tsoutsouras + Ren.

- **Chokkalingam & Ganesan (2026), `r1`** — RAIRO Operations Research, DOI 10.1051/ro/2025159. POPSO + dynamic parameter adaptation + Pareto-entropy archive. Final route via weighted-sum scalarisation, weights set empirically per mission. Gain: front diversity, adaptability. Limit: final pick manually re-tuned, payload ignored.
- **Tsoutsouras et al. (2020), `r10`** — arXiv:2001.02531. Flight speed + energy model adapt to instantaneous payload mass, multi-user delivery. Gain: less energy vs mass-agnostic. Limit: mass only physical parameter in energy model; no Pareto front; no discrete metadata (fragility/weight class/urgency) selecting among precomputed trade-offs.
- **Ren et al. (2019), `r3`** — IEEE SOSE, DOI 10.1109/SOSE.2019.00059. CDNSGA-II, distance + safety, octree urban env. Gain: well-spread collision-free Pareto paths. Limit: crowding distance = geometric spread only; no urgency/fragility/weight; mission context absent.

**Common gap:** Chokkalingam + Ren focus on front diversity; only Ren's crowding distance actually selects (Chokkalingam falls back to manual weighted sum). Tsoutsouras builds no front. None uses payload property as discrete input to Pareto-point *selection*.

## Architecture: Two-Tier Separation

Pareto front construction (Tier 1) split from route selection (Tier 2).

- Front built per delivery region, refreshed periodically (or when environmental drift, e.g. wind shift, exceeds threshold). Not per mission.
- New mission in refresh window → only Tier 2 runs: read payload tag, pick route from current front.
- No planner re-run between missions in window.

### Tier 1 — Deterministic Pareto Front

Multi-objective A*/Dijkstra (NAMOA*) over airspace graph, precomputes + stores well-distributed Pareto set.

- Objectives: distance (km), energy proxy, cumulative risk proxy.
- Per partial path keeps cost vector `g` (not scalar). Dominated paths pruned early via `UpdateParetoFront`. Node-level label-dominance check discards partial path whose cost vector dominated by earlier label at same node, before open set → avoids exponential blow-up of naive MO-A*.
- Graph source: two builders. Synthetic seeded grid (`AirspaceGraphBuilder`, benchmarks). Real-geography lattice (`LatticeAirspaceBuilder`, `run_demo_osm.py`): 200 m lattice, 6 km box, Thiruvanmiyur. Airspace = lattice, **not** road network; OSM supplies buildings/restricted/water/landuse layers, roads only drawn for visualisation.
- Altitude: node alt = ground (DEM) + max(60 m, building + 15 m); > 120 m AGL → node blocked. Restricted zone: node inside removed, edge touching removed, soft buffer `exp(−d/λ)` outside.
- Risk: weighted sum, 11 components in `config.RISK_WEIGHTS` (obstacle, building, terrain, wind, gust, weather, population, nfz, water, comm, turbulence). Weights **uncalibrated**. Cost vector `[d, ε, ρ]`, ρ = cumulative exposure. Wind: `wind_integration_plan.md` v1.1 (six v1.0 bugs fixed).
- Weather gate: live Open-Meteo → cache → calm. Storm/gust over limit → `no_go` → Tier 2 HOLD.
- Terrain: `LocalRasterProvider` → OpenTopography (key in `.env`) → Open-Meteo elevation → flat.
- Output: set of routes, none dominating another on all three objectives. Stored by `ParetoStore`, reused until expiry/drift/graph change.

### Tier 2 — Payload-Driven Selection

Reads four operator tags, maps to one Pareto point autonomously. No weight re-tuning.

1. Fragility class `F` (1–5)
2. Payload weight class `W` (1–3)
3. Delivery urgency `U` (1–3)
4. Priority level `P` (1–5)

## Critical: Tier 1 → Tier 2 Connection

Answers Review 0 panel critique: "no correlation between Tier 1 factors and Tier 2 factors."

**Old approach (rejected):** "shortest for emergency, lowest-risk for fragile" = picking extreme points → full Pareto computation unnecessary, single-objective A* gives same result.

**Fix:** formula maps payload metadata → objective weights; weights score every Pareto point → true optimum, not just extremes. **Keep this. Reverting to extreme-point picks kills novelty claim.**

### Step 1 — Semantic Mapping

| Tag | Maps To | Reason |
|---|---|---|
| Fragility `F` (1–5) | `w_risk` | fragile → low-vibration/turbulence corridors → min risk |
| Weight class `W` (1–3) | `w_energy` | heavier → more hover/lift power → min energy |
| Urgency `U` (1–3) | `w_distance` | time-critical → shortest path |
| Priority `P` (1–5) | dominant-weight amplifier | `P ≥ 4`: largest weight += `0.1 × w_max`, renormalise to sum 1.0 |

### Step 2 — Weight Derivation (no expert tuning)

```
r_raw = F × 2        (fragility heavily penalizes risk)
d_raw = U × 3        (urgency heavily penalizes distance)
e_raw = W × 2        (payload weight penalizes energy)

w_risk     = r_raw / (r_raw + d_raw + e_raw)
w_distance = d_raw / (r_raw + d_raw + e_raw)
w_energy   = e_raw / (r_raw + d_raw + e_raw)
```

Priority amplification (`P ≥ 4`): `w_max = max(w_risk, w_distance, w_energy)` += `0.1 × w_max`, renormalise. Flat additive score bonus tried first, rejected: shifts all scores equally, never moves argmax. `P < 4`: weights untouched.

### Step 3 — Utility Scoring

Per Pareto point `p_i`, normalized objectives `(d̃, ẽ, r̃)`:

```
score(pᵢ) = w_distance × (1 − d̃ᵢ) + w_energy × (1 − ẽᵢ) + w_risk × (1 − r̃ᵢ)
```

- `d̃ = (d − d_min) / (d_max − d_min)` (0 = shortest)
- `ẽ = (e − e_min) / (e_max − e_min)` (0 = least energy)
- `r̃ = (r − r_min) / (r_max − r_min)` (0 = lowest risk)

**Selected route = `argmax(score)` over full front.**

### Why This Answers Panel ("no use if you just pick extreme points")

| Preset | F | W | U | w_risk | w_dist | w_energy | Expected Pick |
|---|---|---|---|---|---|---|---|
| Fragile medical sample | 5 | 1 | 1 | 0.667 | 0.200 | 0.133 | long, low-risk, NOT extreme lowest-risk |
| Emergency medicine | 1 | 1 | 3 | 0.144 | 0.712 | 0.144 | shortest distance (extreme — correct here) |
| Disaster relief | 1 | 3 | 1 | 0.182 | 0.273 | 0.545 | near-lowest energy, non-extreme, distinct from emergency |
| Mixed cargo | 3 | 2 | 2 | 0.375 | 0.375 | 0.250 | middle, balanced |

Only emergency medicine picks extreme, correct for it. Others pick intermediate points single-objective A* and MOPSO cannot find.

**Key sentence:** "Tier 2 does not pick a priority — it picks a point. Payload metadata derives weights, weights score every Pareto point via utility function, argmax selects a route no single-objective baseline produces for a mixed payload."

## Dataset and Benchmarking

### Dataset: Synthetic Airspace Graph

- Benchmark dataset: synthetic grid airspace graph from `AirspaceGraphBuilder._build_synthetic_graph`.
- Real-geography dataset: Thiruvanmiyur lattice (`run_demo_osm.py`, `Simulation/`), OSM buildings + DEM + live weather. Real lattice benchmark not yet run (defect 6).
- Edge costs: **distance** (3-D geodesic), **energy** (wind-aware, grade-aware, payload-mass-aware proxy; battery constants PLACEHOLDER), **risk** (11-component weighted score, 0–1, cumulative exposure).
- Obstacles/threats simulated programmatically → reproducible.
- Other papers' datasets unavailable (custom/proprietary/unreleased: Chokkalingam, Datsko, Ntakolia). Building own synthetic env is field standard.

### Scenarios

| Scenario | Nodes | Obstacle Density | Purpose |
|---|---|---|---|
| S1 small sparse | ~50 | 5% | sanity/unit |
| S2 medium urban | ~200 | 20% | core |
| S3 large dense | ~500 | 40% | scalability stress |

Stochastic baselines (MOPSO): 30 runs, mean ± std.

### Baselines

- Single-objective A* (distance only)
- Single-objective A* (energy only)
- Single-objective A* (risk only)
- Compact MOPSO-style baseline in `module3_baselines.py`; report-scale swarm/repetition settings not run yet.

### Metrics

- Hypervolume (HV), higher better
- Pareto Spread (Δ)
- IGD vs reference front = union of all non-dominated solutions across methods + repetitions, per scenario
- Tier 2 evidence: selected objective ranks, point type, non-extreme picks, latency. Avoid fake "accuracy" labels.
- Computation time: Tier 1 precompute vs MOPSO

## Objectives

### Project

1. **Tier 1:** MO A*/Dijkstra, precompute + store well-distributed Pareto set (distance, energy, risk) per region; periodic refresh, reuse across missions.
2. **Tier 2:** payload metadata interpreter, four tags → one Pareto point, no weight re-tuning.
3. **Ground-station dashboard:** tag entered once → `QGC WPL 110` mission file exported. Minimal Tkinter version built (defects 1–2 open); Mission Planner/QGroundControl import not verified.
4. **Simulation validation:** Gazebo + ArduPilot SITL city flight of Tier 2 pick; measure cross-track error, clearance, landing offset. Mission runner proven on bare SITL; Gazebo world not yet run.
5. **Hardware validation:** quadcopter outdoor flights; measure route adherence + end-to-end latency (tag → mission execution).

### Research

1. Formulate MO UAV routing as Pareto front precomputation over airspace graph; distinguish deterministic graph-search from stochastic population methods.
2. Payload metadata schema + principled mapping from mission context (fragility, urgency, weight) → Pareto point, no expert tuning.
3. Benchmark Tier 1 vs single-objective A* and MOPSO: hypervolume, Pareto spread.
4. Quantify Tier 2 selection accuracy across mission types.
5. **Novelty:** payload metadata as sole input to Pareto point selection, replacing context-blind entropy/diversity criteria; deterministic front refreshed periodically, reused across missions.

## Feasibility

| Component | Tool |
|---|---|
| Graph search | Python, NetworkX |
| Objectives | NumPy |
| Visualization | Matplotlib |
| Mission upload | ArduPilot / Mission Planner |
| Ground-to-UAV comms | MAVLink |
| Dashboard | Tkinter or React |

- Hardware: Pixhawk quadcopter, F450 frame, Raspberry Pi 5, servo payload release.
- Compute: Tier 1 on laptop, no GPU/cloud. Tier 2 negligible.
- Risk mitigation: validate + benchmark in ArduPilot SITL first; flights only confirm.

## Timeline

| Quarter | Period | Activities | Deliverables |
|---|---|---|---|
| Q1 | Jul–Sep | Lit review, problem formalisation, airspace graph design; Tier 1 A*/Dijkstra + unit tests | Lit review report, graph model, Tier 1 impl |
| Q2 | Oct–Nov | Tier 1 Pareto construction, normalisation, storage; Tier 2 tag schema + selection mapping | Pareto module, normalisation scheme, tag schema, selection mapping |
| Q3 | Dec–Jan | Dashboard + mission file generator; integration + SITL validation; A*/MOPSO baseline comparison | Dashboard, mission generator, SITL integration, benchmark results |
| Q4 | Feb–Mar | Outdoor flight tests, end-to-end measurement; analysis, final report | Flight data, performance measurements, final thesis |

## Review 0 — Panel Comments

**Verbatim:**
1. No correlation.
2. Criteria are not considered to map together.
3. Application of Pareto needs some rethinking.
4. Correlation between Tier 1 and Tier 2 factors, since there are more than 2 objectives and complexity only in Tier 2.

**Verbal:** no connection between Tier 1 factors (distance, energy, risk) and Tier 2 factors (fragility, temperature sensitivity, urgency). Told Tier 2 is rule-based (shortest for emergency, lowest-risk for fragile) → "there is no use if you just choose the extreme points."

**Resolutions:**
1. *No correlation* → explicit algebraic mapping: `F`→`w_risk`, `W`→`w_energy`, `U`→`w_distance`. `w_risk = (F×2)/(F×2 + U×3 + W×2)` etc. links every Tier 2 input to every Tier 1 objective.
2. *Criteria don't map together* → same formula. Continuous, not lookup/rule. Each weight depends on all three raw scores; one tag change shifts all three weights.
3. *Pareto rethink* → utility scoring over full front replaces extreme-point selection. `argmax` can be any point. Mixed payloads pick intermediate points single-objective A* cannot produce.
4. *Complexity only in Tier 2* → Tier 1 = MO graph-search complexity; Tier 2 = metadata→weight mapping + full-front scoring, uses all three objectives. Neither tier reducible to single-objective.

## Final Abstract (submitted in BID_28.docx, frozen)

UAV systems for medical and emergency last-mile delivery must balance conflicting objectives: shorter routes carry higher flight risk, low-risk corridors drain more battery, energy-efficient paths rarely fastest. No single fixed trade-off serves all payloads, yet existing planners apply the same weights to every mission.

Existing multi-objective planners (entropy-based, diversity-driven, crowding-distance Pareto selection) encode weights as fixed scalar before search. Fragility class, temperature sensitivity, delivery urgency have no influence on chosen route. Payload changes → operators re-tune weights manually and re-run planner, unworkable in field.

Proposes Payload-Aware Two-Tier UAV Route Planning: separates Pareto front construction from route selection, with explicit algebraic bridge between Tier 1 objectives and Tier 2 payload metadata.

Tier 1: multi-objective A*/Dijkstra over OSM airspace graph → Pareto set across distance, energy, risk. Tier 2: reads four operator tags, derives weights via fixed formula (fragility scales w_risk, urgency scales w_distance, temperature sensitivity scales w_energy), scores every Pareto point via utility function, argmax = mission route. Evaluation: programmatically generated graphs at 50/200/500 nodes vs single-objective A* and MOPSO, metrics hypervolume, Pareto spread, Tier 2 selection accuracy. Hardware: Pixhawk quadcopter, ArduPilot SITL then outdoor flights.

Experiments quantify Tier 1 hypervolume/spread gains over both baselines, test whether metadata-derived utility scoring selects non-extreme Pareto points for mixed-priority payloads that neither A* nor MOPSO produce.

Principal contribution: formula-driven mapping from payload metadata to objective weights, linking mission context to Pareto point selection without expert intervention. Replaces context-blind diversity criteria; front computed once, reused across mission types without planner re-run.

## Technical Decisions Log

| Decision | Rationale |
|---|---|
| Deterministic A*/Dijkstra Tier 1 (not NSGA-II/MOPSO) | Reproducible, precomputable, no stochastic variance |
| Utility scoring over full front (not extreme picks) | Non-extreme picks for mixed payloads; answers panel |
| Four-tag schema (fragility, weight, urgency, priority) | Maps to three objectives; priority amplifies dominant weight; covers all three mission types |
| OSM airspace graph | Reproducible, public; real geography + controlled benchmarks |
| ArduPilot SITL before real flight | Validate before hardware risk; sim = primary eval, flights = confirmation |
| Gazebo Harmonic + ArduPilot SITL city sim (`Simulation/`) | 3-D buildings/terrain check flyability, not just graph math. Planner code imported, not copied. |
| Airspace = lattice, not road graph | UAV not road-bound. Roads only visual + population-exposure layer. Abstract still says "OSM airspace graph": OSM = data source, graph = lattice. |
| Hard clearance filter before NAMOA* (`route_clearance.py`) | Roof +15 m (5 m buffer), terrain +15 m, 120 m AGL. Waypoints never raised after selection. Infeasible → "NO FEASIBLE MISSION". |
| Mission frame 3 = AMSL − home AMSL | `agl_m` wrong over varying terrain (audit defect). Sim mission = TAKEOFF → route → LAND, no unvalidated RTL. |
| Bundle hash ties planner, front, mission, world | Runner refuses to fly on mismatch. Stops stale-data flights. |
| Env/API keys via `.env`, loaded in entry scripts only | Tests stay keyless/offline. Real env wins. |
| Multi-endpoint Overpass w/ identifying User-Agent, cache first | Public endpoints flaky. Failed fetch → empty layers, so check layer counts. |
| No GPU/cloud | Tier 1 on laptop, Tier 2 instant; field-deployable |

## Novelty Statement (Final)

Formula-driven mapping from payload metadata to objective weights: fragility class scales w_risk, urgency scales w_distance, payload weight class scales w_energy. Links mission context to Pareto point selection, replacing context-blind diversity criteria with direct algebraic link from payload description to route choice. Front refreshed periodically, reused across mission types in window, no per-mission planner re-run.

---

*Compiled from Review 0 presentation (FYP_B27.pdf), BID_28.docx abstract, panel session, Review 1 deliverables, Implementation audit 2026-10-06, Simulation README. Last updated: 2026-10-07.*

*Post-Review-0 lit change: Datsko (2024), Ntakolia (2023) dropped from `BID_REPORT_28.tex`; Tsoutsouras `r10` + Ren `r3` added beside Chokkalingam `r1`. Real-world evidence + Gap 1/2/3 added to Background/Related Work. Reflected above.*

*Frozen abstract "temperature sensitivity" → "payload weight class" (tag `W`, 1–3) post-submission: temperature no provable link to energy; heavier payload raises hover/lift power. Use `W` everywhere, never `T`.*
