# FYP B28 — Project Context Document

**Payload-Aware Two-Tier UAV Route Planning for Last-Mile Delivery**

Sri Sivasubramaniya Nadar College of Engineering
Department of Computer Science and Engineering

---

## Team

| Name | Roll Number |
|---|---|
| Thirumurugan RA | 3122235001149 |
| Shrika SP Thota | 3122235001125 |
| Meghna Manimaran | 3122235001079 |

**Guide:** Dr. Sujaudeen N, Assistant Professor, Department of Computer Science and Engineering

**Current Status (2026-10-05):** Review 1 complete. Review 2 in preparation — `Review_2/Report/r2_reviewreport.tex` and `Review_2/PPT/R2_UG_Project_Presentation_Template.tex` are still blank college templates (no project content yet). Review 2 report needs chapters: Introduction/Objectives, First Review Queries and Answers, Problem Statement, Methodology (architecture + modules), Implementation, Results and Discussion. Review 0 presentation (FYP_B27.pdf) submitted and evaluated; Phase 1 Report, Review 1 Report, Review 1 PPT done. Python simulation (Tier 1 + Tier 2, synthetic + OSM demos) implemented and running under `Implementation/` (repo root). Not yet built: wind-aware edge costs (`Implementation/wind_integration_plan.md`), benchmarks vs A*/MOPSO, dashboard, SITL, hardware.

**Repo layout (restructured, no spaces in paths):** `Abstract/`, `Initial Screening PPT/`, `Review_0/`, `Review_1/` (`Report/`, `PPT/`, `diagrams/`), `Review_2/` (`Report/`, `PPT/`), `Phase_1/` (`Phase 1 Report/`, seniors' format reference), `Reference_Papers/`, `Implementation/` (Python sim, moved up from `Review 1/` to root), this file at root.

---

## Problem Domain

Autonomous UAV systems for medical and emergency last-mile delivery, where route planning must simultaneously satisfy conflicting objectives (travel distance, battery consumption, and flight risk) under dynamic field conditions without on-site expert supervision.

### Motivation

- Fragile payloads require low-vibration, low-risk paths. Emergency medicine requires minimum-time routes. Disaster-relief supplies require energy-efficient coverage. Every payload type demands a fundamentally different route trade-off.
- No current planner adjusts its routing priorities to match what is being carried. The same fixed objective weights are applied to every mission, forcing field operators to manually re-calibrate between every deployment, which is impractical and error-prone in time-critical emergency scenarios.

---

## Problem Statement

Existing UAV route planners encode distance, energy, and flight risk into a **single hand-tuned weighted-sum scalar** before path search begins, permanently fixing the objective trade-off regardless of what is being carried.

When payload context changes from fragile diagnostic samples requiring vibration-minimal routing to time-critical emergency medicine requiring shortest-path routing, an expert must manually re-tune planner weights and re-run the search from scratch — an intervention that is **infeasible in field conditions**.

The proposed system resolves this by separating Pareto front generation from mission-context-driven route selection. Payload metadata (fragility class, payload weight class, delivery urgency) has never been used as the sole input to autonomous Pareto point selection in the UAV delivery literature, leaving a structural gap this project directly addresses.

### Formal Problem Statement (report Chapter "Problem Statement")

Given:
(i) airspace graph `G = (V, E)`, edge cost vector `c(e) = [d(e), ε(e), ρ(e)]` — geodesic distance, energy, flight risk;
(ii) Pareto-optimal front `P` of non-dominated routes pre-computed over `G`;
(iii) payload metadata tuple `(F, W, U, P_lvl)` — fragility `F∈{1..5}`, payload weight `W∈{1..3}`, urgency `U∈{1..3}`, priority `P_lvl∈{1..5}`.

Problem: select route `p* ∈ P` best satisfying routing requirements implied by payload metadata, no expert re-tuning, no re-running the search.

Constraints: selection deterministic, reproducible, finishes within 1s of tag entry. Success criterion: selected route's characteristics (distance, energy, risk) match payload's stated requirements, checked via Tier 2 selection accuracy metric across payload presets.

Three conditions (from Gaps below): `p*` must depend on `(F,W,U,P_lvl)`, not front geometry alone (Gap 1). Front `P` reused across missions between refreshes, not rebuilt each time (Gap 2). Map from `(F,W,U,P_lvl)` to `p*` explicit algebraic formula, not expert rule or lookup table (Gap 3).

### Three Gaps (report Chapter "Related Work" → Research Gap)

1. **Gap 1 — no payload-aware selection.** Existing planners apply fixed geometric criterion (entropy/crowding) independent of payload, or no selection mechanism at all (Tsoutsouras folds mass into cost model, never selects). Tier 2 maps four payload tags directly to Pareto point selection through formula.
2. **Gap 2 — no front reuse across missions.** Existing planners re-run full multi-objective search whenever mission context changes. Two-tier split decouples front construction from mission arrivals — Tier 1 refreshes periodically, independent of payload; Tier 2 selects per new tag against current front, no re-run.
3. **Gap 3 — no algebraic bridge.** No formalised mapping links payload description to routing trade-off it implies (fragility → minimise risk, urgency → minimise distance). Weight-derivation formula supplies that bridge, deterministic and reproducible.

### Real-World Evidence (report Background, added post-Review-0)

Payload-blind routing has documented real-world cost, cited to ground novelty claim:
- **Swiss Post–Matternet M2 crashes (Zürich, Jan + May 2019)** — drones carrying clinical lab specimens on fixed, payload-invariant routes crashed; post-incident analysis confirmed identical corridors flown regardless of cargo type, no routing adjustment for fragility (`r12`).
- **Vibration/turbulence degradation studies** — controlled lab work shows drone-transported medicines suffer measurable degradation from flight-path vibration: insulin fibrillation risk, biologic protein denaturation, elevated haemolysis in blood samples on high-vibration segments (`r13`, `r14`). Comparative UAV blood-transport study confirms pre-analytic integrity varies with flight-path characteristics — greater altitude variation/turbulence → higher lab rejection rates (`r15`).
- **Zipline Rwanda blood-delivery network** (up to 80 km/single charge) — illustrates energy dimension: heavier blood-product payloads draw more hover/lift power, shorten range, yet system plans same fixed route regardless of weight, no energy-efficient detour for heavy cargo or direct short route for urgent light payload (`r16`).

Root cause common to all: planner has no knowledge of what's carried, can't prefer smoother/lower-risk corridors for fragile payloads or shortest paths for time-critical medicine.

---

## Literature Review — Existing Work

Table in report [r1, r3, r10]. Three papers reviewed — post-Review-0 set, replaces older Datsko/Ntakolia draft:

### Chokkalingam & Ganesan (2026) — `r1`
**Journal:** RAIRO — Operations Research
**DOI:** 10.1051/ro/2025159

- **Technique:** Pareto-optimal particle swarm optimisation (POPSO) w/ dynamic parameter adaptation + Pareto-entropy archive selection (keeps front diverse during optimisation). Final single route read off via weighted-sum scalarisation, weights set empirically per mission.
- **Finding:** Entropy-guided archiving improves front diversity + adaptability to real-time environmental change over static-weight baselines.
- **Limitation:** Final route choice still manually re-tuned weighted sum per mission. Payload metadata never incorporated — fragile samples and emergency medicine get same hand-tuned selection from same front.

### Tsoutsouras et al. (2020) — `r10`
**Source:** arXiv:2001.02531 (eess.SY)

- **Technique:** Adapts flight speed + energy-consumption model to UAV's instantaneous payload mass during trajectory planning, multi-user delivery missions. Trajectory updates as mass changes between drop-offs.
- **Finding:** Mass-aware trajectory adaptation cuts energy consumption vs mass-agnostic planning across multi-stop routes.
- **Limitation:** Continuous payload mass used only as physical parameter inside energy/speed model itself. Builds no multi-objective Pareto front. No discrete payload metadata (fragility, weight class, urgency) selecting among pre-computed routing trade-offs — mass reshapes cost model, never enters a selection formula.

### Ren et al. (2019) — `r3`
**Venue:** IEEE SOSE
**DOI:** 10.1109/SOSE.2019.00059

- **Technique:** Crowding-distance-based NSGA-II (CDNSGA-II) for multi-objective UAV path planning, balances distance + safety over octree-represented urban environment.
- **Finding:** CDNSGA-II finds well-spread, collision-free Pareto-optimal paths efficiently in octree-modelled urban airspace.
- **Limitation:** Crowding distance encodes only geometric spread of front. Cannot represent urgency, fragility, payload weight. Mission context plays no role in selection.

### Common Gap Across All Three Papers

Two of three (Chokkalingam & Ganesan, Ren) treat front diversity (entropy / crowding distance) as central design concern; only Ren's crowding distance actually *selects* a route from it — Chokkalingam & Ganesan falls back to manual weighted-sum re-tuning for final pick. Tsoutsouras builds no Pareto front at all — folds continuous payload mass into energy model equations of motion, never selects among competing routing trade-offs by payload. None of three use payload weight or any payload property as discrete input to a Pareto-point *selection* mechanism. This is structural gap proposed system closes.

---

## Proposed System Architecture

### Overview: Two-Tier Separation

The core architectural decision is to **separate Pareto front construction (Tier 1) from route selection (Tier 2)**. This means:

- The Pareto front is built for a delivery region and refreshed on a periodic cadence (or when environmental drift, e.g. a wind-condition shift, exceeds a threshold) — not rebuilt per mission.
- For each new mission arriving within a refresh window, only Tier 2 runs — reading the payload tag and selecting the appropriate route from the current front.
- No re-running the planner between missions within a refresh window.

---

## Tier 1 — Deterministic Pareto Front Construction

**What it does:**
Implements a multi-objective A*/Dijkstra graph-search that pre-computes and stores a well-distributed Pareto-optimal set of routes across three objectives for a defined delivery region. The front is refreshed on a periodic cadence and reused across successive missions within that window, without re-running the planner per mission.

**Three Objectives:**
- Route distance (km)
- Energy consumption (battery %)
- Flight risk (% — derived from obstacle density, no-fly zones, wind exposure)

**Algorithm:** Multi-objective A*/Dijkstra graph-search (NAMOA*) over airspace graph. Maintains cost vector `g` (not scalar) per partial path; dominated paths pruned early via `UpdateParetoFront`. Node-level label-dominance check additionally discards any partial path whose cost vector already dominated by previously-seen label at same node, before it enters open set — avoids exponential blow-up of naive MO-A* on dense graphs.

**Graph Source:** OpenStreetMap (OSM) via `osmnx` Python library. The OSM demo (`run_demo_osm.py`) uses Thiruvanmiyur, Chennai as the reference delivery region — the real deployment target, yielding a ~79-route Pareto front.

**Output:** A stored set of Pareto-optimal routes where no single route dominates another on all three objectives simultaneously.

---

## Tier 2 — Payload-Driven Pareto Selection

**What it does:**
Implements a structured payload metadata interpreter that reads four operator-assigned tags and maps them to a single Pareto-front point autonomously — no weight re-tuning at any stage.

**Four Payload Tags:**
1. Fragility class (1–5): How fragile the payload is
2. Payload weight class (1–3): How heavy the payload is
3. Delivery time-window / urgency (1–3): How time-critical the delivery is
4. Priority level (1–5): Mission priority for tie-breaking

---

## Critical: The Tier 1 to Tier 2 Connection

> This section directly addresses the Review 0 panel critique that "there is no correlation between the factors used in Tier 1 and the factors used in Tier 2."

### The Problem with the Original Approach

The original Tier 2 description said: "pick the route with shortest distance for emergency, pick the lowest-risk route for fragile payloads." The panel correctly identified that this is equivalent to just picking **extreme points** on the Pareto front — which means the full Pareto front computation is unnecessary. A single-objective A* run would have produced the same result.

### The Fix: Explicit Algebraic Mapping + Weighted Utility Scoring

The solution introduces a **formula-driven mapping from payload metadata to routing objective weights**, which then scores every Pareto point to find the true optimal — not just the extremes.

#### Step 1 — Semantic Mapping (The Connection)

| Payload Metadata Tag | Maps To (Tier 1 Objective) | Reasoning |
|---|---|---|
| Fragility class (F, 1–5) | `w_risk` | Fragile items need low-vibration, low-turbulence corridors → minimize flight risk |
| Payload weight class (W, 1–3) | `w_energy` | Heavier payload directly increases hover/lift power draw → minimize energy consumption |
| Delivery urgency (U, 1–3) | `w_distance` | Time-critical deliveries need the shortest path → minimize route distance |
| Priority level (P, 1–5) | Dominant-weight amplifier | If `P ≥ 4`, the largest of the three weights is amplified by `0.1 × w_max`, then all three weights are renormalised to sum to 1.0 — biasing high-priority missions further toward their dominant objective |

#### Step 2 — Weight Derivation Formula (No Expert Tuning)

```
r_raw = F × 2        (fragility heavily penalizes risk)
d_raw = U × 3        (urgency heavily penalizes distance)
e_raw = W × 2        (payload weight penalizes energy)

w_risk     = r_raw / (r_raw + d_raw + e_raw)
w_distance = d_raw / (r_raw + d_raw + e_raw)
w_energy   = e_raw / (r_raw + d_raw + e_raw)
```

Weights are derived **algorithmically from metadata** — no human expert tunes them.

**High-priority amplification (`P ≥ 4`):** after the three weights are derived, the dominant weight `w_max = max(w_risk, w_distance, w_energy)` is increased by `0.1 × w_max` and all three weights are renormalised to sum to 1.0 before scoring. A flat additive bonus to every point's score was tried first and rejected — it shifts all scores equally and never moves the argmax. `P < 4` leaves the weights untouched.

#### Step 3 — Utility Scoring Every Pareto Point

For each point `p_i` on the Pareto front with normalized objectives `(d̃, ẽ, r̃)`:

```
score(pᵢ) = w_distance × (1 − d̃ᵢ) + w_energy × (1 − ẽᵢ) + w_risk × (1 − r̃ᵢ)
```

Where:
- `d̃ = (d − d_min) / (d_max − d_min)` — normalized distance (0 = shortest, 1 = longest)
- `ẽ = (e − e_min) / (e_max − e_min)` — normalized energy (0 = least, 1 = most)
- `r̃ = (r − r_min) / (r_max − r_min)` — normalized risk (0 = lowest, 1 = highest)

**Selected route = argmax(score)** over all Pareto points.

#### Why This Answers the Panel

**For the panel comment "no use if you just pick extreme points":**

The utility scoring approach selects **different, non-extreme points** for most real payloads:

| Payload Preset | F | W | U | w_risk | w_dist | w_energy | Expected Selected Point |
|---|---|---|---|---|---|---|---|
| Fragile medical sample | 5 | 1 | 1 | 0.667 | 0.200 | 0.133 | Long, low-risk — but NOT the extreme lowest-risk point |
| Emergency medicine | 1 | 1 | 3 | 0.144 | 0.712 | 0.144 | Shortest distance (extreme — correct for this case) |
| Disaster relief supplies | 1 | 3 | 1 | 0.182 | 0.273 | 0.545 | Near-lowest energy, non-extreme, distinct from emergency medicine |
| Mixed cargo | 3 | 2 | 2 | 0.375 | 0.375 | 0.250 | Middle — balanced trade-off, non-extreme |

Only emergency medicine selects an extreme — and that is the **correct** answer for that payload. All other mixed-priority payload types select intermediate non-extreme points that neither single-objective A* nor MOPSO can find.

**Key sentence for the panel:**
> "Tier 2 does not pick a priority — it picks a point. The payload metadata derives weights, the weights score every Pareto point via a utility function, and the argmax selects a route that no single-objective baseline can produce for a mixed payload."

---

## Dataset and Benchmarking Strategy

### Primary Dataset: Synthetic Airspace Graph

- Extract road/path network from **OpenStreetMap (OSM)** using `osmnx`
- Assign three objective costs to each edge:
  - **Distance:** Euclidean or geodesic edge length
  - **Energy:** Proxy via distance × altitude-change factor
  - **Flight Risk:** Simulated threat zones, obstacle density, wind exposure (scored 0–1)
- Obstacle and threat distributions are **simulated programmatically** for controlled, reproducible benchmarking

### Why Not Use Existing Papers' Datasets

| Paper | Dataset | Available? |
|---|---|---|
| Chokkalingam & Ganesan (2026) | Custom simulation, not released | No |
| Datsko et al. (2024) | Proprietary test environment | No |
| Ntakolia et al. (2023) | Custom grid world, no public repo | No |

This is standard in the field — papers construct their own synthetic environments for reproducibility.

### Benchmark Scenario Set

| Scenario | Nodes | Obstacle Density | Purpose |
|---|---|---|---|
| S1 — Small sparse | ~50 nodes | Low (5%) | Sanity check / unit benchmark |
| S2 — Medium urban | ~200 nodes | Medium (20%) | Core benchmark |
| S3 — Large dense | ~500 nodes | High (40%) | Stress test scalability |

Each stochastic scenario (MOPSO) runs **30 times** for mean ± std dev.

### Baselines

- **Single-objective A*** (distance only)
- **MOPSO** (Multi-Objective Particle Swarm Optimization)
  - Swarm size: 100 particles
  - Iterations: 200
  - Repository size: 100 (Pareto archive)

### Evaluation Metrics

- **Hypervolume (HV):** Volume of objective space dominated by the Pareto front. Higher = better.
- **Pareto Spread (Δ metric):** standard spread metric, captures extent + uniformity of front.
- **IGD (Inverted Generational Distance):** computed against reference front formed from union of all non-dominated solutions found across every method + repetition, per scenario.
- **Tier 2 Selection Accuracy:** Alignment between selected route characteristics and payload tag requirements across mission types (fragile medical, emergency medicine, disaster relief).
- **Computation Time:** Tier 1 pre-computation time vs MOPSO run time.

---

## Project Objectives

### Project (Implementation) Objectives

1. **Tier 1 — Deterministic Pareto Front Construction:** Implement a multi-objective A*/Dijkstra graph-search that pre-computes and stores a well-distributed Pareto-optimal set of routes across route distance, energy consumption, and flight risk for a defined delivery region. The front is refreshed on a periodic cadence and reused across successive missions within that window, without re-running the planner per mission.

2. **Tier 2 — Payload-Driven Pareto Selection:** Implement a structured payload metadata interpreter that reads four operator-assigned tags (fragility class, payload weight class, delivery time-window, priority level) and maps them to a single Pareto-front point autonomously — no weight re-tuning at any stage.

3. **Ground-Station Dashboard:** Build an operator interface where a payload tag is entered once and a ready-to-fly mission file (compatible with ArduPilot Mission Planner) is generated without further manual input.

4. **Hardware Validation:** Demonstrate the end-to-end pipeline on a quadcopter UAV through real outdoor flight tests, measuring route adherence and end-to-end latency from payload tagging to mission execution.

### Research Objectives

1. Formulate multi-objective UAV route planning as a structured **Pareto front pre-computation** problem over a delivery airspace graph, theoretically distinguishing the deterministic graph-search approach from stochastic population-based alternatives.

2. Design a **payload metadata schema** and a principled rule-based mapping that translates mission context (fragility, urgency, payload weight) into Pareto point selection without any expert weight tuning.

3. **Benchmark Tier 1** route quality and Pareto front coverage against single-objective A* and MOPSO baselines on identical delivery scenarios, measuring hypervolume and Pareto spread.

4. **Quantify Tier 2 selection accuracy** by measuring alignment between selected route characteristics and payload tag requirements across diverse mission types (medical sample, emergency medicine, disaster relief supplies).

5. **Novelty:** The system uses payload metadata as the sole input to Pareto point selection, replacing entropy and diversity-based criteria that are blind to delivery context, with a deterministic graph-search Pareto front refreshed on a periodic cadence and reused across all missions within that window.

---

## Feasibility

### Tools and Software Stack

| Component | Tool |
|---|---|
| Graph-search | Python, NetworkX |
| Objective computation | NumPy |
| Pareto front visualization | Matplotlib |
| UAV mission upload | ArduPilot / Mission Planner |
| Ground-to-UAV communication | MAVLink |
| Operator dashboard | Tkinter or React |

### Hardware

- Quadcopter UAV with Pixhawk flight controller
- F450 quadcopter frame
- Raspberry Pi 5 (onboard compute)
- Servo-actuated payload release mechanism

### Compute Requirements

- **Tier 1:** Graph-search pre-computation runs entirely on a standard laptop with no GPU or cloud dependency.
- **Tier 2:** Metadata-to-Pareto mapping is a near-instantaneous rule lookup with negligible compute cost.

### Risks and Mitigations

All algorithm validation and baseline benchmarking is completed through **ArduPilot SITL (software-in-the-loop simulation)** first, so real flight tests serve as confirmation of an already-validated system rather than the primary evaluation environment.

---

## Project Timeline

| Quarter | Period | Key Activities | Deliverables |
|---|---|---|---|
| Q1 | Jul – Sep | Literature review, problem formalisation, airspace graph model design; Tier 1 A*/Dijkstra implementation and unit tests | Literature review report, airspace graph model, Tier 1 implementation with unit tests |
| Q2 | Oct – Nov | Tier 1 Pareto front construction, objective normalisation, and front storage; Tier 2 payload metadata schema design and Pareto selection mapping | Pareto front construction module, objective normalisation scheme, payload metadata schema, Pareto selection mapping |
| Q3 | Dec – Jan | Ground-station dashboard and mission file generator; system integration and ArduPilot SITL simulation validation; baseline comparison against A* and MOPSO | Ground-station dashboard, mission file generator, validated ArduPilot SITL integration, benchmarking results |
| Q4 | Feb – Mar | Real outdoor flight tests and end-to-end performance measurement; results analysis, documentation, and final project report | Flight test data, end-to-end performance measurements, final thesis and project report |

---

## Review 0 — Panel Comments and Resolutions

### Panel Comments (Verbatim)

1. No correlation.
2. Criteria are not considered to map together.
3. Application of Pareto needs some rethinking.
4. Correlation between Tier 1 and Tier 2 factors, since there are more than 2 objectives and complexity only in Tier 2.

### Additional Verbal Feedback

- The panel said there is no connection between the factors used in Tier 1 Pareto Front generation (distance, energy, flight risk) and the factors used in Tier 2 for payload-aware route selection (fragility, temperature sensitivity, urgency).
- When told Tier 2 uses simple rule-based selection — pick shortest distance for emergency, pick lowest risk for fragile — the panel responded: "there is no use if you just choose the extreme points."

### Resolution of Each Comment

**Comment 1 — "No correlation"**
Resolved by introducing the explicit algebraic mapping:
- Fragility class (Tier 2 tag) → scales `w_risk` (Tier 1 objective)
- Payload weight class (Tier 2 tag) → scales `w_energy` (Tier 1 objective)
- Delivery urgency (Tier 2 tag) → scales `w_distance` (Tier 1 objective)

The formula `w_risk = (F×2) / (F×2 + U×3 + W×2)` and equivalents create an explicit, deterministic connection between every Tier 2 input and every Tier 1 objective.

**Comment 2 — "Criteria are not considered to map together"**
Resolved by the same algebraic mapping. The mapping is not a lookup table or rule — it is a continuous formula where all four metadata tags simultaneously influence all three objective weights. Each weight is a function of all three raw scores, so a change in any single tag affects all three weights proportionally.

**Comment 3 — "Application of Pareto needs some rethinking"**
Resolved by replacing extreme-point selection with **utility scoring over the full Pareto front**. The selection mechanism now:
- Scores every Pareto point using `score(p) = w_dist × (1−d̃) + w_energy × (1−ẽ) + w_risk × (1−r̃)`
- Selects `argmax(score)` — which can be any point on the front, not just extremes
- For mixed-priority payloads (disaster relief, mixed cargo, fragile samples), the selected point is an intermediate non-extreme point that single-objective A* cannot produce

**Comment 4 — "Correlation between Tier 1 and Tier 2 factors, more than 2 objectives, complexity only in Tier 2"**
Resolved by demonstrating that Tier 1 produces three objectives (distance, energy, risk) and Tier 2 uses all three simultaneously through the utility scoring formula. The complexity now exists in both tiers: Tier 1 manages the multi-objective graph-search complexity, and Tier 2 manages the metadata-to-weight mapping and full-front scoring complexity. Neither tier is trivially reducible to a single-objective problem.

---

## Final Abstract (as submitted in BID_28.docx)

UAV systems for medical and emergency last-mile delivery must balance objectives that conflict: shorter routes carry higher flight risk, low-risk corridors drain more battery, and energy-efficient paths are rarely the fastest. No single fixed trade-off serves all payload types, yet existing planners apply the same objective weights to every mission regardless of what is being carried.

Existing multi-objective planners, including entropy-based, diversity-driven, and crowding-distance Pareto selection methods, encode objective weights as a fixed scalar before path search begins. Payload properties such as fragility class, temperature sensitivity, and delivery urgency have no influence over which Pareto-optimal route is chosen. When the payload changes, operators must re-tune weights manually and re-run the planner, which is not workable under field conditions.

This work proposes a Payload-Aware Two-Tier UAV Route Planning system that separates Pareto front construction from route selection, introducing an explicit algebraic bridge between Tier 1 routing objectives and Tier 2 payload metadata.

Tier 1 runs a multi-objective A*/Dijkstra graph-search over a delivery airspace graph sourced from OpenStreetMap, producing a Pareto-optimal route set across distance, energy consumption, and flight risk. Tier 2 reads four operator-assigned payload tags and derives objective weights through a fixed formula: fragility class scales w_risk, delivery urgency scales w_distance, and temperature sensitivity scales w_energy. These weights score every Pareto point through a utility function; the argmax is selected as the mission route. The system will be evaluated on programmatically generated airspace graphs at three scales (50, 200, and 500 nodes) against single-objective A* and MOPSO baselines, using hypervolume, Pareto spread, and Tier 2 selection accuracy as metrics. Hardware validation will use a Pixhawk-equipped quadcopter through ArduPilot SITL simulation and outdoor flight tests.

Planned experiments will quantify Tier 1 improvements in hypervolume and Pareto spread over both baselines and will determine whether metadata-derived utility scoring selects non-extreme Pareto points for mixed-priority payloads that neither A* nor MOPSO can produce.

The principal contribution is a formula-driven mapping from payload metadata to routing objective weights, connecting mission context directly to Pareto point selection without expert intervention. This replaces context-blind diversity criteria with a direct algebraic link from payload description to route choice, and the Pareto front is computed once and reused across all mission types without re-running the planner.

---

## Key Technical Decisions Log

| Decision | Rationale |
|---|---|
| Deterministic A*/Dijkstra for Tier 1 (not NSGA-II or MOPSO) | Produces reproducible, deterministic Pareto front; pre-computable once; no stochastic variance between runs |
| Utility scoring over full Pareto front (not rule-based extreme selection) | Enables non-extreme point selection for mixed-priority payloads; addresses panel critique |
| Four-tag payload schema (fragility, payload weight, urgency, priority) | Maps directly to three routing objectives; priority used as tie-breaker; sufficient to cover all three mission types |
| OpenStreetMap for airspace graph | Reproducible, publicly available; allows real-geography testing while keeping benchmarks controlled |
| ArduPilot SITL before real flight | Validates algorithm before hardware risk; simulation serves as primary evaluation, real tests as confirmation |
| No GPU or cloud dependency | Tier 1 runs on standard laptop; Tier 2 is near-instantaneous; field-deployable with minimal hardware |

---

## Novelty Statement (Final)

The principal contribution is a formula-driven mapping from payload metadata to routing objective weights, where fragility class scales w_risk, delivery urgency scales w_distance, and payload weight class scales w_energy. This connection between mission context and Pareto point selection replaces context-blind diversity criteria with a direct algebraic link from payload description to route choice, with the Pareto front refreshed on a periodic cadence and reused across all mission types within that window, without re-running the planner per mission.

---

*Document compiled from Review 0 presentation (FYP_B27.pdf), submitted abstract (BID_28.docx), panel review session, and Review 1 deliverables (Phase 1 Report, Review 1 Report/PPT, Python simulation). Last updated: 2026-10-05.*

*Note: literature review set changed post-Review-0 — Datsko et al. (2024) and Ntakolia et al. (2023) dropped from Review1_Report.tex; replaced with Tsoutsouras et al. (2020) `r10` and Ren et al. (2019) `r3` alongside retained Chokkalingam & Ganesan (2026) `r1`. Real-world evidence (Swiss Post–Matternet crash, vibration-degradation studies, Zipline Rwanda) and formal Gap 1/2/3 framing added to Background/Related Work — both reflected above.*

*Note: "temperature sensitivity" in the frozen abstract above was renamed to "payload weight class" after submission — temperature sensitivity had no provable causal link to energy consumption, whereas heavier payload directly raises hover/lift power draw. All current text uses payload weight class (tag `W`, 1–3).*
