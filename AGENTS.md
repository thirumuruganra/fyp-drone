# AGENTS.md

Agent guidance, this repo. `CLAUDE.md` points here.

## Communication Style — Caveman Mode

Terse like caveman. Technical substance exact. Only fluff die.
Drop: articles, filler (just/really/basically), pleasantries, hedging.
Fragments OK. Short synonyms. Code unchanged.
Pattern: [thing] [action] [reason]. [next step].
ACTIVE EVERY RESPONSE. No revert after many turns. No filler drift.
Code/commits/PRs: normal. Off: "stop caveman" / "normal mode".

**Project context file (`FYP_B28_Project_Context.md`):** agent-facing, not human reference. Whenever updating it, write in caveman style too (terse, fragments, no filler). Keep formulas, code blocks, IDs, DOIs, verbatim panel quotes exact. Update status + open defects when code state changes.

## Git Workflow

Semantic commits, all commits.

## Project Overview

FYP (B28) repo. **"Payload-Aware Two-Tier UAV Route Planning for Last-Mile Delivery"** — Thirumurugan RA, Shrika SP Thota, Meghna Manimaran (Guide: Sujaudeen N), SSN College of Engineering, Dept. of CSE.

Academic deliverables (LaTeX reports, PPTs) + Python planner under `Implementation/` + Gazebo/SITL sim under `Simulation/`. No build system beyond LaTeX compile. Python tests use `unittest`.

**Core idea:** split Pareto-front build (Tier 1) from mission route pick (Tier 2). Planner run once per delivery region, reused. Payload metadata picks point on front per mission — no manual weight re-tune between deliveries. Read `FYP_B28_Project_Context.md` first — problem statement, lit gap, panel feedback history, tech decisions log, current status + open defects. Canonical source for *why*, not just what. Read before touching algorithm concepts.

## Repository Layout

- `Abstract/` — submitted abstract (`BID_28.docx`). Frozen.
- `Review_0/` — Review 0 artifacts. Frozen/historical. `Initial Screening PPT/` git-ignored.
- `Review_1/` — Review 1 deliverables: `Report/` (`BID_REPORT_28.tex`, `references.bib`, `ssnreview.cls`), `PPT/` (`BID_PPT_28.tex`), `diagrams/` (`*.puml` + `.png`).
- `Review_2/` — Review 2: `Report/BID_REPORT_28.tex` done (`ssnreview.cls` = Review 1 copy, `references.bib`, `figures/` = result plots copied from `Implementation/outputs/` + wind sweep), `diagrams/` (`*.puml` + `.png`, PlantUML), `PPT/R2_UG_Project_Presentation_Template.tex` still blank template.
- `Phase_1/` — `Phase 1 Report/` (`main.tex`, `ch1..ch5_*.tex`, `Thesis.cls`) + seniors' format reference (git-ignored).
- `Implementation/` — Python planner (below).
- `Simulation/` — Gazebo (Harmonic) + ArduPilot SITL city sim. Imports `Implementation/` (sys.path), no copies. Region bundle (`prepare_sim_region.py`, OSM bldgs + DEM, `bundle_hash`) → world (`gazebo_world_builder.py`) + clearance-checked lattice (`route_clearance.py`, via builder `edge_filter`) → Tier 1 cached / Tier 2 → `.plan`/`.waypoints` (`run_sim_mission.py`) → pymavlink fly (`sitl_mission_runner.py`) → `evaluate_gazebo_run.py`. Read `Simulation/README.md` (frame contract, WSL setup). Tests: `python -m unittest discover -s Simulation/tests` (needs `trimesh`). Generated `regions/ worlds/ missions/ runs/` git-ignored. Gazebo not yet run; mission runner proven vs bare ArduCopter SITL.
- `Reference_Papers/` — cited papers and baseline references.
- `.env` (root) — tracked in git, holds `OPENTOPO_API_KEY` + `OVERPASS_EXTRA_ENDPOINTS`. Never print or echo values in replies.
- `.agents/skills/`, `.claude/skills/` — mirrored skill copies. Keep both in sync if editing.

## Implementation (`Implementation/`)

Python planner, flat scripts, no package structure — imported direct by demo and benchmark entry points.

- `config.py` — every tunable (region, heights, wind consts, risk weights, battery/mass, Overpass endpoints). Edit here, not inline. Battery constants + `NO_GO_GUST_KMH` marked PLACEHOLDER.
- `env_data.py` — `fetch_weather()` → `WeatherSnapshot` (Open-Meteo, no key; live → cache → calm). Terrain: `ElevationProvider` chain (`LocalRasterProvider` → `OpenTopographyProvider` [env `OPENTOPO_API_KEY`, optional; `load_dotenv()` pulls it from `<repo>/.env` — called by entry scripts `run_demo_osm`, `run_ablation`, `Simulation/prepare_sim_region`, never at import, so tests stay keyless; real env wins] → `OpenMeteoElevationProvider` [keyless] → `FlatProvider`), `fetch_elevations()`. OSM: `fetch_osm_layers()` (buildings/restricted/water/landuse, GeoPackage cache), `BuildingIndex` (STRtree; heights, cell density), `building_height_m()` (height → levels×3 → type default), `ZoneIndex` (restricted zones / water: contains, distance, segment hit; `sea_polygons()` from coastline), `ClassIndex` + `population_class()` (SORA-style exposure), `build_layer_indexes()`, `WeatherSnapshot.no_go_reasons()` (weather gate). Disk cache in `cache/`.
- `overpass_client.py` — multi-endpoint Overpass fetch: retry, busy-wait cap, identifying User-Agent, extra endpoints from env `OVERPASS_EXTRA_ENDPOINTS`. `check_overpass.py` = connectivity probe. `OVERPASS_NOTES.md` = findings. Failed fetch → empty layers, exit still 0: check console layer counts.
- `module1_graph_builder.py` — shared cost pipeline: static edge attrs (`d_h`, `bearing`, `grade`, `obstacle`) → `apply_weather(G, snapshot)` → `energy`, `risk`, `risk_components`, `cost=[d, ε, ρ]`. Builders: `AirspaceGraphBuilder` (seeded synthetic grid, seeded wind/terrain/obstacles), `LatticeAirspaceBuilder` (200 m lattice, 6 km box, Thiruvanmiyur; optional DEM + `BuildingIndex`: node alt = ground + max(60 m, bldg + 15 m), >120 m AGL → node blocked, grade = Δalt/d_h, 3-D distance, plus restricted zones: node inside → removed, edge touching zone → removed, soft buffer `exp(−d/λ)` outside; optional `edge_filter` hook, used by `Simulation/`). Risk = `clip(Σ w_k·ρ_k)` over 11 components in `config.RISK_WEIGHTS` (obstacle, building, terrain, wind, gust, weather, population, nfz, water, comm, turbulence); edge `rho` = that score, edge `risk` = cumulative exposure `rho·d_h/spacing` (cost[2]); per-edge `risk_components` kept. Weather gate → `G.graph["no_go"]`, `["no_go_reasons"]`. Airspace = grid lattice, not road network. Weights uncalibrated.
- `module2_tier1_moastar.py` — `TierOneEngine`: NAMOA*. Label table (node, g, parent id), no per-push dict copy, no visited set (cycles die in node dominance). Equal vectors = dominated (no duplicate routes). Heuristic `heuristic_vector` = `[h, h, 0]`, h = haversine km (admissible: distance ≥ d_h, energy ≥ distance). Prunes on `g + h` vs front. After `run()`: `truncated` (cap hit with live labels left → front maybe incomplete, warns), `expanded`, `generated`, `runtime_s`, `diagnostics()`. `thin_front(front, n)`: optional ε-dominance cap, keeps 3 extremes, off by default. Tier 1.
- `pareto_store.py` — `ParetoStore.get_or_build()`: JSON fronts + metadata (`graph_hash` over static attrs only, weather snapshot, `valid_until`, `truncated`). Rebuild when missing / `--refresh` / graph changed / expired (3 h) / weather drift (|Δv| > 10 km/h, |Δdir| > 45° if v > 10, |Δgust| > 15, go/no-go flip). Rebuild = `apply_weather` + Tier 1 only. `run_region(G, depot, points)`: one front per delivery point (lat/lon snapped to lattice). Files in `pareto_front_storage/fronts/` (git-ignored).
- `module3_baselines.py` — single-objective A* (distance/energy/risk), `weighted_sum_astar`, compact `mopso_baseline`. Baselines for benchmarks.
- `module4_tier2_payload.py` — `TierTwoSelector` + `derive_weights(F, W, U, P)`: converts four payload tags (fragility F, payload weight class W, urgency U, priority P) into objective weights via fixed formula (below + project context doc), scores every Pareto point, picks `argmax(score)`. Tier 2. `select_detailed()` = validated, battery-feasibility-filtered, returns `SelectionResult` (JSON/CSV export). `select()` / `run_all_presets()` skip battery filter (open defect). Respects weather gate: `TierTwoSelector(front, no_go_reasons)` → `select()` raises `WeatherNoGo`, `run_all_presets()` prints HOLD, returns `{}`.
- `module5_evaluation.py` — HV (exact 3-D), IGD, spread Δ, union reference front, `tier2_evidence`, `tier2_sweep` (225 tags), `monotonicity_violations`, component-share tables.
- `run_demo_syn.py` / `run_demo_osm.py` — end-to-end demo scripts (`main()`), wire modules 1 → 2 → 4 on synthetic and lattice graphs, print step banners, render summary figures via Matplotlib.
- `run_demo_osm.py` — lattice demo (name historic). Flags `--offline`, `--wind-override SPEED DIR_FROM`, `--no-terrain`, `--no-buildings`, `--no-risk-layers`, `--manual-nfz`, `--refresh`, `--front-max N` (approximate unverified zones, off by default), `--box-km`, `--spacing-m`, `--payload F W U P`, `--preset NAME`. `--offline` falls back to hand-made fixtures in `tests/fixtures/` (not real data; regenerate with `make_fixtures.py`). Outputs `outputs/osm_*`, `outputs/tier2/selection_*.json|csv`. Source/target auto-picked distant pair; no CLI depot/dest yet.
- `run_ablation.py` — Phase 3 ablation: all / no_wind / no_terrain / no_buildings → Tier 1 fronts + edge-risk maps + component maps in `outputs/ablation/`.
- `run_benchmarks.py` — synthetic S1/S2/S3 scenarios + baselines. Flags `--scenario`, `--mopso-repetitions` (30), `--swarm-size` (100), `--iterations` (200), `--archive-size`, `--output-dir`, `--smoke`. Outputs `outputs/benchmarks/`.
- `mission_export.py` — QGroundControl / Mission Planner `QGC WPL 110` export from `SelectionResult`; frame 3 = above HOME: `alt_amsl_m − home_alt_amsl` (home = ground under first wp unless given), `agl_m` fallback only without DEM; optional RTL ending.
- `dashboard.py` — minimal Tkinter operator dashboard: load stored graph/front, enter tags, show route, export mission. Known defects: bypasses weather HOLD, graph/front pair mismatch after OSM demo.
- `IMPLEMENTATION_PLAN.md` — phases 0–7 plan; all merged. `IMPLEMENTATION_AUDIT_2026-10-06.md` — what merged ≠ validated, defect list. Check both before claiming a feature done.
- `SITL_NOTES.md` — SITL checklist, points to `Simulation/`.
- `wind_integration_plan.md` — v1.1, implemented. Wind formulas, constants, six v1.0 bugs fixed. Read before touching wind-related edge costs.
- `cache/`, `outputs/`, `pareto_front_storage/`, `__pycache__/` — generated artifacts, not source.

### Running the simulation

```bash
cd Implementation
pip install -r requirements.txt   # networkx, numpy, matplotlib, requests, shapely, geopandas, pyogrio, osmnx
python run_demo_syn.py            # synthetic airspace graph demo
python run_demo_osm.py            # lattice demo, live OSM + DEM + weather (network needed)
python run_demo_osm.py --offline --box-km 2 --payload 3 2 2 3   # fixtures/cache, fast
python dashboard.py               # local operator dashboard, exports .waypoints
python -m unittest discover -s tests
```

Tests (unittest, offline): `python -m unittest discover -s Implementation/tests` (~223) and `Simulation/tests` (~42). Also run demo scripts, inspect printed Pareto front + figures under `outputs/`. Sandbox blocks Open-Meteo/Overpass: use `--offline` or `--wind-override`. Windows: set `PYTHONUTF8=1` — cp1252 console crashes on `↳` and similar prints (2 tests error otherwise). `test_dashboard_helpers` needs Tk; `test_world_builder` needs `trimesh`.

### Tier 1 → Tier 2 connection (matters when editing Tier 2 logic)

Payload-tag-to-weight mapping: not lookup table, continuous formula. Deliberate — answers review-panel critique, documented `FYP_B28_Project_Context.md`:

```
r_raw = F × 2        d_raw = U × 3        e_raw = W × 2
w_risk = r_raw / (r_raw + d_raw + e_raw)
w_distance = d_raw / (r_raw + d_raw + e_raw)
w_energy = e_raw / (r_raw + d_raw + e_raw)

P ≥ 4: w_max += 0.1 × w_max, renormalise to sum 1.0   # P < 4: untouched

score(p_i) = w_distance × (1 − d̃ᵢ) + w_energy × (1 − ẽᵢ) + w_risk × (1 − r̃ᵢ)   # normalized objectives
```

Tags: F fragility 1–5, W payload weight class 1–3 (was "temperature sensitivity" in frozen abstract; renamed — no causal link to energy), U urgency 1–3, P priority 1–5. Use W everywhere, never T.

Selected route = `argmax(score)` over full Pareto front, not just extreme points. Keep this — revert to min-distance/min-risk extreme picks kills novelty claim.

## LaTeX Reports

- Compile targets: `Phase_1/Phase 1 Report/main.tex` (`Thesis.cls`, `bmacros.tex`, `setspace.sty`, `lstpatch.sty`, `ssn.jpeg`, all local), `Review_1/Report/BID_REPORT_28.tex` (`ssnreview.cls`), `Review_1/PPT/BID_PPT_28.tex`, `Review_2/Report/BID_REPORT_28.tex` (`graphicspath` = `../diagrams/` + `figures/`; uses `\flowchapter` like Review 1), `Review_2/PPT/*.tex`. Each own dir. Compile from same dir — relative includes need it. Seniors' format reference (git-ignored) only for missing class/asset copies; see `Phase 1 Report/README_COMPILE.txt`.
- Editor latexmk integration builds PDF on save. In-place, from target dir — never `-outdir=build` (MiKTeX bibtex can't read `references.bib` from parent dir). `Review_1/Report/.latexmkrc` and `Review_1/PPT/.latexmkrc` set `$pdf_mode = 1`, `$clean_ext = "synctex.gz nav snm thm"`, `$bibtex_use = 2`. Refs in `references.bib`, style `unsrtnat` (natbib). Regenerate build files, don't hand-edit.
- **`BID_REPORT_28.tex` chapter flow:** chapters run continuous — no chapter starts fresh page. Use `\flowchapter{Title}` (preamble: `\par\addvspace{30pt}{\let\clearpage\relax\chapter{#1}}`) for every chapter except first (`Introduction` keeps plain `\chapter`, opens body). Never revert `\flowchapter` to bare `\chapter` — brings back page break + big end-of-page gaps. `\label` goes right after `\flowchapter{...}` call.
- `Review_1/diagrams/*.puml` — PlantUML sources for `arch`, `tier1`, `tier2`, `gantt`. Render: `java -jar plantuml.jar <name>.puml` (or plantuml.com) → `.png` LaTeX includes. Missing PNG → placeholder, not fail.
- Non-default LaTeX packages: `algpseudocode`/`algorithm` (algorithmicx), `booktabs`, `multirow`, `xcolor`, `url`. `glossaries` errors → comment out `\makeglossaries` in `Thesis.cls`.
- **After compile, clean with `latexmk -c <file>.tex`** (`BID_REPORT_28.tex`, or `main.tex` for Phase 1). With `.latexmkrc` (`$bibtex_use = 2`) removes every generated file (`.aux`, `.bbl`, `.blg`, `.fls`, `.fdb_latexmk`, `.log`, `.out`, `.toc`, `.lof`, `.lot`, `.synctex.gz`), leaves only `.tex` source(s), final `.pdf`, and style/class/asset files the `.tex` links (`.cls`, `.sty`, `bmacros.tex`, `references.bib`, images/diagrams).
