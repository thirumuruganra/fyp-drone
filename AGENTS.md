# CLAUDE.md

Claude Code guidance, this repo.

## Communication Style — Caveman Mode

Terse like caveman. Technical substance exact. Only fluff die.
Drop: articles, filler (just/really/basically), pleasantries, hedging.
Fragments OK. Short synonyms. Code unchanged.
Pattern: [thing] [action] [reason]. [next step].
ACTIVE EVERY RESPONSE. No revert after many turns. No filler drift.
Code/commits/PRs: normal. Off: "stop caveman" / "normal mode".

**Project context file (`FYP_B28_Project_Context.md`):** agent-facing, not human reference. Whenever updating it, write in caveman style too (terse, fragments, no filler). Keep formulas, code blocks, IDs, DOIs, verbatim panel quotes exact.

## Git Workflow

Semantic commits, all commits.

## Project Overview

FYP (B28) repo. **"Payload-Aware Two-Tier UAV Route Planning for Last-Mile Delivery"** — Thirumurugan RA, Shrika SP Thota, Meghna Manimaran (Guide: Sujaudeen N), SSN College of Engineering, Dept. of CSE.

Mostly academic deliverables (LaTeX reports, PPTs) + small Python sim under `Implementation/`. No build system beyond LaTeX compile. Python tests use `unittest`.

**Core idea:** split Pareto-front build (Tier 1) from mission route pick (Tier 2). Planner run once per delivery region, reused. Payload metadata picks point on front per mission — no manual weight re-tune between deliveries. Read `FYP_B28_Project_Context.md` first — problem statement, lit gap, panel feedback history, tech decisions log. Canonical source for *why*, not just what. Read before touching algorithm concepts.

## Repository Layout

- `Abstract/`, `Initial Screening PPT/`, `Review_0/` — early submission artifacts. Frozen/historical.
- `Review_1/` — Review 1 deliverables (`Report/`, `PPT/`, `diagrams/`).
- `Review_2/` — Review 2 templates (`Report/`, `PPT/`).
- `Phase_1/` — Phase 1 report and seniors' format reference.
- `Implementation/` — Python sim (below).
- `Simulation/` — Gazebo (Harmonic) + ArduPilot SITL city sim. Imports `Implementation/` (sys.path), no copies. Region bundle (`prepare_sim_region.py`, OSM bldgs + DEM, `bundle_hash`) → world (`gazebo_world_builder.py`) + clearance-checked lattice (`route_clearance.py`, via builder `edge_filter`) → Tier 1 cached / Tier 2 → `.plan`/`.waypoints` (`run_sim_mission.py`) → pymavlink fly (`sitl_mission_runner.py`) → `evaluate_gazebo_run.py`. Read `Simulation/README.md` (frame contract, WSL setup). Tests: `python -m unittest discover -s Simulation/tests`. Generated `regions/ worlds/ missions/ runs/` git-ignored.
- `Reference_Papers/` — cited papers and baseline references.

## Implementation (`Implementation/`)

Python sim, five modules, no package structure — flat scripts, imported direct by demo and benchmark entry points.

- `config.py` — every tunable (region, heights, wind consts, risk weights, battery/mass). Edit here, not inline.
- `env_data.py` — `fetch_weather()` → `WeatherSnapshot` (Open-Meteo, no key; live → cache → calm). Terrain: `ElevationProvider` chain (`LocalRasterProvider` → `OpenTopographyProvider` [env `OPENTOPO_API_KEY`, optional; `load_dotenv()` pulls it from `<repo>/.env` — called by entry scripts `run_demo_osm`, `run_ablation`, `Simulation/prepare_sim_region`, never at import, so tests stay keyless; real env wins] → `OpenMeteoElevationProvider` [keyless] → `FlatProvider`), `fetch_elevations()`. OSM: `fetch_osm_layers()` (buildings/restricted/water/landuse, GeoPackage cache), `BuildingIndex` (STRtree; heights, cell density), `building_height_m()` (height → levels×3 → type default), `ZoneIndex` (restricted zones / water: contains, distance, segment hit; `sea_polygons()` from coastline), `ClassIndex` + `population_class()` (SORA-style exposure), `build_layer_indexes()`, `WeatherSnapshot.no_go_reasons()` (weather gate). Disk cache in `cache/`.
- `module1_graph_builder.py` — shared cost pipeline: static edge attrs (`d_h`, `bearing`, `grade`, `obstacle`) → `apply_weather(G, snapshot)` → `energy`, `risk`, `risk_components`, `cost=[d, ε, ρ]`. Builders: `AirspaceGraphBuilder` (seeded synthetic grid, seeded wind/terrain/obstacles), `LatticeAirspaceBuilder` (200 m lattice, 6 km box, Thiruvanmiyur; optional DEM + `BuildingIndex`: node alt = ground + max(60 m, bldg + 15 m), >120 m AGL → node blocked, grade = Δalt/d_h, 3-D distance, plus restricted zones: node inside → removed, edge touching zone → removed, soft buffer `exp(−d/λ)` outside). Risk = `clip(Σ w_k·ρ_k)` over 11 components in `config.RISK_WEIGHTS` (obstacle, building, terrain, wind, gust, weather, population, nfz, water, comm, turbulence); edge `rho` = that score, edge `risk` = cumulative exposure `rho·d_h/spacing` (cost[2]); per-edge `risk_components` kept. Weather gate → `G.graph["no_go"]`, `["no_go_reasons"]`. Airspace = grid lattice, not road network.
- `run_demo_osm.py` — lattice demo (name historic). Flags `--offline`, `--wind-override SPEED DIR_FROM`, `--no-terrain`, `--no-buildings`, `--no-risk-layers`, `--manual-nfz`, `--refresh`, `--front-max N` (approximate unverified zones, off by default), `--box-km`, `--spacing-m`. `--offline` falls back to hand-made fixtures in `tests/fixtures/` (not real data; regenerate with `make_fixtures.py`). Outputs `outputs/osm_*`.
- `module2_tier1_moastar.py` — `TierOneEngine`: NAMOA*. Label table (node, g, parent id), no per-push dict copy, no visited set (cycles die in node dominance). Equal vectors = dominated (no duplicate routes). Heuristic `heuristic_vector` = `[h, h, 0]`, h = haversine km (admissible: distance ≥ d_h, energy ≥ distance). Prunes on `g + h` vs front. After `run()`: `truncated` (cap hit with live labels left → front maybe incomplete, warns), `expanded`, `generated`, `runtime_s`, `diagnostics()`. `thin_front(front, n)`: optional ε-dominance cap, keeps 3 extremes, off by default. Tier 1.
- `pareto_store.py` — `ParetoStore.get_or_build()`: JSON fronts + metadata (`graph_hash` over static attrs only, weather snapshot, `valid_until`, `truncated`). Rebuild when missing / `--refresh` / graph changed / expired (3 h) / weather drift (|Δv| > 10 km/h, |Δdir| > 45° if v > 10, |Δgust| > 15, go/no-go flip). Rebuild = `apply_weather` + Tier 1 only. `run_region(G, depot, points)`: one front per delivery point (lat/lon snapped to lattice). Files in `pareto_front_storage/fronts/` (git-ignored).
- `module4_tier2_payload.py` — `TierTwoSelector` + `derive_weights(F, T, U)`: converts four payload tags (fragility, temp sensitivity, urgency, priority) into objective weights via fixed formula (project context doc), scores every Pareto point, picks `argmax(score)`. Tier 2.
- `run_demo_syn.py` / `run_demo_osm.py` — end-to-end demo scripts (`main()`), wire modules 1 → 2 → 4 on synthetic and OSM graphs, print step banners, render summary figures via Matplotlib.
- `run_ablation.py` — Phase 3 ablation: all / no_wind / no_terrain / no_buildings → Tier 1 fronts + edge-risk maps + component maps in `outputs/ablation/`.
- `run_benchmarks.py` — benchmark entry point for synthetic scenarios and baselines.
- `module4_tier2_payload.py` respects the weather gate: `TierTwoSelector(front, no_go_reasons)` → `select()` raises `WeatherNoGo`, `run_all_presets()` prints HOLD, returns `{}`.
- `mission_export.py` — QGroundControl / Mission Planner `QGC WPL 110` export from `SelectionResult`; frame 3 = above HOME: `alt_amsl_m − home_alt_amsl` (home = ground under first wp unless given), `agl_m` fallback only without DEM; optional RTL ending.
- `dashboard.py` — minimal Tkinter operator dashboard: load stored graph/front, enter tags, show route, export mission.
- `SITL_NOTES.md` — SITL validation checklist; hardware/SITL not run yet.
- `wind_integration_plan.md` — v1.1, implemented. Wind formulas, constants, six v1.0 bugs fixed. Read before touching wind-related edge costs.
- `cache/`, `outputs/`, `pareto_front_storage/`, `__pycache__/` — generated artifacts, not source.

### Running the simulation

```bash
cd Implementation
pip install -r requirements.txt   # networkx, numpy, matplotlib, osmnx (osmnx optional — synthetic path falls back without it)
python run_demo_syn.py            # synthetic airspace graph demo
python run_demo_osm.py            # OpenStreetMap-backed demo (requires network access for OSM + optionally osmnx)
python dashboard.py               # local operator dashboard, exports .waypoints
python -m unittest discover -s tests
```

Tests (unittest, offline): `python -m unittest discover -s Implementation/tests`. Also run demo scripts, inspect printed Pareto front + figures under `outputs/`. Sandbox blocks Open-Meteo: use `--offline` or `--wind-override`.

### Tier 1 → Tier 2 connection (matters when editing Tier 2 logic)

Payload-tag-to-weight mapping: not lookup table, continuous formula. Deliberate — answers review-panel critique, documented `FYP_B28_Project_Context.md`:

```
r_raw = F × 2        d_raw = U × 3        e_raw = W × 2
w_risk = r_raw / (r_raw + d_raw + e_raw)
w_distance = d_raw / (r_raw + d_raw + e_raw)
w_energy = e_raw / (r_raw + d_raw + e_raw)

score(p_i) = w_distance × (1 − d̃ᵢ) + w_energy × (1 − ẽᵢ) + w_risk × (1 − r̃ᵢ)   # normalized objectives
```

Selected route = `argmax(score)` over full Pareto front, not just extreme points. Keep this — revert to min-distance/min-risk extreme picks kills novelty claim.

## LaTeX Reports

- Compile targets: `Phase 1 Report/main.tex`, `Review 1 Report/Review1_Report.tex`. Own dirs. Need local `.cls`/`.sty` (`Thesis.cls` or `ssnreview.cls`, `bmacros.tex`, `setspace.sty`, `ssn.jpeg`, etc., sourced from `Reference Material (From Seniors - just for format)/FYP_Phase1_Report/`). Compile from same dir — relative includes need it.
- Editor latexmk integration builds PDF on save. In-place, from target dir — never `-outdir=build` (MiKTeX bibtex can't read `references.bib` from parent dir). `Review 1 Report/.latexmkrc` sets `$pdf_mode = 1`, `$clean_ext = "synctex.gz nav snm thm"`, `$bibtex_use = 2`. Refs in `references.bib`, style `unsrtnat` (natbib). Regenerate build files, don't hand-edit.
- **`Review1_Report.tex` chapter flow:** chapters run continuous — no chapter starts fresh page. Use `\flowchapter{Title}` (preamble: `\par\addvspace{30pt}{\let\clearpage\relax\chapter{#1}}`) for every chapter except first (`Introduction` keeps plain `\chapter`, opens body). Never revert `\flowchapter` to bare `\chapter` — brings back page break + big end-of-page gaps. `\label` goes right after `\flowchapter{...}` call.
- `Phase 1 Report/diagrams/*.puml` — PlantUML sources for `arch`, `tier1`, `tier2`, `gantt`. Render: `java -jar plantuml.jar diagrams\<name>.puml` (or plantuml.com) → `.png` LaTeX includes. Missing PNG → placeholder, not fail.
- Non-default LaTeX packages: `algpseudocode`/`algorithm` (algorithmicx), `booktabs`, `multirow`, `xcolor`, `url`. `glossaries` errors → comment out `\makeglossaries` in `Thesis.cls`.
- **After compile, clean with `latexmk -c Review1_Report.tex`** (`latexmk -c main.tex` for Phase 1 Report). Only command to run for report — with repo `.latexmkrc` (`$bibtex_use = 2`) removes every generated file (`.aux`, `.bbl`, `.blg`, `.fls`, `.fdb_latexmk`, `.log`, `.out`, `.toc`, `.lof`, `.lot`, `.synctex.gz`), leaves only `.tex` source(s), final `.pdf`, and style/class/asset files the `.tex` links (`.cls`, `.sty`, `bmacros.tex`, `references.bib`, images/diagrams).
