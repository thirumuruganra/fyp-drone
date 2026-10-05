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

Mostly academic deliverables (LaTeX reports, PPTs) + small Python sim under `Review 1/Implementation/`. No build system beyond LaTeX compile. No Python test suite.

**Core idea:** split Pareto-front build (Tier 1) from mission route pick (Tier 2). Planner run once per delivery region, reused. Payload metadata picks point on front per mission — no manual weight re-tune between deliveries. Read `Review 1/FYP_B28_Project_Context.md` first — problem statement, lit gap, panel feedback history, tech decisions log. Canonical source for *why*, not just what. Read before touching algorithm concepts.

## Repository Layout

- `Abstract/`, `Initial Screening PPT/`, `Review 0/` — early submission artifacts. Frozen/historical.
- `Review 1/` — current phase deliverables:
  - `FYP_B28_Project_Context.md` — canonical context (problem, arch, math, panel Q&A, timeline). Read first.
  - `Implementation/` — Python sim (below).
  - `Phase 1 Report/` — LaTeX thesis chapters (`ch1_intro.tex` … `ch5_timeline.tex`), compiled via `main.tex` (uses `Thesis.cls`, `bmacros.tex`).
  - `Review 1 PPT/`, `Review 1 Report/` — LaTeX presentation/report, `ssnreview.cls`.
- `Reference Material (From Seniors - just for format)/` — prior year thesis, format reference only. Not project content.

## Implementation (`Review 1/Implementation/`)

Python sim, four modules, no package structure — flat scripts, imported direct by two demo entry points.

- `config.py` — every tunable (region, heights, wind consts, risk weights, battery/mass). Edit here, not inline.
- `env_data.py` — `fetch_weather()` → `WeatherSnapshot` (Open-Meteo, no key; live → cache → calm). Disk cache in `cache/`.
- `module1_graph_builder.py` — shared cost pipeline: static edge attrs (`d_h`, `bearing`, `grade`, `obstacle`) → `apply_weather(G, snapshot)` → `energy`, `risk`, `risk_components`, `cost=[d, ε, ρ]`. Builders: `AirspaceGraphBuilder` (seeded synthetic grid, seeded wind/terrain/obstacles), `LatticeAirspaceBuilder` (200 m lattice, 6 km box, Thiruvanmiyur). Airspace = grid lattice, not road network.
- `run_demo_osm.py` — lattice demo (name historic). Flags `--offline`, `--wind-override SPEED DIR_FROM`, `--box-km`, `--spacing-m`. Outputs `outputs/osm_*`.
- `module2_tier1_moastar.py` — `TierOneEngine`: multi-objective A*/Dijkstra search (`pareto_dominates`, `update_pareto_front`), produces non-dominated route set for source/target pair. Tier 1.
- `module4_tier2_payload.py` — `TierTwoSelector` + `derive_weights(F, T, U)`: converts four payload tags (fragility, temp sensitivity, urgency, priority) into objective weights via fixed formula (project context doc), scores every Pareto point, picks `argmax(score)`. Tier 2.
- `run_demo_syn.py` / `run_demo_osm.py` — end-to-end demo scripts (`main()`), wire modules 1 → 2 → 4 on synthetic and OSM graphs, print step banners, render summary figures via Matplotlib.
- `wind_integration_plan.md` — v1.1, implemented. Wind formulas, constants, six v1.0 bugs fixed. Read before touching wind-related edge costs.
- `cache/`, `outputs/`, `pareto_front_storage/`, `__pycache__/` — generated artifacts, not source.

Module numbering skips 3 (1, 2, 4 — no `module3`) — matches tier terms in report. Don't "fix" by renumbering.

### Running the simulation

```bash
cd "Review 1/Implementation"
pip install -r requirements.txt   # networkx, numpy, matplotlib, osmnx (osmnx optional — synthetic path falls back without it)
python run_demo_syn.py            # synthetic airspace graph demo
python run_demo_osm.py            # OpenStreetMap-backed demo (requires network access for OSM + optionally osmnx)
```

Tests (unittest, offline): `python -m unittest discover -s Implementation/tests`. Also run demo scripts, inspect printed Pareto front + figures under `outputs/`. Sandbox blocks Open-Meteo: use `--offline` or `--wind-override`.

### Tier 1 → Tier 2 connection (matters when editing Tier 2 logic)

Payload-tag-to-weight mapping: not lookup table, continuous formula. Deliberate — answers review-panel critique, documented `FYP_B28_Project_Context.md`:

```
r_raw = F × 2        d_raw = U × 3        e_raw = T × 2
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
