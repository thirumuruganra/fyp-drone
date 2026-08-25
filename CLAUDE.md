# CLAUDE.md

Guidance for Claude Code in this repo.

## Communication Style — Caveman Mode

Terse like caveman. Technical substance exact. Only fluff die.
Drop: articles, filler (just/really/basically), pleasantries, hedging.
Fragments OK. Short synonyms. Code unchanged.
Pattern: [thing] [action] [reason]. [next step].
ACTIVE EVERY RESPONSE. No revert after many turns. No filler drift.
Code/commits/PRs: normal. Off: "stop caveman" / "normal mode".

## Git Workflow

Semantic commits for all git commits.

After code change, AskUserQuestion. Print exact choices:
1. Commit + push
2. Commit only
3. Neither
Wait for user pick 1, 2, or 3.

## Project Overview

FYP (B28) repo. **"Payload-Aware Two-Tier UAV Route Planning for Last-Mile Delivery"** — Thirumurugan RA, Shrika SP Thota, Meghna Manimaran (Guide: Sujaudeen N), SSN College of Engineering, Dept. of CSE.

Mostly academic deliverables (LaTeX reports, presentations) + small Python sim under `Review 1/Implementation/`. No build system beyond LaTeX compile. No test suite for Python.

**Core idea:** split Pareto-front build (Tier 1) from mission route pick (Tier 2). Planner run once per delivery region, reused. Payload metadata picks point on front per mission — no manual weight re-tune between deliveries. Read `Review 1/FYP_B28_Project_Context.md` first — problem statement, lit gap, panel feedback history, tech decisions log. Canonical source for *why*, not just what. Read before touching algorithm concepts.

## Repository Layout

- `Abstract/`, `Initial Screening PPT/`, `Review 0/` — early submission artifacts. Frozen/historical.
- `Review 1/` — current phase deliverables:
  - `FYP_B28_Project_Context.md` — canonical context (problem, arch, math, panel Q&A, timeline). Read first.
  - `Implementation/` — actual Python sim (below).
  - `Phase 1 Report/` — LaTeX thesis chapters (`ch1_intro.tex` … `ch5_timeline.tex`), compiled via `main.tex` (uses `Thesis.cls`, `bmacros.tex`).
  - `Review 1 PPT/`, `Review 1 Report/` — LaTeX presentation/report, `ssnreview.cls`.
- `Reference Material (From Seniors - just for format)/` — prior year thesis, format reference only. Not project content.

## Implementation (`Review 1/Implementation/`)

Python sim, four modules, no package structure — flat scripts, imported direct by two demo entry points.

- `module1_graph_builder.py` — `AirspaceGraphBuilder`: builds airspace graph (synthetic, or subclass for OSM), assigns each edge 3-vector cost `[distance, energy, risk]`.
- `run_demo_osm.py` — `OSMAirspaceGraphBuilder(AirspaceGraphBuilder)`: OSM-backed variant, `osmnx`, real-world edge costs (grade-based energy proxy, obstacle-proximity risk).
- `module2_tier1_moastar.py` — `TierOneEngine`: multi-objective A*/Dijkstra search (`pareto_dominates`, `update_pareto_front`), produces non-dominated route set for source/target pair. Tier 1.
- `module4_tier2_payload.py` — `TierTwoSelector` + `derive_weights(F, T, U)`: converts four payload tags (fragility, temp sensitivity, urgency, priority) into objective weights via fixed formula (project context doc), scores every Pareto point, picks `argmax(score)`. Tier 2.
- `run_demo_syn.py` / `run_demo_osm.py` — end-to-end demo scripts (`main()`), wire modules 1 → 2 → 4 on synthetic and OSM graphs, print step banners, render summary figures via Matplotlib.
- `wind_integration_plan.md` — design doc, not-yet-built wind-aware extension (headwind/crosswind energy penalty, wind-speed risk term via Open-Meteo API). Read before touching wind-related edge costs — exact formulas, constants, which functions change.
- `cache/`, `outputs/`, `pareto_front_storage/`, `__pycache__/` — generated artifacts, not source.

Module numbering skips 3 (1, 2, 4 — no `module3`) — matches tier terms in report. Don't "fix" by renumbering.

### Running the simulation

```bash
cd "Review 1/Implementation"
pip install -r requirements.txt   # networkx, numpy, matplotlib, osmnx (osmnx optional — synthetic path falls back without it)
python run_demo_syn.py            # synthetic airspace graph demo
python run_demo_osm.py            # OpenStreetMap-backed demo (requires network access for OSM + optionally osmnx)
```

No automated tests. Correctness check: run demo scripts, inspect printed Pareto front + generated summary figures under `outputs/`.

### The Tier 1 → Tier 2 connection (important when editing Tier 2 logic)

Payload-tag-to-weight mapping: not lookup table, continuous formula. Deliberate — answers review-panel critique documented in `FYP_B28_Project_Context.md`:

```
r_raw = F × 2        d_raw = U × 3        e_raw = T × 2
w_risk = r_raw / (r_raw + d_raw + e_raw)
w_distance = d_raw / (r_raw + d_raw + e_raw)
w_energy = e_raw / (r_raw + d_raw + e_raw)

score(p_i) = w_distance × (1 − d̃ᵢ) + w_energy × (1 − ẽᵢ) + w_risk × (1 − r̃ᵢ)   # normalized objectives
```

Selected route = `argmax(score)` over full Pareto front, not just extreme points. Keep this — reverting to min-distance/min-risk extreme picks kills project's novelty claim.

## LaTeX Reports

- `Phase 1 Report/main.tex` and `Review 1 Report/Review1_Report.tex` — main compile targets, own dirs. Both need local `.cls`/`.sty` files (`Thesis.cls` or `ssnreview.cls`, `bmacros.tex`, `setspace.sty`, `ssn.jpeg`, etc., sourced from `Reference Material (From Seniors - just for format)/FYP_Phase1_Report/`). Compile from same dir — relative includes need it.
- BibTeX between first two passes (per `Review 1/Phase 1 Report/README_COMPILE.txt`):
  ```bash
  pdflatex main.tex
  bibtex main
  pdflatex main.tex
  pdflatex main.tex
  ```
  Refs in `references.bib`, style `unsrtnat` (natbib). Build artifacts (`.aux`, `.bbl`, `.fls`, `.fdb_latexmk`, `.log`, `.out`, `.toc`) already present from prior builds — regenerate, don't hand-edit.
- `Phase 1 Report/diagrams/*.puml` — PlantUML sources for `arch`, `tier1`, `tier2`, `gantt` diagrams. Render: `java -jar plantuml.jar diagrams\<name>.puml` (or plantuml.com) → `.png` files LaTeX includes. Missing PNG → placeholder, not fail.
- Non-default LaTeX packages needed: `algpseudocode`/`algorithm` (algorithmicx), `booktabs`, `multirow`, `xcolor`, `url`. `glossaries` errors → comment out `\makeglossaries` in `Thesis.cls`.
- **After compile, delete all generated build artifacts** — keep only `.tex` source(s), final `.pdf`, style/class files `.tex` links (`.cls`, `.sty`, `bmacros.tex`, `references.bib`, image/diagram assets doc `\include`s). Remove rest: `.aux`, `.bbl`, `.blg`, `.fls`, `.fdb_latexmk`, `.log`, `.out`, `.toc`, `.lof`, `.lot`, `.synctex.gz`, similar. Cleanup pass after successful compile, not before — intermediates needed during multi-pass `pdflatex`/`bibtex` sequence itself.
