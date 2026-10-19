# Implementation audit — 2026-10-06

All phase commits merged into latest remote `main`. Planned work still incomplete; merged ≠ fully implemented/validated.

## Branch evidence

- Remote refreshed with `git fetch origin main`.
- Audited `origin/main`: `bd3622f51470878fbc2b116e5b14c2108acee19e`, PR #8 merge.
- Current checkout: `codex/phase7-integration-extras`, `f4b33bc`. Tree identical to refreshed `origin/main`.
- Local `main`: `5a09428`, 25 commits behind remote. Branch unchanged during audit.
- Tests/demos ran against isolated `git archive origin/main` snapshot: `/private/tmp/fyp-main-audit-20261006`.
- Audit adds this document only. Algorithm code unchanged.

## Phase coverage

| Phase | Code coverage | Remaining |
|---|---|---|
| 0 — sync/config/dependencies | Present | Local `main` needs fast-forward; docs stale in places |
| 1 — lattice/shared wind pipeline | Present; offline runs pass | Full live OSM layer verification |
| 2 — terrain/building altitude | Present; fixtures pass; live Open-Meteo DEM works | Actual OSM building fetch failed during audit |
| 3 — component risk/NFZ/weather gate | Present; offline tests pass | Risk calibration; live layer validation; dashboard gate propagation |
| 4 — NAMOA*/front storage | Present; complete offline fronts; cache reuse observed | Full report-scale performance validation |
| 5 — validation/feasibility/export | Present in detailed selection/CLI | All-presets path still bypasses battery filtering; battery model placeholder |
| 6 — evaluation finish | Partial | Real lattice benchmark, calm/windy comparison, true density variation, per-run MOPSO statistics; monotonicity aggregation bug |
| 7 — mission/dashboard | Present; mission serialization tests pass | Dashboard defects, altitude-frame correction, Tk runtime, SITL/MAVLink/hardware validation |

## Runs and outcomes

Python: existing repo `.venv`, Python 3.14.7, OSMnx 2.1.1. `MPLCONFIGDIR=/private/tmp/fyp-mpl-audit` for writable plot cache.

| Command | Result |
|---|---|
| `python -m unittest discover -s Implementation/tests -v` | 193 tests pass; one failed module import. Loader reports 194 entries. `test_dashboard_helpers` cannot import: `ModuleNotFoundError: No module named '_tkinter'`; its three tests not executed |
| `python run_demo_syn.py` | Exit 0; complete 5-route front; figures written; 2.2 s |
| `python run_demo_osm.py --offline --payload 3 2 2 3` | Exit 0; 893 nodes; complete 205-route front; 5.6 s; battery HOLD at 59.20 Wh usable capacity |
| `python run_demo_osm.py --offline --box-km 2 --payload 3 2 2 3` | Exit 0; complete 13-route front; selected index 5; JSON/CSV written; 1.1 s |
| Same 2 km offline command again | Existing 13-route front reused; selected index 5 |
| `python run_benchmarks.py --smoke --scenario S1 --output-dir outputs/audit_benchmarks` | Exit 0; three scenarios; complete fronts of 5/1/1 routes; 675 sweep rows |
| Live 2 km demo, same payload | Exit 0; 25.9 s; live weather and DEM; OSM fetch connection reset → empty OSM layers. One-route front; selection index 0 |
| Selected JSON → QGC WPL export → parser | Four mission items; final command 20/RTL. Serialization succeeds; external import/flight unverified |

Live run bounded to 90 s; OSMnx request timeout 12 s, rate-limit waiting disabled for audit. Live weather: 3.9 km/h FROM 270°, gust 19.8 km/h. DEM: Open-Meteo, 100 points, 0–14 m. OSM log: `ConnectionResetError(54, 'Connection reset by peer')`, followed by zero buildings/restricted/water/exposure layers. Successful process exit does **not** prove real OSM integration.

Offline OSM fixtures hand-made; not real geographic evidence. Default 6 km route spans opposite lattice corners; mixed-cargo battery HOLD expected under current placeholder energy calibration.

Logs: `/private/tmp/fyp-main-audit-tests.log`, `fyp-main-audit-syn.log`, `fyp-main-audit-osm.log`, `fyp-main-audit-osm-small.log`, `fyp-main-audit-osm-reuse.log`, `fyp-main-audit-benchmark.log`, `fyp-main-audit-live.log`. Generated audit outputs stay in temporary snapshot.

## Confirmed defects / missing implementation

1. **Dashboard bypasses weather HOLD.** `dashboard.py:52` constructs `TierTwoSelector(pareto_front)` without graph `no_go_reasons`; `select_detailed()` does not infer gate from graph. Isolated execution of actual helper function: graph storm gate true → selected path, `hold_advisory=None`. Equivalent CLI selector with reasons → HOLD. Helper extracted through AST because installed Python lacks Tk; full UI not run.

2. **Dashboard graph/front mismatch after OSM demo.** Defaults in `dashboard.py:27` load synthetic `airspace_graph.pkl`; OSM demo saves `airspace_graph_osm.pkl` but overwrites shared `pareto_front.pkl`. After synthetic → 6 km OSM run: default graph 196 nodes, front 205 routes, 5,495 path-node references absent from graph. UI has no graph/front file picker. Needs matched artifact pair and validation.

3. **Mission altitude frames wrong for varying terrain.** `mission_export.py:185` writes per-node `agl_m` into frame 3. Frame 3 means altitude above home, not local terrain. Correct conversion needs planned AMSL minus home ground AMSL, or deliberately supported terrain frame. Home item uses global frame 0 with AGL too (`:167`). See [official MAVLink frame definitions](https://mavlink.io/en/messages/common.html#MAV_FRAME). Serialization tests cannot validate frame semantics.

4. **Default all-presets route selection bypasses battery filtering.** `run_all_presets()` calls `select()`, whose `:333` argmax ignores feasibility. Only `select_detailed()` filters Wh. Use explicit `--payload` or `--preset` for current feasibility-aware CLI. Unify selection paths before treating every demo/dashboard/benchmark pick as feasible.

5. **Benchmark monotonicity mixes scenarios.** `module5_evaluation.py:400` keys rows only by F/W/U/P. `run_benchmarks.py:237` supplies all scenarios together; last scenario overwrites earlier tag rows. S1 smoke reports 660 violations; each scenario checked separately has zero. Independent reproduction: two single-route fronts individually yield zero; combined yield 480 false violations. Include scenario identity/grouping in comparisons and output.

6. **Density settings do not reach obstacle builder.** `run_benchmarks.py:74` omits `obstacle_density=density`. `_apply_density_risk()` scales/clips final risk instead. Labels 5/20/40% therefore do not measure intended obstacle coverage. Scaling final cumulative exposure also diverges from shared component cost pipeline.

7. **Phase 6 missing comparisons/statistics.** Runner has synthetic S1/S2/S3 only. No real Thiruvanmiyur lattice benchmark; no calm-vs-windy HV/front/selected-route comparison. MOPSO repeats pooled into one route union; only runtime averaged; per-repeat HV/IGD/spread mean ± std absent. Defaults 100 particles / 200 iterations / 30 reps present, but report-scale experiment not run in audit. Weighted A*, component tables, payload-alignment, 225-tag sweep, latency outputs present.

8. **Validation/calibration unfinished.** Tk-enabled Python absent here. OSM live layer fetch failed; fail-fast/data-quality status needed before claiming realistic OSM result. Battery constants explicitly PLACEHOLDER in `config.py:133`; risk coefficients need empirical calibration. Mission Planner/QGroundControl import, MAVLink upload, SITL route adherence/latency, hardware flights unverified. `SITL_NOTES.md` already states not run.

9. **Documentation drift.** `FYP_B28_Project_Context.md` still describes road graph/no DEM/no building layer despite current lattice implementation. `AGENTS.md` retains F/T/U temperature wording in one module description. Plan's initial status historical; phase merges alone insufficient completion evidence.

## Run algorithm with OSM

From clean working tree, update local main:

```bash
cd /Users/meghna/Desktop/fyp-drone
git switch main
git pull --ff-only origin main
source .venv/bin/activate
python -m pip install -r Implementation/requirements.txt
cd Implementation
python run_demo_osm.py --box-km 2 --payload 3 2 2 3
```

Tags: F fragility 1–5; W payload weight class 1–3; U urgency 1–3; P priority 1–5. Alternative: `--preset mixed_cargo`. Default region Thiruvanmiyur; actual UAV graph airspace lattice, OSM supplies buildings/restrictions/water/exposure. Source/target auto-selected distant pair; no CLI depot/destination coordinates yet. `ParetoStore.run_region()` supports points programmatically.

Check console for live/cached **real** OSM layer counts, DEM provider, weather source, complete front, and GO/HOLD. `ALL STEPS COMPLETE` alone insufficient: failed layer fetch deliberately falls back to empty layers. OSM cache preferred over new fetch; `--refresh` refreshes Tier 1 front, not OSM layers.

```bash
# Offline demo: cache, else hand-made fixtures.
python run_demo_osm.py --offline --box-km 2 --payload 3 2 2 3

# Controlled windy demo; rebuild front.
python run_demo_osm.py --offline --box-km 2 --wind-override 30 90 --refresh --payload 3 2 2 3

# Full default 6 km region; battery HOLD possible.
python run_demo_osm.py --payload 3 2 2 3
```

Outputs: `outputs/osm_*.png`, `outputs/tier2/selection_custom.json`, `.csv`; persistent fronts under `pareto_front_storage/fronts/`. Repeat unchanged run reuses front. Fix dashboard/export defects before flight integration; current CLI suitable simulation/demo.
