# Implementation: Payload-Aware Two-Tier UAV Route Planner

Python implementation of the planner described in the report.

- **Tier 1** builds the full Pareto front of routes (distance, wind-aware energy, flight risk)
  over a 3-D airspace lattice with NAMOA*. It runs once per delivery region and weather window,
  and the front is stored.
- **Tier 2** reads four payload tags, turns them into objective weights with a fixed formula and
  picks the best route from the stored front with `argmax(score)`. No re-search and no manual
  re-tuning per mission.

Payload tags: **F** fragility 1-5, **W** payload weight class 1-3, **U** urgency 1-3, **P** priority 1-5.

```
r_raw = 2F      d_raw = 3U      e_raw = 2W        S = r_raw + d_raw + e_raw
w_risk = r_raw/S    w_distance = d_raw/S    w_energy = e_raw/S
P >= 4: the largest weight gets +10 %, then renormalise to 1
score(p) = w_distance(1 - d~) + w_energy(1 - e~) + w_risk(1 - r~)     (objectives min-max normalised)
```

## Getting the code

The code is shared only through the OneDrive folder `Batch_28`; there is no Git repository to
clone. Download `Batch_28` from OneDrive (it arrives as a zip), extract it outside OneDrive and run
from the extracted `Batch_28` folder. The scripts write `cache/`, `outputs/` and
`pareto_front_storage/` next to themselves, and those should not be synced back into OneDrive.

Keep the layout:

```
Batch_28/SourceCode/
  .env.example      copy to .env for the optional OpenTopography key
  Implementation/   this folder; run every command below from here
  Simulation/       Gazebo + SITL (imports this folder)
```

## Setup

```bash
cd Batch_28/SourceCode/Implementation
python -m venv .venv                # optional; activate: .venv\Scripts\activate (Windows) or . .venv/bin/activate
pip install -r requirements.txt     # Python 3, tested on 3.12
```

Windows: set `PYTHONUTF8=1` before running (PowerShell: `$env:PYTHONUTF8 = "1"`), otherwise some
console prints crash on the cp1252 code page. Tkinter (bundled with Python) is needed only for
`dashboard.py`.

Optional: an OpenTopography key gives the 30 m Copernicus DEM. Copy `Batch_28/SourceCode/.env.example` to
`Batch_28/SourceCode/.env` (one level above this folder) and fill in `OPENTOPO_API_KEY`. The entry scripts
read it from there. Without a key the 90 m Open-Meteo elevation API is used.

## Run

```bash
python run_demo_syn.py                                        # synthetic airspace, no network
python run_demo_osm.py                                        # real region: live OSM + DEM + weather
python run_demo_osm.py --offline --box-km 2 --payload 3 2 2 3 # fixtures, fast, no network
python run_benchmarks.py --smoke                              # quick benchmark check
python run_benchmarks.py --scenario S2                        # all, S1, S2 or S3
python run_ablation.py                                        # risk-layer ablation
python dashboard.py                                           # operator dashboard (run run_demo_syn.py once first)
python -m unittest discover -s tests                          # 224 offline tests
```

Useful `run_demo_osm.py` options: `--preset NAME` (`fragile_medical`, `emergency_medicine`,
`disaster_relief`, `mixed_cargo`), `--payload F W U P`, `--wind-override SPEED_KMH DIR_FROM_DEG`,
`--refresh` (rebuild the stored front), `--box-km`, `--spacing-m`, `--no-terrain`,
`--no-buildings`, `--no-risk-layers`.

`--offline` uses the hand-made fixtures in `tests/fixtures/`. They are **not real data** and exist
for tests and quick runs only.

Results go to `outputs/` (figures, CSV/JSON selections, `.waypoints` mission files), created on
first run. Downloaded layers are cached in `cache/`. Stored graphs and fronts go to
`pareto_front_storage/`. All three folders are created automatically and are not part of this submission.

## Files

| File | Role |
|---|---|
| `config.py` | Every tunable constant (region, heights, wind, risk weights, battery, endpoints) |
| `env_data.py` | Weather (Open-Meteo), DEM provider chain, OSM layers, spatial indexes, `.env` loader |
| `overpass_client.py` | Multi-endpoint Overpass client with retry; `check_overpass.py` is a connectivity probe |
| `module1_graph_builder.py` | Module 1: synthetic and lattice airspace graphs, edge cost pipeline (distance, energy, 11-component risk) |
| `module2_tier1_moastar.py` | Module 2: Tier 1 NAMOA* engine |
| `pareto_store.py` | Front store: build once, refresh when the graph changes, the 3 h window expires or the weather drifts |
| `module3_baselines.py` | Module 3: single-objective A*, weighted-sum A*, MOPSO |
| `module4_tier2_payload.py` | Module 4: payload tags to weights, scoring, selection, battery and weather HOLD |
| `module5_evaluation.py` | Module 5: hypervolume, IGD, spread, Tier 2 evidence, 225-tag sweep |
| `mission_export.py` | Module 6: QGroundControl / Mission Planner `QGC WPL 110` export |
| `dashboard.py` | Module 6: Tkinter operator dashboard |
| `run_demo_syn.py`, `run_demo_osm.py` | End-to-end demos (Modules 1, 2, 4) |
| `run_benchmarks.py`, `run_ablation.py` | Benchmark and ablation entry points |
| `map_data/` | Historical OSM road snapshot, used only as a display fallback when the live road fetch fails |
| `tests/` | `unittest` suite and offline fixtures |

## Known limits

- Battery constants and the weather no-go gust limit in `config.py` are placeholders, not calibrated.
- Risk weights are uncalibrated.
- The airspace is a grid lattice, not a road network.
- The dashboard does not enforce the weather HOLD and can load a graph and front that do not match.
- Data: OpenStreetMap contributors (ODbL), Copernicus DEM via OpenTopography, Open-Meteo.
