# Wind Speed Integration Plan
## FYP B28 — Payload-Aware Two-Tier UAV Route Planning
### Wind-aware airspace graph (Module 1) — **v1.1, implemented in Phase 1**

Status: implemented in `config.py`, `env_data.py` and `module1_graph_builder.py`
(tests: `tests/test_wind.py`, `tests/test_lattice.py`). This revision supersedes v1.0,
which had six defects, listed in §1.

---

## 1. Changes from v1.0

| # | v1.0 defect | v1.1 fix |
|---|---|---|
| 1 | Headwind sign inverted: `θ_rel = θ_edge − (θ_from + 180°)` made wind FROM 90° on an edge heading 90° a *tailwind* | `HW = v·cos(θ_edge − θ_from)` (positive = headwind), `CW = v·|sin(θ_edge − θ_from)|` |
| 2 | UTC hour indexed an IST (`Asia/Kolkata`) series: 5.5 h off | Hourly row chosen by matching the response's own `current.time` string (floored to the hour) against `hourly.time`; no clock conversion |
| 3 | Endpoint example (12.9020, 80.1322) did not match the constants (12.9829, 80.2555) | One source of truth: `config.CENTER_LAT/LON` = Thiruvanmiyur (12.9830, 80.2594), used for the lattice and the API call |
| 4 | Edge bearing `atan2(Δlon, Δlat)` had no `cos(lat)` scaling | Forward-azimuth formula (`initial_bearing_deg`); the east component carries `cos(lat)` |
| 5 | 10 m wind used for a UAV flying at 60–120 m | `wind_speed_80m` / `wind_direction_80m` from the hourly block; fallback: 10 m wind lifted with `v·(h/10)^0.25` to the 60 m cruise height |
| 6 | `ρ_wind` identical on every edge, so it only penalised hop count | Direction-dependent: `ρ_wind = clip((CW + k_g·max(0, gust − v)) / V_MAX_SAFE)` |

The gust excess uses a gust factor measured at 10 m (`wind_gusts_10m / wind_speed_10m`)
applied to the wind at flight height.

---

## 2. Data source: Open-Meteo (free, no API key)

```
GET https://api.open-meteo.com/v1/forecast
    latitude=12.983 & longitude=80.2594
    current=wind_speed_10m,wind_direction_10m,wind_gusts_10m,
            precipitation,weather_code,visibility
    hourly=wind_speed_80m,wind_direction_80m
    wind_speed_unit=kmh & forecast_days=1 & timezone=Asia/Kolkata
```

`env_data.fetch_weather()` returns a `WeatherSnapshot` and never raises:

1. live call (skipped with `--offline`); the snapshot is cached in `cache/weather_<lat>_<lon>.json`
2. otherwise the last cached snapshot (marked `cache(...)`)
3. otherwise calm conditions (0 km/h) — deterministic

Wind direction is meteorological (the direction the wind blows FROM, degrees from north).
The sandbox used for development blocks `api.open-meteo.com`; live runs happen on a
networked machine. An offline test fixture shaped like the response lives in
`tests/fixtures/open_meteo_chennai.json`.

---

## 3. Models

All constants live in `config.py`.

### 3.1 Edge bearing

`θ_edge = atan2(sin Δλ·cos φ₂, cos φ₁·sin φ₂ − sin φ₁·cos φ₂·cos Δλ)` in [0°, 360°).

### 3.2 Wind-adjusted energy

```
HW = v · cos(θ_edge − θ_from)         headwind (+) / tailwind (−)
CW = v · |sin(θ_edge − θ_from)|

ε = d · (1 + |grade| + β_head·max(0, HW)/V_cruise + β_cross·CW/V_cruise)      ε ≥ d
```

`d` is the 3-D edge length `√(d_h² + Δalt²)` and `grade = Δalt/d_h` (0 on the flat
lattice). Tailwind is clamped to zero benefit, so `ε ≥ d` and the Tier 1 heuristic
stays admissible.

| Symbol | Default | Meaning |
|---|---|---|
| `V_CRUISE_KMH` | 36 | UAV airspeed (10 m/s) |
| `BETA_HEAD` | 0.40 | headwind energy coefficient |
| `BETA_CROSS` | 0.15 | crosswind energy coefficient |

### 3.3 Risk

```
ρ = clip( Σ_k w_k · ρ_k , 0, 1 ),   every ρ_k ∈ [0, 1]

ρ_wind    = clip( (CW + K_GUST·max(0, gust − v)) / V_MAX_SAFE )     w = GAMMA_WIND = 0.30
ρ_weather = W_PRECIP·clip(precip / 7.5 mm/h) + W_VIS·clip(1 − visibility / 10 km)   w = 0.20
ρ_obstacle = static, from the builder                                w = 1.0
```

Each edge stores the weighted contributions in `risk_components` next to `risk`.
Components added in later phases (buildings, terrain, population, restricted zones,
comm/GPS, water, turbulence) extend `config.RISK_WEIGHTS`.

### 3.4 Pipeline

```
builder → static edge attrs (d_h, distance, bearing, grade, obstacle)
        → apply_weather(G, snapshot) → energy, risk, risk_components, cost, time_s
```

`apply_weather` is idempotent and can be re-run on the same graph with a new snapshot
(calm vs windy comparison, drift refresh). `time_s = d / ground_speed` with
`ground_speed = √(V² − CW²) − HW` is informational only.

---

## 4. Usage

```
python run_demo_osm.py                        # live wind → cache → calm
python run_demo_osm.py --offline              # cached snapshot, else calm
python run_demo_osm.py --wind-override 30 90  # 30 km/h blowing FROM the east
```

The wind is printed to the console and in the Module 1 figure title. The synthetic
benchmark graphs use a seeded wind (`env_data.seeded_weather`) and a seeded terrain
field, so they stay deterministic.

---

## 5. Testing

1. **Sign convention** — wind FROM 90° gives HW > 0 on an east-bound edge and HW < 0 on
   a west-bound edge (`test_wind.py`).
2. **Headwind sensitivity** — with `--wind-override 30 90`, east-bound lattice edges cost
   more energy than west-bound edges of the same length (`test_lattice.py`).
3. **Offline** — no network and no cache gives calm conditions; with a cache it reuses
   the stored snapshot.
4. **Timestamp** — the hourly row follows the response's IST time string.
5. **Front shift / route divergence** (Phase 6) — compare calm vs `--wind-override`
   fronts, and how far Fragile Medical's pick moves versus Emergency Medicine's.

---

## 6. Limitations

| Limitation | Future improvement |
|---|---|
| One wind sample for the whole 6 km box (spatially uniform) | Grid of API points, interpolated per edge |
| Wind at 80 m (or 60 m by power law), not per-edge altitude | Per-edge altitude once terrain/building clearance sets node heights (Phase 2) |
| Static snapshot at run time | Forecast per time window; drift-triggered refresh (Phase 4) |
| Uniform wind on a flat, obstacle-free lattice gives (nearly) no objective conflict, so the Phase 1 lattice front is often a single route | Terrain, buildings, restricted zones and population exposure (Phases 2–3) |
| No turbulence model | Building density × wind (Phase 3) |

---

*Plan version: 1.1 — implemented in Phase 1*
