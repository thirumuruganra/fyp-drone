# Wind Speed Integration Plan
## FYP B28 — Payload-Aware Two-Tier UAV Route Planning
### Phase II Extension: Wind-Aware Airspace Graph (`run_demo_osm.py`)

---

## 1. Motivation

The current `run_demo_osm.py` computes edge risk using only obstacle proximity and adds
a random noise term labelled "environmental variation." Wind is listed as a planned
risk component in Section 3.2.3 of the report but is not yet implemented.

Wind affects UAV flight in two independent ways that map directly onto existing cost
objectives:

| Effect | Objective affected | Mechanism |
|---|---|---|
| Headwind increases motor load | Energy `ε(e)` | Drone pushes against oncoming air |
| High wind speed causes instability | Risk `ρ(e)` | Lateral correction increases crash probability |

Integrating a live wind API completes the edge cost model, makes the energy and risk
objectives reflect real atmospheric conditions, and produces a Pareto front that
genuinely changes with weather — a strong demonstration for Review 2 / Phase II.

---

## 2. Selected API: Open-Meteo

**Why Open-Meteo:**

- Completely free, no API key or account required
- Global coverage (works for Thiruvanmiyur / Thiruvanmiyur, Tamil Nadu)
- Returns hourly wind speed (km/h) and wind direction (°) at 10 m altitude
- JSON response, no authentication headers needed
- Python `requests` call — no extra library beyond what is already installed

**Endpoint:**

```
GET https://api.open-meteo.com/v1/forecast
    ?latitude=12.9020
    &longitude=80.1322
    &hourly=windspeed_10m,winddirection_10m
    &forecast_days=1
    &timezone=Asia%2FKolkata
```

**Response (relevant excerpt):**

```json
{
  "hourly": {
    "time": ["2026-08-24T00:00", ...],
    "windspeed_10m": [8.2, 9.1, ...],     // km/h
    "winddirection_10m": [220, 215, ...]   // degrees (meteorological: FROM direction)
  }
}
```

Use the value at the **current hour** (index matched by wall-clock time) so each demo
run reflects the live wind at Thiruvanmiyur at the moment it runs.

---

## 3. Mathematical Models

### 3.1 Edge Bearing

To compute the wind component along each edge, the bearing (compass heading) of each
edge must be derived from its endpoint coordinates:

```
θ_edge = atan2(lon_v − lon_u, lat_v − lat_u)   [converted to 0–360°]
```

### 3.2 Wind-Adjusted Energy Cost  (Eq. 3.1 extension)

Decompose the wind vector into a **headwind component** (along the edge) and a
**crosswind component** (perpendicular to the edge):

```
θ_rel        = θ_edge − θ_wind_to          (where θ_wind_to = θ_wind_from + 180°)

HW(e)        = v_wind × cos(θ_rel)         headwind (+) / tailwind (−)
CW(e)        = v_wind × |sin(θ_rel)|       always ≥ 0

penalty_hw   = β_head  × max(0, HW(e)) / V_CRUISE
penalty_cw   = β_cross × CW(e)         / V_CRUISE

ε_wind(e) = d(e) × (1 + |grade| + penalty_hw + penalty_cw)
```

**Constants (tunable):**

| Symbol | Default | Meaning |
|---|---|---|
| `V_CRUISE` | 10 m/s (36 km/h) | UAV cruising air speed |
| `β_head` | 0.40 | Headwind energy penalty coefficient |
| `β_cross` | 0.15 | Crosswind energy penalty coefficient |

Tailwind (`HW < 0`) is clamped to zero — it reduces flight time but the motor load
model does not yet account for energy *recovery* (conservative assumption).

### 3.3 Wind-Adjusted Risk Score  (Eq. 3.2 extension — new term)

High wind speed raises the probability of loss of attitude control:

```
ρ_wind(e) = γ × clip(v_wind / V_MAX_SAFE, 0, 1)

ρ_total(e) = clip(ρ_obstacle(e) + ρ_wind(e) + noise, 0, 1)
```

| Symbol | Default | Meaning |
|---|---|---|
| `γ` | 0.30 | Maximum wind contribution to risk (at gale force) |
| `V_MAX_SAFE` | 40 km/h | Wind speed above which UAV operation is considered high-risk |

At 8 km/h (light breeze) → ρ_wind ≈ 0.06 (negligible).  
At 25 km/h (fresh breeze) → ρ_wind ≈ 0.19 (meaningful).  
At 40 km/h (gale) → ρ_wind = 0.30 (approaches the open-sky base risk).

---

## 4. Implementation Plan

### Step 1 — Add `fetch_wind()` utility  (new function in `run_demo_osm.py`)

```python
import requests
from datetime import datetime, timezone

THIRUVANMIYUR_LAT =  12.9829
THIRUVANMIYUR_LON =  80.2555
V_CRUISE_KMH   =  36.0       # 10 m/s
V_MAX_SAFE_KMH =  40.0
BETA_HEAD       =  0.40
BETA_CROSS      =  0.15
GAMMA_WIND      =  0.30

def fetch_wind() -> tuple[float, float]:
    """
    Fetch current wind speed (km/h) and direction (°, FROM) from Open-Meteo.
    Returns (wind_speed_kmh, wind_direction_deg).
    Falls back to (0.0, 0.0) on network error so the demo still runs offline.
    """
    url = (
        "https://api.open-meteo.com/v1/forecast"
        f"?latitude={THIRUVANMIYUR_LAT}&longitude={THIRUVANMIYUR_LON}"
        "&hourly=windspeed_10m,winddirection_10m"
        "&forecast_days=1&timezone=Asia%2FKolkata"
    )
    try:
        resp = requests.get(url, timeout=10)
        resp.raise_for_status()
        data   = resp.json()["hourly"]
        hour   = datetime.now(timezone.utc).hour
        speed  = float(data["windspeed_10m"][hour])
        dirn   = float(data["winddirection_10m"][hour])
        print(f"[Wind API] Speed={speed:.1f} km/h  Direction={dirn:.0f}° (from)")
        return speed, dirn
    except Exception as exc:
        print(f"[Wind API] Fetch failed ({exc}) — using calm conditions (0 km/h)")
        return 0.0, 0.0
```

### Step 2 — Add `_edge_bearing()` helper to `OSMAirspaceGraphBuilder`

```python
import math

def _edge_bearing(self, G: nx.DiGraph, u, v) -> float:
    """Compass bearing (0–360°) from node u to node v."""
    dx = G.nodes[v]["x"] - G.nodes[u]["x"]   # Δlon
    dy = G.nodes[v]["y"] - G.nodes[u]["y"]   # Δlat
    bearing = math.degrees(math.atan2(dx, dy)) % 360
    return bearing
```

### Step 3 — Replace `_energy_cost()` with `_energy_cost_wind()`

```python
def _energy_cost_wind(
    self, distance_km: float, edge_data: dict,
    bearing_deg: float, wind_speed_kmh: float, wind_dir_from_deg: float
) -> float:
    """
    Wind-adjusted energy proxy (Eq. 3.1 extended).
    Headwind and crosswind add a proportional energy penalty.
    """
    grade = abs(edge_data.get("grade", 0.0)) if edge_data else 0.0
    if grade == 0.0:
        rng   = np.random.default_rng(int(distance_km * 1e4) % (2**31))
        grade = float(rng.uniform(0.0, 0.3))

    wind_to_deg  = (wind_dir_from_deg + 180.0) % 360.0
    theta_rel    = math.radians(bearing_deg - wind_to_deg)
    hw           = wind_speed_kmh * math.cos(theta_rel)   # + = headwind
    cw           = wind_speed_kmh * abs(math.sin(theta_rel))

    penalty_hw   = BETA_HEAD  * max(0.0, hw) / V_CRUISE_KMH
    penalty_cw   = BETA_CROSS * cw           / V_CRUISE_KMH

    return round(distance_km * (1.0 + grade + penalty_hw + penalty_cw), 5)
```

### Step 4 — Extend `_risk_score_scaled()` with a wind term

Add one parameter `wind_speed_kmh` and append the wind risk term:

```python
def _risk_score_scaled(
    self, u, v, G, x_min, y_min, span_x, span_y,
    wind_speed_kmh: float = 0.0          # ← new parameter
) -> float:
    # ... existing obstacle proximity logic unchanged ...
    rng        = np.random.default_rng(int(abs(hash((u, v)))) % (2**31))
    noise      = float(rng.uniform(-0.05, 0.05))
    rho_wind   = GAMMA_WIND * min(wind_speed_kmh / V_MAX_SAFE_KMH, 1.0)
    return round(float(np.clip(base_risk + rho_wind + noise, 0.0, 1.0)), 4)
```

### Step 5 — Update `fetch_graph()` in `OSMAirspaceGraphBuilder`

Call `fetch_wind()` once at the start of graph construction and pass both values to
the edge-building loop:

```python
def fetch_graph(self) -> nx.DiGraph:
    # ... existing OSM download + node building ...

    wind_speed, wind_dir = fetch_wind()     # ← fetch once, use for all edges
    print(f"[Module 1 — OSM] Wind conditions: "
          f"{wind_speed:.1f} km/h from {wind_dir:.0f}°")

    for (u, v), data in best_edges.items():
        d_km    = data["length"] / 1000.0
        bearing = self._edge_bearing(G, u, v)
        eps     = self._energy_cost_wind(d_km, data, bearing, wind_speed, wind_dir)
        rho     = self._risk_score_scaled(
            u, v, G, x_min, y_min, span_x, span_y,
            wind_speed_kmh=wind_speed           # ← pass to risk
        )
        G.add_edge(u, v,
                   distance=round(d_km, 5),
                   energy=round(eps, 5),
                   risk=round(rho, 5),
                   cost=[round(d_km, 5), round(eps, 5), round(rho, 5)])
    # ...
```

### Step 6 — Print wind conditions in the final summary

In `main()`, after Step 1 returns, add a one-line wind summary to the console output
and to the Module 1 graph title so the condition is visible in the saved PNG.

---

## 5. Files Changed

| File | Change |
|---|---|
| `run_demo_osm.py` | Add `fetch_wind()`, `_edge_bearing()`, `_energy_cost_wind()`, update `_risk_score_scaled()` and `fetch_graph()` |
| `module1_graph_builder.py` | No change (OSM subclass lives in `run_demo_osm.py`) |
| `module2_tier1_moastar.py` | No change (consumes edge costs, agnostic to how they're set) |
| `module4_tier2_payload.py` | No change (consumes Pareto front, agnostic to cost model) |

One new dependency: `requests` (already in most Python environments; add to
`requirements.txt`).

---

## 6. Testing Approach

1. **Offline fallback** — disconnect Wi-Fi and run: output should warn "Fetch failed"
   and produce identical results to the current (calm) baseline.

2. **Headwind sensitivity** — manually override `wind_speed_kmh=30, wind_dir_from_deg=90`
   and check that east-bound edges have higher energy than west-bound edges of the
   same length.

3. **Pareto front change** — compare `osm_module2_pareto_front_3d.png` on a calm day
   vs a windy day: the Energy axis should shift right (higher energy costs across all
   routes) and the Risk axis should shift up uniformly (wind risk added to every edge).

4. **Route divergence** — confirm that high wind conditions cause Fragile Medical
   (high w_risk) to select a different Pareto point than on a calm day, while
   Emergency Medicine (high w_dist) is less affected.

---

## 7. Limitations and Future Work

| Limitation | Future improvement |
|---|---|
| Single wind sample for the whole graph (spatially uniform) | Use a grid of lat/lon points and interpolate per-edge |
| 10 m altitude wind used; UAV flies at 50–120 m | Use Open-Meteo pressure-level data (925 hPa ≈ 75 m) |
| Static wind (snapshot at run time) | Fetch forecast per time window; replan if wind changes mid-flight |
| No turbulence model | Add terrain-induced turbulence near buildings (OSM building footprints) |
| No wind-adjusted flight time | Derive ETA from ground speed = airspeed ± headwind component |

---

*Plan version: 1.0 — August 2026*  
*Author: FYP B28 Implementation Notes*
