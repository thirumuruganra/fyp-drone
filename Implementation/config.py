"""
config.py — single home for every tunable constant.

Nothing here reads the network or the disk; modules import what they need.
Values marked PLACEHOLDER are used by later phases (Tier 2 feasibility) and
must be calibrated before being quoted in the report.
"""

# ── Region (Thiruvanmiyur, Chennai) ─────────────────────────────────────────
LOCATION_NAME = "Thiruvanmiyur, Chennai, Tamil Nadu, India"
CENTER_LAT = 12.9830
CENTER_LON = 80.2594
BOX_M = 6000.0            # square side of the lattice box
LATTICE_SPACING_M = 200.0
EARTH_RADIUS_KM = 6371.0088

# ── Flight heights (Drone Rules 2021: ≤ 120 m AGL in the green zone) ───────
H_CRUISE_AGL = 60.0       # m, nominal cruise height above ground
CLEARANCE = 15.0          # m, vertical margin above the tallest obstacle
H_MAX_AGL = 120.0         # m, legal ceiling

# ── Wind / energy model (see wind_integration_plan.md v1.1) ────────────────
V_CRUISE_KMH = 36.0       # UAV airspeed, 10 m/s
BETA_HEAD = 0.40          # energy penalty per (headwind / V_CRUISE)
BETA_CROSS = 0.15         # energy penalty per (crosswind / V_CRUISE)
GAMMA_WIND = 0.30         # max contribution of the wind term to risk
V_MAX_SAFE_KMH = 40.0     # wind speed at which the wind risk term saturates
K_GUST = 0.5              # weight of (gust − mean wind) in the wind risk term
ALPHA_SHEAR = 0.25        # power-law exponent for 10 m → flight height
WIND_REF_HEIGHT_M = 10.0  # height of Open-Meteo's wind_speed_10m
WIND_HEIGHT_80M = 80.0    # height of Open-Meteo's wind_speed_80m

# ── Weather risk term (uniform over the graph) ─────────────────────────────
PRECIP_HEAVY_MM_H = 7.5   # precipitation at which the rain term saturates
VIS_CLEAR_M = 10_000.0    # visibility at/above which the visibility term is 0
W_PRECIP = 0.5            # share of the weather term from rain
W_VIS = 0.5               # share of the weather term from poor visibility

# ── Terrain / buildings (Phase 2) ──────────────────────────────────────────
GRADE_MAX = 0.30                 # |grade| at which the sudden-altitude-change term saturates
EDGE_SAMPLE_STEP_M = 25.0        # along-edge sampling step for clearance / footprint terms
BUILDING_NEAR_M = 30.0           # "near a footprint" distance
BUILDING_MARGIN_REF_M = 60.0     # vertical margin above a nearby roof at which risk is 0
OSM_FETCH_RADIUS_M = 4500.0      # features_from_point radius (covers the 6 km box corners)
DEM_PROVIDER_TYPE = "COP30"      # OpenTopography global DEM (COP30 | COP90 | SRTMGL1 | ...)
OPENTOPO_ENV_KEY = "OPENTOPO_API_KEY"
DEM_RASTER_ENV = "DEM_RASTER_PATH"   # optional local GeoTIFF (SRTM / Copernicus)
ELEVATION_BATCH = 100            # Open-Meteo elevation API: coordinates per request
ELEVATION_ROUND_DEG = 5          # cache key precision (≈ 1 m)
# Building height assumption (Chennai OSM height coverage is sparse):
#   height=*  →  building:levels × LEVEL_HEIGHT_M  →  per-type default
LEVEL_HEIGHT_M = 3.0
BUILDING_DEFAULT_HEIGHT_M = {"residential": 9.0, "commercial": 15.0, "other": 10.0}
BUILDING_RESIDENTIAL_TYPES = {"residential", "house", "apartments", "detached",
                              "semidetached_house", "terrace", "dormitory",
                              "bungalow", "hut", "cabin"}
BUILDING_COMMERCIAL_TYPES = {"commercial", "retail", "office", "hotel", "supermarket",
                             "kiosk", "mall", "warehouse", "industrial"}

# ── Risk weights: ρ = clip(Σ w_k · ρ_k, 0, 1), every ρ_k ∈ [0, 1] ───────────
# Components so far: obstacle, wind, weather (Phase 1), building, terrain
# (Phase 2).  Phase 3 adds population, NFZ, comm/GPS, water and turbulence.
RISK_WEIGHTS = {
    "obstacle": 1.0,
    "wind": GAMMA_WIND,
    "weather": 0.20,
    "building": 0.60,
    "terrain": 0.30,
}

# ── Synthetic scenarios (seeded, deterministic) ────────────────────────────
SYN_ORIGIN_LAT = 12.70
SYN_ORIGIN_LON = 80.10
SYN_SPACING_DEG = 0.012           # ≈ 1.3 km between grid nodes
SYN_JITTER_DEG = 0.002
SYN_OBSTACLE_DENSITY = 0.20       # fraction of the area covered by obstacle discs
SYN_OBSTACLE_RADIUS_FRAC = 0.12   # disc radius as a fraction of the grid span
SYN_OBSTACLE_PEAK = 0.90          # obstacle risk at a disc centre
SYN_WIND_SPEED_RANGE_KMH = (12.0, 28.0)
SYN_GUST_FACTOR = 1.4
SYN_TERRAIN_HILLS = 4
SYN_TERRAIN_AMP_M = 220.0         # hill height range upper bound
SYN_TERRAIN_SIGMA_FRAC = 0.09     # hill width as a fraction of the grid span

# ── Battery / payload (Tier 2 feasibility, Phase 5) — PLACEHOLDER ──────────
M_UAV_KG = 1.5                    # F450 airframe + battery
BATTERY_WH = 74.0                 # 4S 5000 mAh
BATTERY_RESERVE = 0.20
K_CAL_WH_PER_KM = 6.0             # Wh per km of energy proxy at M_UAV
PAYLOAD_MASS_KG = {1: 0.25, 2: 0.50, 3: 1.00}   # payload weight class W → kg

# ── Front-store drift thresholds (Phase 4) ─────────────────────────────────
DRIFT_WIND_SPEED_KMH = 10.0
DRIFT_WIND_DIR_DEG = 45.0
DRIFT_DIR_MIN_SPEED_KMH = 10.0
DRIFT_GUST_KMH = 15.0
FRONT_VALID_HOURS = 3.0

# ── Paths ───────────────────────────────────────────────────────────────────
CACHE_DIR = "cache"
