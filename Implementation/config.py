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
# Global-data public instances; Private.coffee formerly used the Kumi hostname.
# Base URLs, without /interpreter. Override here for a self-hosted deployment.
OVERPASS_ENDPOINTS = (
    "https://overpass-api.de/api",
    "https://overpass.private.coffee/api",
    "https://overpass.kumi.systems/api",
)
OVERPASS_USER_AGENT = (
    "FYP-B28-UAV-Route-Planner/1.0 "
    "(SSN College of Engineering; academic OSM research)"
)
OVERPASS_QUERY_TIMEOUT_S = 60
OVERPASS_CONNECT_TIMEOUT_S = 8
OVERPASS_READ_TIMEOUT_S = 90     # allow server query timeout plus response transfer
OVERPASS_ATTEMPTS_PER_ENDPOINT = 2
OVERPASS_RETRY_DELAY_S = 2
OVERPASS_BUSY_DELAY_S = 30       # minimum pause for HTTP 406/429/5xx
OVERPASS_MAX_RETRY_WAIT_S = 60   # longer Retry-After: stop rather than retry early
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

# ── Population / road exposure (SORA-style ground-risk class lookup) ───────
# Class ∈ [0, 1] per OSM feature; the edge term is the mean over along-edge samples.
POPULATION_CLASS_ASSEMBLY = 1.0       # schools, hospitals, universities, marketplaces
POPULATION_CLASS_COMMERCIAL = 0.7     # commercial / retail land use
POPULATION_CLASS_ROAD = 0.6           # motorway / trunk / primary roads (people and vehicles)
POPULATION_CLASS_RESIDENTIAL = 0.5
POPULATION_CLASS_INDUSTRIAL = 0.3
POPULATION_ASSEMBLY_AMENITIES = {"school", "hospital", "college", "university", "marketplace"}
POPULATION_MAJOR_ROADS = {"motorway", "trunk", "primary"}
ROAD_BUFFER_M = 15.0                  # half-width given to road centre-lines
POI_BUFFER_M = 60.0                   # radius given to point amenities

# ── Restricted / no-fly zones ──────────────────────────────────────────────
# Hard: a node inside a zone, or an edge whose segment intersects one, is removed.
# Soft: outside a zone, ρ_nfz = exp(−distance / NFZ_SOFT_LAMBDA_M), taken at the
# closest approach along the edge and dropped beyond NFZ_SOFT_CUTOFF · λ.
NFZ_SOFT_LAMBDA_M = 150.0
NFZ_SOFT_CUTOFF = 4.0
RESTRICTED_LINE_POINT_BUFFER_M = 100.0   # lines/points tagged restricted (runways, helipads)
# Manual list: (name, lat, lon, radius_m).  The centres and radii are APPROXIMATE, typed
# from memory and NOT verified — check them against the Digital Sky map / DGCA before
# enabling.  Off by default: an unverified zone must not silently remove real airspace
# (the Raj Bhavan buffer clips the NW corner of the default box).
USE_MANUAL_NO_FLY_ZONES = False
MANUAL_NO_FLY_ZONES = [
    ("Chennai airport (MAA) red-zone buffer", 12.9941, 80.1709, 5000.0),
    ("Raj Bhavan, Guindy", 13.0090, 80.2290, 600.0),
    ("Guindy National Park", 13.0067, 80.2206, 1200.0),
]

# ── Water (payload loss), comm/GPS and turbulence ──────────────────────────
WATERWAY_BUFFER_M = 15.0              # half-width given to waterway centre-lines
SEA_OFFSET_M = 10_000.0               # coastline → sea polygon (sea is on the way's right)
DENSITY_REF = 0.30                    # building plan-area ratio at which density saturates
COMM_HEIGHT_REF_M = 30.0              # mean building height at which the canyon term saturates

# ── Weather go/no-go gate (stored on the graph, respected by Tier 2) ───────
NO_GO_WEATHER_CODES = (95, 96, 97, 98, 99)   # WMO thunderstorm
NO_GO_PRECIP_MM_H = PRECIP_HEAVY_MM_H
NO_GO_VISIBILITY_M = 1000.0
NO_GO_GUST_KMH = 45.0                        # PLACEHOLDER limit for an F450-class UAV

# ── Risk weights: ρ = clip(Σ w_k · ρ_k, 0, 1), every ρ_k ∈ [0, 1] ───────────
# ρ is the per-edge probability-like score; the objective sums the length-weighted
# exposure ρ · d_h / spacing (see module1.apply_weather).
RISK_WEIGHTS = {
    "obstacle": 1.0,        # synthetic obstacle discs
    "building": 0.60,       # footprint proximity × vertical margin
    "terrain": 0.30,        # clearance margin, sudden altitude change
    "wind": GAMMA_WIND,     # crosswind
    "gust": GAMMA_WIND,     # gust excess over the mean wind
    "weather": 0.20,        # rain, visibility
    "population": 0.50,     # people / road exposure
    "nfz": 0.80,            # soft buffer around restricted zones
    "water": 0.30,          # fraction of the edge over water
    "comm": 0.20,           # urban-canyon comm / GPS loss
    "turbulence": 0.20,     # building density × wind
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
