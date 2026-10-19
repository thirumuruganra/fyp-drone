"""
Simulation settings (Gazebo world + ArduPilot SITL).  Planner tunables stay in
Implementation/config.py; only simulation-specific values live here.

Default region: a 1.5 km crop of Thiruvanmiyur around the planner centre, with
the depot in the south-west and three delivery pads in the other corners so
routes have to cross the built-up area.
"""

import _paths  # noqa: F401  (puts Implementation/ on sys.path)
import config as cfg

# ── Region ───────────────────────────────────────────────────────────────────
SIM_REGION_ID = "thiru_1500"
SIM_CENTER = (cfg.CENTER_LAT, cfg.CENTER_LON)   # lattice centre (planner frame origin)
SIM_BOX_M = 1500.0                              # square crop, side in metres
SIM_SPACING_M = 100.0                           # Tier 1 lattice spacing inside the crop
SIM_MARGIN_M = 150.0                            # extra terrain / buildings around the box

# Depot = Gazebo world origin = SITL home.  Destinations = landing pads.
SIM_DEPOT = (12.978683, 80.254601)
SIM_DESTINATIONS = [
    (12.987227, 80.264199),     # north-east
    (12.979133, 80.263830),     # south-east
    (12.987676, 80.255431),     # north-west
]

# Desktop profile: real Thiruvanmiyur/Adyar scene, short local delivery for recording.
# Pads are virtual simulation points, checked against downloaded OSM geometry.
DESKTOP_REGION_ID = "thiruvanmiyur_adyar"
DESKTOP_CENTER = (12.9955, 80.2580)
DESKTOP_BOX_M = 3200.0
DESKTOP_OSM_RADIUS_M = 1900.0
DESKTOP_SPACING_M = 150.0
DESKTOP_TERRAIN_RES_M = 30.0      # lighter city mesh; same frozen DEM and clearance checks
DESKTOP_DEPOT = (12.9860000, 80.2605000)
DESKTOP_DESTINATION = (12.9861799, 80.2625304)  # nearby virtual pad (~221 m east/north of depot)
DESKTOP_QGC_APP = "/Applications/QGroundControl.app"
# Tight drone framing for recording; wide city view remains an explicit option.
DESKTOP_CAMERA_OFFSETS = {"takeoff": (-4.0, -6.0, 3.0),
                          "follow": (-6.0, -8.0, 8.0),
                          "wide": (-120.0, -90.0, 35.0), "close": (-6.0, -8.0, 8.0)}
DESKTOP_OVERVIEW_ALT_M = 6.0    # visible ground liftoff → close follow through landing
# Ordered recording checkpoints: fraction along source→destination, lateral metres.
# Intentional corridor with eight corners; not an unconstrained shortest-path claim.
DESKTOP_TURN_CHECKPOINTS = ((0.0, 0.0), (0.12, 0.0), (0.12, 35.0),
    (0.34, 35.0), (0.34, 0.0), (0.57, 0.0), (0.57, 40.0),
    (0.79, 40.0), (0.79, 0.0), (1.0, 0.0))

# ── Terrain ──────────────────────────────────────────────────────────────────
SIM_DEM_FETCH_M = 90.0          # DEM sampling step (Copernicus 90 m via Open-Meteo); finer adds nothing
SIM_TERRAIN_RES_M = 10.0        # Gazebo terrain mesh resolution (bilinear from the DEM grid)
SIM_DSM_FILTER = "p10"          # "p10": 10th percentile in a 3×3 window (strips DSM roof bias); "none"

# ── Clearance (hard feasibility, Phase 3) ────────────────────────────────────
SIM_EDGE_BUFFER_M = 5.0         # horizontal buffer: drone size + tracking error + footprint error
SIM_TERRAIN_STEP_M = 10.0       # terrain sampling step along an edge
SIM_ROOF_CLEARANCE_M = cfg.CLEARANCE     # 15 m above any roof the buffered path crosses
SIM_TERRAIN_CLEARANCE_M = cfg.CLEARANCE  # 15 m above ground along the cruise path
SIM_BUILDING_BASE_SINK_M = 0.5  # sink building bases so none float on sloped ground
SIM_MIN_FOOTPRINT_M2 = 4.0      # drop slivers
SIM_PAD_CONNECT_MAX_M = 160.0   # furthest lattice node a pad may connect to

# ── Gazebo world ─────────────────────────────────────────────────────────────
SIM_TILE_M = 250.0              # building meshes merged per tile (fewer entities)
SIM_VEHICLE_MODEL = "iris_with_gimbal"   # from ardupilot_gazebo/models
SIM_VEHICLE_SPAWN_Z = 0.195              # as in ardupilot_gazebo iris_runway.sdf
SIM_MAX_FRONT_MARKERS = 20      # Tier 1 front routes drawn as thin lines
ROAD_WIDTH_M = {"motorway": 14.0, "trunk": 12.0, "primary": 10.0, "secondary": 8.0,
                "tertiary": 7.0, "residential": 5.0}
ROAD_WIDTH_DEFAULT_M = 4.0
HEIGHT_SOURCE_COLOURS = {       # RGBA; colour shows where each building height came from
    "tagged": (0.62, 0.62, 0.66, 1.0),
    "levels": (0.86, 0.78, 0.62, 1.0),
    "default": (0.90, 0.60, 0.58, 1.0),
}

# ── SITL / mission ───────────────────────────────────────────────────────────
SIM_CRUISE_SPEED_MS = 5.0       # WPNAV_SPEED (conservative: limits corner cutting)
SIM_MAVLINK_URL = "tcp:127.0.0.1:5762"   # SITL SERIAL1, unused by MAVProxy / QGC
SIM_HOME_TOL_H_M = 2.0          # SITL home must match the depot pad within this
SIM_HOME_TOL_V_M = 1.0
SIM_LAND_TOL_M = 3.0            # landed within this of the destination pad = delivered
SIM_MISSION_TIMEOUT_S = 1800.0
SIM_FRONT_VALID_HOURS = 24.0 * 30   # frozen (calm) simulation weather: no 3 h live-weather expiry
