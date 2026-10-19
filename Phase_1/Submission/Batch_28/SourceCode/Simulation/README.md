# Simulation: Gazebo + ArduPilot SITL

Flies the route chosen by the two-tier planner through a 3-D Gazebo copy of Thiruvanmiyur
(Chennai) and compares the flown track with the plan.

1. OSM building footprints are extruded to their heights and placed on the DEM terrain.
2. Tier 1 (NAMOA*) builds the Pareto front once.
3. Tier 2 picks a route from the payload tags `(F, W, U, P)`.
4. An ArduCopter SITL drone flies that route in AUTO mode.
5. The flown track is evaluated against the plan.

The planner stays in `../Implementation`. This folder imports it through the Python path, so keep
`Implementation/` and `Simulation/` side by side.

```
OSM + DEM -> prepare_sim_region.py -> regions/<id>/   (frozen data bundle, bundle_hash)
                                          |
              +---------------------------+---------------------------+
              v                                                       v
   gazebo_world_builder.py                                  run_sim_mission.py
   worlds/<id>/ (terrain, buildings, roads, iris)           lattice with hard clearance checks
              |                                             -> Tier 1 front (cached) -> Tier 2 pick
              |                                             -> missions/<run>/ mission.plan, .waypoints
              v                                                       |
   gazebo/launch_region.sh  <--- route markers -----------------------+
   gz sim <-JSON-> ArduCopter SITL <-- MAVLink -- sitl_mission_runner.py
                         |                                            |
                         v UDP 14550                                  v
                 QGroundControl                           runs/<run>/ telemetry + Gazebo pose
                                                                      v
                                                        evaluate_gazebo_run.py -> metrics + plots
```

## Files

| File | What it does |
|---|---|
| `sim_config.py` | Region (1.5 km box, 100 m lattice), depot and destination pads, clearance, world and SITL settings |
| `prepare_sim_region.py` | Builds the frozen bundle: DEM grid, OSM buildings with heights, roads, pads, `manifest.json` |
| `sim_region.py` | `RegionBundle` loader and frame helpers (planner frame = box centre, world frame = depot) |
| `route_clearance.py` | Hard checks on every lattice edge and pad connector (15 m over roofs and terrain, 120 m AGL cap) |
| `gazebo_world_builder.py` | Writes the SDF world, terrain mesh, building tiles, roads, pads and route markers |
| `run_sim_mission.py` | Bundle -> graph -> Tier 1 (cached) -> Tier 2 -> ArduCopter mission and markers |
| `sim_mission_export.py` | Writes QGC `.plan`, `QGC WPL 110` and `mission.json` (TAKEOFF -> route -> LAND) |
| `sitl_mission_runner.py` | pymavlink: preflight checks, mission upload and read-back, GUIDED -> arm -> AUTO, logging |
| `simulation_logger.py` | MAVLink CSVs plus Gazebo ground-truth pose |
| `evaluate_gazebo_run.py` | Cross-track error, clearance, landing offset, plots, `runs/summary.csv` |
| `gazebo/launch_region.sh` | Starts `gz sim` with the world and route markers, prints the matching SITL command |
| `regions/` | Frozen bundles: `thiru_1500` (default) and `thiru_demo` (short demo region) |
| `tests/` | Offline unit tests |

## Requirements

- Ubuntu 24.04 (WSL2 or native), Python 3
- Gazebo Harmonic (gz-sim 8)
- ArduPilot SITL (ArduCopter) with `sim_vehicle.py` and MAVProxy
- `ardupilot_gazebo` plugin
- QGroundControl (optional)
- Python packages: `pip install -r requirements-simulation.txt` (includes the planner requirements)

Planning steps (`run_sim_mission.py`, `evaluate_gazebo_run.py`, the tests) need only Python, not Gazebo.

## One-time setup (Ubuntu 24.04)

```bash
gz sim --versions                  # expect 8.x (Harmonic)
export GZ_VERSION=harmonic

# ArduPilot SITL
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl && ./waf copter

# ArduPilot Gazebo plugin
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev libopencv-dev libgstreamer1.0-dev \
     libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
git clone https://github.com/ArduPilot/ardupilot_gazebo ~/ardupilot_gazebo
cd ~/ardupilot_gazebo && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4

# Environment (append to ~/.bashrc)
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Python environment, from the SourceCode folder
cd Simulation
python3 -m venv ~/venvs/fyp && . ~/venvs/fyp/bin/activate
pip install -r requirements-simulation.txt
```

Under WSL2, put the project in the Linux filesystem (not `/mnt/c`) for speed.

**Smoke test (run once before the city):**

```bash
gz sim -v4 -r iris_runway.sdf                                  # terminal 1
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -w \
    --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/copter.parm \
    --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm   # terminal 2
# MAVProxy: mode guided -> arm throttle -> takeoff 10
```

Expect a stable hover at 10 m and a real-time factor of at least 0.8.

## Running a simulated delivery

Use the short demo region `thiru_demo`: the depot and one pad are about 450 m apart (5 lattice
nodes, about 2 minutes at 5 m/s). The default `thiru_1500` depot sits inside a footprint buffer
with live OSM data, so `run_sim_mission.py` reports NO FEASIBLE MISSION for it.

```bash
cd Simulation

# Phase 1: frozen region. regions/thiru_demo is already included; rebuild only to refresh the data
# (needs network; OPENTOPO_API_KEY in ../.env gives the 30 m DEM, otherwise a 90 m DEM is used)
python prepare_sim_region.py --region-id thiru_demo --box-m 1500 \
    --depot 12.978683 80.254140 --dest 12.982188 80.256170 --strict

# Phase 2: Gazebo world
python gazebo_world_builder.py thiru_demo

# Phase 3: plan (Tier 1 once, Tier 2 per payload)
python run_sim_mission.py --region-id thiru_demo --preset fragile_medical
python run_sim_mission.py --region-id thiru_demo --all-presets      # four picks from ONE cached front
python run_sim_mission.py --region-id thiru_demo --baseline min-distance
#   also: --payload F W U P, --dest-index N, --wind-override SPEED_KMH DIR_FROM_DEG, --refresh

# Phase 4: fly (three terminals)
gazebo/launch_region.sh thiru_demo missions/<run_id>
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -w \
    --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/copter.parm \
    --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm \
    --custom-location=<printed by the launcher>
python sitl_mission_runner.py missions/<run_id>
#   or in QGroundControl: Plan -> Open -> mission.plan -> Upload -> Start Mission

# Phase 5: evaluate
python evaluate_gazebo_run.py runs/<run_id>__<time>
```

In Gazebo the thin blue lines are the Tier 1 Pareto front and the thick red line is the Tier 2
pick. Pads are blue (depot) and orange (destination). Building colours show where the height came
from: grey is an OSM `height` tag, beige is `levels x 3 m`, light red is the type default.

**Restart Gazebo and SITL between flights.** ArduPilot only moves home when it arms, so after a
delivery the drone still stands on the destination pad and the runner refuses to start.

### QGroundControl on Windows

MAVProxy sends to UDP 14550. With Windows 11, set `networkingMode=mirrored` under `[wsl2]` in
`%UserProfile%\.wslconfig`, run `wsl --shutdown`, and QGroundControl connects automatically. In the
default NAT mode, add `--out=udp:<windows host ip>:14550` to the `sim_vehicle.py` command and allow
UDP 14550 through the firewall.

### Gazebo GUI black or slow under WSLg

Update the GPU driver, launch with `gazebo/launch_region.sh <id> "" --render-engine ogre`, build the
world with `python gazebo_world_builder.py <id> --no-sky`, or run the server headless
(`gz sim -s -r <world>.sdf`) with the GUI separate (`gz sim -g`).

## Frame contract (why the drone lands where the planner says)

```
Gazebo world origin = depot pad ground       x east, y north, z up, metres
xy_world = LocalFrame(depot).to_local(lat, lon)
H_home   = depot ground AMSL, read from the same terrain grid as the planner and the mesh
z_world  = H_amsl - H_home
SITL     = sim_vehicle.py ... --custom-location=<depot lat>,<depot lon>,<H_home>,0
mission  = frame 3 (GLOBAL_RELATIVE_ALT): rel_alt = alt_amsl_m - H_home
```

The world origin and the SITL home must be the same point. `gazebo_world_builder.py` writes the
matching command to `worlds/<id>/sitl_command.txt`, and the runner refuses to fly if the SITL home is
more than 2 m horizontally or 1 m vertically from the depot pad.

## What is checked

- **Same data everywhere.** The bundle hash is stored in the graph, the front, `mission.json` and
  `world_manifest.json`. The runner refuses to fly if they differ.
- **Flyable edges.** Before NAMOA* runs, `route_clearance.ClearanceChecker` removes edges less than
  15 m above a roof (with a 5 m buffer), edges whose terrain clearance sampled every 10 m drops below
  15 m, and edges above 120 m AGL. The final flown polyline, pad connectors included, is checked again.
- **Pads.** Take-off and landing columns must be clear of any buffered footprint. Otherwise the planner
  reports NO FEASIBLE MISSION and writes nothing.
- **Mission upload.** It must end in `MISSION_ACCEPTED` and the read-back must match what was sent.
  The mission is TAKEOFF -> route -> LAND at the destination pad.
- **Tier 2 reuse.** A tag-only change prints `Reusing stored front` and no `[Module 2] NAMOA*` line.

## Tests

```bash
python -m unittest discover -s tests        # needs trimesh (in requirements-simulation.txt)
```

`python prepare_sim_region.py --offline --region-id smoke_offline` builds a bundle from the hand-made
fixtures in `../Implementation/tests/fixtures`. They are not real data; `--strict` rejects them.

## Known limits

- Terrain is the Copernicus DSM with a 10th-percentile filter. It is approximate ground, and
  Thiruvanmiyur is nearly flat (about 0-15 m).
- Building heights fall back to `levels x 3 m`, then to a type default. Counts are in `manifest.json`.
- The Gazebo world has no wind. Plan calm, or add wind to the world and check that it reaches the
  vehicle before comparing wind cases.
- Battery and energy figures are not calibrated in Gazebo; planner energy is a proxy.
- Results from SITL runs are simulated outputs, not flight tests.
