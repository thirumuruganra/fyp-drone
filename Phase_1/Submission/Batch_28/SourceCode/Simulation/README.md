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

## Getting the code

Download the `Batch_28` folder and copy it into the Linux filesystem of Ubuntu / WSL2 (running
from `/mnt/c/...` works but is much slower).

```bash
cp -r /mnt/c/<path to>/Batch_28 ~/Batch_28
cd ~/Batch_28/SourceCode/Simulation                  # all commands below run from here
```

Keep the layout:

```
Batch_28/SourceCode/
  .env.example      copy to .env for the optional OpenTopography key
  Implementation/   planner (imported by this folder)
  Simulation/       this folder
```

The shell scripts and the parameter file are saved with Linux (LF) line endings. If one was opened
and saved on Windows and bash reports `$'\r': command not found`, convert it back:
`sed -i 's/\r$//' gazebo/*.sh gazebo/*.parm`.

The output folders `worlds/`, `missions/`, `runs/` and `.local/` are not shipped; they are created on
the first run. `regions/` ships with the two frozen bundles.

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
| `sim_config.py` | Region (`thiru_demo`: 1.5 km box, 100 m lattice), depot and destination pads, clearance, camera views, world and SITL settings |
| `launch_simulation.py` | One command: prepare (if missing) -> world -> mission -> Gazebo + bare SITL -> fly -> evaluate |
| `prepare_sim_region.py` | Builds the frozen bundle: DEM grid, OSM buildings with heights, roads, pads, `manifest.json` |
| `sim_region.py` | `RegionBundle` loader and frame helpers (planner frame = box centre, world frame = depot) |
| `route_clearance.py` | Hard checks on every lattice edge and pad connector (15 m over roofs and terrain, 120 m AGL cap) |
| `gazebo_world_builder.py` | Writes the SDF world, terrain mesh, building tiles, roads, pads, `gui.config` (follow camera) and route markers |
| `run_sim_mission.py` | Bundle -> graph -> Tier 1 (cached) -> Tier 2 -> ArduCopter mission and markers |
| `sim_mission_export.py` | Writes QGC `.plan`, `QGC WPL 110` and `mission.json` (TAKEOFF -> route -> LAND) |
| `sitl_mission_runner.py` | pymavlink: preflight checks, mission upload and read-back, GUIDED -> arm -> AUTO, logging |
| `simulation_logger.py` | MAVLink CSVs plus Gazebo ground-truth pose |
| `evaluate_gazebo_run.py` | Cross-track error, clearance, landing offset, plots, `runs/summary.csv` |
| `gazebo/launch_region.sh` | Starts `gz sim` with the world, route markers and follow camera, prints the matching SITL command |
| `gazebo/env.sh` | Sets the ArduPilot / `ardupilot_gazebo` paths (default `~/ardupilot`, `~/ardupilot_gazebo`) and Gazebo plugin and resource paths |
| `gazebo/camera_director.py` | Recording camera: pad shot at take-off, then follow camera from 6 m altitude |
| `gazebo/sitl_overrides.parm` | SITL parameters layered after the Iris defaults (rangefinder and precision landing off) |
| `regions/` | Frozen bundles: `thiru_demo` (default) and `thiru_1500` (former default, depot infeasible) |
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

# Gazebo Python bindings for the recording camera (system python3, not the venv)
sudo apt install python3-gz-transport13 python3-gz-msgs10

# Python environment
cd ~/Batch_28/SourceCode/Simulation
python3 -m venv ~/venvs/fyp && . ~/venvs/fyp/bin/activate
pip install -r requirements-simulation.txt

# Environment: source once per shell, from this folder (or copy its exports to ~/.bashrc).
# It expects ~/ardupilot and ~/ardupilot_gazebo; for other locations set
# FYP_ARDUPILOT_DIR / FYP_ARDUPILOT_GAZEBO_DIR first.
source gazebo/env.sh
```

**Smoke test (run once before the city):**

```bash
gz sim -v4 -r iris_runway.sdf                                  # terminal 1
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -w \
    --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/copter.parm \
    --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm   # terminal 2
# MAVProxy: mode guided -> arm throttle -> takeoff 10
```

Expect a stable hover at 10 m and a real-time factor of at least 0.8.

**SITL overrides.** `gazebo/sitl_overrides.parm` is layered after the Iris defaults, both in
`worlds/<id>/sitl_command.txt` and in the launcher. It sets two parameters:

- `RNGFND1_TYPE 0`. SITL has no terrain model, so it reports the rangefinder as height above the
  *home* altitude. On a pad higher than the depot the reading after touchdown stays at the height
  difference (5.95 m on `thiru_demo`), the land detector never fires, and the drone never disarms.
- `PLND_ENABLED 0`. `gazebo-iris.parm` enables precision landing on an IR-LOCK beacon the city world
  does not have, which triggers `PrecLand: Failsafe Measures` during LAND.

## Running a simulated delivery: one command

The default region is `thiru_demo`: the depot and one pad are about 450 m apart (5 lattice nodes,
about 2 minutes at 5 m/s), both at least 12 m clear of any building footprint. Its frozen bundle is
already in `regions/`.

```bash
. ~/venvs/fyp/bin/activate                            # once per shell
cd ~/Batch_28/SourceCode/Simulation
source gazebo/env.sh                                   # once per shell
python launch_simulation.py                            # thiru_demo, mixed_cargo: prepare -> world -> plan -> fly -> evaluate
python launch_simulation.py --preset fragile_medical   # same stored Tier 1 front, different Tier 2 pick
python launch_simulation.py --scene-only               # world and route markers only
python launch_simulation.py --wait-for-start           # recording: hold on the pad until Enter
python launch_simulation.py --view follow              # chase camera from the start (default: take-off shot)
python launch_simulation.py --gcs-host 127.0.0.1       # WSL mirrored networking (default auto)
```

What it does:

- Prepares the region if `regions/<id>/` is missing (live data with `--strict`).
- Rebuilds the world with a 30 m display terrain mesh and the follow camera. The planner and the
  clearance checks still use the bundle DEM.
- Runs `run_sim_mission.py`, which reuses the cached graph and Tier 1 front.
- Starts Gazebo through `launch_region.sh` and the bare ArduCopter SITL binary (same `--model JSON`,
  Iris parameter files, overrides and depot home as `sitl_command.txt`, wiped EEPROM, no MAVProxy).
- Waits for TCP 5762, flies the mission with `sitl_mission_runner.py`, and runs
  `evaluate_gazebo_run.py` after landing.

Notes:

- Every launch starts a fresh Gazebo and SITL, so no restart is needed between flights. It refuses to
  start a second session, or to start while TCP 5762 is in use (for example by PX4 SITL).
- **Recording.** With `--wait-for-start` the runner uploads and verifies the mission, holds the drone
  disarmed on the depot pad and prints `READY ON GROUND`. Start the screen recorder, then press Enter
  in the launcher terminal to arm and take off.
- **Camera.** The default `--view takeoff` places the camera beside the depot pad, (-4, -6, +3) m,
  and watches the lift-off. At 6 m altitude `gazebo/camera_director.py` switches to the close follow
  camera, (-6, -8, +8) m. The director runs under the system `python3`, because the
  `python3-gz-transport13` and `python3-gz-msgs10` bindings are Debian packages, not part of the venv.
  `--view follow` and `--view wide` (-120, -90, +35) m follow from the start.
- **QGroundControl.** Telemetry goes to `--gcs-host`:14550. With the default `auto`, WSL2 sends to the
  Windows host (default gateway under NAT networking) and other systems send to 127.0.0.1. On WSL2
  the launcher also opens Windows QGroundControl if it is not running (default
  `C:\Program Files\QGroundControl\bin\QGroundControl.exe`; override with `--qgc-exe` or
  `FYP_QGC_EXE`, skip with `--no-ground-station`).
- Logs go to `runs/launch_<time>/` (`gazebo.log`, `ardupilot.log`) plus the runner's
  `runs/<run_id>__<time>/`.

## Running a simulated delivery: step by step

The same pipeline, run by hand.

```bash
cd ~/Batch_28/SourceCode/Simulation
. ~/venvs/fyp/bin/activate

# Phase 1: frozen region. regions/thiru_demo is already included; rebuild only to refresh the data
# (needs network; OPENTOPO_API_KEY in Batch_28/SourceCode/.env gives the 30 m DEM, otherwise a 90 m DEM is used)
python prepare_sim_region.py --strict                  # default: thiru_demo (sim_config.py)
#   other area: --region-id my_area --center LAT LON --box-m 1500 --depot LAT LON --dest LAT LON

# Phase 2: Gazebo world
python gazebo_world_builder.py thiru_demo              # also: --view wide, --terrain-res-m 30, --no-sky

# Phase 3: plan (Tier 1 once, Tier 2 per payload)
python run_sim_mission.py --preset fragile_medical
python run_sim_mission.py --all-presets                # four picks from ONE cached front
python run_sim_mission.py --baseline min-distance
#   also: --payload F W U P, --dest-index N, --wind-override SPEED_KMH DIR_FROM_DEG, --refresh

# Phase 4: fly (three terminals)
bash gazebo/launch_region.sh thiru_demo missions/<run_id>
sim_vehicle.py ...                                     # exact command in worlds/thiru_demo/sitl_command.txt
python sitl_mission_runner.py missions/<run_id>
#   or in QGroundControl: Plan -> Open -> mission.plan -> Upload -> Start Mission

# Phase 5: evaluate
python evaluate_gazebo_run.py runs/<run_id>__<time>
```

The former default region `thiru_1500` is kept in `regions/`, but its depot (12.978683, 80.254601)
sits inside a footprint buffer with live OSM data, so `run_sim_mission.py` reports NO FEASIBLE
MISSION for it.

In Gazebo the thin blue lines are the Tier 1 Pareto front and the thick red line is the Tier 2
pick. Pads are blue (depot) and orange (destination). Building colours show where the height came
from: grey is an OSM `height` tag, beige is `levels x 3 m`, light red is the type default.

**Restart Gazebo and SITL between manual flights.** ArduPilot only moves home when it arms, so after
a delivery the drone still stands on the destination pad and the runner refuses to start.

### QGroundControl on Windows (manual runs)

MAVProxy sends to UDP 14550. With Windows 11, set `networkingMode=mirrored` under `[wsl2]` in
`%UserProfile%\.wslconfig`, run `wsl --shutdown`, and QGroundControl connects automatically. In the
default NAT mode, add `--out=udp:<windows host ip>:14550` to the `sim_vehicle.py` command and allow
UDP 14550 through the firewall.

### Gazebo GUI black or slow under WSLg

- **Blank window, log repeats "GUI requesting list of world names":** the GUI cannot reach the server
  over gz-transport. Add `export GZ_IP=127.0.0.1` and `export GZ_RELAY=127.0.0.1` to `~/.bashrc` and
  open a new terminal.
- Otherwise: update the GPU driver, launch with `--render-engine ogre`, build the world with
  `--no-sky`, use `--terrain-res-m 30` (the launcher default), or run the server headless
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

## Verified in WSL2 Gazebo (2026-10-19)

Ubuntu 24.04 WSL2 (NAT), Gazebo Harmonic 8.15, ArduCopter master, RTX 4060 laptop. `thiru_demo`
with real OSM and DEM data, `--preset fragile_medical`; Tier 2 picked the min-risk route (0.524 km)
from a front of 2 routes.

| Run | Lands + disarms | Xtrack RMSE / max | Landing offset | Min roof clearance | Violations | RTF |
|---|---|---|---|---|---|---|
| Manual pipeline, stock params | no (rangefinder 5.95 m on pad) | 0.19 / 1.11 m | 0.49 m | 49.67 m | 0 | n/a |
| Manual pipeline + `sitl_overrides.parm` | yes | 0.21 / 1.13 m | 0.50 m | 49.67 m | 0 | 0.33 |
| `launch_simulation.py` | yes | 0.22 / 1.11 m | 0.50 m | 49.67 m | 0 | 0.69 |

The launcher roughly doubles the real-time factor (30 m display mesh, camera-free
`iris_with_ardupilot`). Recording mode (`--wait-for-start`, `--view takeoff`) held the drone on the
pad until Enter, switched the camera to follow at 6 m, and gave the same flight results.

## Known limits

- Terrain is the Copernicus DSM with a 10th-percentile filter. It is approximate ground, and
  Thiruvanmiyur is nearly flat (about 0-15 m).
- Building heights fall back to `levels x 3 m`, then to a type default. Counts are in `manifest.json`.
- The Gazebo world has no wind. Plan calm, or add wind to the world and check that it reaches the
  vehicle before comparing wind cases.
- Battery and energy figures are not calibrated in Gazebo; planner energy is a proxy.
- Results from SITL runs are simulated outputs, not flight tests.
