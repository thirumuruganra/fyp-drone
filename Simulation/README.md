# Gazebo + ArduPilot SITL simulation: Two-Tier planner over an OSM city

This folder flies the route chosen by the two-tier planner through a 3-D Gazebo copy of
Thiruvanmiyur. The pipeline:

1. OSM building footprints are extruded to their heights and placed on the DEM terrain.
2. Tier 1 (NAMOA*) builds the Pareto front once.
3. Tier 2 picks a route from the payload tags `(F, W, U, P)`.
4. An ArduCopter SITL drone flies that route in AUTO mode.
5. The flown track is compared with the plan.

The planner code stays in `../Implementation`. Everything here imports it; nothing is copied.

```
OSM + DEM ──► prepare_sim_region.py ──► regions/<id>/  (ONE frozen data bundle, bundle_hash)
                                          │
              ┌───────────────────────────┴───────────────────────────┐
              ▼                                                       ▼
   gazebo_world_builder.py                                  run_sim_mission.py
   worlds/<id>/ (terrain + LoD1 buildings + roads + iris)   lattice + HARD roof/terrain clearance
              │                                             → Tier 1 front (cached) → Tier 2 pick
              │                                             → missions/<run>/ mission.plan / .waypoints
              ▼                                                       │
   gazebo/launch_region.sh ◄──── route_markers.sdf ───────────────────┤
   gz sim  ◄─JSON─►  ArduCopter SITL (sim_vehicle.py)  ◄── MAVLink ── sitl_mission_runner.py
                         │ UDP 14550                                  │ (or QGC: open mission.plan)
                         ▼                                            ▼
                 QGroundControl (Windows)                  runs/<run>/ telemetry + gz pose
                                                                      ▼
                                                        evaluate_gazebo_run.py → metrics + plots
```

| File | What it does |
|---|---|
| `sim_config.py` | Region (1.5 km box, 100 m lattice), depot/destination pads, clearance, world and SITL settings |
| `prepare_sim_region.py` | Builds the frozen bundle: DEM grid, OSM buildings with heights, roads, pads, `manifest.json` |
| `sim_region.py` | `RegionBundle` loader; frame helpers (planner frame = box centre, world frame = depot) |
| `route_clearance.py` | Hard checks on every lattice edge and pad connector (roof +15 m with a 5 m buffer, terrain +15 m, 120 m AGL) |
| `gazebo_world_builder.py` | Writes the SDF world, terrain mesh, building tiles, roads and pads, plus `route_markers_sdf()` |
| `run_sim_mission.py` | Runs bundle → graph → Tier 1 (cached) → Tier 2 → ArduCopter mission + markers |
| `sim_mission_export.py` | Writes QGC `.plan`, `QGC WPL 110` and `mission.json` (TAKEOFF → route → LAND, no RTL) |
| `sitl_mission_runner.py` | pymavlink: preflight identity checks, upload + read-back, GUIDED → arm → AUTO, logging |
| `simulation_logger.py` | MAVLink CSVs plus Gazebo ground-truth pose (`gz topic … dynamic_pose/info`) |
| `evaluate_gazebo_run.py` | Cross-track error, clearance, violations, landing offset, map and altitude plots, `runs/summary.csv` |
| `gazebo/launch_region.sh` | Starts `gz sim` with the world, spawns route markers, prints the matching SITL command |

## Altitude and frame contract (why the drone lands where the planner says)

```
Gazebo world origin (0,0,0) = depot pad ground          x east, y north, z up (ENU), metres
xy_world  = LocalFrame(depot).to_local(lat, lon)
H_home    = depot ground AMSL, read from the SAME terrain grid as the planner and the mesh
z_world   = H_amsl − H_home
SITL      = sim_vehicle.py … --custom-location=<depot lat>,<depot lon>,<H_home>,0
mission   = frame 3 (GLOBAL_RELATIVE_ALT): rel_alt = alt_amsl_m − H_home      (never agl_m)
```

With `--model JSON`, ArduPilot derives GPS from its own home position plus the Gazebo pose. The world origin and the SITL home must therefore be the same point. `gazebo_world_builder.py` writes the matching command to `worlds/<id>/sitl_command.txt`, and the runner refuses to fly if the SITL home is more than 2 m horizontally or 1 m vertically from the depot pad.

## Phase 0: one-time setup in WSL2 (Ubuntu 24.04)

PX4's `ubuntu.sh` has already installed Gazebo. Keep PX4, but **never run PX4 SITL and ArduPilot SITL at the same time**: their UDP ports overlap.

```bash
# 0. Which Gazebo? Expect "8.x" (Harmonic). Use GZ_VERSION=ionic / jetty if that is what you have.
gz sim --versions
export GZ_VERSION=harmonic

# 1. ArduPilot SITL
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile                              # 24.04: activates ~/venv-ardupilot, puts sim_vehicle.py on PATH
./waf configure --board sitl && ./waf copter

# 2. ArduPilot Gazebo plugin (Harmonic: libgz-sim8-dev)
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev libopencv-dev libgstreamer1.0-dev \
     libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
git clone https://github.com/ArduPilot/ardupilot_gazebo ~/ardupilot_gazebo
cd ~/ardupilot_gazebo && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4

# 3. Environment (append to ~/.bashrc, then open a new terminal)
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# 4. This repo inside WSL (faster than /mnt/c), opened from VS Code with "WSL: Open Folder"
git clone https://github.com/thirumuruganra/fyp-drone.git ~/fyp-drone
cd ~/fyp-drone/Simulation
python3 -m venv ~/venvs/fyp && . ~/venvs/fyp/bin/activate
pip install -r requirements-simulation.txt
```

### Smoke test (must pass before the city)

```bash
# terminal 1
gz sim -v4 -r iris_runway.sdf
# terminal 2
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
#   MAVProxy:  mode guided  →  arm throttle  →  takeoff 10
```

Gate for this smoke test:

- Stable hover at 10 m.
- A 3-waypoint AUTO mission flown from QGC.
- Real-time factor of at least 0.8 (shown at the bottom right of Gazebo).

### QGroundControl on Windows

ArduPilot's MAVProxy sends to **UDP 14550**. The PX4 link you made earlier used port 18570 and does not apply here.

- **Recommended (Windows 11), mirrored networking:**
  1. Create `%UserProfile%\.wslconfig` containing:

     ```ini
     [wsl2]
     networkingMode=mirrored
     ```

  2. Run `wsl --shutdown`.

  QGC then connects automatically on 14550.
- **NAT mode (default WSL networking):**
  1. Add `--out=udp:$(ip route show default | awk '{print $3}'):14550` to the `sim_vehicle.py` line. That address is the Windows host as seen from WSL.
  2. Allow UDP 14550 through the Windows firewall.
  3. If QGC needs a manual link, add a UDP link on port 14550.

### Gazebo GUI under WSLg is slow or black

- Update the Windows GPU driver.
- Install a newer Mesa: `sudo add-apt-repository ppa:kisak/kisak-mesa && sudo apt upgrade`.
- Launch with `gazebo/launch_region.sh <id> "" --render-engine ogre`.
- Build the world without the sky: `python gazebo_world_builder.py <id> --no-sky`.
- As a last resort, run the server headless (`gz sim -s -r <world>.sdf`) and the GUI separately (`gz sim -g`).

## Running a simulated delivery

```bash
cd ~/fyp-drone/Simulation

# Phase 1: freeze the region (live OSM + Open-Meteo DEM, cached in Implementation/cache)
python prepare_sim_region.py --strict                 # default: thiru_1500 (sim_config.py)
#   other area:  --region-id my_area --center LAT LON --box-m 1500 --depot LAT LON --dest LAT LON --dest LAT LON

# Phase 2: build the Gazebo world
python gazebo_world_builder.py thiru_1500

# Phase 3: plan (Tier 1 once, Tier 2 per payload)
python run_sim_mission.py --preset fragile_medical    # or --payload F W U P, --dest-index 1
python run_sim_mission.py --all-presets               # four picks from ONE cached front
python run_sim_mission.py --baseline min-distance     # single-objective baseline

# Phase 4: fly
gazebo/launch_region.sh thiru_1500 missions/<run_id>                  # terminal 1: world + markers
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console \
    --custom-location=<printed by the launcher>                         # terminal 2
python sitl_mission_runner.py missions/<run_id>                       # terminal 3
#   …or in Windows QGC: Plan → Open → mission.plan → Upload → Start Mission
#   (mission.plan is at \\wsl$\Ubuntu-24.04\home\<user>\fyp-drone\Simulation\missions\<run_id>)

# Phase 5: evaluate
python evaluate_gazebo_run.py runs/<run_id>__<time>
```

In Gazebo, the thin blue lines are the Tier 1 Pareto front and the thick red line is the Tier 2 pick. The pads are blue (depot) and orange (destination). Building colours show where each height came from: grey means an OSM `height` tag, beige means `building:levels × 3 m`, and light red means the type default.

## What is checked and where

- **Planner and world use the same data.** The bundle hash is stored in the graph, the front identity, `mission.json` and `world_manifest.json`. The runner refuses to fly if they don't match.
- **Edges are flyable.** `route_clearance.ClearanceChecker` removes, before NAMOA* runs:
  - edges whose buffered segment passes less than 15 m above a roof;
  - edges whose terrain clearance, sampled every 10 m, drops below 15 m;
  - edges that exceed 120 m AGL.

  Selected waypoints are never raised afterwards. The exact flown polyline, including the pad connectors, is checked again before export.
- **Pads.** The take-off and landing columns must be clear of any (buffered) footprint, and the connector leg to the lattice is clearance-checked. If either check fails, the planner reports "NO FEASIBLE MISSION" and writes no mission.
- **Mission.** The upload must end in `MISSION_ACCEPTED`, and the read-back must match what was sent. The mission is TAKEOFF → route → LAND at the destination pad, with no unvalidated RTL.
- **Start position.** Before arming, the runner checks both the SITL home and the vehicle's *current* position against the depot pad. ArduPilot only moves home when it arms, so after a delivery the drone is still standing on the destination pad. **Restart Gazebo and SITL between flights.**
- **Tier 2 reuse.** A tag-only change prints `Reusing stored front` and no `[Module 2] NAMOA*` line.

## Tests (offline, synthetic data)

```bash
python -m unittest discover -s Simulation/tests       # 45 tests
python -m unittest discover -s Implementation/tests   # planner suite (edge_filter / identity / altitude-frame tests added)
```

`python prepare_sim_region.py --offline --region-id smoke_offline` builds a bundle from the hand-made fixtures in `Implementation/tests/fixtures`. These fixtures are **not real data** and are for smoke tests only; `--strict` rejects them.

## Verified so far (2026-10-06, cloud sandbox without Gazebo)

- All of `Simulation/tests` passes offline, and so does `Implementation/tests` except for the existing Tkinter import in `test_dashboard_helpers` (no Tk in the sandbox).
- `sitl_mission_runner.py` was flown against a **real ArduCopter SITL** (4.8.0-dev, built-in `quad` physics, `--speedup 5`), with home at the depot pad, using fixture-bundle missions:
  - preflight, upload, read-back, GUIDED → arm → AUTO, landing and disarm all worked;
  - cross-track RMSE was 0.11–0.13 m, and the landing offset from the pad was 0.01–0.02 m.
  - Two fixes came out of these runs: ArduCopter 4.7+ renamed `WPNAV_SPEED` (cm/s) to `WP_SPD` (m/s), and the runner now sets whichever one exists. ArduPilot also returns `DO_CHANGE_SPEED` with frame 0, which the read-back now accepts.
- **Not run here:**
  - Gazebo itself (no `gz` in the sandbox). The world SDF and meshes are only checked structurally (XML parse, mesh heights against the bundle).
  - The real OSM / Open-Meteo build, because the network policy blocks it.
  - Run Phase 0 and the Phase 2 gates (load the world, drop-test roof and ground, check real-time factor) on the WSL machine before any experiments.

## Known limits

- The terrain is Copernicus 90 m (a DSM) with a 10th-percentile filter. It is approximate ground, and Thiruvanmiyur is nearly flat (about 0–15 m).
- Building heights fall back to `levels × 3 m`, then to a type default. The counts are in `manifest.json`.
- The Gazebo world has no wind. Plan calm, or add wind to the world and verify it reaches the iris before comparing wind cases.
- Battery and energy figures in Gazebo are not calibrated. Planner energy is reported as a proxy.
