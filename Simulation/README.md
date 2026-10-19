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
| `sim_config.py` | Region (`thiru_demo`: 1.5 km box, 100 m lattice), depot/destination pads, clearance, camera views, world and SITL settings |
| `launch_simulation.py` | One command: prepare (if missing) → world → mission → Gazebo + bare SITL → fly → evaluate |
| `gazebo/env.sh` | Sets `FYP_ARDUPILOT_DIR`, `FYP_ARDUPILOT_GAZEBO_DIR` (default `~/ardupilot`, `~/ardupilot_gazebo`) and Gazebo plugin/resource paths |
| `prepare_sim_region.py` | Builds the frozen bundle: DEM grid, OSM buildings with heights, roads, pads, `manifest.json` |
| `sim_region.py` | `RegionBundle` loader; frame helpers (planner frame = box centre, world frame = depot) |
| `route_clearance.py` | Hard checks on every lattice edge and pad connector (roof +15 m with a 5 m buffer, terrain +15 m, 120 m AGL) |
| `gazebo_world_builder.py` | Writes the SDF world, terrain mesh, building tiles, roads and pads, `gui.config` (follow camera), plus `route_markers_sdf()` |
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

**SITL overrides: rangefinder and precision landing off.** `gazebo/sitl_overrides.parm` is layered after the Iris defaults, both in `sitl_command.txt` and in the launcher. It sets two parameters:

- `RNGFND1_TYPE 0`. SITL simulates the rangefinder as height above the *home* altitude, because it has no terrain model. On a pad higher than the depot, the reading after touchdown is the height difference (5.95 m on `thiru_demo`). Copter's land detector needs a reading under 2 m, so LAND never completes, the vehicle never disarms, and the runner never finishes. The Iris models have no rangefinder anyway.
- `PLND_ENABLED 0`. `gazebo-iris.parm` enables strict precision landing for an IR-LOCK beacon that the city world does not have, which causes `PrecLand: Failsafe Measures` during LAND.

`--model JSON` hides the `gazebo-iris` frame from newer ArduPilot SITL binaries (defaults are looked up by model name), so `FRAME_CLASS` stays 0 and arming fails with `Motors: Check frame class and type`. The two `--add-param-file` options restore the defaults; `-w` wipes a stale `eeprom.bin` once.

```bash
# terminal 1
gz sim -v4 -r iris_runway.sdf
# terminal 2
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -w \
    --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/copter.parm \
    --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm
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

- **Blank window, and the log repeats "GUI requesting list of world names"** (even with `empty.sdf`): the GUI cannot reach the server over gz-transport. Pin transport to loopback in `~/.bashrc`, then open a new terminal:

  ```bash
  export GZ_IP=127.0.0.1
  export GZ_RELAY=127.0.0.1
  ```

  The server alone (`gz sim -s -r empty.sdf`) still loads in this state, which is how to tell this case apart from a rendering problem.

- Update the Windows GPU driver.
- Install a newer Mesa: `sudo add-apt-repository ppa:kisak/kisak-mesa && sudo apt upgrade`.
- Launch with `gazebo/launch_region.sh <id> "" --render-engine ogre` (or `launch_simulation.py --render-engine ogre`).
- Build the world without the sky: `python gazebo_world_builder.py <id> --no-sky` (or `launch_simulation.py --no-sky`).
- Use a coarser terrain mesh: `--terrain-res-m 30` (the launcher's default). The planner and clearance checks still use the bundle DEM.
- As a last resort, run the server headless (`gz sim -s -r <world>.sdf`) and the GUI separately (`gz sim -g`).

## Running a simulated delivery: one command

```bash
cd ~/fyp-drone/Simulation
source gazebo/env.sh                          # once per shell
python launch_simulation.py                   # thiru_demo, mixed_cargo: prepare → world → plan → fly → evaluate
python launch_simulation.py --preset fragile_medical   # same stored Tier 1 front, different Tier 2 pick
python launch_simulation.py --scene-only      # world + route markers only
python launch_simulation.py --gcs-host 127.0.0.1   # WSL mirrored networking (default auto = Windows host under NAT)
```

What it does:

- Prepares the region if `regions/<id>/` is missing. `thiru_demo` uses live data with `--strict`, and `smoke_offline` uses the fixtures.
- Rebuilds the world with a 30 m display terrain mesh and the follow camera.
- Runs `run_sim_mission.py`, which reuses the cached graph and Tier 1 front.
- Starts Gazebo through `launch_region.sh`.
- Starts the bare ArduCopter SITL binary. It uses the same `--model JSON`, Iris parameter files and depot home as `sitl_command.txt`, with a wiped EEPROM and no MAVProxy.
- Waits for TCP 5762, then flies the mission with `sitl_mission_runner.py` and runs `evaluate_gazebo_run.py` after landing.

Notes:

- Every launch starts a fresh Gazebo and SITL, so no manual restart is needed between flights.
- It refuses to start a second session, or to start when TCP 5762 is in use (for example by PX4 SITL).
- Logs go to `runs/launch_<time>/` (`gazebo.log`, `ardupilot.log`), plus the runner's usual `runs/<run_id>__<time>/`.
- Telemetry for QGroundControl goes to `--gcs-host`:14550. The default is `auto`. On WSL2 it sends to the default gateway, which is the Windows host under NAT networking (`sim_vehicle.py` does the same); elsewhere it sends to 127.0.0.1. With mirrored networking, pass `--gcs-host 127.0.0.1`.

The steps below are the same pipeline, run by hand.

## Running a simulated delivery: step by step

```bash
cd ~/fyp-drone/Simulation

# Phase 1: freeze the region (live OSM + DEM, cached in Implementation/cache)
#   OPENTOPO_API_KEY is read automatically from <repo>/.env → Copernicus 30 m DEM;
#   without a key the Open-Meteo 90 m DEM is used.
python prepare_sim_region.py --strict                 # default: thiru_demo (sim_config.py)
#   other area:  --region-id my_area --center LAT LON --box-m 1500 --depot LAT LON --dest LAT LON --dest LAT LON

# Phase 2: build the Gazebo world
python gazebo_world_builder.py thiru_demo                # --view wide, --terrain-res-m 30, --no-sky

# Phase 3: plan (Tier 1 once, Tier 2 per payload)
python run_sim_mission.py --preset fragile_medical    # or --payload F W U P, --dest-index 1
python run_sim_mission.py --all-presets               # four picks from ONE cached front
python run_sim_mission.py --baseline min-distance     # single-objective baseline

# Phase 4: fly
gazebo/launch_region.sh thiru_demo missions/<run_id>                  # terminal 1: world + markers + follow camera
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -w \
    --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/copter.parm \
    --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm \
    --custom-location=<printed by the launcher>                         # terminal 2
python sitl_mission_runner.py missions/<run_id>                       # terminal 3
#   …or in Windows QGC: Plan → Open → mission.plan → Upload → Start Mission
#   (mission.plan is at \\wsl$\Ubuntu-24.04\home\<user>\fyp-drone\Simulation\missions\<run_id>)

# Phase 5: evaluate
python evaluate_gazebo_run.py runs/<run_id>__<time>
```

**Default region (`thiru_demo`).** The depot and one pad are about 450 m apart, so the route has 5 lattice nodes (6 navigation waypoints, about 2 minutes at 5 m/s). Both pads were picked at least 12 m clear of any footprint.

The former default, `thiru_1500`, had its depot (12.978683, 80.254601) inside a footprint buffer with live OSM data, so `run_sim_mission.py` reported NO FEASIBLE MISSION. Its three corner pads are listed in `sim_config.py` but have not been checked. To try them, use `prepare_sim_region.py --region-id <id> --dest LAT LON …`.

**Gazebo camera.** `gazebo_world_builder.py` writes `worlds/<id>/gui.config`. The camera starts behind the depot and follows the vehicle: `follow` = (−6, −8, +8) m, `wide` = (−120, −90, +35) m. `launch_region.sh` passes it with `--gui-config`, then re-sends the follow request on `/gui/track` for about 10 s in case the GUI started late. After that, the camera is free to move.

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
python -m unittest discover -s Simulation/tests       # needs trimesh
python -m unittest discover -s Implementation/tests   # planner suite (edge_filter / identity / altitude-frame tests added)
```

`python prepare_sim_region.py --offline --region-id smoke_offline` builds a bundle from the hand-made fixtures in `Implementation/tests/fixtures`. These fixtures are **not real data** and are for smoke tests only; `--strict` rejects them.

## Verified in WSL2 Gazebo (2026-10-19)

Setup: Ubuntu 24.04 WSL2 (NAT networking), Gazebo Harmonic 8.15, ArduCopter master `9dd2e0f350`, RTX 4060 laptop.

All three runs used `thiru_demo` with real OSM and DEM data and `--preset fragile_medical`. Every run flew the same route: Tier 2 picked the min-risk point (0.524 km) from a front of 2 routes.

| Run | Lands + disarms | Xtrack RMSE / max | Vert RMSE | Landing offset | Min roof clearance | Violations | RTF (10 s median) | Wall time to disarm |
|---|---|---|---|---|---|---|---|---|
| Old manual pipeline, stock params | no: stuck on pad (rangefinder 5.95 m) | 0.19 / 1.11 m | 0.19 m | 0.49 m | 49.67 m | 0 | n/a | never |
| Old manual pipeline + `sitl_overrides.parm` | yes | 0.21 / 1.13 m | 0.19 m | 0.50 m | 49.67 m | 0 | 0.33 | 606 s |
| `launch_simulation.py` (30 m mesh, `iris_with_ardupilot`) | yes | 0.22 / 1.11 m | 0.19 m | 0.50 m | 49.67 m | 0 | 0.69 | 277 s |

- Flight accuracy is the same across all three runs.
- The launcher roughly doubles the real-time factor: the lighter terrain mesh and the camera-free vehicle avoid server-side gimbal camera rendering.
- QGroundControl on Windows connected through the launcher (default `--gcs-host auto` → the Windows host).
- The GUI confirmed the follow camera (`/gui/currently_tracked`: FOLLOW `iris_with_ardupilot`).

## Earlier verification (2026-10-06, cloud sandbox without Gazebo)

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
