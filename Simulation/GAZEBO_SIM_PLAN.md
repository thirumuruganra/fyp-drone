# Gazebo + ArduPilot SITL simulation plan: Two-Tier planner over OSM city (FYP B28)

Date 2026-10-06. Replaces the teammate plan `gazebo_implementation_plan.md` for this setup.

Status: Phases 1–5 code implemented in this folder and tested offline (see README.md). Phase 0 (WSL install) and the Gazebo gates still need running on the team machine.

## Context

The team needs a Gazebo demo of the two-tier planner. The world should be a Thiruvanmiyur city crop with terrain and OSM buildings extruded to their heights. Tier 1 (NAMOA*) builds the Pareto front once, Tier 2 picks a route from the payload tags (F, W, U, P), and an ArduPilot SITL drone flies that route through the 3-D city.

The teammate plan has the right architecture: one shared region bundle feeds both the planner and the world, plus hard clearance checks and a fixed altitude frame. It assumed macOS or an Ubuntu box, a 1 km crop and no repo changes yet. This plan adapts it to the actual machine and to the answers given:

- **Machine:** Windows host running QGroundControl and VS Code, with WSL2 **Ubuntu 24.04**. PX4-Autopilot plus Gazebo was installed through `ubuntu.sh`, which installs Gazebo **Harmonic** or newer.
- **Flight stack:** **ArduPilot SITL** via the official `ardupilot_gazebo` plugin, which supports Harmonic, Ionic and Jetty. PX4 stays installed but is not used, and the two must never run at the same time because their UDP ports overlap.
- **Region:** a crop of about **1.5 km**, with the 100 m lattice inside it.
- **Mission delivery:** both an automated **pymavlink runner** and an exported **QGC `.plan`** file that the Windows QGC can load for the live demo.

Repo facts that drive the work (paths relative to `Implementation/`):

- `LatticeAirspaceBuilder(center, box_m, spacing_m, …)` is in `module1_graph_builder.py:532`.
  - Each node stores `lat, lon, x_m, y_m, ground_elev_m, bldg_h_m, agl_m, alt_amsl_m`.
  - Node AGL is `max(60, tallest building in cell + 15)`. A node above 120 m AGL, or inside a no-fly zone, is removed.
  - **Edges are never hard-checked against buildings.** `_edge_terms`, :645-690, only turns them into risk scores. A segment can therefore cross a roof that is taller than the interpolated altitude.
- Reusable pieces in `env_data.py`:
  - `LocalFrame` (:253): equirectangular projection, x east and y north, `to_local` / `to_latlon`.
  - `fetch_osm_layers` (:852): writes `cache/osm_*.gpkg`, with layers `buildings`, `restricted`, `water`, `landuse`.
  - `fetch_roads` (:919).
  - `building_height_m` (:542).
  - `BuildingIndex` (:562): STRtree of footprints in local metres, plus heights.
  - `fetch_elevations(coords, …)` (:482): takes a list of points; `ElevationGrid` (:288) samples bilinearly.
- `SelectionResult.path` (`module4_tier2_payload.py:93`) is a list of `{node, lat, lon, alt_amsl_m, agl_m}`. `select_detailed(graph=G)` writes `outputs/tier2/selection_*.json`.
- `mission_export.py` has the altitude bug recorded as audit defect #3. It writes `agl_m` into frame 3, which is height above home, and the frame-0 home row also carries AGL. Its collinear merge works in degree space.
- `run_demo_osm.py` has no CLI for centre, depot or destination. `pick_distant_pair` chooses them at random. The graph pickle goes to `pareto_front_storage/airspace_graph_osm.pkl`, and `pareto_store.run_region(G, depot, points)` accepts `(lat, lon)` pairs.

## Altitude / frame contract (single source of truth)

```
Gazebo world origin (0,0,0) = depot pad ground.   x east, y north, z up (ENU), metres
xy_world   = LocalFrame(depot_lat, depot_lon).to_local(lat, lon)
H_home     = depot ground AMSL (from the SAME DEM grid used by planner + world)
z_world    = H_amsl − H_home
SITL home  = --custom-location=depot_lat,depot_lon,H_home,0
mission alt (MAV_FRAME_GLOBAL_RELATIVE_ALT=3) = alt_amsl_m − H_home     # never agl_m
```

With the JSON backend, ArduPilot derives GPS from its own home plus the Gazebo pose. Matching the world origin to the SITL home is therefore what makes the map, the planner and the autopilot agree. The world's `<spherical_coordinates>` element is set to the same values, but only for the GUI and NavSat. Set `TERRAIN_ENABLE 0`, since terrain frame 10 is deferred.

## Phase 0: Stack install + smoke test in WSL (1–2 days)

1. **Check the Gazebo version:** run `gz sim --versions`; expect Harmonic 8.x. Set `export GZ_VERSION=harmonic`, or `jetty` / `ionic` if that is what is installed.
2. **Install ArduPilot:**

   ```bash
   git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
   cd ~/ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y && . ~/.profile   # 24.04 → creates ~/venv-ardupilot
   ./waf configure --board sitl && ./waf copter
   ```

3. **Build the plugin.** Install `libgz-sim8-dev rapidjson-dev` and the gstreamer/opencv dependencies listed in the README, then:

   ```bash
   git clone https://github.com/ArduPilot/ardupilot_gazebo ~/ardupilot_gazebo
   cd ~/ardupilot_gazebo && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4
   ```

   Add to `~/.bashrc`:

   ```bash
   export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
   export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$HOME/fyp-drone/Simulation/worlds:$GZ_SIM_RESOURCE_PATH
   ```

4. **Run the smoke test:**
   - Terminal 1: `gz sim -v4 -r iris_runway.sdf`
   - Terminal 2: `sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console`
   - Then `mode guided`, `arm throttle`, `takeoff 10`.
5. **Connect the Windows QGC.** ArduPilot sends on 14550, not the PX4 port 18570 that was used before. Pick one of these:
   - (a) Recommended: turn on WSL mirrored networking by adding `[wsl2] networkingMode=mirrored` to `%UserProfile%\.wslconfig` and running `wsl --shutdown`. QGC then auto-connects on UDP 14550.
   - (b) NAT mode: add `--out=udp:$(ip route show default | awk '{print $3}'):14550` to `sim_vehicle.py`. Allow UDP 14550 through the Windows firewall.
6. **If graphics are slow or broken under WSLg:**
   - Update the Windows GPU driver.
   - Install kisak-mesa (per the PX4 WSL doc).
   - Fall back to `--render-engine ogre`, or run the server with `-s` and the GUI separately with `-g`.
7. **Clone `fyp-drone` into the WSL home** (`~/fyp-drone`) and open it with VS Code Remote-WSL. Create a venv: `pip install -r requirements.txt -r requirements-simulation.txt`.
8. **Pin the versions:** record the gz version and the ArduPilot / ardupilot_gazebo commit hashes in `Simulation/README.md`.

**Gate:** stable hover; a 3-waypoint AUTO mission on the flat runway world flown from QGC; real-time factor ≥ 0.8.

## Phase 1: Simulation region bundle (2–3 days)

Add `SIM_*` settings to the new `Simulation/sim_config.py` (`Implementation/config.py` is not changed):

- `SIM_CENTER` (lat, lon), `SIM_BOX_M=1500`, `SIM_SPACING_M=100`.
- `SIM_DEPOT` (lat, lon), `SIM_DESTINATIONS`: 2–3 (lat, lon) points chosen so that tall buildings lie between them and the depot.
- `SIM_TERRAIN_RES_M=10`, `SIM_EDGE_BUFFER_M=5`, `SIM_REGION_ID`.

New `Simulation/prepare_sim_region.py` writes `Simulation/regions/<region_id>/`:

- **`buildings.gpkg`:** footprints clipped to the box (plus a buffer), repaired with `make_valid`, with `height_m`, `height_source` (tagged / levels / default), base elevation and `osm_id`. It reuses `fetch_osm_layers`, `building_height_m` and `_local_rows`.
- **`terrain.json`:** an `ElevationGrid` at 10 m spacing over the box, from `fetch_elevations` (Open-Meteo / Copernicus, cached). It is clamped to the 10th percentile within a footprint-sized window, to strip the DSM roof bias noted in the teammate plan. Thiruvanmiyur is about 0–15 m and nearly flat, so this is low-risk.
- **`pads.json`:** depot and destinations with their ground AMSL and `H_home`.
- **`roads.geojson`:** clipped roads, used for visuals only.
- **`manifest.json`:** bounds, origin, the frame contract above, data dates, provider names, height-source counts, a `bundle_hash` (sha256 of the processed files and settings), and `offline/fixture` flags.
- **Strict mode:** refuse to build if the OSM layer is empty or elevation fell back to `FlatProvider`, so a real-map run can't silently use fixtures or flat ground. The audit saw empty layers after a failed fetch.

Planner graph change in `module1_graph_builder.py`:

- Add an optional `terrain_grid=ElevationGrid` (from the bundle) so node ground and edge sampling read **the same surface** as the Gazebo mesh.
- Add an optional `building_index` built from the bundle's `buildings.gpkg`.
- Store `G.graph["bundle_hash"]`.

## Phase 2: Gazebo world generation (3–4 days)

New `Simulation/gazebo_world_builder.py` takes the bundle and writes `Simulation/worlds/<region_id>/`. Add `trimesh`, `mapbox_earcut` and `lxml` (optional) to `requirements-simulation.txt`.

- **Terrain:** triangulate the 10 m grid into one static mesh (OBJ), with z = `H − H_home` and the same mesh used for `<visual>` and `<collision>`. A mesh is used rather than a heightmap to avoid heightmap size and orientation pitfalls; the 150×150 grid is small.
- **Buildings:**
  - For each footprint (handling holes and multipolygons), call `trimesh.creation.extrude_polygon(poly_local, height_m)`.
  - Base = minimum terrain under the footprint minus 0.5 m, so no building floats. Roof = base + `height_m`. Store the absolute roof AMSL, which the planner clearance check uses.
  - Merge into **tile meshes** of 250 m × 250 m (≈36 models) to keep the entity count low.
  - Give each tile a static model with a mesh visual and an identical mesh collision.
  - Colour by `height_source` (tagged = grey, levels = beige, default = light red) so the review panel can see data provenance.
- **Visual-only extras** (no collision): roads as flat dark ribbons; depot and destination pads; **all Tier 1 front routes as thin translucent polylines plus the selected route as a thick coloured polyline**, generated per mission run. This shows the two-tier idea inside Gazebo.
- **World template:** `gazebo/templates/city.sdf.j2` copies the systems block from `iris_runway.sdf`: Physics, Sensors (ogre2), UserCommands, SceneBroadcaster, Imu, NavSat, AirPressure, Magnetometer.
  - Physics: `max_step_size 0.001`, `real_time_factor 1`.
  - `<spherical_coordinates>`: depot lat/lon, `H_home`, ENU.
  - Includes `model://iris_with_gimbal` (or `iris_with_ardupilot`) at pose `0 0 0.2 0 0 90` (ENU yaw, so the drone faces north, matching SITL heading 0). Confirm the yaw in the Phase 0 smoke test.
- **Launcher:** `gazebo/launch_region.sh <region_id>` exports the resource path and runs `gz sim -v4 -r <world>.sdf`. A second command prints the matching `sim_vehicle.py … --custom-location=<lat,lon,H_home,0> --out=…` line read from the manifest.

**Gate:**

- The city loads and drops no more than 0.8 below real-time.
- Drop a box onto a roof and onto the ground. It must rest at the expected z, within 0.5 m of the planner's roof and terrain value.
- A known building corner in Gazebo matches its OSM lat/lon in QGC map view, within 1 m.

## Phase 3: Make planner routes flyable (3–4 days)

New `route_clearance.py`:

- **`edge_clearance(a, b, building_index, terrain_grid, buffer_m)`:**
  - Buffer the horizontal segment by `SIM_EDGE_BUFFER_M`, which covers drone size plus tracking error.
  - Query the STRtree for intersecting footprints.
  - For each footprint, compute the parametric interval `[t0, t1]` of the segment inside the buffered polygon. Altitude is linear, so the minimum altitude is at an interval end. Compare it with roof AMSL plus `cfg.CLEARANCE`.
  - Sample terrain along the edge every ≤ 10 m from the grid, not interpolated from the endpoints.
  - Return the minimum clearance and a pass/fail result.
- **Pass `edge_clearance` as the builder's new `edge_filter` hook** (`hard_clearance=True` in the simulation run): drop edges that fail the check before keeping the strongly connected component.
  - Record `edges_rejected_clearance` in `build_info`.
  - Never raise waypoints after selection; fix the graph before Tier 1, as the teammate plan says.
- **Pad connectors:**
  - Snap the depot and each destination to the nearest node that has a clearance-checked straight link.
  - Check the vertical takeoff column at the depot and the descent column at the destination against nearby roofs.
  - If no connector exists, return "no feasible mission" explicitly.
- **Fix `Implementation/mission_export.py` and add `Simulation/sim_mission_export.py`:**
  - Use frame 3 with `alt_amsl_m − H_home`. The home row is frame 0 with `H_home` AMSL.
  - Reject missing `alt_amsl_m` in real-map mode (no 60 m default).
  - Disable collinear merge by default for the simulation. If it is kept, recompute it in metres and re-run `edge_clearance` on every merged segment.
  - Mission: `NAV_TAKEOFF` (22) to the first cruise rel-alt, then the depot node, path waypoints, destination node, then `NAV_LAND` (21) at the destination pad. **No RTL**, because the return route is not validated.
- **New `export_qgc_plan(result, path, home)` in `Simulation/sim_mission_export.py`:** writes QGC `.plan` JSON (`fileType: Plan`, `firmwareType: 3` ArduPilot, `vehicleType: 2`, `plannedHomePosition: [lat, lon, H_home]`, SimpleItems with `AltitudeMode: 1` and frame 3). Keep `.waypoints` (WPL 110) as well.
- **Orchestrator CLI: `Simulation/run_sim_mission.py --region-id ID --depot LAT LON --dest LAT LON [...] (--payload F W U P | --preset NAME)`** that:
  1. Loads the bundle and builds or loads the graph (`bundle_hash` must match).
  2. Runs `pareto_store.run_region(G, depot, dests)` for cached fronts.
  3. Runs Tier 2 for the given tags or a preset (`select_detailed(graph=G)`).
  4. Writes `missions/<run_id>/{selection.json, mission.waypoints, mission.plan, route_markers.sdf}`.
- **Pareto store:** add `bundle_hash`, `hard_clearance` and the builder version to the front metadata, and rebuild on a mismatch.

**Gate:**

- New unit tests pass:
  - A narrow tall building between two nodes is rejected.
  - A diagonal edge clipping a roof corner is rejected.
  - An uphill route gives rel-alt = AMSL − home, not AGL.
  - An unreachable pad returns no mission.
  - The `.plan` round trip is correct.
- QGC loads the `.plan` and shows the correct altitudes.

## Phase 4: SITL runner + logging (2–3 days)

New `sitl_mission_runner.py` (pymavlink), connecting by default to `tcp:127.0.0.1:5762`, the free SITL serial port that MAVProxy and QGC don't use:

1. Check the identity: the mission's `bundle_hash` equals the world manifest's, the front is not truncated, and the weather gate passes (`no_go_reasons`).
2. Wait for heartbeat and EKF/GPS readiness. Read `HOME_POSITION` and require it to be within 2 m and 1 m of `pads.json` (horizontal and altitude).
3. Set the parameters: `WPNAV_SPEED 500`, `WPNAV_ACCEL` default, `TERRAIN_ENABLE 0`, `FENCE_ENABLE 0`, `RTL_ALT` safe. Save the full parameter dump.
4. Upload with the mission protocol (`MISSION_COUNT` → `MISSION_REQUEST_INT` → `MISSION_ITEM_INT`, then a required `MISSION_ACCEPTED` ACK). Download the mission back and diff it against what was sent.
5. Fly: mode GUIDED → arm → mode AUTO → `MAV_CMD_MISSION_START`. Verify this sequence in Phase 0 and adjust if ArduCopter needs a different one. Monitor `MISSION_CURRENT`/`MISSION_ITEM_REACHED`, `STATUSTEXT`, landing and disarm, with a timeout and abort.
6. Log with `simulation_logger.py`:
   - MAVLink `GLOBAL_POSITION_INT`, `LOCAL_POSITION_NED`, `ATTITUDE`, `BATTERY_STATUS` to CSV.
   - Gazebo ground-truth pose via `gz topic -e -t /world/<w>/dynamic_pose/info --json-output` as a subprocess, filtered to the drone model.
   - `run_manifest.json` with tags, front id, selected index, wall and sim timestamps.
   - The ArduPilot `.bin` dataflash logs come from `~/ardupilot/logs`.

**Manual demo path:** load `mission.plan` in Windows QGC, upload, press Start Mission. Gazebo shows the drone following the highlighted route through the buildings.

**Gate:** mission accepted; the drone lands on the destination pad; there is a trajectory CSV and no contact with buildings or terrain.

## Phase 5: Evaluation + demo suite (3–5 days)

New `evaluate_gazebo_run.py` computes, from the logs and the bundle:

- Cross-track error (RMSE and max) of the flown path against the planned polyline, cruise segment only.
- Minimum roof and terrain clearance along the flown track, reusing `BuildingIndex` and `ElevationGrid`.
- Completion, flight time, distance.
- Planner energy proxy versus flown distance.

Plots:

- 2-D map overlay: buildings coloured by height, front routes in grey, selected route, flown track.
- Altitude profile: planned AMSL, flown, roof and terrain under the track.

Experiments:

- **Selection proof:** one cached front, four project presets plus single-objective baselines (min-distance, min-risk), each flown 3–5 times. Show different routes picked and **no NAMOA* re-run** (same front hash, Tier 2 latency ≤ 1 s).
- **Stretch:**
  - Wind via the Gazebo `WindEffects` system, with the teammate's from-direction to ENU formula. Confirm that the iris receives the force and that it isn't applied twice.
  - Payload mass via the iris `base_link` mass, kept separate from tag-only runs.

Deliverables: screen-recorded video of Gazebo plus QGC, a metrics CSV, figures, and an update to `SITL_NOTES.md` that replaces the frame-3/AGL text.

## Files: all new simulation code in a new top-level `Simulation/` folder

`Simulation/` sits beside `Implementation/` and reuses the planner by import, not by copying. Each script starts with the same `sys.path.insert(0, <repo>/Implementation)` header that the existing tests use, then imports `config`, `env_data`, `module1_graph_builder`, `pareto_store`, `module4_tier2_payload` and `mission_export` directly.

```text
Simulation/
  README.md                    # WSL setup (Phase 0), pinned versions, QGC networking, run commands
  requirements-simulation.txt  # trimesh, mapbox_earcut, pymavlink, jinja2 (+ -r ../Implementation/requirements.txt)
  sim_config.py                # SIM_CENTER, SIM_BOX_M, SIM_SPACING_M, SIM_DEPOT, SIM_DESTINATIONS, SIM_TERRAIN_RES_M, SIM_EDGE_BUFFER_M, SIM_REGION_ID
  _paths.py                    # adds ../Implementation to sys.path; shared output dirs
  prepare_sim_region.py        # Phase 1 bundle
  gazebo_world_builder.py      # Phase 2 world + meshes + route markers
  route_clearance.py           # Phase 3 hard edge / pad clearance
  sim_mission_export.py        # .plan + .waypoints writer (frame 3 = AMSL − H_home, takeoff/land, no RTL)
  run_sim_mission.py           # orchestrator: bundle → graph → Tier 1 cache → Tier 2 → mission + markers
  sitl_mission_runner.py       # Phase 4 pymavlink upload/fly
  simulation_logger.py
  evaluate_gazebo_run.py       # Phase 5 metrics + plots
  gazebo/templates/city.sdf.j2
  gazebo/launch_region.sh
  tests/test_route_clearance.py, test_sim_region.py, test_world_builder.py, test_sim_mission_export.py
  # generated (git-ignored): regions/<id>/, worlds/<id>/, missions/<run_id>/, runs/<run_id>/
```

`GZ_SIM_RESOURCE_PATH` in Phase 0 points at `$HOME/fyp-drone/Simulation/worlds` instead of `Implementation/gazebo/generated`. The bundle, world and run paths in the phases above live under `Simulation/` (`Simulation/regions/<id>/`, `Simulation/worlds/<id>/`, `Simulation/runs/<run_id>/`). `run_sim_mission.py` carries the centre, depot and destination CLI, so `run_demo_osm.py` is left untouched.

Small, backward-compatible changes to `Implementation/`, all optional keyword arguments whose defaults keep current behaviour and tests:

- `module1_graph_builder.py`:
  - `LatticeAirspaceBuilder(..., terrain_grid=None, edge_filter=None)`.
  - `edge_filter(a_attrs, b_attrs) -> bool` is called before the strongly-connected-component step. `Simulation/route_clearance.py` supplies it.
  - The builder counts the edges it rejects in `build_info`.
- `pareto_store.py`: an optional `extra_identity` dict (`bundle_hash`, `hard_clearance`, builder version) is merged into the metadata and the rebuild check.
- `mission_export.py`: fix audit defect #3 for every user. Frame-3 altitude becomes `alt_amsl_m − home_alt_amsl`, and the home row becomes frame 0 with AMSL. This is a bug fix, so the change stays there.
- `SITL_NOTES.md`: point to `Simulation/README.md`.

New tests run with `python -m unittest discover -s Simulation/tests`; the existing suite still runs with `python -m unittest discover -s Implementation/tests`.

`.gitignore`: add `Simulation/regions/`, `Simulation/worlds/`, `Simulation/missions/`, `Simulation/runs/`.

Commit messages follow the semantic style, for example `feat(sim): gazebo world builder`.

## Verification (end-to-end)

1. Run `python -m unittest discover -s Implementation/tests` (existing suite, unchanged) and `python -m unittest discover -s Simulation/tests` (new tests). Both run offline on fixtures.
2. In WSL, from `Simulation/`:
   - `python prepare_sim_region.py --region-id thiru_1500 --strict`
   - `python gazebo_world_builder.py thiru_1500`
   - `python run_sim_mission.py --region-id thiru_1500 --preset fragile_medical`, which writes the mission and markers.
3. `gazebo/launch_region.sh thiru_1500` in terminal 1, then `sim_vehicle.py … --custom-location=…` in terminal 2.
4. Run `python sitl_mission_runner.py missions/<run_id>`. Alternatively, load `mission.plan` in Windows QGC.
5. Run `python evaluate_gazebo_run.py outputs/sim_runs/<run_id>` and check:
   - Clearance ≥ 15 m throughout.
   - Landed on the pad.
   - Cross-track ≤ 3 m RMSE at 5 m/s.
6. Repeat step 4 with a different preset. The log must show the front loaded from cache, with no Tier 1 rebuild.
