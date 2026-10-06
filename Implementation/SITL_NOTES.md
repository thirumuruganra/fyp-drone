# SITL Notes

Status: not run. Phase 7 adds mission-file export and dashboard path only.

**Gazebo + ArduPilot SITL simulation now lives in `../Simulation/` — see `Simulation/README.md`.**

## Planned validation

1. Generate mission:
   `python dashboard.py`, select tags, export `.waypoints`.
2. Open `.waypoints` in QGroundControl or Mission Planner.
3. Confirm:
   - header `QGC WPL 110`
   - relative-altitude waypoint frame
   - waypoint altitude never above 120 m AGL
   - final command returns to launch
4. In ArduPilot SITL:
   - start vehicle
   - upload mission
   - arm and fly in guided/auto mode
   - record cross-track error and upload-to-arm latency

## Current implementation choice

Mission export uses relative altitude (`MAV_FRAME_GLOBAL_RELATIVE_ALT`, frame `3`) = height above HOME:
`rel_alt = alt_amsl_m − home_alt_amsl`, home row frame `0` in AMSL (home defaults to the ground under
the first waypoint). Waypoints without `alt_amsl_m` (no DEM) fall back to `agl_m`, correct on flat
ground only; missing `agl_m` → 60 m. Export rejects any waypoint above 120 m AGL.

## Not verified

- Mission Planner / QGroundControl import.
- ArduPilot SITL upload.
- MAVLink route adherence logs.
