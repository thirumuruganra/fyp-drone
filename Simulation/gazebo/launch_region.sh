#!/usr/bin/env bash
# Start the generated Gazebo city world and (optionally) spawn a mission's route markers.
#
#   gazebo/launch_region.sh <region_id> [mission_dir] [extra gz sim args...]
#
#   gazebo/launch_region.sh thiru_demo
#   gazebo/launch_region.sh thiru_demo missions/20261018_224344_thiru_demo_dest0_fragile_medical
#   gazebo/launch_region.sh thiru_demo "" --render-engine ogre     # WSLg / VM rendering fallback
#
# Needs: Gazebo Harmonic+ (gz) and ardupilot_gazebo built (see Simulation/README.md); gazebo/env.sh
# puts it on GZ_SIM_SYSTEM_PLUGIN_PATH / GZ_SIM_RESOURCE_PATH. Then start SITL in another terminal
# with the sim_vehicle.py line this script prints (its --custom-location MUST match the world origin),
# or run launch_simulation.py, which does all of this in one command.
set -euo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SIM_DIR/gazebo/env.sh"
REGION="${1:-thiru_demo}"
MISSION="${2:-}"
shift $(( $# > 2 ? 2 : $# ))

WORLD_DIR="$SIM_DIR/worlds/$REGION"
if [[ ! -f "$WORLD_DIR/world_manifest.json" ]]; then
  echo "No world for '$REGION'. Build it first:" >&2
  echo "  python prepare_sim_region.py --region-id $REGION --strict && python gazebo_world_builder.py $REGION" >&2
  exit 1
fi
WORLD_NAME="fyp_${REGION}"
WORLD_SDF="$WORLD_DIR/${WORLD_NAME}.sdf"

if ! command -v gz >/dev/null; then
  echo "'gz' not found — install Gazebo Harmonic (PX4 ubuntu.sh does) and open a new shell." >&2
  exit 1
fi
FOUND_PLUGIN=0
IFS=':' read -ra PLUGIN_DIRS <<< "${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
for d in "${PLUGIN_DIRS[@]}"; do
  [[ -n "$d" && -f "$d/libArduPilotPlugin.so" ]] && FOUND_PLUGIN=1
done
if [[ $FOUND_PLUGIN -eq 0 ]]; then
  echo "WARNING: libArduPilotPlugin.so not found on GZ_SIM_SYSTEM_PLUGIN_PATH — the iris will not talk to SITL." >&2
fi

export GZ_SIM_RESOURCE_PATH="$WORLD_DIR/models:$WORLD_DIR:${GZ_SIM_RESOURCE_PATH:-}"

echo "──────────────────────────────────────────────────────────────────────"
echo " World : $WORLD_SDF"
echo " SITL  : run in a second terminal (from ~/ardupilot or with sim_vehicle.py on PATH):"
echo "   $(cat "$WORLD_DIR/sitl_command.txt")"
echo " QGC   : Windows QGroundControl auto-connects on UDP 14550 (WSL mirrored networking),"
echo "         else add --out=udp:<windows-ip>:14550 to the sim_vehicle.py line."
echo "──────────────────────────────────────────────────────────────────────"

GUI_ARGS=()
[[ -f "$WORLD_DIR/gui.config" ]] && GUI_ARGS=(--gui-config "$WORLD_DIR/gui.config")   # follow camera
gz sim -v3 -r "${GUI_ARGS[@]}" "$@" "$WORLD_SDF" &
GZ_PID=$!
trap 'kill $GZ_PID 2>/dev/null || true' EXIT
trap 'exit 130' INT TERM

wait_for_world() {
  for _ in $(seq 1 60); do
    if gz service -l 2>/dev/null | grep -q "^/world/$WORLD_NAME/create$"; then return 0; fi
    sleep 1
  done
  return 1
}

if [[ -n "$MISSION" ]]; then
  MARKERS="$(cd "$MISSION" && pwd)/route_markers.sdf"
  echo "Waiting for world $WORLD_NAME to spawn route markers …"
  wait_for_world || true
  gz service -s "/world/$WORLD_NAME/create" --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
    --timeout 5000 --req "sdf_filename: \"$MARKERS\", name: \"route_markers\"" \
    && echo "Route markers spawned (thin blue = Tier 1 front, thick red = Tier 2 pick)." \
    || echo "WARNING: could not spawn $MARKERS" >&2
fi

# Re-send the follow request on /gui/track in case the GUI ignored gui.config's CameraTracking
# settings or started late. Stops after ~10 s so the user can move the camera freely afterwards.
if [[ -n "${GUI_ARGS[*]}" ]] && wait_for_world && read -r VEHICLE OX OY OZ < <(python3 -c 'import json,sys; m=json.load(open(sys.argv[1])); print(m["vehicle_model"], *m.get("gui", {}).get("camera_offset", [-6, -8, 8]))' "$WORLD_DIR/world_manifest.json"); then
  ( for _ in 1 2 3 4 5; do
      gz topic -t /gui/track -m gz.msgs.CameraTrack -p "track_mode: FOLLOW, follow_target: {name: \"$VEHICLE\"}, follow_offset: {x: $OX, y: $OY, z: $OZ}, follow_pgain: 0.05" >/dev/null 2>&1 || true
      sleep 2
    done ) &
fi

wait $GZ_PID
