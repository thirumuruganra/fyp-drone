#!/usr/bin/env bash
# Start the generated Gazebo city world and (optionally) spawn a mission's route markers.
#
#   gazebo/launch_region.sh <region_id> [mission_dir] [extra gz sim args...]
#
#   gazebo/launch_region.sh thiru_1500
#   gazebo/launch_region.sh thiru_1500 missions/20261006_101500_thiru_1500_dest0_fragile_medical
#   gazebo/launch_region.sh thiru_1500 "" --render-engine ogre     # WSLg / VM rendering fallback
#
# Needs: Gazebo Harmonic+ (gz), ardupilot_gazebo built and on GZ_SIM_SYSTEM_PLUGIN_PATH /
# GZ_SIM_RESOURCE_PATH (see Simulation/README.md). Then start SITL in another terminal with the
# sim_vehicle.py line this script prints (its --custom-location MUST match the world origin).
set -euo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REGION="${1:-thiru_1500}"
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

gz sim -v3 -r "$@" "$WORLD_SDF" &
GZ_PID=$!
trap 'kill $GZ_PID 2>/dev/null || true' INT TERM

if [[ -n "$MISSION" ]]; then
  MARKERS="$(cd "$MISSION" && pwd)/route_markers.sdf"
  echo "Waiting for world $WORLD_NAME to spawn route markers …"
  for _ in $(seq 1 60); do
    if gz service -l 2>/dev/null | grep -q "^/world/$WORLD_NAME/create$"; then break; fi
    sleep 1
  done
  gz service -s "/world/$WORLD_NAME/create" --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
    --timeout 5000 --req "sdf_filename: \"$MARKERS\", name: \"route_markers\"" \
    && echo "Route markers spawned (thin blue = Tier 1 front, thick red = Tier 2 pick)." \
    || echo "WARNING: could not spawn $MARKERS" >&2
fi

wait $GZ_PID
