#!/usr/bin/env bash
# Source (bash) before running the simulation stack:  source Simulation/gazebo/env.sh
# Points at the ArduPilot and ardupilot_gazebo checkouts from README Phase 0. Override with
# FYP_ARDUPILOT_DIR / FYP_ARDUPILOT_GAZEBO_DIR; a checkout under Simulation/.local/ wins otherwise.
FYP_SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_fyp_pick() {    # first existing directory, else the last candidate
  local d
  for d in "$@"; do [[ -d "$d" ]] && { echo "$d"; return; }; done
  echo "${@: -1}"
}
export GZ_VERSION="${GZ_VERSION:-harmonic}"
export FYP_ARDUPILOT_DIR="${FYP_ARDUPILOT_DIR:-$(_fyp_pick "$FYP_SIM_DIR/.local/ardupilot" "$HOME/ardupilot")}"
export FYP_ARDUPILOT_GAZEBO_DIR="${FYP_ARDUPILOT_GAZEBO_DIR:-$(_fyp_pick "$FYP_SIM_DIR/.local/ardupilot_gazebo" "$HOME/ardupilot_gazebo")}"
unset -f _fyp_pick
case ":$PATH:" in *":$FYP_ARDUPILOT_DIR/Tools/autotest:"*) ;; *) export PATH="$FYP_ARDUPILOT_DIR/Tools/autotest:$PATH" ;; esac
case ":${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:" in *":$FYP_ARDUPILOT_GAZEBO_DIR/build:"*) ;;
  *) export GZ_SIM_SYSTEM_PLUGIN_PATH="$FYP_ARDUPILOT_GAZEBO_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}" ;; esac
case ":${GZ_SIM_RESOURCE_PATH:-}:" in *":$FYP_ARDUPILOT_GAZEBO_DIR/models:"*) ;;
  *) export GZ_SIM_RESOURCE_PATH="$FYP_ARDUPILOT_GAZEBO_DIR/models:$FYP_ARDUPILOT_GAZEBO_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}" ;; esac
