#!/usr/bin/env bash
# Source from bash or zsh before running the project-local stack.
if [[ -n "${BASH_VERSION:-}" ]]; then
  FYP_ENV_SCRIPT="${BASH_SOURCE[0]}"
elif [[ -n "${ZSH_VERSION:-}" ]]; then
  FYP_ENV_SCRIPT="${(%):-%x}"
else
  echo "Gazebo environment requires bash or zsh." >&2
  return 1
fi
FYP_SIM_DIR="$(cd "$(dirname "$FYP_ENV_SCRIPT")/.." && pwd)"
# Repair paths exported by the former bash-only loader when sourced in zsh
# from Simulation/. Keep valid custom installations intact.
FYP_REPO_DIR="$(dirname "$FYP_SIM_DIR")"
if [[ "${FYP_ARDUPILOT_DIR:-}" == "$FYP_REPO_DIR/.local/ardupilot" && ! -d "$FYP_ARDUPILOT_DIR" ]]; then
  unset FYP_ARDUPILOT_DIR
fi
if [[ "${FYP_ARDUPILOT_GAZEBO_DIR:-}" == "$FYP_REPO_DIR/.local/ardupilot_gazebo" && ! -d "$FYP_ARDUPILOT_GAZEBO_DIR" ]]; then
  unset FYP_ARDUPILOT_GAZEBO_DIR
fi
FYP_GZ_ENV="${FYP_GAZEBO_ENV:-$FYP_SIM_DIR/.gazebo-env}"
if [[ -d "$FYP_GZ_ENV/conda-meta" ]]; then
  export CONDA_PREFIX="$FYP_GZ_ENV"
  export PATH="$FYP_GZ_ENV/bin:$PATH"
  export CMAKE_PREFIX_PATH="$FYP_GZ_ENV:${CMAKE_PREFIX_PATH:-}"
  export PKG_CONFIG_PATH="$FYP_GZ_ENV/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
  export GZ_CONFIG_PATH="$FYP_GZ_ENV/share/gz:${GZ_CONFIG_PATH:-}"
  # Keep Qt 5 (Gazebo Harmonic) separate from any Qt 6 packages on this Mac.
  export QT_PLUGIN_PATH="$FYP_GZ_ENV/plugins"
  export QML2_IMPORT_PATH="$FYP_GZ_ENV/qml"
  export OGRE_RESOURCE_PATH="$FYP_GZ_ENV/lib/OGRE"
  export OGRE2_RESOURCE_PATH="$FYP_GZ_ENV/lib/OGRE-Next"
fi
export GZ_VERSION=harmonic
export FYP_ARDUPILOT_DIR="${FYP_ARDUPILOT_DIR:-$FYP_SIM_DIR/.local/ardupilot}"
export FYP_ARDUPILOT_GAZEBO_DIR="${FYP_ARDUPILOT_GAZEBO_DIR:-$FYP_SIM_DIR/.local/ardupilot_gazebo}"
export PATH="$FYP_ARDUPILOT_DIR/Tools/autotest:$PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$FYP_ARDUPILOT_GAZEBO_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$FYP_ARDUPILOT_GAZEBO_DIR/models:$FYP_ARDUPILOT_GAZEBO_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}"
