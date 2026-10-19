#!/usr/bin/env bash
# One-time local install. Uses existing Conda and Xcode Command Line Tools.
set -euo pipefail
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONDA_BIN="${CONDA_EXE:-$(command -v conda || true)}"
[[ -n "$CONDA_BIN" ]] || { echo "Install Conda/Miniforge first." >&2; exit 1; }
[[ "$(uname -sm)" == "Darwin arm64" ]] || { echo "This setup targets Apple Silicon macOS." >&2; exit 1; }
xcode-select -p >/dev/null
mkdir -p "$SIM_DIR/.local"
if [[ ! -x "$SIM_DIR/.gazebo-env/bin/gz" ]]; then
  "$CONDA_BIN" env create --prefix "$SIM_DIR/.gazebo-env" --file "$SIM_DIR/gazebo/environment-macos.yml" -y
fi
source "$SIM_DIR/gazebo/env.sh"
python -m pip install -r "$SIM_DIR/requirements-simulation.txt" 'empy==3.3.4' \
  pexpect future pyserial psutil pyparsing dronecan setuptools
if [[ ! -d "$FYP_ARDUPILOT_GAZEBO_DIR/.git" ]]; then
  git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$FYP_ARDUPILOT_GAZEBO_DIR"
fi
if [[ ! -d "$FYP_ARDUPILOT_DIR/.git" ]]; then
  git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$FYP_ARDUPILOT_DIR"
fi
cmake -S "$FYP_ARDUPILOT_GAZEBO_DIR" -B "$FYP_ARDUPILOT_GAZEBO_DIR/build" -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build "$FYP_ARDUPILOT_GAZEBO_DIR/build" --parallel 4
(
  cd "$FYP_ARDUPILOT_DIR"
  NM=/usr/bin/nm SIZE=/usr/bin/size python waf configure --board sitl
  python waf copter -j4
)
cmake -S "$SIM_DIR/gazebo/macos" -B "$SIM_DIR/.local/macos" -G Ninja
cmake --build "$SIM_DIR/.local/macos"
echo "Ready. Open $SIM_DIR/Launch Simulation.command"
