#!/usr/bin/env bash
set -e
SIM_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SIM_DIR/gazebo/env.sh"
cd "$SIM_DIR"
python -u launch_simulation.py "$@"
