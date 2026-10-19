"""
Shared paths for the Gazebo / ArduPilot SITL simulation scripts.

Importing this module puts ../Implementation on sys.path, so the simulation
reuses the planner modules (config, env_data, module1..4, pareto_store,
mission_export) directly instead of copying them.
"""

from __future__ import annotations

import sys
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
REPO_DIR = SIM_DIR.parent
IMPL_DIR = REPO_DIR / "Implementation"

if str(IMPL_DIR) not in sys.path:
    sys.path.insert(0, str(IMPL_DIR))
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

IMPL_CACHE_DIR = IMPL_DIR / "cache"            # OSM / elevation / weather cache shared with the planner
FIXTURE_DIR = IMPL_DIR / "tests" / "fixtures"  # hand-made offline fixtures (NOT real data)

# Generated artefacts (git-ignored)
REGIONS_DIR = SIM_DIR / "regions"              # frozen region bundles
WORLDS_DIR = SIM_DIR / "worlds"                # generated Gazebo worlds + meshes
MISSIONS_DIR = SIM_DIR / "missions"            # planned missions (selection, .plan, .waypoints, markers)
RUNS_DIR = SIM_DIR / "runs"                    # flown runs (telemetry, metrics, plots)
TEMPLATES_DIR = SIM_DIR / "gazebo" / "templates"
SITL_OVERRIDES = SIM_DIR / "gazebo" / "sitl_overrides.parm"   # layered after the ArduPilot Iris defaults
