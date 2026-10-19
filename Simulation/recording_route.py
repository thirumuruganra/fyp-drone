"""Ordered recording corridor, using the shared environment/cost/clearance pipeline.

The ten checkpoints are operator constraints. Tier 1 / Tier 2 still run on this
corridor, whose single feasible path is not a region-wide optimality claim.
"""
import hashlib
import json
from datetime import datetime

import networkx as nx
import numpy as np

import _paths  # noqa: F401
import config as cfg
import sim_config as scfg
from env_data import LocalRasterProvider, build_layer_indexes
from module1_graph_builder import (EdgeFeatures, LatticeAirspaceBuilder, apply_weather,
                                  haversine_km, initial_bearing_deg)
from route_clearance import ClearanceChecker, NoFeasibleMission


def build_recording_graph(bundle, spacing_m, weather, *, risk_layers=True):
    """Ten ordered points, nine checked legs, no shortcut edges."""
    frame = bundle.planner_frame
    depot, dest = bundle.pads["depot"], bundle.pads["destinations"][0]
    start = np.array(frame.to_local(depot["lat"], depot["lon"]), dtype=float)
    end = np.array(frame.to_local(dest["lat"], dest["lon"]), dtype=float)
    direction = end - start
    length = float(np.linalg.norm(direction))
    if length <= 1.0:
        raise NoFeasibleMission("recording checkpoints need distinct source/destination")
    side = np.array([-direction[1], direction[0]]) / length
    points = np.array([start + t * direction + offset * side
                       for t, offset in scfg.DESKTOP_TURN_CHECKPOINTS])
    if len(points) != 10 or len(np.unique(points, axis=0)) != 10:
        raise NoFeasibleMission("recording corridor needs ten distinct checkpoints")
    if np.max(np.abs(points)) > bundle.box_m / 2:
        raise NoFeasibleMission("recording turn leaves the region's flyable box")
    xs, ys = points.T
    lats, lons = frame.to_latlon(xs, ys)
    # Keep pad endpoints bit-identical, including exported coordinate comparisons.
    lats[0], lons[0] = depot["lat"], depot["lon"]
    lats[-1], lons[-1] = dest["lat"], dest["lon"]
    layers = build_layer_indexes(bundle.planner_layers(), frame, buildings=False,
                                 risk_layers=risk_layers)
    builder = LatticeAirspaceBuilder(center=bundle.center, box_m=bundle.box_m,
        spacing_m=spacing_m, weather=weather,
        elevation_providers=[LocalRasterProvider(str(bundle.terrain_path))],
        building_index=bundle.building_index(), **layers)
    ground, buildings, agl, density, mean_height = builder._node_environment(lats, lons, xs, ys)
    if np.any(agl > cfg.H_MAX_AGL):
        raise NoFeasibleMission("recording checkpoint exceeds flight ceiling")
    zones = builder.restricted_zones
    if zones is not None and (zones.contains(xs, ys).any() or
            zones.intersects_segments(points[:-1], points[1:]).any()):
        raise NoFeasibleMission("recording corridor crosses a restricted zone")
    G = nx.DiGraph(spacing_km=spacing_m / 1000.0)
    for i in range(len(points)):
        altitude = float(ground[i] + agl[i])
        G.add_node(i, lat=float(lats[i]), lon=float(lons[i]),
            x=float(lons[i]), y=float(lats[i]), x_m=float(xs[i]), y_m=float(ys[i]),
            ground_elev_m=float(ground[i]), bldg_h_m=float(buildings[i]),
            bldg_density=float(density[i]), bldg_mean_h_m=float(mean_height[i]),
            agl_m=float(agl[i]), alt_amsl_m=altitude, alt_m=altitude)
    checker = ClearanceChecker.from_bundle(bundle)
    for i in range(len(points) - 1):
        a, b = G.nodes[i], G.nodes[i + 1]
        if not checker.edge_filter(a, b):
            raise NoFeasibleMission(f"recording leg {i + 1} fails roof/terrain clearance")
        horizontal = haversine_km(a["lat"], a["lon"], b["lat"], b["lon"])
        dz = b["alt_amsl_m"] - a["alt_amsl_m"]
        features = EdgeFeatures.build(horizontal,
            initial_bearing_deg(a["lat"], a["lon"], b["lat"], b["lon"]), dz,
            **builder._edge_terms(a, b, horizontal, dz / (horizontal * 1000)))
        G.add_edge(i, i + 1, **features.edge_attrs())
    identity = {"bundle": bundle.bundle_hash, "checkpoints": scfg.DESKTOP_TURN_CHECKPOINTS,
                "spacing_m": spacing_m, "weather": weather.to_dict(),
                "risk_layers": risk_layers, "version": "recording-corridor-1",
                "buffer_m": scfg.SIM_EDGE_BUFFER_M, "roof_m": scfg.SIM_ROOF_CLEARANCE_M,
                "terrain_m": scfg.SIM_TERRAIN_CLEARANCE_M, "max_agl_m": cfg.H_MAX_AGL}
    key = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()[:12]
    G.graph.update(bundle_hash=bundle.bundle_hash, region_id=bundle.region_id,
        graph_key=key, hard_clearance=True, route_mode="ordered_recording_corridor",
        location=f"sim-{bundle.region_id}-recording-turns", built_at=datetime.now().isoformat(),
        **builder.build_info)
    apply_weather(G, weather)
    return G, checker, key
