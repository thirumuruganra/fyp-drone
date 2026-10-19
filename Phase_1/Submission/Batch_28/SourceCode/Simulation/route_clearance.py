"""
Phase 3: hard clearance checks so a planned route is physically flyable in the
Gazebo city.

The planner's lattice turns building/terrain proximity into *risk*; it never
removes an edge whose straight segment cuts a roof (node altitude only covers
the tallest building in each node's own cell, and edge altitude is linear
between nodes).  ClearanceChecker rejects such edges BEFORE Tier 1 runs, so
the Pareto front only holds routes the drone can actually fly — waypoints are
never raised after selection.

Segment test (exact for straight, linearly climbing segments):
  * buffer each footprint by `buffer_m` (drone size + tracking + map error)
  * part of the horizontal segment inside the buffered footprint: t ∈ [t0, t1]
  * altitude is linear in t → minimum over the part is at t0 or t1
  * clearance = that minimum − roof AMSL ≥ roof_clearance_m
Terrain: sampled every `terrain_step_m` from the bundle DEM (never interpolated
from the endpoints only, so a ridge between two safe nodes is caught).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

import _paths  # noqa: F401
import config as cfg
import sim_config as scfg
from env_data import ElevationGrid, LocalFrame
from sim_region import sample_grid


class NoFeasibleMission(RuntimeError):
    """No clearance-checked way to connect a pad / fly the mission."""


@dataclass
class ClearanceResult:
    ok: bool
    min_roof_clearance_m: float          # +inf when the buffered segment crosses no footprint
    min_terrain_clearance_m: float
    max_agl_m: float
    blocking: list = field(default_factory=list)   # indices of footprints violating clearance
    reason: str = ""


class RoofIndex:
    """Footprints (local metres) with absolute roof altitude, buffered for the clearance test."""

    def __init__(self, geoms: Sequence, roof_amsl: Sequence[float], buffer_m: float = scfg.SIM_EDGE_BUFFER_M):
        from shapely import STRtree
        self.geoms = list(geoms)
        self.roof_amsl = np.asarray(roof_amsl, dtype=float)
        self.buffer_m = float(buffer_m)
        self.buffered = [g.buffer(self.buffer_m) for g in self.geoms]
        self.tree = STRtree(self.buffered) if self.buffered else None

    def __len__(self):
        return len(self.geoms)

    @classmethod
    def from_bundle(cls, bundle, frame: LocalFrame, buffer_m: float = scfg.SIM_EDGE_BUFFER_M) -> "RoofIndex":
        geoms, roofs = [], []
        for geom, row in bundle.local_footprints(frame):
            geoms.append(geom)
            roofs.append(float(row["roof_amsl_m"]))
        return cls(geoms, roofs, buffer_m)

    def candidates(self, geom) -> np.ndarray:
        if self.tree is None:
            return np.zeros(0, dtype=int)
        return np.asarray(self.tree.query(geom, predicate="intersects"), dtype=int)

    def roof_at(self, x: float, y: float, buffered: bool = True) -> float:
        """Highest roof AMSL whose (buffered) footprint contains (x, y); −inf if none."""
        from shapely.geometry import Point
        pt = Point(x, y)
        best = -math.inf
        for k in self.candidates(pt):
            g = self.buffered[k] if buffered else self.geoms[k]
            if g.contains(pt) or g.touches(pt):
                best = max(best, float(self.roof_amsl[k]))
        return best


def _interval_ts(part, p0: np.ndarray, d: np.ndarray, length2: float):
    """Parametric t of the end points of an intersection piece along p0 + t·d."""
    from shapely.geometry import LineString, Point
    pieces = getattr(part, "geoms", [part])
    out = []
    for g in pieces:
        if g.is_empty:
            continue
        if isinstance(g, Point):
            pts = [np.array(g.coords[0])]
        elif isinstance(g, LineString):
            c = np.asarray(g.coords)
            pts = [c[0], c[-1]]
        else:                                           # GeometryCollection etc.
            out.extend(_interval_ts(g, p0, d, length2))
            continue
        ts = [float(np.clip(np.dot(q - p0, d) / length2, 0.0, 1.0)) for q in pts]
        out.append((min(ts), max(ts)))
    return out


class ClearanceChecker:
    """Hard roof / terrain / ceiling checks in one local frame (the planner frame)."""

    def __init__(self, roof_index: RoofIndex, terrain: ElevationGrid, frame: LocalFrame, *,
                 roof_clearance_m: float = scfg.SIM_ROOF_CLEARANCE_M,
                 terrain_clearance_m: float = scfg.SIM_TERRAIN_CLEARANCE_M,
                 max_agl_m: float = cfg.H_MAX_AGL,
                 terrain_step_m: float = scfg.SIM_TERRAIN_STEP_M,
                 ceiling_tol_m: float = 1.0):
        self.roofs = roof_index
        self.terrain = terrain
        self.frame = frame
        self.roof_clearance_m = roof_clearance_m
        self.terrain_clearance_m = terrain_clearance_m
        self.max_agl_m = max_agl_m
        self.terrain_step_m = terrain_step_m
        self.ceiling_tol_m = ceiling_tol_m
        self.n_checked = 0
        self.n_rejected = 0

    @classmethod
    def from_bundle(cls, bundle, frame: Optional[LocalFrame] = None, **kw) -> "ClearanceChecker":
        frame = frame or bundle.planner_frame
        buffer_m = kw.pop("buffer_m", scfg.SIM_EDGE_BUFFER_M)
        return cls(RoofIndex.from_bundle(bundle, frame, buffer_m), bundle.terrain, frame, **kw)

    def ground_xy(self, xs, ys) -> np.ndarray:
        lat, lon = self.frame.to_latlon(np.asarray(xs, float), np.asarray(ys, float))
        return sample_grid(self.terrain, lat, lon)

    # ── segments ────────────────────────────────────────────────────────────
    def segment(self, p0, p1, z0: float, z1: float, *, check_terrain: bool = True,
                check_ceiling: bool = True) -> ClearanceResult:
        """Straight segment p0 → p1 (local x, y) flown from z0 to z1 m AMSL."""
        from shapely.geometry import LineString, Point
        p0 = np.asarray(p0, dtype=float)
        p1 = np.asarray(p1, dtype=float)
        d = p1 - p0
        length2 = float(np.dot(d, d))
        geom = Point(p0) if length2 < 1e-9 else LineString([p0, p1])

        min_roof = math.inf
        blocking = []
        for k in self.roofs.candidates(geom):
            poly = self.roofs.buffered[k]
            if length2 < 1e-9:
                zmin = min(z0, z1)
            else:
                ts = _interval_ts(geom.intersection(poly), p0, d, length2)
                if not ts:
                    continue
                zmin = min(min(z0 + (z1 - z0) * a, z0 + (z1 - z0) * b) for a, b in ts)
            c = zmin - float(self.roofs.roof_amsl[k])
            min_roof = min(min_roof, c)
            if c < self.roof_clearance_m:
                blocking.append(int(k))

        min_terr, max_agl = math.inf, -math.inf
        if check_terrain or check_ceiling:
            n = max(2, int(math.ceil(math.sqrt(length2) / self.terrain_step_m)) + 1)
            t = np.linspace(0.0, 1.0, n)
            g = self.ground_xy(p0[0] + t * d[0], p0[1] + t * d[1])
            agl = (z0 + t * (z1 - z0)) - g
            min_terr, max_agl = float(agl.min()), float(agl.max())

        reasons = []
        if blocking:
            reasons.append(f"roof clearance {min_roof:.1f} m < {self.roof_clearance_m:.0f} m")
        if check_terrain and min_terr < self.terrain_clearance_m:
            reasons.append(f"terrain clearance {min_terr:.1f} m < {self.terrain_clearance_m:.0f} m")
        if check_ceiling and max_agl > self.max_agl_m + self.ceiling_tol_m:
            reasons.append(f"{max_agl:.1f} m AGL above the {self.max_agl_m:.0f} m ceiling")
        return ClearanceResult(not reasons, min_roof, min_terr, max_agl, blocking, "; ".join(reasons))

    def edge_filter(self, a: dict, b: dict) -> bool:
        """LatticeAirspaceBuilder(edge_filter=...) hook: keep the edge only if it clears."""
        r = self.segment((a["x_m"], a["y_m"]), (b["x_m"], b["y_m"]), a["alt_amsl_m"], b["alt_amsl_m"])
        self.n_checked += 1
        if not r.ok:
            self.n_rejected += 1
        return r.ok

    # ── pads (vertical take-off / landing columns + connector legs) ─────────
    def pad_column(self, xy, ground_amsl: float, top_amsl: float) -> ClearanceResult:
        """Vertical climb/descent at a pad: no (buffered) footprint may cover the pad."""
        roof = self.roofs.roof_at(xy[0], xy[1], buffered=True)
        if roof > -math.inf:
            return ClearanceResult(False, -math.inf, 0.0, top_amsl - ground_amsl, [],
                                   "pad lies on / within the buffer of a building footprint")
        return ClearanceResult(True, math.inf, 0.0, top_amsl - ground_amsl)


@dataclass
class PadConnection:
    node: object
    pad: dict
    node_alt_amsl_m: float
    distance_m: float
    leg: ClearanceResult


def connect_pad(G, pad: dict, checker: ClearanceChecker, *, max_dist_m: float = scfg.SIM_PAD_CONNECT_MAX_M,
                frame: Optional[LocalFrame] = None) -> PadConnection:
    """
    Nearest lattice node reachable from the pad by a checked profile:
    vertical column at the pad up to the node's cruise altitude, then a level
    leg to the node.  Raises NoFeasibleMission when no node within max_dist_m works.
    """
    frame = frame or checker.frame
    px, py = (float(v) for v in frame.to_local(pad["lat"], pad["lon"]))
    ground = float(pad["ground_amsl_m"])
    nodes = sorted(G.nodes, key=lambda n: math.hypot(G.nodes[n]["x_m"] - px, G.nodes[n]["y_m"] - py))
    reasons = []
    for n in nodes:
        a = G.nodes[n]
        dist = math.hypot(a["x_m"] - px, a["y_m"] - py)
        if dist > max_dist_m:
            break
        z = float(a["alt_amsl_m"])
        col = checker.pad_column((px, py), ground, z)
        if not col.ok:
            raise NoFeasibleMission(f"pad {pad.get('name', '?')}: {col.reason}")
        leg = checker.segment((px, py), (a["x_m"], a["y_m"]), z, z)
        if leg.ok:
            return PadConnection(n, pad, z, dist, leg)
        reasons.append(f"node {n}: {leg.reason}")
    raise NoFeasibleMission(f"pad {pad.get('name', '?')}: no clearance-checked lattice node within "
                            f"{max_dist_m:.0f} m" + (f" ({'; '.join(reasons[:3])})" if reasons else ""))


def check_polyline(checker: ClearanceChecker, pts_xy, alts_amsl) -> list:
    """ClearanceResult of every leg of a polyline (used to re-verify a final mission)."""
    return [checker.segment(pts_xy[i], pts_xy[i + 1], alts_amsl[i], alts_amsl[i + 1])
            for i in range(len(pts_xy) - 1)]
