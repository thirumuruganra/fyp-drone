"""
Module 1 — Airspace Graph Builder
==================================
FYP: Payload-Aware Two-Tier UAV Route Planning for Last-Mile Delivery

Builds the delivery airspace graph G = (V, E).  Every edge carries a
3-objective cost vector

      c(e) = [d(e),  ε(e),  ρ(e)]

  d(e) : flight distance of the segment (km, 3-D: √(d_h² + Δalt²))
  ε(e) : energy proxy (km-equivalent) = d · (1 + |grade| + headwind + crosswind)
  ρ(e) : flight risk ∈ [0, 1] = clip(Σ w_k · ρ_k)

Node altitude (lattice): alt_amsl = ground + max(H_CRUISE_AGL, tallest building
in the node's cell + CLEARANCE); nodes needing more than H_MAX_AGL are blocked.
Edge grade = Δalt / d_h from the endpoint altitudes.

Shared cost pipeline (used by every builder):

      static edge attributes  →  apply_weather(G, snapshot)  →  energy / risk
      (d_h, bearing, grade,        energy_cost(), risk_components()
       obstacle, building, terrain)

Static attributes never change after construction; apply_weather() can be
re-run on the same graph with a new WeatherSnapshot (calm vs windy, refresh).

Builders:
  AirspaceGraphBuilder     — seeded synthetic grid (benchmarks S1–S3, demo)
  LatticeAirspaceBuilder   — regular lattice over Thiruvanmiyur, Chennai

HOW TO RUN:
    python module1_graph_builder.py

OUTPUT:
    pareto_front_storage/airspace_graph.pkl  — serialised NetworkX DiGraph
    outputs/module1_airspace_graph.png       — visualisation
"""

import math
import os
import pickle
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np
import networkx as nx
import matplotlib
matplotlib.use("Agg")          # non-interactive backend — safe on all OS
import matplotlib.pyplot as plt
from matplotlib.figure import Figure

import config as cfg
from env_data import (BuildingIndex, ElevationProvider, LocalFrame, WeatherSnapshot,
                      fetch_elevations, fetch_weather, seeded_weather)


# ─────────────────────────────────────────────────────────────────────────────
#  Geometry
# ─────────────────────────────────────────────────────────────────────────────

def haversine_km(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in km."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = p2 - p1
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
    return 2.0 * cfg.EARTH_RADIUS_KM * math.asin(min(1.0, math.sqrt(a)))


def initial_bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Compass bearing (0° = N, 90° = E) of the great circle from point 1 to point 2."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlmb = math.radians(lon2 - lon1)
    y = math.sin(dlmb) * math.cos(p2)          # cos(lat) scaling of the east component
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dlmb)
    return math.degrees(math.atan2(y, x)) % 360.0


# ─────────────────────────────────────────────────────────────────────────────
#  Shared cost pipeline:  EdgeFeatures → apply_weather → energy / risk
# ─────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class EdgeFeatures:
    """Static (weather-independent) description of one flight segment."""
    d_h_km: float          # horizontal haversine length
    distance_km: float     # 3-D length √(d_h² + Δalt²)
    bearing_deg: float     # heading u → v
    grade: float           # signed Δalt / d_h
    obstacle: float        # ρ_obstacle ∈ [0, 1]   (synthetic obstacle discs)
    building: float = 0.0  # ρ_building ∈ [0, 1]   (footprint proximity × vertical margin)
    terrain: float = 0.0   # ρ_terrain ∈ [0, 1]    (clearance margin, sudden altitude change)

    @classmethod
    def build(cls, d_h_km: float, bearing_deg: float,
              dalt_m: float = 0.0, obstacle: float = 0.0,
              building: float = 0.0, terrain: Optional[float] = None) -> "EdgeFeatures":
        """terrain=None → clip(|grade| / GRADE_MAX): the sudden-altitude-change term alone."""
        grade = (dalt_m / 1000.0) / d_h_km if d_h_km > 0 else 0.0
        distance = math.hypot(d_h_km, dalt_m / 1000.0)
        if terrain is None:
            terrain = abs(grade) / cfg.GRADE_MAX
        unit = lambda x: float(np.clip(x, 0.0, 1.0))
        return cls(d_h_km, distance, bearing_deg % 360.0, grade,
                   unit(obstacle), unit(building), unit(terrain))

    @classmethod
    def from_edge(cls, data: dict) -> "EdgeFeatures":
        try:
            return cls(data["d_h"], data["distance"], data["bearing"],
                       data["grade"], data["obstacle"],
                       data.get("building", 0.0), data.get("terrain", 0.0))
        except KeyError as exc:
            raise KeyError(f"edge lacks static attribute {exc}; rebuild the graph "
                           f"with a current builder") from None

    def edge_attrs(self) -> dict:
        return {"d_h": round(self.d_h_km, 5), "distance": round(self.distance_km, 5),
                "bearing": round(self.bearing_deg, 3), "grade": round(self.grade, 5),
                "obstacle": round(self.obstacle, 5), "building": round(self.building, 5),
                "terrain": round(self.terrain, 5)}


def wind_components(bearing_deg: float, speed_kmh: float,
                    dir_from_deg: float) -> Tuple[float, float]:
    """
    Split the wind into (headwind, crosswind) for an edge flown on `bearing_deg`.

    dir_from_deg is the meteorological direction the wind blows FROM.  An edge
    heading 90° (east) in wind from 90° meets the wind head-on:
        HW = v · cos(θ_edge − θ_from)     positive = headwind, negative = tailwind
        CW = v · |sin(θ_edge − θ_from)|   ≥ 0
    """
    rel = math.radians(bearing_deg - dir_from_deg)
    return speed_kmh * math.cos(rel), speed_kmh * abs(math.sin(rel))


def energy_cost(f: EdgeFeatures, w: WeatherSnapshot) -> float:
    """
    ε = d · (1 + |grade| + β_head·max(0, HW)/V_cruise + β_cross·CW/V_cruise)  ≥  d.

    Tailwind is clamped to zero benefit (no energy recovery), which keeps
    ε ≥ d and the Tier 1 heuristic admissible.
    """
    hw, cw = wind_components(f.bearing_deg, w.wind_speed_kmh, w.wind_dir_from_deg)
    factor = (1.0 + abs(f.grade)
              + cfg.BETA_HEAD * max(0.0, hw) / cfg.V_CRUISE_KMH
              + cfg.BETA_CROSS * cw / cfg.V_CRUISE_KMH)
    return f.distance_km * factor


def risk_components(f: EdgeFeatures, w: WeatherSnapshot) -> dict:
    """
    Weighted risk contributions w_k·ρ_k (each ρ_k ∈ [0, 1]); ρ = clip(Σ, 0, 1).

      obstacle : static, synthetic obstacle discs
      building : static, fraction of along-edge samples near a footprint × vertical margin
      terrain  : static, max(clearance margin, |grade| / GRADE_MAX)
      wind     : (CW + k_g·max(0, gust − v)) / V_MAX_SAFE   — crosswind + gust instability
      weather  : uniform; rain and poor visibility
    """
    _, cw = wind_components(f.bearing_deg, w.wind_speed_kmh, w.wind_dir_from_deg)
    wind_term = (cw + cfg.K_GUST * max(0.0, w.gust_kmh - w.wind_speed_kmh)) / cfg.V_MAX_SAFE_KMH
    weather_term = (cfg.W_PRECIP * min(1.0, max(0.0, w.precipitation_mm / cfg.PRECIP_HEAVY_MM_H))
                    + cfg.W_VIS * min(1.0, max(0.0, 1.0 - w.visibility_m / cfg.VIS_CLEAR_M)))
    terms = {
        "obstacle": f.obstacle,
        "building": f.building,
        "terrain": f.terrain,
        "wind": min(1.0, wind_term),
        "weather": min(1.0, weather_term),
    }
    return {k: cfg.RISK_WEIGHTS[k] * v for k, v in terms.items()}


def ground_speed_kmh(f: EdgeFeatures, w: WeatherSnapshot) -> float:
    """Speed over ground when crabbing into the wind to hold the track (informational)."""
    hw, cw = wind_components(f.bearing_deg, w.wind_speed_kmh, w.wind_dir_from_deg)
    along = math.sqrt(max(cfg.V_CRUISE_KMH ** 2 - cw ** 2, 0.0)) - hw
    return max(along, 1.0)


def apply_weather(G: nx.DiGraph, snapshot: Optional[WeatherSnapshot] = None) -> nx.DiGraph:
    """
    (Re)compute energy, risk, cost, risk_components and time_s on every edge
    from its static attributes and `snapshot` (calm when None).  Idempotent;
    safe to call repeatedly with different snapshots.
    """
    w = snapshot if snapshot is not None else WeatherSnapshot.calm()
    for _, _, data in G.edges(data=True):
        f = EdgeFeatures.from_edge(data)
        eps = round(energy_cost(f, w), 5)
        comps = risk_components(f, w)
        rho = round(float(np.clip(sum(comps.values()), 0.0, 1.0)), 5)
        data["energy"] = eps
        data["risk"] = rho
        data["risk_components"] = {k: round(v, 5) for k, v in comps.items()}
        data["cost"] = [data["distance"], eps, rho]
        data["time_s"] = round(3600.0 * f.distance_km / ground_speed_kmh(f, w), 2)
    G.graph["weather"] = w.to_dict()
    return G


# ─────────────────────────────────────────────────────────────────────────────
#  Synthetic builder
# ─────────────────────────────────────────────────────────────────────────────

class AirspaceGraphBuilder:
    """
    Builds the UAV delivery airspace graph.  The base class is the seeded
    synthetic grid used by the benchmarks and the synthetic demo; subclasses
    (LatticeAirspaceBuilder) override fetch_graph().

    Parameters
    ----------
    location : str
        Label of the region (informational for the synthetic grid).
    dist : int
        Region radius in metres (informational for the synthetic grid).
    seed : int
        Seed for the synthetic obstacles, terrain, wind and node jitter.
    weather : WeatherSnapshot, optional
        Wind/weather applied to the graph.  Synthetic default: seeded wind.
    obstacle_density : float
        Fraction of the synthetic area covered by obstacle discs.
    """

    def __init__(self,
                 location: str = cfg.LOCATION_NAME,
                 dist: int = 2000,
                 seed: int = 42,
                 weather: Optional[WeatherSnapshot] = None,
                 obstacle_density: float = cfg.SYN_OBSTACLE_DENSITY):
        self.location = location
        self.dist = dist
        self.seed = seed
        self.weather = weather
        self.obstacle_density = obstacle_density
        self.G: Optional[nx.DiGraph] = None   # the airspace graph (populated by fetch_graph)

    # ------------------------------------------------------------------
    #  Public API
    # ------------------------------------------------------------------

    def fetch_graph(self) -> nx.DiGraph:
        """
        Build the synthetic airspace graph (196 nodes).

        Returns
        -------
        G : nx.DiGraph
            Each edge carries distance, energy, risk, cost=[d, ε, ρ], plus the
            static attributes (d_h, bearing, grade, obstacle) and risk_components.
        """
        self.G = self._build_synthetic_graph(n_nodes=196)
        print(f"[Module 1] Graph ready: "
              f"{self.G.number_of_nodes()} nodes, "
              f"{self.G.number_of_edges()} edges")
        return self.G

    def save(self, path: str = "pareto_front_storage/airspace_graph.pkl") -> None:
        """Serialise the airspace graph to disk for Tier 1 re-use."""
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "wb") as fh:
            pickle.dump(self.G, fh)
        print(f"[Module 1] Graph saved → {path}")

    @staticmethod
    def load(path: str = "pareto_front_storage/airspace_graph.pkl") -> nx.DiGraph:
        """Load a previously saved airspace graph."""
        with open(path, "rb") as fh:
            return pickle.load(fh)

    def visualise(self,
                  source=None,
                  target=None,
                  route: Optional[list] = None,
                  title: str = "Module 1 — Airspace Graph") -> Figure:
        """
        Draw the airspace graph.

        Parameters
        ----------
        source, target : node IDs to highlight in green / red.
        route          : list of node IDs forming the selected UAV path.
        """
        assert self.G is not None, "Call fetch_graph() first."
        pos = {n: (d["x"], d["y"]) for n, d in self.G.nodes(data=True)}

        fig, ax = plt.subplots(figsize=(11, 9))
        ax.set_facecolor("#F8F8F8")

        # All edges (grey, thin)
        nx.draw_networkx_edges(self.G, pos, ax=ax,
                               edge_color="#C0C0C0", arrows=False,
                               width=0.6, alpha=0.7)

        # All nodes: shaded by altitude when it varies, else blue
        alts = [d.get("alt_m", 0.0) for _, d in self.G.nodes(data=True)]
        if max(alts) - min(alts) > 0.5:
            sc = nx.draw_networkx_nodes(self.G, pos, ax=ax, node_size=30,
                                        node_color=alts, cmap=plt.cm.viridis, alpha=0.9)
            fig.colorbar(sc, ax=ax, shrink=0.6, pad=0.01, label="Node altitude AMSL (m)")
        else:
            nx.draw_networkx_nodes(self.G, pos, ax=ax,
                                   node_size=25, node_color="#4A90D9", alpha=0.85)

        # Source / target highlights
        if source is not None:
            nx.draw_networkx_nodes(self.G, pos, nodelist=[source], ax=ax,
                                   node_size=180, node_color="#27AE60")
            ax.annotate("Source", xy=pos[source],
                        xytext=(pos[source][0]+0.001, pos[source][1]+0.001),
                        fontsize=9, color="#27AE60", fontweight="bold")
        if target is not None:
            nx.draw_networkx_nodes(self.G, pos, nodelist=[target], ax=ax,
                                   node_size=180, node_color="#E74C3C")
            ax.annotate("Target", xy=pos[target],
                        xytext=(pos[target][0]+0.001, pos[target][1]+0.001),
                        fontsize=9, color="#E74C3C", fontweight="bold")

        # Selected route (orange, thick)
        if route and len(route) > 1:
            route_edges = list(zip(route[:-1], route[1:]))
            nx.draw_networkx_edges(self.G, pos, edgelist=route_edges, ax=ax,
                                   edge_color="#E67E22", width=3.5,
                                   arrows=True, arrowsize=15,
                                   connectionstyle="arc3,rad=0.1")

        weather = self.G.graph.get("weather")
        wind_line = ""
        if weather:
            w = WeatherSnapshot.from_dict(weather)
            wind_line = f"\nWind: {w.describe()}"
        ax.set_title(title +
                     f"\n{self.G.number_of_nodes()} waypoints  |  "
                     f"{self.G.number_of_edges()} flight segments" + wind_line,
                     fontsize=13, fontweight="bold")
        ax.axis("off")
        plt.tight_layout()
        return fig

    # ------------------------------------------------------------------
    #  Private helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _add_edge(G: nx.DiGraph, u, v, features: EdgeFeatures) -> None:
        """Add an edge with its static attributes; costs are filled by apply_weather."""
        G.add_edge(u, v, **features.edge_attrs())

    def _build_synthetic_graph(self, n_nodes: int = 50) -> nx.DiGraph:
        """
        Seeded synthetic airspace graph.

        Nodes lie on a ~1.3 km grid with small jitter; each connects to its 8
        grid neighbours.  Static edge features come from a seeded terrain field
        (grade) and seeded obstacle discs; wind is a seeded snapshot.  Every
        draw uses a seed-derived generator, so the graph is deterministic.
        """
        print(f"[Module 1] Building synthetic graph with ~{n_nodes} nodes ...")
        rng = np.random.default_rng(self.seed)
        G = nx.DiGraph()
        side = int(np.ceil(np.sqrt(n_nodes)))

        offsets = {}
        for i in range(n_nodes):
            row, col = divmod(i, side)
            x = col * cfg.SYN_SPACING_DEG + float(rng.uniform(-cfg.SYN_JITTER_DEG, cfg.SYN_JITTER_DEG))
            y = row * cfg.SYN_SPACING_DEG + float(rng.uniform(-cfg.SYN_JITTER_DEG, cfg.SYN_JITTER_DEG))
            offsets[i] = (x, y)

        span = max((side - 1) * cfg.SYN_SPACING_DEG, 1e-9)
        terrain = self._seeded_terrain(span)
        obstacles = self._seeded_obstacles(span)

        for i, (x, y) in offsets.items():
            lat, lon = cfg.SYN_ORIGIN_LAT + y, cfg.SYN_ORIGIN_LON + x
            G.add_node(i, x=lon, y=lat, lat=lat, lon=lon, alt_m=round(terrain(x, y), 3))

        for i in range(n_nodes):
            ri, ci = divmod(i, side)
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rj, cj = ri + dr, ci + dc
                    j = rj * side + cj
                    if (dr == 0 and dc == 0) or not (0 <= cj < side) or not (0 <= j < n_nodes):
                        continue
                    ni, nj = G.nodes[i], G.nodes[j]
                    d_h = haversine_km(ni["lat"], ni["lon"], nj["lat"], nj["lon"])
                    brg = initial_bearing_deg(ni["lat"], ni["lon"], nj["lat"], nj["lon"])
                    xm = (offsets[i][0] + offsets[j][0]) / 2.0
                    ym = (offsets[i][1] + offsets[j][1]) / 2.0
                    self._add_edge(G, i, j, EdgeFeatures.build(
                        d_h, brg, nj["alt_m"] - ni["alt_m"], obstacles(xm, ym)))

        weather = self.weather if self.weather is not None else seeded_weather(self.seed)
        apply_weather(G, weather)
        print(f"[Module 1] Synthetic graph: {G.number_of_nodes()} nodes, "
              f"{G.number_of_edges()} edges | wind {weather.describe()}")
        return G

    def _seeded_terrain(self, span: float):
        """Smooth terrain: a few seeded Gaussian hills.  Returns alt(x, y) in metres."""
        rng = np.random.default_rng([self.seed, 1])
        k = cfg.SYN_TERRAIN_HILLS
        cx = rng.uniform(0.0, span, k)
        cy = rng.uniform(0.0, span, k)
        amp = rng.uniform(0.4, 1.0, k) * cfg.SYN_TERRAIN_AMP_M
        sig = cfg.SYN_TERRAIN_SIGMA_FRAC * span

        def alt(x: float, y: float) -> float:
            d2 = (cx - x) ** 2 + (cy - y) ** 2
            return float(np.sum(amp * np.exp(-d2 / (2.0 * sig ** 2))))
        return alt

    def _seeded_obstacles(self, span: float):
        """
        Seeded obstacle discs covering ≈ obstacle_density of the area.  Half are
        placed along the source→target diagonal (the shortest path must cross
        risk), the rest uniformly.  Returns ρ_obstacle(x, y) ∈ [0, 1].
        """
        rng = np.random.default_rng([self.seed, 2])
        r = cfg.SYN_OBSTACLE_RADIUS_FRAC
        k = max(1, int(np.ceil(self.obstacle_density / (np.pi * r ** 2)))) \
            if self.obstacle_density > 0 else 0
        k_diag = (k + 1) // 2
        centres = []
        for n in range(k):
            if n < k_diag:
                t = rng.uniform(0.15, 0.85)
                off = rng.normal(0.0, 0.05)
                centres.append((t + off / np.sqrt(2), t - off / np.sqrt(2)))
            else:
                centres.append(tuple(rng.uniform(0.05, 0.95, 2)))
        centres = np.array(centres).reshape(-1, 2)

        def rho(x: float, y: float) -> float:
            if len(centres) == 0:
                return 0.0
            d = np.hypot(centres[:, 0] - x / span, centres[:, 1] - y / span)
            return float(max(0.0, np.max(cfg.SYN_OBSTACLE_PEAK * (1.0 - d / r))))
        return rho


# ─────────────────────────────────────────────────────────────────────────────
#  Lattice builder (Thiruvanmiyur airspace grid)
# ─────────────────────────────────────────────────────────────────────────────

class LatticeAirspaceBuilder(AirspaceGraphBuilder):
    """
    Regular airspace lattice around a centre point (grid-lattice airspace, not
    the road network).

    Local equirectangular grid: node (i, j) sits at east/north offsets
    (x_m, y_m) from the centre; lat/lon follow from the sphere.  8-neighbour
    edges in both directions; the largest strongly connected component is kept.

    Terrain and height (optional):
      elevation_providers : list of ElevationProvider → ground_elev_m from the DEM
                            (None: flat, 0 m, no lookups)
      building_index      : BuildingIndex → bldg_h_m = tallest footprint touching
                            the node's cell (None: no buildings)
      alt_amsl_m = ground + max(H_CRUISE_AGL, bldg_h + CLEARANCE); a node needing
      more than H_MAX_AGL is blocked (removed).

    Node attributes : x = lon, y = lat, lat, lon, x_m, y_m, ground_elev_m,
                      bldg_h_m, agl_m, alt_amsl_m, alt_m (= alt_amsl_m)
    Edge attributes : d_h, distance (3-D), bearing, grade (Δalt/d_h), obstacle,
                      building, terrain, time_s, energy, risk, risk_components, cost
    """

    def __init__(self,
                 center: Tuple[float, float] = (cfg.CENTER_LAT, cfg.CENTER_LON),
                 box_m: float = cfg.BOX_M,
                 spacing_m: float = cfg.LATTICE_SPACING_M,
                 seed: int = 42,
                 weather: Optional[WeatherSnapshot] = None,
                 offline: bool = False,
                 wind_override: Optional[Tuple[float, float]] = None,
                 location: str = cfg.LOCATION_NAME,
                 elevation_providers: Optional[Sequence[ElevationProvider]] = None,
                 building_index: Optional[BuildingIndex] = None):
        super().__init__(location=location, dist=int(box_m / 2), seed=seed, weather=weather)
        self.center = center
        self.box_m = box_m
        self.spacing_m = spacing_m
        self.offline = offline
        self.wind_override = wind_override
        self.elevation_providers = elevation_providers
        self.building_index = building_index
        self.frame = LocalFrame(*center)
        self.build_info: dict = {}

    def resolve_weather(self) -> WeatherSnapshot:
        """Explicit snapshot, else fetch (live → cache → calm); then apply any wind override."""
        w = self.weather if self.weather is not None else \
            fetch_weather(self.center[0], self.center[1], offline=self.offline)
        if self.wind_override is not None:
            w = w.with_wind(*self.wind_override)
        return w

    def fetch_graph(self) -> nx.DiGraph:
        print(f"[Module 1] Building {self.spacing_m:.0f} m lattice over a "
              f"{self.box_m / 1000:.1f} km box at "
              f"({self.center[0]:.4f}, {self.center[1]:.4f}) — {self.location}")
        G = self._build_lattice()
        weather = self.resolve_weather()
        apply_weather(G, weather)
        print(f"[Module 1] Wind conditions: {weather.describe()}")
        G.graph.update(source="lattice", location=self.location,
                       center=self.center, box_m=self.box_m, spacing_m=self.spacing_m,
                       **self.build_info)
        self.G = G
        print(f"[Module 1] Lattice ready: {G.number_of_nodes()} nodes, "
              f"{G.number_of_edges()} edges")
        return G

    # ------------------------------------------------------------------

    def _node_environment(self, lats, lons, xs, ys):
        """Ground elevation, tallest building per cell and the resulting altitudes (arrays)."""
        if self.elevation_providers is None:
            ground, provider = np.zeros(len(xs)), "none (flat)"
        else:
            ground, provider = fetch_elevations(list(zip(lats, lons)), self.elevation_providers,
                                                offline=self.offline)
        if self.building_index is not None and len(self.building_index):
            bldg = self.building_index.max_height_in_cells(xs, ys, self.spacing_m / 2.0)
        else:
            bldg = np.zeros(len(xs))
        agl = np.maximum(cfg.H_CRUISE_AGL, bldg + cfg.CLEARANCE)
        self.build_info = {
            "elevation_provider": provider,
            "n_buildings": len(self.building_index) if self.building_index is not None else 0,
        }
        return ground, bldg, agl

    def _edge_terms(self, a: dict, b: dict, d_h_km: float, grade: float) -> Tuple[float, float]:
        """
        (ρ_building, ρ_terrain) from samples every EDGE_SAMPLE_STEP_M along the edge,
        flying the straight line between the endpoint altitudes.

          margin_k   = flight altitude − ground − tallest footprint under the sample
          terrain    = max(1 − clip(min margin / CLEARANCE), clip(|grade| / GRADE_MAX))
          building   = mean_k [ near_k · (1 − clip((flight − ground − roof near) / REF)) ]
                       where near_k = a footprint lies within BUILDING_NEAR_M of the sample
        """
        n = max(2, int(math.ceil(d_h_km * 1000.0 / cfg.EDGE_SAMPLE_STEP_M)) + 1)
        t = np.linspace(0.0, 1.0, n)
        flight = a["alt_amsl_m"] + t * (b["alt_amsl_m"] - a["alt_amsl_m"])
        ground = a["ground_elev_m"] + t * (b["ground_elev_m"] - a["ground_elev_m"])
        agl = flight - ground
        idx = self.building_index
        if idx is not None and len(idx):
            xs = a["x_m"] + t * (b["x_m"] - a["x_m"])
            ys = a["y_m"] + t * (b["y_m"] - a["y_m"])
            under = idx.tallest_under(xs, ys)
            near = idx.tallest_within(xs, ys, cfg.BUILDING_NEAR_M)
            margin = agl - under
            building = float(np.mean((near > 0) * (1.0 - np.clip((agl - near) / cfg.BUILDING_MARGIN_REF_M, 0.0, 1.0))))
        else:
            margin, building = agl, 0.0
        clearance_risk = 1.0 - float(np.clip(margin.min() / cfg.CLEARANCE, 0.0, 1.0))
        terrain = max(clearance_risk, min(1.0, abs(grade) / cfg.GRADE_MAX))
        return building, terrain

    def _build_lattice(self) -> nx.DiGraph:
        n = max(2, int(round(self.box_m / self.spacing_m)))
        half = (n - 1) / 2.0
        cols, rows = np.meshgrid(np.arange(n), np.arange(n))      # node id = r * n + c
        xs = ((cols - half) * self.spacing_m).ravel()
        ys = ((rows - half) * self.spacing_m).ravel()
        lats, lons = self.frame.to_latlon(xs, ys)
        ground, bldg, agl = self._node_environment(lats, lons, xs, ys)

        blocked = agl > cfg.H_MAX_AGL
        if blocked.any():
            print(f"[Module 1] Blocked {int(blocked.sum())} node(s): required height above "
                  f"ground exceeds {cfg.H_MAX_AGL:.0f} m")
        self.build_info["blocked_nodes"] = int(blocked.sum())

        G = nx.DiGraph()
        for k in range(n * n):
            if blocked[k]:
                continue
            alt = float(ground[k] + agl[k])
            G.add_node(k, x=float(lons[k]), y=float(lats[k]), lat=float(lats[k]), lon=float(lons[k]),
                       x_m=float(xs[k]), y_m=float(ys[k]),
                       ground_elev_m=round(float(ground[k]), 3), bldg_h_m=round(float(bldg[k]), 3),
                       agl_m=round(float(agl[k]), 3), alt_amsl_m=round(alt, 3), alt_m=round(alt, 3))

        for r in range(n):
            for c in range(n):
                u = r * n + c
                if u not in G:
                    continue
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        rr, cc = r + dr, c + dc
                        if (dr == 0 and dc == 0) or not (0 <= rr < n and 0 <= cc < n):
                            continue
                        v = rr * n + cc
                        if v not in G:
                            continue
                        a, b = G.nodes[u], G.nodes[v]
                        d_h = haversine_km(a["lat"], a["lon"], b["lat"], b["lon"])
                        brg = initial_bearing_deg(a["lat"], a["lon"], b["lat"], b["lon"])
                        dalt = b["alt_amsl_m"] - a["alt_amsl_m"]
                        grade = (dalt / 1000.0) / d_h
                        building, terrain = self._edge_terms(a, b, d_h, grade)
                        self._add_edge(G, u, v, EdgeFeatures.build(
                            d_h, brg, dalt, building=building, terrain=terrain))

        scc = max(nx.strongly_connected_components(G), key=len)
        if len(scc) < G.number_of_nodes():
            print(f"[Module 1] Keeping largest SCC: {len(scc)} of {G.number_of_nodes()} nodes")
            G = G.subgraph(scc).copy()
        return G


# ─────────────────────────────────────────────────────────────────────────────
#  Script entry-point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    os.makedirs("outputs", exist_ok=True)
    os.makedirs("pareto_front_storage", exist_ok=True)

    builder = AirspaceGraphBuilder(
        location="Thiruvanmiyur, Tamil Nadu, India",
        dist=2000,
        seed=42
    )
    G = builder.fetch_graph()
    builder.save()

    # Pick arbitrary source and target for visualisation
    nodes = list(G.nodes())
    source = nodes[0]
    target = nodes[len(nodes) // 2]

    fig = builder.visualise(source=source, target=target)
    out_path = "outputs/module1_airspace_graph.png"
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[Module 1] Visualisation saved → {out_path}")
    print("[Module 1] DONE ✓")
