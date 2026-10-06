"""
Frozen simulation region bundle: one set of map data shared by the planner
(graph, clearance checks) and the Gazebo world, identified by `bundle_hash`.

Layout (Simulation/regions/<region_id>/, written by prepare_sim_region.py):

    manifest.json        bounds, origin, frame contract, provenance, counts, hashes
    terrain.json         ElevationGrid (lat/lon grid, m AMSL) — the ONE ground surface
    buildings.geojson    footprints + height_m, height_source, base_amsl_m, roof_amsl_m
    restricted.geojson   OSM restricted zones (planner risk / hard NFZ)   [optional]
    water.geojson        OSM water                                        [optional]
    landuse.geojson      OSM land use / POIs (population exposure)        [optional]
    roads.geojson        road centre-lines (visual only)                  [optional]
    pads.json            depot + destinations with ground AMSL; h_home_amsl_m

Frame contract (see Simulation/README.md):
    Gazebo origin = depot ground, ENU metres:  xy = LocalFrame(depot).to_local(lat, lon)
    z_world = H_amsl − H_home,  H_home = depot ground AMSL from terrain.json
    mission rel-alt (MAV_FRAME_GLOBAL_RELATIVE_ALT) = H_amsl − H_home
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import numpy as np

import _paths
from env_data import BuildingIndex, ElevationGrid, LocalFrame

DATA_LAYERS = ("buildings", "restricted", "water", "landuse", "roads")


class BundleError(RuntimeError):
    """Missing, inconsistent or rejected region bundle."""


def region_dir(region_id: str, root: Optional[Path] = None) -> Path:
    return Path(root or _paths.REGIONS_DIR) / region_id


def sample_grid(grid: ElevationGrid, lats, lons) -> np.ndarray:
    """Vectorised bilinear sample of an ElevationGrid (clamped to the grid edge)."""
    lats = np.atleast_1d(np.asarray(lats, dtype=float))
    lons = np.atleast_1d(np.asarray(lons, dtype=float))
    n_i, n_j = grid.values.shape
    fi = np.clip((lats - grid.lat_south) / grid.dlat, 0.0, n_i - 1.0)
    fj = np.clip((lons - grid.lon_west) / grid.dlon, 0.0, n_j - 1.0)
    i0 = np.minimum(fi.astype(int), n_i - 2)
    j0 = np.minimum(fj.astype(int), n_j - 2)
    ti, tj = fi - i0, fj - j0
    v = grid.values
    return ((1 - ti) * (1 - tj) * v[i0, j0] + (1 - ti) * tj * v[i0, j0 + 1]
            + ti * (1 - tj) * v[i0 + 1, j0] + ti * tj * v[i0 + 1, j0 + 1])


class RegionBundle:
    """Read-only view of a prepared region bundle."""

    def __init__(self, root: Path):
        self.root = Path(root)
        mpath = self.root / "manifest.json"
        if not mpath.exists():
            raise BundleError(f"no region bundle at {self.root} "
                              f"(run: python prepare_sim_region.py --region-id {self.root.name})")
        self.manifest = json.loads(mpath.read_text(encoding="utf-8"))
        self.pads = json.loads((self.root / "pads.json").read_text(encoding="utf-8"))
        self.terrain = ElevationGrid.from_json(str(self.terrain_path))
        self._gdf_cache: dict = {}

    @classmethod
    def load(cls, region_id: str, root: Optional[Path] = None) -> "RegionBundle":
        return cls(region_dir(region_id, root))

    # ── identity / geometry ─────────────────────────────────────────────────
    @property
    def region_id(self) -> str:
        return self.manifest["region_id"]

    @property
    def bundle_hash(self) -> str:
        return self.manifest["bundle_hash"]

    @property
    def center(self) -> tuple:
        return tuple(self.manifest["center"])

    @property
    def box_m(self) -> float:
        return float(self.manifest["box_m"])

    @property
    def margin_m(self) -> float:
        return float(self.manifest["margin_m"])

    @property
    def depot(self) -> tuple:
        return (self.pads["depot"]["lat"], self.pads["depot"]["lon"])

    @property
    def destinations(self) -> list:
        return [(d["lat"], d["lon"]) for d in self.pads["destinations"]]

    @property
    def h_home(self) -> float:
        return float(self.pads["h_home_amsl_m"])

    @property
    def is_fixture(self) -> bool:
        return bool(self.manifest["data_mode"].get("fixture"))

    @property
    def planner_frame(self) -> LocalFrame:
        """Frame of the lattice graph (x_m, y_m node attributes)."""
        return LocalFrame(*self.center)

    @property
    def world_frame(self) -> LocalFrame:
        """Gazebo world frame: origin at the depot pad."""
        return LocalFrame(*self.depot)

    @property
    def terrain_path(self) -> Path:
        return self.root / "terrain.json"

    def ground(self, lat, lon) -> np.ndarray:
        return sample_grid(self.terrain, lat, lon)

    def to_world(self, lat, lon, alt_amsl=None):
        """(x, y[, z]) in the Gazebo world frame."""
        x, y = self.world_frame.to_local(lat, lon)
        if alt_amsl is None:
            return x, y
        return x, y, np.asarray(alt_amsl, dtype=float) - self.h_home

    # ── data layers ─────────────────────────────────────────────────────────
    def layer(self, name: str):
        """GeoDataFrame (EPSG:4326) of a bundle layer; empty frame when absent."""
        import geopandas as gpd
        if name not in self._gdf_cache:
            path = self.root / f"{name}.geojson"
            if path.exists():
                self._gdf_cache[name] = gpd.read_file(path)
            else:
                self._gdf_cache[name] = gpd.GeoDataFrame({"geometry": []}, geometry="geometry",
                                                         crs="EPSG:4326")
        return self._gdf_cache[name]

    def buildings(self):
        return self.layer("buildings")

    def planner_layers(self) -> dict:
        """Layers in the shape env_data.build_layer_indexes() expects."""
        return {name: self.layer(name) for name in ("buildings", "restricted", "water", "landuse")}

    def local_footprints(self, frame: LocalFrame):
        """(local polygon, row) per building, projected into `frame`."""
        import shapely
        gdf = self.buildings()
        for geom, (_, row) in zip(gdf.geometry, gdf.iterrows()):
            if geom is None or geom.is_empty:
                continue
            yield shapely.transform(
                geom, lambda xy: np.column_stack(frame.to_local(xy[:, 1], xy[:, 0]))), row

    def building_index(self, frame: Optional[LocalFrame] = None) -> BuildingIndex:
        """Planner BuildingIndex (height above ground) built from the bundle footprints."""
        frame = frame or self.planner_frame
        geoms, heights = [], []
        for geom, row in self.local_footprints(frame):
            geoms.append(geom)
            heights.append(float(row["height_m"]))
        return BuildingIndex(geoms, heights)
