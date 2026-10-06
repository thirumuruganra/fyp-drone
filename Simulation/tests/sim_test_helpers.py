"""Synthetic region bundles for the simulation tests (no network, no fixtures)."""

import io
import sys
from contextlib import redirect_stdout
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))

import _paths  # noqa: E402,F401
import geopandas as gpd  # noqa: E402
from shapely.geometry import box as sbox  # noqa: E402

from env_data import ElevationProvider, LocalFrame  # noqa: E402
import prepare_sim_region  # noqa: E402

CENTER = (12.9830, 80.2594)


class FnProvider(ElevationProvider):
    """Elevation from a function of local (x, y) metres around CENTER."""
    name = "test-dem"

    def __init__(self, fn, center=CENTER):
        self.fn, self.frame = fn, LocalFrame(*center)

    def fetch(self, coords):
        out = []
        for lat, lon in coords:
            x, y = self.frame.to_local(lat, lon)
            out.append(float(self.fn(float(x), float(y))))
        return out


def latlon(x, y, center=CENTER):
    lat, lon = LocalFrame(*center).to_latlon(x, y)
    return float(lat), float(lon)


def buildings_gdf(specs, center=CENTER):
    """specs: [(xmin, ymin, xmax, ymax, tags dict)] in local metres around center."""
    rows = []
    for x0, y0, x1, y1, tags in specs:
        (la0, lo0), (la1, lo1) = latlon(x0, y0, center), latlon(x1, y1, center)
        rows.append({"building": tags.get("building", "yes"), "building:levels": tags.get("building:levels"),
                     "height": tags.get("height"), "name": tags.get("name"),
                     "geometry": sbox(lo0, la0, lo1, la1)})
    if not rows:
        return gpd.GeoDataFrame({"geometry": []}, geometry="geometry", crs="EPSG:4326")
    return gpd.GeoDataFrame(rows, geometry="geometry", crs="EPSG:4326")


def make_bundle(root, region_id="t", *, buildings=(), terrain=lambda x, y: 5.0, box_m=400.0,
                depot_xy=(-150.0, -150.0), dests_xy=((150.0, 150.0),), margin_m=60.0,
                dem_step_m=20.0, dsm_filter="none", strict=False, provider=None):
    layers = {"buildings": buildings_gdf(buildings)}
    empty = gpd.GeoDataFrame({"highway": [], "geometry": []}, geometry="geometry", crs="EPSG:4326")
    with redirect_stdout(io.StringIO()):
        return prepare_sim_region.build_bundle(
            region_id, CENTER, box_m, latlon(*depot_xy), [latlon(*d) for d in dests_xy],
            margin_m=margin_m, dem_step_m=dem_step_m, dsm_filter_mode=dsm_filter, strict=strict,
            out_root=Path(root), layers=layers, roads=empty,
            elevation_providers=[provider or FnProvider(terrain)])
