"""
Phase 1: freeze a simulation region bundle (Simulation/regions/<region_id>/).

Reuses the planner's data path (env_data.fetch_osm_layers / fetch_osm_roads /
fetch_elevations / building_height_m, shared cache in Implementation/cache) and
writes ONE copy of terrain + buildings that both the planner graph and the
Gazebo world are built from.  See sim_region.py for the layout.

    python prepare_sim_region.py                         # default region (sim_config)
    python prepare_sim_region.py --strict                # refuse fixtures / flat ground / no buildings
    python prepare_sim_region.py --offline               # hand-made fixtures (smoke tests only)
    python prepare_sim_region.py --region-id my_area --center 12.98 80.26 --box-m 1500 \
        --depot 12.9787 80.2546 --dest 12.9872 80.2642 --dest 12.9791 80.2638
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

import _paths
import config as cfg
import sim_config as scfg
from env_data import (ElevationGrid, LocalFrame, LocalRasterProvider, OpenMeteoElevationProvider, _first_number,
                      building_height_m, default_elevation_providers, fetch_elevations,
                      fetch_osm_layers, fetch_osm_roads, load_dotenv, FIXTURE_DEM)
from sim_region import BundleError, region_dir, sample_grid
from terrain_tiles import TerrariumElevationProvider

BUNDLE_VERSION = 1


# ── helpers ──────────────────────────────────────────────────────────────────

def height_source(tags) -> str:
    """Which rule of env_data.building_height_m() produced the height."""
    h = _first_number(tags.get("height"))
    if h is not None and h > 0:
        return "tagged"
    lv = _first_number(tags.get("building:levels"))
    if lv is not None and lv > 0:
        return "levels"
    return "default"


def bbox_latlon(center, half_m: float):
    """(lat_s, lon_w, lat_n, lon_e) of a square of half-side half_m around center."""
    f = LocalFrame(*center)
    lat_s, lon_w = f.to_latlon(-half_m, -half_m)
    lat_n, lon_e = f.to_latlon(half_m, half_m)
    return float(lat_s), float(lon_w), float(lat_n), float(lon_e)


def dsm_filter(values: np.ndarray, mode: str) -> np.ndarray:
    """'p10': 10th percentile of each 3×3 window (lowers DSM roof/canopy bias)."""
    if mode in (None, "", "none"):
        return values.copy()
    if mode != "p10":
        raise ValueError(f"unknown DSM filter {mode!r}")
    p = np.pad(values, 1, mode="edge")
    n_i, n_j = values.shape
    stack = np.stack([p[di:di + n_i, dj:dj + n_j] for di in range(3) for dj in range(3)])
    return np.percentile(stack, 10, axis=0)


def fetch_terrain_grid(center, half_m: float, step_m: float, providers, offline: bool,
                       cache_dir: str):
    """Regular lat/lon DEM grid covering ±half_m (+1 cell) around center."""
    lat_s, lon_w, lat_n, lon_e = bbox_latlon(center, half_m + step_m)
    f = LocalFrame(*center)
    dlat = step_m / f.m_per_deg_lat
    dlon = step_m / f.m_per_deg_lon
    n_i = int(math.ceil((lat_n - lat_s) / dlat)) + 1
    n_j = int(math.ceil((lon_e - lon_w) / dlon)) + 1
    lats = lat_s + dlat * np.arange(n_i)
    lons = lon_w + dlon * np.arange(n_j)
    coords = [(round(float(la), 6), round(float(lo), 6)) for la in lats for lo in lons]
    z, provider = fetch_elevations(coords, providers, offline=offline, cache_dir=cache_dir)
    return lat_s, lon_w, dlat, dlon, np.asarray(z, dtype=float).reshape(n_i, n_j), provider


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    h.update(path.read_bytes())
    return h.hexdigest()


def _write_geojson(gdf, path: Path) -> bool:
    if gdf is None or len(gdf) == 0:
        return False
    gdf.to_file(path, driver="GeoJSON")
    return True


def _clip(gdf, bbox):
    """Features whose geometry intersects bbox (lat_s, lon_w, lat_n, lon_e); not cut."""
    from shapely.geometry import box
    if gdf is None or len(gdf) == 0:
        return gdf
    lat_s, lon_w, lat_n, lon_e = bbox
    keep = gdf.geometry.intersects(box(lon_w, lat_s, lon_e, lat_n))
    return gdf[keep].reset_index(drop=True)


# ── buildings ────────────────────────────────────────────────────────────────

def normalise_buildings(gdf, terrain: ElevationGrid, bbox, *,
                        sink_m: float = scfg.SIM_BUILDING_BASE_SINK_M,
                        min_area_m2: float = scfg.SIM_MIN_FOOTPRINT_M2,
                        frame: Optional[LocalFrame] = None):
    """
    Valid single polygons with height_m, height_source, base_amsl_m, roof_amsl_m.

    base = lowest terrain under the footprint (vertices + interior point) − sink_m,
    roof = base + height_m: a conservative flat LoD1 envelope over sloped ground.
    """
    import geopandas as gpd
    from shapely import make_valid
    from shapely.geometry import Polygon
    import shapely

    gdf = _clip(gdf, bbox)
    rows = []
    stats = {"input": 0 if gdf is None else len(gdf), "invalid_fixed": 0,
             "dropped_small": 0, "dropped_non_polygon": 0}
    if gdf is None or len(gdf) == 0:
        return gpd.GeoDataFrame({"geometry": []}, geometry="geometry", crs="EPSG:4326"), stats
    frame = frame or LocalFrame(*gdf.geometry.iloc[0].representative_point().coords[0][::-1])
    for bid, (geom, (_, row)) in enumerate(zip(gdf.geometry, gdf.iterrows())):
        if geom is None or geom.is_empty:
            continue
        if not geom.is_valid:
            geom = make_valid(geom)
            stats["invalid_fixed"] += 1
        parts = [g for g in getattr(geom, "geoms", [geom]) if isinstance(g, Polygon)]
        if not parts:
            stats["dropped_non_polygon"] += 1
            continue
        tags = {k: row.get(k) for k in ("building", "building:levels", "height", "name")}
        h = float(building_height_m(tags))
        src = height_source(tags)
        for k, poly in enumerate(parts):
            local = shapely.transform(poly, lambda xy: np.column_stack(frame.to_local(xy[:, 1], xy[:, 0])))
            if local.area < min_area_m2:
                stats["dropped_small"] += 1
                continue
            ext = np.asarray(poly.exterior.coords)
            ip = poly.representative_point()
            lons = np.append(ext[:, 0], ip.x)
            lats = np.append(ext[:, 1], ip.y)
            base = float(np.min(sample_grid(terrain, lats, lons))) - sink_m
            rows.append({
                "bid": f"{bid}" if len(parts) == 1 else f"{bid}.{k}",
                "building": tags.get("building"), "name": tags.get("name"),
                "levels": tags.get("building:levels"), "height_tag": tags.get("height"),
                "height_m": round(h, 2), "height_source": src,
                "base_amsl_m": round(base, 2), "roof_amsl_m": round(base + h, 2),
                "area_m2": round(float(local.area), 1), "geometry": poly,
            })
    out = gpd.GeoDataFrame(rows, geometry="geometry", crs="EPSG:4326")
    return out, stats


# ── main build ───────────────────────────────────────────────────────────────

def build_bundle(region_id: str, center: Sequence[float], box_m: float, depot: Sequence[float],
                 destinations: Sequence[Sequence[float]], *,
                 margin_m: float = scfg.SIM_MARGIN_M,
                 dem_step_m: float = scfg.SIM_DEM_FETCH_M,
                 dsm_filter_mode: str = scfg.SIM_DSM_FILTER,
                 offline: bool = False, strict: bool = False,
                 dem_path: Optional[str] = None,
                 osm_radius_m: float = cfg.OSM_FETCH_RADIUS_M,
                 cache_dir: Optional[str] = None,
                 out_root: Optional[Path] = None,
                 layers: Optional[dict] = None,
                 roads=None,
                 elevation_providers=None) -> Path:
    """
    Build and write the bundle; returns its directory.  `layers`, `roads` and
    `elevation_providers` override the data sources (tests).
    """
    center = (float(center[0]), float(center[1]))
    cache_dir = str(cache_dir or _paths.IMPL_CACHE_DIR)
    half = box_m / 2.0 + margin_m
    bbox = bbox_latlon(center, half)
    fixture = False

    # Pads must sit inside the flyable box.
    pf = LocalFrame(*center)
    for name, (la, lo) in [("depot", depot)] + [(f"dest{i}", d) for i, d in enumerate(destinations)]:
        x, y = pf.to_local(la, lo)
        if max(abs(float(x)), abs(float(y))) > box_m / 2.0:
            raise BundleError(f"{name} ({la}, {lo}) lies outside the {box_m:.0f} m box")
    if not destinations:
        raise BundleError("at least one destination is required")

    # ── terrain ──
    if elevation_providers is None:
        if offline:
            fixture_dem = _paths.FIXTURE_DIR / FIXTURE_DEM
            elevation_providers = [LocalRasterProvider(dem_path or str(fixture_dem))]
            fixture = dem_path is None
        else:
            elevation_providers = default_elevation_providers(dem_path)
    lat_s, lon_w, dlat, dlon, raw, provider = fetch_terrain_grid(
        center, half, dem_step_m, elevation_providers, offline, cache_dir)
    if provider == "flat":
        msg = "elevation fell back to FlatProvider (no DEM reachable)"
        if strict:
            raise BundleError(f"strict mode: {msg}")
        print(f"[Region] WARNING: {msg}")
    values = np.round(dsm_filter(raw, dsm_filter_mode), 3)     # exactly what terrain.json stores
    terrain = ElevationGrid(lat_s, lon_w, dlat, dlon, values)

    # ── OSM ──
    if layers is None:
        layers = fetch_osm_layers(center[0], center[1], dist_m=osm_radius_m, cache_dir=cache_dir,
                                  offline=offline,
                                  fixture_dir=str(_paths.FIXTURE_DIR) if offline else None)
        fixture = fixture or (offline and len(layers.get("buildings", [])) > 0)
    if roads is None:
        roads = fetch_osm_roads(center[0], center[1], dist_m=osm_radius_m, cache_dir=cache_dir,
                                offline=offline)
    buildings, bstats = normalise_buildings(layers.get("buildings"), terrain, bbox, frame=pf)
    if len(buildings) == 0:
        msg = "no building footprints in the region (OSM fetch failed or empty area)"
        if strict:
            raise BundleError(f"strict mode: {msg}")
        print(f"[Region] WARNING: {msg}")
    if strict and fixture:
        raise BundleError("strict mode: hand-made fixtures are not real map data")

    # ── pads ──
    def pad(lat, lon, name):
        g = float(sample_grid(terrain, [lat], [lon])[0])
        return {"name": name, "lat": float(lat), "lon": float(lon), "ground_amsl_m": g}

    depot_pad = pad(*depot, "depot")
    pads = {"depot": depot_pad,
            "destinations": [pad(la, lo, f"dest{i}") for i, (la, lo) in enumerate(destinations)],
            "h_home_amsl_m": depot_pad["ground_amsl_m"]}

    # ── write ──
    root = region_dir(region_id, out_root)
    if root.exists():
        shutil.rmtree(root)
    root.mkdir(parents=True)
    with open(root / "terrain.json", "w", encoding="utf-8") as fh:
        json.dump({"lat_south": lat_s, "lon_west": lon_w, "dlat": dlat, "dlon": dlon,
                   "values": values.tolist(),
                   "_comment": f"DEM grid for {region_id}; provider {provider}; filter {dsm_filter_mode}"}, fh)
    written = {"terrain": "terrain.json"}
    if _write_geojson(buildings, root / "buildings.geojson"):
        written["buildings"] = "buildings.geojson"
    for name in ("restricted", "water", "landuse"):
        if _write_geojson(_clip(layers.get(name), bbox), root / f"{name}.geojson"):
            written[name] = f"{name}.geojson"
    road_gdf = _clip(roads, bbox)
    if road_gdf is not None and len(road_gdf):
        road_gdf = road_gdf[road_gdf.geometry.geom_type.isin(["LineString", "MultiLineString"])]
    if _write_geojson(road_gdf, root / "roads.geojson"):
        written["roads"] = "roads.geojson"
    (root / "pads.json").write_text(json.dumps(pads, indent=2), encoding="utf-8")
    written["pads"] = "pads.json"

    settings = {
        "bundle_version": BUNDLE_VERSION, "region_id": region_id, "center": list(center),
        "box_m": box_m, "margin_m": margin_m, "dem_step_m": dem_step_m,
        "dsm_filter": dsm_filter_mode, "building_base_sink_m": scfg.SIM_BUILDING_BASE_SINK_M,
        "min_footprint_m2": scfg.SIM_MIN_FOOTPRINT_M2, "level_height_m": cfg.LEVEL_HEIGHT_M,
        "building_default_height_m": cfg.BUILDING_DEFAULT_HEIGHT_M,
    }
    files = {name: {"path": p, "sha256": _sha256(root / p)} for name, p in sorted(written.items())}
    h = hashlib.sha256(json.dumps(settings, sort_keys=True).encode())
    for name in sorted(files):
        h.update(f"{name}:{files[name]['sha256']};".encode())
    counts = buildings["height_source"].value_counts().to_dict() if len(buildings) else {}
    manifest = {
        **settings,
        "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "bundle_hash": h.hexdigest()[:16],
        "bbox_latlon": {"south": bbox[0], "west": bbox[1], "north": bbox[2], "east": bbox[3]},
        "origin": {"lat": depot_pad["lat"], "lon": depot_pad["lon"],
                   "h_home_amsl_m": pads["h_home_amsl_m"]},
        "frame_contract": {
            "world": "ENU metres, origin = depot ground; xy = LocalFrame(depot).to_local(lat, lon)",
            "z_world": "H_amsl - h_home_amsl_m",
            "mission_rel_alt": "frame 3 (GLOBAL_RELATIVE_ALT) = H_amsl - h_home_amsl_m",
            "sitl_home": "--custom-location=depot_lat,depot_lon,h_home_amsl_m,0",
        },
        "terrain": {"provider": provider, "source_step_m": dem_step_m, "filter": dsm_filter_mode,
                    "shape": list(values.shape), "min_m": float(values.min()),
                    "max_m": float(values.max()), "raw_max_m": float(raw.max()),
                    "surface_note": "Public DEM surface; filtered, approximate ground, not surveyed DTM",
                    "source": next((getattr(p, "provenance", {}) for p in elevation_providers
                                    if p.name == provider), {})},
        "buildings": {"count": int(len(buildings)), "height_source_counts": counts, **bstats},
        "layers": {name: int(len(_clip(layers.get(name), bbox)) if layers.get(name) is not None else 0)
                   for name in ("restricted", "water", "landuse")},
        "roads": int(len(road_gdf)) if road_gdf is not None else 0,
        "data_mode": {"offline": bool(offline), "fixture": bool(fixture), "strict": bool(strict)},
        "map_source": {"attribution": "© OpenStreetMap contributors",
                       "url": "https://www.openstreetmap.org/copyright",
                       "height_note": "OSM height tag, else building:levels × configured level height, else type default"},
        "files": files,
    }
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"[Region] Bundle {region_id} → {root}")
    print(f"[Region]   hash {manifest['bundle_hash']}, terrain {values.shape} from {provider} "
          f"({values.min():.1f}–{values.max():.1f} m), H_home {pads['h_home_amsl_m']:.2f} m")
    print(f"[Region]   buildings {len(buildings)} {counts}, roads {manifest['roads']}")
    if fixture:
        print("[Region]   NOTE: built from HAND-MADE FIXTURES — smoke tests only, not real data")
    return root


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--region-id", default=scfg.SIM_REGION_ID)
    p.add_argument("--center", nargs=2, type=float, metavar=("LAT", "LON"), default=scfg.SIM_CENTER)
    p.add_argument("--box-m", type=float, default=scfg.SIM_BOX_M)
    p.add_argument("--margin-m", type=float, default=scfg.SIM_MARGIN_M)
    p.add_argument("--depot", nargs=2, type=float, metavar=("LAT", "LON"), default=scfg.SIM_DEPOT)
    p.add_argument("--dest", nargs=2, type=float, action="append", metavar=("LAT", "LON"),
                   help="delivery pad (repeat); default: sim_config.SIM_DESTINATIONS")
    p.add_argument("--dem", default=None, help="local DEM (GeoTIFF or ElevationGrid JSON)")
    p.add_argument("--dem-step-m", type=float, default=scfg.SIM_DEM_FETCH_M)
    p.add_argument("--osm-radius-m", type=float, default=cfg.OSM_FETCH_RADIUS_M)
    p.add_argument("--dsm-filter", choices=["p10", "none"], default=scfg.SIM_DSM_FILTER)
    p.add_argument("--offline", action="store_true", help="hand-made fixtures (smoke tests only)")
    p.add_argument("--keyless", action="store_true",
                   help="use public AWS terrain tiles / Open-Meteo only; do not load .env credentials/endpoints")
    p.add_argument("--strict", action="store_true",
                   help="refuse fixtures, flat-ground fallback and empty building layers")
    args = p.parse_args(argv)
    if args.keyless and (args.offline or args.dem):
        p.error("--keyless cannot be combined with --offline or --dem")
    if not args.keyless:
        load_dotenv()                               # OPENTOPO_API_KEY etc. from <repo>/.env
    try:
        build_bundle(args.region_id, args.center, args.box_m, args.depot,
                     args.dest or scfg.SIM_DESTINATIONS, margin_m=args.margin_m,
                     dem_step_m=args.dem_step_m, dsm_filter_mode=args.dsm_filter,
                     offline=args.offline, strict=args.strict, dem_path=args.dem,
                     osm_radius_m=args.osm_radius_m,
                     elevation_providers=[TerrariumElevationProvider(), OpenMeteoElevationProvider()]
                     if args.keyless else None)
    except BundleError as exc:
        raise SystemExit(f"[Region] ERROR: {exc}")


if __name__ == "__main__":
    main()
