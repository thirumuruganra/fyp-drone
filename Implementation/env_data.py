"""
env_data.py — environmental data for Module 1: weather, terrain, OSM layers.

fetch_weather() returns a WeatherSnapshot.  Order of attempts:
    1. live Open-Meteo call (skipped when offline=True)   → cached on disk
    2. last cached snapshot for the same location
    3. calm conditions
so the result is deterministic whenever the network is unavailable.

Open-Meteo is free and keyless for non-commercial use, so no API key is needed.

Terrain (Phase 2): ElevationProvider implementations, tried in this order by
fetch_elevations(): LocalRasterProvider (GeoTIFF or JSON grid) →
OpenTopographyProvider (needs OPENTOPO_API_KEY) → OpenMeteoElevationProvider
(Copernicus 90 m, keyless) → FlatProvider (0 m, warns).  Network providers cache
node elevations in cache/elevation_<provider>.json keyed by rounded coordinates.

OSM layers (Phase 2): fetch_osm_layers() returns buildings, restricted zones,
water and land use/POIs from osmnx, cached as a GeoPackage.  BuildingIndex wraps
the building footprints (STRtree) for node-cell and along-edge height queries.

Wind height: the UAV cruises at H_CRUISE_AGL (60 m).  Open-Meteo gives
wind_speed_80m / wind_direction_80m (used as-is, nearest to the flight band)
and wind_speed_10m.  If the 80 m row is missing, 10 m wind is lifted to the
cruise height with a power law  v_h = v_10 · (h / 10) ** ALPHA_SHEAR.
"""

from __future__ import annotations

import json
import math
import os
import re
from dataclasses import asdict, dataclass
from datetime import datetime
from typing import Callable, Mapping, Optional, Sequence, Tuple

import numpy as np

import config as cfg

OPEN_METEO_URL = "https://api.open-meteo.com/v1/forecast"
OPEN_METEO_TZ = "Asia/Kolkata"
_CURRENT_VARS = ("wind_speed_10m,wind_direction_10m,wind_gusts_10m,"
                 "precipitation,weather_code,visibility")
_HOURLY_VARS = "wind_speed_80m,wind_direction_80m"


@dataclass
class WeatherSnapshot:
    """Conditions at flight height, spatially uniform over the graph."""
    wind_speed_kmh: float = 0.0          # mean wind at flight height
    wind_dir_from_deg: float = 0.0       # meteorological: direction wind blows FROM
    gust_kmh: float = 0.0                # gust at flight height (≥ wind_speed_kmh)
    precipitation_mm: float = 0.0
    visibility_m: float = cfg.VIS_CLEAR_M
    weather_code: int = 0                # WMO code
    time: str = ""                       # local (IST) ISO time of the observation
    source: str = "calm"                 # open-meteo:80m | open-meteo:10m-power-law |
                                         # cache | override | synthetic | calm
    wind_height_m: float = cfg.H_CRUISE_AGL
    latitude: float = cfg.CENTER_LAT
    longitude: float = cfg.CENTER_LON
    fetched_at: str = ""

    def __post_init__(self):
        self.wind_dir_from_deg = float(self.wind_dir_from_deg) % 360.0
        self.gust_kmh = max(float(self.gust_kmh), float(self.wind_speed_kmh))

    @classmethod
    def calm(cls, lat: float = cfg.CENTER_LAT, lon: float = cfg.CENTER_LON):
        return cls(latitude=lat, longitude=lon)

    def with_wind(self, speed_kmh: float, dir_from_deg: float) -> "WeatherSnapshot":
        """Copy with the wind replaced (gust = mean, since an override has no gust info)."""
        data = asdict(self)
        data.update(wind_speed_kmh=float(speed_kmh),
                    wind_dir_from_deg=float(dir_from_deg),
                    gust_kmh=float(speed_kmh), source="override")
        return WeatherSnapshot(**data)

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> "WeatherSnapshot":
        known = {f for f in cls.__dataclass_fields__}
        return cls(**{k: v for k, v in data.items() if k in known})

    def describe(self) -> str:
        return (f"{self.wind_speed_kmh:.1f} km/h from {self.wind_dir_from_deg:.0f}° "
                f"(gust {self.gust_kmh:.1f}) @ {self.wind_height_m:.0f} m  "
                f"[{self.source}]")


def power_law_wind(v_ref: float, h_ref: float, h: float,
                   alpha: float = cfg.ALPHA_SHEAR) -> float:
    """Wind-speed shear between two heights."""
    return float(v_ref) * (float(h) / float(h_ref)) ** alpha


def seeded_weather(seed: int) -> WeatherSnapshot:
    """Deterministic wind for synthetic scenarios (no network, no wall clock)."""
    import numpy as np
    rng = np.random.default_rng(seed)
    lo, hi = cfg.SYN_WIND_SPEED_RANGE_KMH
    speed = float(rng.uniform(lo, hi))
    direction = float(rng.uniform(0.0, 360.0))
    return WeatherSnapshot(wind_speed_kmh=round(speed, 2),
                           wind_dir_from_deg=round(direction, 1),
                           gust_kmh=round(speed * cfg.SYN_GUST_FACTOR, 2),
                           source="synthetic",
                           latitude=cfg.SYN_ORIGIN_LAT, longitude=cfg.SYN_ORIGIN_LON)


# ─────────────────────────────────────────────────────────────────────────────
#  Open-Meteo parsing
# ─────────────────────────────────────────────────────────────────────────────

def parse_open_meteo(payload: dict,
                     lat: float = cfg.CENTER_LAT,
                     lon: float = cfg.CENTER_LON,
                     cruise_h: float = cfg.H_CRUISE_AGL) -> WeatherSnapshot:
    """
    Turn an Open-Meteo /v1/forecast response (timezone=Asia/Kolkata) into a
    WeatherSnapshot.

    The hourly row is chosen by matching the response's own `current.time`
    string (floored to the hour) against `hourly.time`; both are IST, so no
    UTC/IST conversion is involved.
    """
    cur = payload["current"]
    v10 = float(cur.get("wind_speed_10m") or 0.0)
    d10 = float(cur.get("wind_direction_10m") or 0.0)
    g10 = float(cur.get("wind_gusts_10m") or v10)
    when = str(cur.get("time", ""))

    speed, direction, height, source = None, d10, cruise_h, "open-meteo:10m-power-law"
    hourly = payload.get("hourly") or {}
    hour_key = when[:13] + ":00" if len(when) >= 13 else ""
    times = hourly.get("time") or []
    if hour_key in times:
        i = times.index(hour_key)
        v80 = (hourly.get("wind_speed_80m") or [None] * len(times))[i]
        d80 = (hourly.get("wind_direction_80m") or [None] * len(times))[i]
        if v80 is not None and d80 is not None:
            speed, direction = float(v80), float(d80)
            height, source = cfg.WIND_HEIGHT_80M, "open-meteo:80m"
    if speed is None:
        speed = power_law_wind(v10, cfg.WIND_REF_HEIGHT_M, cruise_h)

    # Gust factor measured at 10 m, applied to the wind at flight height.
    gust = max(speed, speed * g10 / v10 if v10 > 0.5 else g10)

    vis = cur.get("visibility")
    return WeatherSnapshot(
        wind_speed_kmh=round(speed, 3),
        wind_dir_from_deg=direction,
        gust_kmh=round(gust, 3),
        precipitation_mm=float(cur.get("precipitation") or 0.0),
        visibility_m=float(vis) if vis is not None else cfg.VIS_CLEAR_M,
        weather_code=int(cur.get("weather_code") or 0),
        time=when, source=source, wind_height_m=height,
        latitude=lat, longitude=lon,
    )


# ─────────────────────────────────────────────────────────────────────────────
#  fetch_weather
# ─────────────────────────────────────────────────────────────────────────────

def _cache_path(lat: float, lon: float, cache_dir: str) -> str:
    return os.path.join(cache_dir, f"weather_{lat:.3f}_{lon:.3f}.json")


def _read_cache(path: str) -> Optional[WeatherSnapshot]:
    try:
        with open(path, "r", encoding="utf-8") as fh:
            return WeatherSnapshot.from_dict(json.load(fh))
    except (OSError, ValueError, TypeError):
        return None


def fetch_weather(lat: float = cfg.CENTER_LAT,
                  lon: float = cfg.CENTER_LON,
                  offline: bool = False,
                  cache_dir: str = cfg.CACHE_DIR,
                  timeout: float = 10.0,
                  http_get: Optional[Callable] = None) -> WeatherSnapshot:
    """
    Current weather at (lat, lon).  Never raises: falls back to the cached
    snapshot, then to calm conditions.

    http_get : callable(url, params=..., timeout=...) → response; defaults to
               requests.get.  Injected by tests.
    """
    path = _cache_path(lat, lon, cache_dir)

    if not offline:
        try:
            if http_get is None:
                import requests
                http_get = requests.get
            resp = http_get(OPEN_METEO_URL, params={
                "latitude": lat, "longitude": lon,
                "current": _CURRENT_VARS, "hourly": _HOURLY_VARS,
                "wind_speed_unit": "kmh", "forecast_days": 1,
                "timezone": OPEN_METEO_TZ,
            }, timeout=timeout)
            resp.raise_for_status()
            snap = parse_open_meteo(resp.json(), lat, lon)
            snap.fetched_at = datetime.now().isoformat(timespec="seconds")
            os.makedirs(cache_dir, exist_ok=True)
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(snap.to_dict(), fh, indent=2)
            print(f"[Weather] Live Open-Meteo: {snap.describe()}")
            return snap
        except Exception as exc:                      # network, HTTP, JSON, schema
            print(f"[Weather] Live fetch failed ({type(exc).__name__}: {exc})")

    cached = _read_cache(path)
    if cached is not None:
        cached.source = f"cache({cached.source})" if not cached.source.startswith("cache") \
            else cached.source
        print(f"[Weather] Using cached snapshot from {cached.fetched_at or cached.time}: "
              f"{cached.describe()}")
        return cached

    print("[Weather] No cached snapshot — using calm conditions (0 km/h)")
    return WeatherSnapshot.calm(lat, lon)


# ═════════════════════════════════════════════════════════════════════════════
#  Local metric frame (shared by the lattice builder and the building index)
# ═════════════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class LocalFrame:
    """Local equirectangular frame: x east, y north, metres from (lat0, lon0)."""
    lat0: float
    lon0: float

    @property
    def m_per_deg_lat(self) -> float:
        return math.radians(1.0) * cfg.EARTH_RADIUS_KM * 1000.0

    @property
    def m_per_deg_lon(self) -> float:
        return self.m_per_deg_lat * math.cos(math.radians(self.lat0))

    def to_local(self, lat, lon):
        return ((np.asarray(lon) - self.lon0) * self.m_per_deg_lon,
                (np.asarray(lat) - self.lat0) * self.m_per_deg_lat)

    def to_latlon(self, x_m, y_m):
        return (self.lat0 + np.asarray(y_m) / self.m_per_deg_lat,
                self.lon0 + np.asarray(x_m) / self.m_per_deg_lon)


# ═════════════════════════════════════════════════════════════════════════════
#  Elevation providers
# ═════════════════════════════════════════════════════════════════════════════

class ElevationUnavailable(Exception):
    """A provider cannot serve this request (no key, no network, outside coverage)."""


def _elev_key(lat: float, lon: float) -> str:
    d = cfg.ELEVATION_ROUND_DEG
    return f"{round(float(lat), d):.{d}f},{round(float(lon), d):.{d}f}"


class ElevationGrid:
    """Regular lat/lon grid of ground elevations with bilinear sampling.

    values[i][j]: row i is the i-th latitude from the south, column j the j-th
    longitude from the west; lat_south/lon_west are cell-centre coordinates.
    """

    def __init__(self, lat_south: float, lon_west: float, dlat: float, dlon: float, values):
        self.lat_south, self.lon_west = float(lat_south), float(lon_west)
        self.dlat, self.dlon = float(dlat), float(dlon)
        self.values = np.asarray(values, dtype=float)
        if self.values.ndim != 2 or min(self.values.shape) < 2:
            raise ValueError("elevation grid must be at least 2 × 2")

    def sample(self, lat: float, lon: float) -> float:
        n_i, n_j = self.values.shape
        fi = (lat - self.lat_south) / self.dlat
        fj = (lon - self.lon_west) / self.dlon
        if not (-1.0 <= fi <= n_i and -1.0 <= fj <= n_j):
            raise ElevationUnavailable(f"({lat:.5f}, {lon:.5f}) is outside the DEM")
        fi, fj = min(max(fi, 0.0), n_i - 1.0), min(max(fj, 0.0), n_j - 1.0)
        i0, j0 = min(int(fi), n_i - 2), min(int(fj), n_j - 2)
        ti, tj = fi - i0, fj - j0
        v = self.values
        out = ((1 - ti) * (1 - tj) * v[i0, j0] + (1 - ti) * tj * v[i0, j0 + 1]
               + ti * (1 - tj) * v[i0 + 1, j0] + ti * tj * v[i0 + 1, j0 + 1])
        if not np.isfinite(out):
            raise ElevationUnavailable("DEM has no data at this point")
        return float(out)

    @classmethod
    def from_json(cls, path: str) -> "ElevationGrid":
        with open(path, "r", encoding="utf-8") as fh:
            d = json.load(fh)
        return cls(d["lat_south"], d["lon_west"], d["dlat"], d["dlon"], d["values"])

    @classmethod
    def from_aaigrid(cls, text: str) -> "ElevationGrid":
        """Parse an ESRI ASCII grid (OpenTopography outputFormat=AAIGrid)."""
        lines = text.strip().splitlines()
        head, rows, i = {}, [], 0
        while i < len(lines) and re.match(r"^[A-Za-z_]+\s", lines[i]):
            k, v = lines[i].split(None, 1)
            head[k.lower()] = float(v)
            i += 1
        for ln in lines[i:]:
            rows.append([float(x) for x in ln.split()])
        nodata = head.get("nodata_value")
        arr = np.array(rows, dtype=float)
        if nodata is not None:
            arr[arr == nodata] = np.nan
        cell = head["cellsize"]
        lat_c = head["yllcorner"] + cell / 2.0       # southernmost row centre
        lon_c = head["xllcorner"] + cell / 2.0
        return cls(lat_c, lon_c, cell, cell, arr[::-1])     # file lists rows north → south


class ElevationProvider:
    """Ground elevation (m AMSL) for a list of (lat, lon).  Network providers cache on disk."""
    name = "base"
    cacheable = False

    def fetch(self, coords: Sequence[Tuple[float, float]]) -> list:
        raise NotImplementedError

    def elevations(self, coords: Sequence[Tuple[float, float]], offline: bool = False,
                   cache_dir: str = cfg.CACHE_DIR) -> list:
        if not self.cacheable:
            if offline and getattr(self, "needs_network", False):
                raise ElevationUnavailable(f"{self.name} needs the network (offline)")
            return self.fetch(coords)
        path = os.path.join(cache_dir, f"elevation_{self.name}.json")
        cache: dict = {}
        try:
            with open(path, "r", encoding="utf-8") as fh:
                cache = json.load(fh)
        except (OSError, ValueError):
            pass
        missing = [c for c in coords if _elev_key(*c) not in cache]
        if missing:
            if offline:
                raise ElevationUnavailable(f"{self.name}: {len(missing)} coordinates not cached (offline)")
            got = self.fetch(missing)
            for c, z in zip(missing, got):
                cache[_elev_key(*c)] = float(z)
            os.makedirs(cache_dir, exist_ok=True)
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(cache, fh)
        return [cache[_elev_key(*c)] for c in coords]


class FlatProvider(ElevationProvider):
    """Last resort: 0 m everywhere."""
    name = "flat"

    def fetch(self, coords):
        print("[Terrain] WARNING: no DEM available — assuming flat ground (0 m)")
        return [0.0] * len(coords)


class LocalRasterProvider(ElevationProvider):
    """Local DEM: a GeoTIFF (SRTM / Copernicus, needs rasterio) or a JSON grid (ElevationGrid)."""
    name = "local-raster"

    def __init__(self, path: Optional[str]):
        self.path = path

    def fetch(self, coords):
        if not self.path or not os.path.exists(self.path):
            raise ElevationUnavailable(f"no local DEM at {self.path!r}")
        if self.path.lower().endswith(".json"):
            grid = ElevationGrid.from_json(self.path)
            return [grid.sample(lat, lon) for lat, lon in coords]
        try:
            import rasterio
        except ImportError:
            raise ElevationUnavailable("rasterio is not installed (pip install rasterio)") from None
        with rasterio.open(self.path) as ds:
            vals = [float(v[0]) for v in ds.sample([(lon, lat) for lat, lon in coords])]
            nodata = ds.nodata
        if any(not np.isfinite(v) or (nodata is not None and v == nodata) for v in vals):
            raise ElevationUnavailable("DEM raster has no data at some nodes")
        return vals


class OpenTopographyProvider(ElevationProvider):
    """OpenTopography global DEM REST API.  Free API key in env OPENTOPO_API_KEY."""
    name = "opentopography"
    cacheable = True
    needs_network = True
    URL = "https://portal.opentopography.org/API/globaldem"

    def __init__(self, api_key: Optional[str] = None, demtype: str = cfg.DEM_PROVIDER_TYPE,
                 http_get: Optional[Callable] = None, timeout: float = 60.0):
        self.api_key = api_key if api_key is not None else os.environ.get(cfg.OPENTOPO_ENV_KEY)
        self.demtype, self.http_get, self.timeout = demtype, http_get, timeout

    def fetch(self, coords):
        if not self.api_key:
            raise ElevationUnavailable(f"no API key (set {cfg.OPENTOPO_ENV_KEY})")
        get = self.http_get
        if get is None:
            import requests
            get = requests.get
        lats = [c[0] for c in coords]
        lons = [c[1] for c in coords]
        pad = 0.005
        resp = get(self.URL, params={
            "demtype": self.demtype, "outputFormat": "AAIGrid", "API_Key": self.api_key,
            "south": min(lats) - pad, "north": max(lats) + pad,
            "west": min(lons) - pad, "east": max(lons) + pad,
        }, timeout=self.timeout)
        resp.raise_for_status()
        grid = ElevationGrid.from_aaigrid(resp.text)
        return [grid.sample(lat, lon) for lat, lon in coords]


class OpenMeteoElevationProvider(ElevationProvider):
    """Open-Meteo elevation API (Copernicus 90 m), no key, up to 100 coordinates per request."""
    name = "open-meteo"
    cacheable = True
    needs_network = True
    URL = "https://api.open-meteo.com/v1/elevation"

    def __init__(self, http_get: Optional[Callable] = None, timeout: float = 15.0):
        self.http_get, self.timeout = http_get, timeout

    def fetch(self, coords):
        get = self.http_get
        if get is None:
            import requests
            get = requests.get
        out: list = []
        for k in range(0, len(coords), cfg.ELEVATION_BATCH):
            batch = coords[k:k + cfg.ELEVATION_BATCH]
            resp = get(self.URL, params={
                "latitude": ",".join(f"{c[0]:.5f}" for c in batch),
                "longitude": ",".join(f"{c[1]:.5f}" for c in batch),
            }, timeout=self.timeout)
            resp.raise_for_status()
            z = resp.json()["elevation"]
            if len(z) != len(batch):
                raise ElevationUnavailable(f"expected {len(batch)} elevations, got {len(z)}")
            out.extend(float(v) for v in z)
        return out


def default_elevation_providers(raster_path: Optional[str] = None) -> list:
    """LocalRaster (explicit path or $DEM_RASTER_PATH) → OpenTopography → Open-Meteo."""
    path = raster_path or os.environ.get(cfg.DEM_RASTER_ENV)
    providers: list = [LocalRasterProvider(path)] if path else []
    return providers + [OpenTopographyProvider(), OpenMeteoElevationProvider()]


def fetch_elevations(coords: Sequence[Tuple[float, float]],
                     providers: Optional[Sequence[ElevationProvider]] = None,
                     offline: bool = False,
                     cache_dir: str = cfg.CACHE_DIR) -> Tuple[np.ndarray, str]:
    """
    Ground elevation for every (lat, lon), from the first provider that can serve
    all of them; FlatProvider (with a warning) if none can.  Returns (array, provider name).
    """
    providers = list(providers) if providers is not None else default_elevation_providers()
    for p in providers:
        try:
            z = p.elevations(coords, offline=offline, cache_dir=cache_dir)
            print(f"[Terrain] Elevation from {p.name}: {len(z)} points, "
                  f"{min(z):.1f}–{max(z):.1f} m")
            return np.asarray(z, dtype=float), p.name
        except Exception as exc:           # ElevationUnavailable, network, JSON, schema
            print(f"[Terrain] {p.name} unavailable ({type(exc).__name__}: {exc})")
    flat = FlatProvider()
    return np.asarray(flat.fetch(coords), dtype=float), flat.name


# ═════════════════════════════════════════════════════════════════════════════
#  OSM layers and building heights
# ═════════════════════════════════════════════════════════════════════════════

OSM_LAYER_TAGS = {
    "buildings": {"building": True},
    "restricted": {"aeroway": True, "military": True, "landuse": ["military"],
                   "amenity": ["prison"], "boundary": ["protected_area"],
                   "leisure": ["nature_reserve"], "office": ["government"]},
    "water": {"natural": ["water", "bay", "coastline"], "waterway": True},
    "landuse": {"landuse": ["residential", "commercial", "retail", "industrial"],
                "amenity": ["school", "hospital", "marketplace", "college", "university"],
                "highway": ["motorway", "trunk", "primary"]},
}
_OSM_KEEP = ["building", "building:levels", "height", "name", "aeroway", "military",
             "landuse", "amenity", "boundary", "leisure", "office", "natural",
             "waterway", "highway"]
FIXTURE_BUILDINGS = "osm_buildings_thiruvanmiyur.geojson"
FIXTURE_DEM = "dem_thiruvanmiyur.json"

_NUM = re.compile(r"[-+]?\d+(?:[.,]\d+)?")


def _first_number(value) -> Optional[float]:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return None
    m = _NUM.search(str(value))
    if not m:
        return None
    x = float(m.group().replace(",", "."))
    return x * 0.3048 if re.search(r"ft|'", str(value)) else x


def building_height_m(tags: Mapping) -> float:
    """
    Building height in metres: height=* → building:levels × 3 m → per-type default
    (residential 9, commercial 15, other 10).  The default is a documented
    assumption, because Chennai's OSM height coverage is sparse.
    """
    h = _first_number(tags.get("height"))
    if h is not None and h > 0:
        return h
    levels = _first_number(tags.get("building:levels"))
    if levels is not None and levels > 0:
        return levels * cfg.LEVEL_HEIGHT_M
    btype = str(tags.get("building") or "").lower()
    if btype in cfg.BUILDING_RESIDENTIAL_TYPES:
        return cfg.BUILDING_DEFAULT_HEIGHT_M["residential"]
    if btype in cfg.BUILDING_COMMERCIAL_TYPES:
        return cfg.BUILDING_DEFAULT_HEIGHT_M["commercial"]
    return cfg.BUILDING_DEFAULT_HEIGHT_M["other"]


class BuildingIndex:
    """Building footprints (shapely geometries in local metres) with heights, in an STRtree."""

    def __init__(self, geoms: Sequence, heights: Sequence[float]):
        from shapely import STRtree
        self.geoms = list(geoms)
        self.heights = np.asarray(heights, dtype=float)
        self.tree = STRtree(self.geoms) if self.geoms else None

    def __len__(self) -> int:
        return len(self.geoms)

    @classmethod
    def empty(cls) -> "BuildingIndex":
        return cls([], [])

    @classmethod
    def from_gdf(cls, gdf, frame: LocalFrame) -> "BuildingIndex":
        """Polygon buildings of an EPSG:4326 GeoDataFrame, projected into `frame`."""
        import shapely
        geoms, heights = [], []
        if gdf is None or len(gdf) == 0:
            return cls.empty()
        for geom, (_, row) in zip(gdf.geometry, gdf.iterrows()):
            if geom is None or geom.is_empty or geom.geom_type not in ("Polygon", "MultiPolygon"):
                continue
            local = shapely.transform(geom, lambda xy: np.column_stack(frame.to_local(xy[:, 1], xy[:, 0])))
            geoms.append(local)
            heights.append(building_height_m(row))
        return cls(geoms, heights)

    def _reduce(self, queries, predicate: str, distance: Optional[float] = None) -> np.ndarray:
        out = np.zeros(len(queries))
        if self.tree is None or len(queries) == 0:
            return out
        kw = {"distance": distance} if distance is not None else {}
        q_idx, t_idx = self.tree.query(queries, predicate=predicate, **kw)
        np.maximum.at(out, q_idx, self.heights[t_idx])
        return out

    def max_height_in_cells(self, xs, ys, half_m: float) -> np.ndarray:
        """Tallest building touching the square cell of half-side half_m around each point (0 if none)."""
        import shapely
        xs, ys = np.asarray(xs, float), np.asarray(ys, float)
        return self._reduce(shapely.box(xs - half_m, ys - half_m, xs + half_m, ys + half_m), "intersects")

    def tallest_under(self, xs, ys) -> np.ndarray:
        """Height of the tallest footprint containing each point (0 if none)."""
        import shapely
        return self._reduce(shapely.points(np.asarray(xs, float), np.asarray(ys, float)), "intersects")

    def tallest_within(self, xs, ys, radius_m: float) -> np.ndarray:
        """Height of the tallest footprint within radius_m of each point (0 if none)."""
        import shapely
        return self._reduce(shapely.points(np.asarray(xs, float), np.asarray(ys, float)),
                            "dwithin", distance=radius_m)


def _empty_layer():
    import geopandas as gpd
    return gpd.GeoDataFrame({c: [] for c in _OSM_KEEP}, geometry=[], crs="EPSG:4326")


def _clean_layer(gdf):
    """Keep geometry + the tag columns we use; flatten the (element, id) index."""
    gdf = gdf.reset_index()
    cols = [c for c in _OSM_KEEP if c in gdf.columns]
    for c in cols:
        gdf[c] = gdf[c].map(lambda v: None if v is None or (isinstance(v, float) and math.isnan(v)) else str(v))
    return gdf[cols + ["geometry"]]


def fetch_osm_layers(lat: float = cfg.CENTER_LAT, lon: float = cfg.CENTER_LON,
                     dist_m: float = cfg.OSM_FETCH_RADIUS_M,
                     cache_dir: str = cfg.CACHE_DIR,
                     offline: bool = False,
                     fixture_dir: Optional[str] = None,
                     fetcher: Optional[Callable] = None) -> dict:
    """
    OSM layers around (lat, lon): {"buildings", "restricted", "water", "landuse"}
    as EPSG:4326 GeoDataFrames, cached as one GeoPackage in cache_dir.

    Order: cached GeoPackage → live osmnx.features_from_point (skipped when
    offline) → hand-made fixture buildings from fixture_dir (only if given;
    announced loudly) → empty layers.

    fetcher(lat, lon, dist_m, tags) → GeoDataFrame replaces osmnx (tests).
    """
    import geopandas as gpd
    path = os.path.join(cache_dir, f"osm_{lat:.4f}_{lon:.4f}_{int(dist_m)}.gpkg")

    if os.path.exists(path):
        layers = {}
        for name in OSM_LAYER_TAGS:
            try:
                layers[name] = gpd.read_file(path, layer=name)
            except Exception:
                layers[name] = _empty_layer()
        print(f"[OSM] Cached layers {path}: " + ", ".join(f"{k}={len(v)}" for k, v in layers.items()))
        return layers

    if not offline:
        try:
            if fetcher is None:
                import osmnx as ox
                fetcher = lambda la, lo, d, tags: ox.features_from_point((la, lo), tags=tags, dist=d)
            layers = {}
            for name, tags in OSM_LAYER_TAGS.items():
                try:
                    layers[name] = _clean_layer(fetcher(lat, lon, dist_m, tags))
                except Exception as exc:
                    if type(exc).__name__ == "InsufficientResponseError":
                        layers[name] = _empty_layer()          # no features of this kind
                    else:
                        raise
            os.makedirs(cache_dir, exist_ok=True)
            for name, gdf in layers.items():
                if len(gdf):
                    gdf.to_file(path, layer=name, driver="GPKG")
            print(f"[OSM] Live osmnx: " + ", ".join(f"{k}={len(v)}" for k, v in layers.items()))
            return layers
        except Exception as exc:
            print(f"[OSM] Live fetch failed ({type(exc).__name__}: {exc})")

    layers = {name: _empty_layer() for name in OSM_LAYER_TAGS}
    fixture = os.path.join(fixture_dir, FIXTURE_BUILDINGS) if fixture_dir else None
    if fixture and os.path.exists(fixture):
        layers["buildings"] = gpd.read_file(fixture)
        print(f"[OSM] USING HAND-MADE FIXTURE buildings ({len(layers['buildings'])}) from {fixture} "
              f"— not real OSM data")
    else:
        print("[OSM] No cached layers and no network — continuing without OSM layers")
    return layers
