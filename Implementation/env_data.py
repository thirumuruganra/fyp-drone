"""
env_data.py — environmental data for Module 1 (weather now; terrain/OSM later).

fetch_weather() returns a WeatherSnapshot.  Order of attempts:
    1. live Open-Meteo call (skipped when offline=True)   → cached on disk
    2. last cached snapshot for the same location
    3. calm conditions
so the result is deterministic whenever the network is unavailable.

Open-Meteo is free and keyless for non-commercial use, so no API key is needed.

Wind height: the UAV cruises at H_CRUISE_AGL (60 m).  Open-Meteo gives
wind_speed_80m / wind_direction_80m (used as-is, nearest to the flight band)
and wind_speed_10m.  If the 80 m row is missing, 10 m wind is lifted to the
cruise height with a power law  v_h = v_10 · (h / 10) ** ALPHA_SHEAR.
"""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass
from datetime import datetime
from typing import Callable, Optional

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
