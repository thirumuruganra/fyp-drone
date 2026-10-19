"""Keyless, cached AWS Terrain Tiles DEM (Terrarium RGB encodes metres).

Source/encoding: https://github.com/tilezen/joerd/tree/master/docs
Public data only; no API keys or .env access. Tiles use Web Mercator; requested
WGS84 points are sampled bilinearly across tile boundaries.
"""
from __future__ import annotations

import io
import math
from pathlib import Path

import numpy as np

import _paths
from env_data import ElevationProvider

TILE_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"


def decode_terrarium(payload: bytes) -> np.ndarray:
    from PIL import Image
    with Image.open(io.BytesIO(payload)) as image:
        if image.size != (256, 256):
            raise ValueError(f"expected 256×256 DEM tile, received {image.size}")
        rgb = np.asarray(image.convert("RGB"), dtype=np.float64)
    return rgb[:, :, 0] * 256.0 + rgb[:, :, 1] + rgb[:, :, 2] / 256.0 - 32768.0


def global_pixel(lat: float, lon: float, zoom: int):
    if not (-85.05112878 <= lat <= 85.05112878 and -180 <= lon <= 180):
        raise ValueError("coordinate outside Web Mercator coverage")
    size = 256 * (2 ** zoom)
    return ((lon + 180.0) / 360.0 * size - 0.5,
            (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) * size / 2.0 - 0.5)


class TerrariumElevationProvider(ElevationProvider):
    cacheable = True
    needs_network = True

    def __init__(self, cache_dir=None, zoom=12, http_get=None):
        self.zoom = int(zoom)
        if not 0 <= self.zoom <= 15:
            raise ValueError("Terrain Tiles zoom must be 0..15")
        self.name = f"aws-terrarium-z{self.zoom}"
        self.cache_dir = Path(cache_dir or _paths.IMPL_CACHE_DIR) / "terrain_tiles"
        self.http_get = http_get
        self.tiles = {}
        self.provenance = {
            "url": "https://registry.opendata.aws/terrain-tiles/",
            "attribution": "Terrain Tiles / Mapzen, Tilezen; underlying source attribution: https://github.com/tilezen/joerd/blob/master/docs/attribution.md",
            "encoding": "Terrarium: R*256 + G + B/256 - 32768 metres",
            "tile_crs": "EPSG:3857", "zoom": self.zoom,
            "resolution_note": f"zoom {self.zoom}; ~38 m tile pixels at Chennai at z12; underlying DEM resolution varies; resampling adds no source detail",
        }

    def tile(self, x, y):
        x %= 2 ** self.zoom
        y = max(0, min(2 ** self.zoom - 1, y))
        key = x, y
        if key not in self.tiles:
            path = self.cache_dir / str(self.zoom) / str(x) / f"{y}.png"
            if path.exists():
                self.tiles[key] = decode_terrarium(path.read_bytes())
            else:
                import requests
                get = self.http_get or requests.get
                response = get(TILE_URL.format(z=self.zoom, x=x, y=y), timeout=30,
                               headers={"User-Agent": "FYP-B28-Gazebo/1.0 (regional academic simulation)"})
                response.raise_for_status()
                tile = decode_terrarium(response.content)
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(response.content)
                self.tiles[key] = tile
        return self.tiles[key]

    def pixel(self, x, y):
        y = max(0, min(256 * 2 ** self.zoom - 1, y))
        return float(self.tile(x // 256, y // 256)[y % 256, x % 256])

    def fetch(self, coords):
        out = []
        for lat, lon in coords:
            x, y = global_pixel(float(lat), float(lon), self.zoom)
            ix, iy = math.floor(x), math.floor(y)
            fx, fy = x - ix, y - iy
            out.append((1 - fy) * ((1 - fx) * self.pixel(ix, iy) + fx * self.pixel(ix + 1, iy))
                       + fy * ((1 - fx) * self.pixel(ix, iy + 1) + fx * self.pixel(ix + 1, iy + 1)))
        return out
