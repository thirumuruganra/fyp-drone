"""
Regenerate the offline terrain fixtures (deterministic):

    python tests/fixtures/make_fixtures.py

  dem_thiruvanmiyur.json              hand-made DEM grid (ElevationGrid JSON)
  osm_buildings_thiruvanmiyur.geojson hand-made building footprints in OSM tag style

These are SYNTHETIC stand-ins shaped like SRTM/OSM data for offline runs and
tests.  They are NOT real elevation or OpenStreetMap data.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))

import config as cfg
from env_data import LocalFrame

COMMENT = ("Hand-made fixture for offline runs and tests; NOT real data. "
           "Regenerate with tests/fixtures/make_fixtures.py")


def make_dem():
    step = 0.005
    n = 15
    lat_s = cfg.CENTER_LAT - step * (n - 1) / 2
    lon_w = cfg.CENTER_LON - step * (n - 1) / 2
    values = []
    for i in range(n):
        row = []
        for j in range(n):
            west = 1.0 - j / (n - 1)                        # coast in the east, higher inland
            bump = 1.5 * math.sin(i / 2.0) * math.cos(j / 3.0)
            row.append(round(2.0 + 9.0 * west + bump, 2))
        values.append(row)
    d = {"_comment": COMMENT, "lat_south": lat_s, "lon_west": lon_w,
         "dlat": step, "dlon": step, "values": values}
    (HERE / "dem_thiruvanmiyur.json").write_text(json.dumps(d, indent=1), encoding="utf-8")


def make_buildings():
    rng = np.random.default_rng(2026)
    frame = LocalFrame(cfg.CENTER_LAT, cfg.CENTER_LON)
    clusters = [(-1800, 1500), (-300, 900), (1200, 1600), (-1500, -900), (400, -400), (1800, -1700)]
    features = []

    def add(cx, cy, w, h, angle, props):
        c, s = math.cos(angle), math.sin(angle)
        corners = [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]
        pts = [(cx + x * c - y * s, cy + x * s + y * c) for x, y in corners]
        ring = []
        for x, y in pts + [pts[0]]:
            lat, lon = frame.to_latlon(x, y)
            ring.append([round(float(lon), 7), round(float(lat), 7)])
        features.append({"type": "Feature", "properties": props,
                         "geometry": {"type": "Polygon", "coordinates": [ring]}})

    for cx, cy in clusters:
        for _ in range(22):
            x, y = cx + rng.normal(0, 320), cy + rng.normal(0, 320)
            w, h, a = rng.uniform(10, 38), rng.uniform(10, 38), rng.uniform(0, math.pi)
            kind = rng.choice(["height", "levels", "type"], p=[0.35, 0.30, 0.35])
            if kind == "height":
                props = {"building": "yes", "height": f"{rng.uniform(6, 45):.0f}"}
            elif kind == "levels":
                props = {"building": "apartments", "building:levels": str(int(rng.integers(2, 13)))}
            else:
                props = {"building": str(rng.choice(["residential", "house", "commercial", "retail", "yes"]))}
            add(x, y, w, h, a, props)
    add(-1200, 800, 30, 30, 0.3, {"building": "office", "height": "62 m", "name": "fixture tower"})
    add(900, -1300, 25, 25, 0.0, {"building": "commercial", "height": "118",
                                  "name": "fixture tower (blocks its node: 118 + 15 > 120 m)"})
    fc = {"type": "FeatureCollection", "_comment": COMMENT, "features": features}
    (HERE / "osm_buildings_thiruvanmiyur.geojson").write_text(json.dumps(fc), encoding="utf-8")


if __name__ == "__main__":
    make_dem()
    make_buildings()
    print("fixtures written to", HERE)
