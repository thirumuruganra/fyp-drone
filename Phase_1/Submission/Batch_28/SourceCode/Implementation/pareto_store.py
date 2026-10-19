"""
pareto_store.py — persistent Tier 1 fronts with refresh metadata.
=================================================================
Tier 1 runs once per delivery region and its front is reused by Tier 2 for every
mission.  A stored record is only trusted while the world it was built for still
holds, so each record carries the metadata needed to decide that:

    {location, graph_hash, graph_timestamp, weather_timestamp, weather_snapshot,
     source, target, created_at, valid_until, truncated, front, ...}

get_or_build() reuses the record unless it is
    • missing                      • forced (--refresh)
    • built on a different static graph (graph_hash)
    • built for a different caller-supplied identity (extra_identity, e.g. the
      simulation region bundle hash and clearance settings)
    • expired (valid_until)
    • stale: the weather drifted past a threshold since the record was built
        |Δ wind speed| > 10 km/h       |Δ direction| > 45° when the wind > 10 km/h
        |Δ gust| > 15 km/h             a weather go/no-go flip
and a rebuild re-runs only apply_weather() (when a new snapshot is given) plus
Tier 1 — the static layers (DEM, buildings, zones) stay cached in the graph.

The graph hash covers the static edge attributes only (never weather-dependent
costs), so a weather change alone does not change it.

Thresholds and the default validity window live in config.py.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
from datetime import datetime, timedelta, timezone
from typing import Iterable, Optional, Sequence

import networkx as nx

import config as cfg
from env_data import WeatherSnapshot
from module1_graph_builder import apply_weather, haversine_km
from module2_tier1_moastar import TierOneEngine, thin_front

STATIC_EDGE_KEYS = ("d_h", "distance", "bearing", "grade", "obstacle", "building", "terrain",
                    "population", "nfz", "water", "comm", "density")
DEFAULT_ROOT = os.path.join("pareto_front_storage", "fronts")


# ─────────────────────────────────────────────────────────────────────────────
#  Hashing and drift
# ─────────────────────────────────────────────────────────────────────────────

def graph_hash(G: nx.DiGraph) -> str:
    """
    SHA-256 of the static graph: node ids and every edge with its static attributes.
    Weather-dependent costs (energy, risk, ...) are excluded.  Graphs built without
    static attributes (hand-made) fall back to their distance/energy/risk costs.
    """
    h = hashlib.sha256()
    for u, v, d in sorted(G.edges(data=True), key=lambda e: (str(e[0]), str(e[1]))):
        keys = STATIC_EDGE_KEYS if "d_h" in d else ("distance", "energy", "risk")
        vals = ",".join(f"{round(float(d.get(k, 0.0)), 5):.5f}" for k in keys)
        h.update(f"{u}>{v}|{vals};".encode())
    h.update(f"n={G.number_of_nodes()}".encode())
    return h.hexdigest()


def _angle_diff(a: float, b: float) -> float:
    """Smallest absolute difference between two compass directions, in degrees."""
    d = abs(float(a) - float(b)) % 360.0
    return min(d, 360.0 - d)


def weather_drift(old: dict, new: WeatherSnapshot) -> list:
    """
    Reasons the weather moved enough to invalidate a front built for `old` (a
    WeatherSnapshot dict); empty when it is still representative.
    """
    o = WeatherSnapshot.from_dict(old)
    reasons = []
    dv = abs(new.wind_speed_kmh - o.wind_speed_kmh)
    if dv > cfg.DRIFT_WIND_SPEED_KMH:
        reasons.append(f"wind speed moved {dv:.1f} km/h (> {cfg.DRIFT_WIND_SPEED_KMH:g})")
    dd = _angle_diff(new.wind_dir_from_deg, o.wind_dir_from_deg)
    if dd > cfg.DRIFT_WIND_DIR_DEG and max(new.wind_speed_kmh, o.wind_speed_kmh) > cfg.DRIFT_DIR_MIN_SPEED_KMH:
        reasons.append(f"wind direction turned {dd:.0f}° (> {cfg.DRIFT_WIND_DIR_DEG:g}°)")
    dg = abs(new.gust_kmh - o.gust_kmh)
    if dg > cfg.DRIFT_GUST_KMH:
        reasons.append(f"gusts moved {dg:.1f} km/h (> {cfg.DRIFT_GUST_KMH:g})")
    if bool(o.no_go_reasons()) != bool(new.no_go_reasons()):
        reasons.append("weather go/no-go flipped")
    return reasons


# ─────────────────────────────────────────────────────────────────────────────
#  Records
# ─────────────────────────────────────────────────────────────────────────────

def _now(now: Optional[datetime]) -> datetime:
    return now if now is not None else datetime.now(timezone.utc)


def _parse(ts: str) -> datetime:
    dt = datetime.fromisoformat(ts)
    return dt if dt.tzinfo else dt.replace(tzinfo=timezone.utc)


def front_from_record(record: dict) -> list:
    """The stored front as [([d, ε, ρ], path), ...] (Tier 2 / engine format)."""
    return [(list(r["g"]), list(r["path"])) for r in record["front"]]


def _slug(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "-", str(text)).strip("-").lower() or "region"


class ParetoStore:
    """JSON files, one per (location, source, target), under `root`."""

    def __init__(self, root: str = DEFAULT_ROOT):
        self.root = root

    def path(self, location: str, source, target) -> str:
        return os.path.join(self.root, f"{_slug(location)}__{_slug(source)}__{_slug(target)}.json")

    def load(self, location: str, source, target) -> Optional[dict]:
        try:
            with open(self.path(location, source, target), "r", encoding="utf-8") as fh:
                return json.load(fh)
        except (OSError, ValueError):
            return None

    def save(self, record: dict) -> str:
        os.makedirs(self.root, exist_ok=True)
        path = self.path(record["location"], record["source"], record["target"])
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(record, fh, indent=1)
        return path

    # ------------------------------------------------------------------

    def staleness(self, record: Optional[dict], G: nx.DiGraph, weather: WeatherSnapshot,
                  refresh: bool = False, now: Optional[datetime] = None,
                  extra_identity: Optional[dict] = None) -> list:
        """Reasons the record cannot be reused (empty list = reuse it)."""
        if record is None:
            return ["no stored front"]
        if refresh:
            return ["refresh requested"]
        reasons = []
        if record.get("graph_hash") != graph_hash(G):
            reasons.append("static graph changed")
        if extra_identity is not None and record.get("identity") != extra_identity:
            reasons.append("identity changed")
        if _now(now) > _parse(record["valid_until"]):
            reasons.append(f"expired at {record['valid_until']}")
        reasons.extend(weather_drift(record["weather_snapshot"], weather))
        return reasons

    def get_or_build(self, G: nx.DiGraph, source, target, *,
                     location: Optional[str] = None,
                     weather: Optional[WeatherSnapshot] = None,
                     refresh: bool = False,
                     valid_hours: float = cfg.FRONT_VALID_HOURS,
                     max_explored: int = 200_000,
                     front_max: Optional[int] = None,
                     now: Optional[datetime] = None,
                     extra_identity: Optional[dict] = None,
                     verbose: bool = True):
        """
        Reuse the stored front for (location, source, target) or rebuild it.

        weather : the conditions to plan for.  Defaults to the snapshot already applied
                  to G (G.graph["weather"]; calm for graphs without one, which are left
                  as they are).  A rebuild re-applies an explicitly given snapshot with
                  apply_weather().
        front_max : optionally thin the front by ε-dominance (off by default).
        extra_identity : JSON-serialisable dict stored with the record; a record built
                  for a different identity is rebuilt (None: not checked).

        Returns (record, rebuilt, reasons).  On reuse G is left untouched, so its edge
        costs may reflect a different (still representative) weather than the stored one.
        """
        location = location or G.graph.get("location") or "region"
        explicit_weather = weather is not None
        if weather is None:
            weather = WeatherSnapshot.from_dict(G.graph["weather"]) if "weather" in G.graph \
                else WeatherSnapshot.calm()
        record = self.load(location, source, target)
        reasons = self.staleness(record, G, weather, refresh, now, extra_identity)
        if not reasons:
            if verbose:
                print(f"[Store] Reusing stored front ({len(record['front'])} routes, built "
                      f"{record['created_at']}, valid until {record['valid_until']})")
            return record, False, []

        if verbose:
            print("[Store] Rebuilding front: " + "; ".join(reasons))
        if explicit_weather and G.graph.get("weather") != weather.to_dict():
            apply_weather(G, weather)                       # weather layer only; static layers stay
        engine = TierOneEngine(G, source, target, max_explored=max_explored)
        front = engine.run()
        n_exact = len(front)
        if front_max is not None:
            front = thin_front(front, front_max)
        created = _now(now)
        record = {
            "location": location,
            "graph_hash": graph_hash(G),
            "graph_timestamp": G.graph.get("built_at", ""),
            "weather_timestamp": weather.time or weather.fetched_at,
            "weather_snapshot": weather.to_dict(),
            "source": source,
            "target": target,
            "created_at": created.isoformat(timespec="seconds"),
            "valid_until": (created + timedelta(hours=valid_hours)).isoformat(timespec="seconds"),
            "truncated": engine.truncated,
            "front_size_exact": n_exact,
            "front_max": front_max,
            "diagnostics": engine.diagnostics(),
            "front": [{"g": list(g), "path": list(p)} for g, p in front],
        }
        if extra_identity is not None:
            record["identity"] = extra_identity
        path = self.save(record)
        if verbose:
            print(f"[Store] Saved {len(front)} routes → {path}")
        return record, True, reasons


# ─────────────────────────────────────────────────────────────────────────────
#  Region runs
# ─────────────────────────────────────────────────────────────────────────────

def snap_to_node(G: nx.DiGraph, lat: float, lon: float):
    """Nearest graph node (great-circle) to a lat/lon point."""
    return min(G.nodes, key=lambda n: haversine_km(lat, lon, G.nodes[n]["lat"], G.nodes[n]["lon"]))


def _as_node(G: nx.DiGraph, place):
    """A node id, or a (lat, lon) pair snapped to the nearest node."""
    if isinstance(place, (tuple, list)) and len(place) == 2 and place not in G:
        return snap_to_node(G, float(place[0]), float(place[1]))
    return place


def run_region(G: nx.DiGraph, depot, delivery_points: Iterable, *,
               store: Optional[ParetoStore] = None, **kwargs) -> dict:
    """
    One Tier 1 front per delivery point, planned from a common depot.  depot and the
    delivery points are node ids or (lat, lon) pairs, snapped to the nearest lattice node.

    Returns {delivery point node: record}.  Keyword arguments go to get_or_build()
    (weather, refresh, valid_hours, max_explored, front_max, now, location).  Points that
    snap to the depot, or to the same node as an earlier point, are skipped.
    """
    store = store or ParetoStore()
    source = _as_node(G, depot)
    if source not in G:
        raise ValueError(f"depot {depot!r} is not in the graph")
    out: dict = {}
    for place in delivery_points:
        target = _as_node(G, place)
        if target not in G:
            raise ValueError(f"delivery point {place!r} is not in the graph")
        if target == source or target in out:
            continue
        out[target], _, _ = store.get_or_build(G, source, target, **kwargs)
    return out
