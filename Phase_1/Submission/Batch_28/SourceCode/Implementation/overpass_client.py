"""Verified HTTPS Overpass transport with finite retries and global-data failover.

OSMnx's public XML reader handles feature geometry. Its download code is avoided
because it retries HTTP 429/504 recursively without an attempt limit. Project
GeoPackage/GeoJSON caches remain managed by env_data.
"""
from __future__ import annotations

import json
import html
import math
import os
import re
import tempfile
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from email.utils import parsedate_to_datetime
from pathlib import Path
from urllib.parse import urlsplit

import requests

import config as cfg


class OverpassUnavailable(RuntimeError):
    """All configured attempts failed; no usable, complete response was obtained."""


class OverpassBackoff(OverpassUnavailable):
    """Server-requested retry delay exceeds this invocation's wait budget."""


def endpoint_base(url: str) -> str:
    """Accept a base or interpreter URL, requiring certificate-verified HTTPS."""
    base = url.rstrip("/")
    if base.endswith("/interpreter"):
        base = base[:-len("/interpreter")]
    parsed = urlsplit(base)
    if (parsed.scheme != "https" or not parsed.hostname or parsed.username
            or parsed.password or parsed.query or parsed.fragment):
        raise ValueError("Overpass endpoint must be an HTTPS base/interpreter URL")
    return base


def _busy_pause(response) -> float:
    delay = float(cfg.OVERPASS_BUSY_DELAY_S)
    value = response.headers.get("Retry-After")
    if value:
        try:
            wait = float(value)
        except ValueError:
            try:
                wait = (parsedate_to_datetime(value) - datetime.now(timezone.utc)).total_seconds()
            except (ValueError, TypeError, OverflowError):
                wait = delay
        if math.isfinite(wait):
            delay = max(delay, wait)
    if delay > cfg.OVERPASS_MAX_RETRY_WAIT_S:
        raise OverpassBackoff(f"Server requests Retry-After={delay:.0f}s; retry later")
    return delay


def request_overpass(query: str, *, output: str = "json", endpoints=None,
                     post=None, sleep=None, report=print):
    """Return (validated payload, successful base URL).

    Retry connection resets/timeouts, temporary HTTP failures and incomplete
    server results. Query/auth errors and certificate failures stop immediately.
    No TLS version, cipher, proxy or CA override; verify=True on every request.
    XML payloads return bytes for OSMnx's public features_from_xml reader.
    """
    if output not in {"json", "xml"}:
        raise ValueError("output must be json or xml")
    if endpoints is None:
        # Private/keyed mirrors (URL embeds a secret) come from env, never from tracked files.
        extra = [u.strip() for u in os.environ.get(cfg.OVERPASS_EXTRA_ENV, "").split(",") if u.strip()]
        endpoints = (*cfg.OVERPASS_ENDPOINTS, *extra)
    bases = tuple(dict.fromkeys(endpoint_base(u) for u in endpoints))
    if not bases:
        raise ValueError("At least one Overpass endpoint is required")
    post = requests.post if post is None else post
    sleep = time.sleep if sleep is None else sleep
    failures = []
    # Overpass's [out:...] setting controls format; no content negotiation needed.
    headers = {"User-Agent": cfg.OVERPASS_USER_AGENT}
    for base in bases:
        for attempt in range(1, cfg.OVERPASS_ATTEMPTS_PER_ENDPOINT + 1):
            pause = cfg.OVERPASS_RETRY_DELAY_S
            try:
                response = post(
                    base + "/interpreter", data={"data": query}, headers=headers,
                    timeout=(cfg.OVERPASS_CONNECT_TIMEOUT_S, cfg.OVERPASS_READ_TIMEOUT_S),
                    verify=True,
                )
                if response.status_code in {406, 408, 429} or 500 <= response.status_code < 600:
                    pause = _busy_pause(response)
                    detail = re.sub(r"\s+", " ", html.unescape(re.sub(r"<[^>]*>", " ", response.text))).strip()
                    raise OverpassUnavailable(f"HTTP {response.status_code}: {detail[:400]}")
                response.raise_for_status()
                if output == "json":
                    payload = response.json()
                    if (not isinstance(payload, dict) or not isinstance(payload.get("elements"), list)
                            or payload.get("remark")):
                        raise OverpassUnavailable(f"Incomplete/non-OSM response: {str(payload)[:200]}")
                else:
                    root = ET.fromstring(response.content)
                    if root.tag != "osm" or root.find("remark") is not None:
                        raise OverpassUnavailable("Incomplete/non-OSM XML response")
                    payload = response.content
                report(f"[Overpass] {base}: HTTP {response.status_code}, attempt {attempt}")
                return payload, base
            except requests.exceptions.SSLError:
                raise  # CA/certificate/TLS failures need investigation, never verify=False
            except requests.exceptions.HTTPError:
                raise  # invalid QL (400), auth/permission errors, etc. are not outages
            except OverpassBackoff:
                raise
            except (requests.exceptions.ConnectionError, requests.exceptions.Timeout,
                    OverpassUnavailable, ValueError, ET.ParseError) as exc:
                message = f"{base} attempt {attempt}: {type(exc).__name__}: {exc}"
                failures.append(message)
                report(f"[Overpass] {message}")
                if attempt < cfg.OVERPASS_ATTEMPTS_PER_ENDPOINT or base != bases[-1]:
                    sleep(pause)
    raise OverpassUnavailable("All endpoints failed: " + "; ".join(failures))


def fetch_osm_features(lat: float, lon: float, dist_m: float, tags: dict):
    """OSM features in the same square bounding box as features_from_point.

    Tags are ORed, including nodes, ways and relations. Downward recursion keeps
    the nodes/member ways required for complete building/multipolygon geometry.
    No OSMnx private APIs or process-global settings/request monkeypatches.
    """
    if (not math.isfinite(lat) or not math.isfinite(lon) or not math.isfinite(dist_m)
            or not -90 < lat < 90 or not -180 <= lon <= 180 or dist_m <= 0):
        raise ValueError("Expected valid latitude/longitude and positive finite radius")
    if not isinstance(tags, dict) or not tags:
        raise ValueError("tags must be a nonempty dictionary")
    delta_lat = math.degrees(dist_m / (cfg.EARTH_RADIUS_KM * 1000))
    delta_lon = delta_lat / math.cos(math.radians(lat))
    bbox = f"{lat-delta_lat:.7f},{lon-delta_lon:.7f},{lat+delta_lat:.7f},{lon+delta_lon:.7f}"
    components = []
    for key, value in tags.items():
        if not isinstance(key, str):
            raise TypeError("OSM tag keys must be strings")
        if value is True:
            filters = [f"[{json.dumps(key, ensure_ascii=False)}]"]
        else:
            values = [value] if isinstance(value, str) else value
            if not isinstance(values, list) or not all(isinstance(v, str) for v in values):
                raise TypeError("OSM tag values must be True, a string or a list of strings")
            filters = [f"[{json.dumps(key, ensure_ascii=False)}={json.dumps(v, ensure_ascii=False)}]" for v in values]
        components.extend(f"nwr{tag_filter}({bbox});" for tag_filter in filters)
    if not components:
        raise ValueError("tags must select at least one value")
    query = (f"[out:xml][timeout:{cfg.OVERPASS_QUERY_TIMEOUT_S}];"
             f"({''.join(components)});(._;>;);out body;")
    import osmnx as ox
    from shapely.geometry import box

    polygon = box(lon-delta_lon, lat-delta_lat, lon+delta_lon, lat+delta_lat)
    payload, base = request_overpass(query, output="xml")
    with tempfile.TemporaryDirectory(prefix="fyp-overpass-") as folder:
        path = Path(folder) / "features.osm"
        path.write_bytes(payload)
        gdf = ox.features_from_xml(path, polygon=polygon, tags=tags)
    gdf.attrs["overpass_endpoint"] = base
    return gdf
