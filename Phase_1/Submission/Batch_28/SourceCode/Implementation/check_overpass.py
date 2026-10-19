"""Live .venv connectivity check; exits nonzero if the corrected query fails."""
import argparse
import socket
import ssl
import sys
from urllib.parse import urlsplit

import requests
import urllib3

import config as cfg
from overpass_client import endpoint_base, fetch_osm_features, request_overpass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", action="append", help="HTTPS base/interpreter URL; repeat for fallback")
    parser.add_argument("--status", action="store_true", help="Also check DNS, verified TLS and status")
    parser.add_argument("--features", action="store_true", help="Also exercise the actual geometry download/parser")
    parser.add_argument("--radius-m", type=float, default=300)
    args = parser.parse_args()
    print("Python:", sys.executable, sys.version.split()[0])
    print("TLS:", ssl.OPENSSL_VERSION)
    print("requests:", requests.__version__, "urllib3:", urllib3.__version__)
    print("CA bundle:", requests.certs.where())
    endpoints = args.endpoint or cfg.OVERPASS_ENDPOINTS
    if args.status:
        for url in endpoints:
            base = endpoint_base(url)
            host = urlsplit(base).hostname
            try:
                print(host, "DNS:", sorted({r[4][0] for r in socket.getaddrinfo(host, 443, type=socket.SOCK_STREAM)}))
                with socket.create_connection((host, 443), timeout=cfg.OVERPASS_CONNECT_TIMEOUT_S) as raw:
                    with ssl.create_default_context(cafile=requests.certs.where()).wrap_socket(raw, server_hostname=host) as tls:
                        print(host, "verified TLS:", tls.version())
                r = requests.get(base + "/status", headers={"User-Agent": cfg.OVERPASS_USER_AGENT},
                                 timeout=(cfg.OVERPASS_CONNECT_TIMEOUT_S, 20), verify=True)
                print(host, "status HTTP:", r.status_code, r.text[:300])
            except (OSError, requests.RequestException) as exc:
                print(host, type(exc).__name__, str(exc))
    query = '[out:json][timeout:20];way["building"](around:300,12.9830,80.2594);out tags;'
    print("Query:", query)
    payload, base = request_overpass(query, endpoints=endpoints)
    print("PASS:", base, "building ways:", len(payload["elements"]))
    print("OSM snapshot:", payload.get("osm3s", {}).get("timestamp_osm_base"))
    if args.features:
        # Use the tested endpoint order for this invocation, including --endpoint.
        cfg.OVERPASS_ENDPOINTS = tuple(endpoints)
        gdf = fetch_osm_features(cfg.CENTER_LAT, cfg.CENTER_LON, args.radius_m, {"building": True})
        print("FEATURES PASS:", len(gdf), "CRS:", gdf.crs, "types:", gdf.geom_type.value_counts().to_dict())


if __name__ == "__main__":
    main()
