# Overpass connectivity (2026-10-06)

Use the existing repository `.venv`. No Python downgrade, dependency reinstall,
CA replacement, TLS-version override, or `verify=False` is needed.

## Findings from this Mac/network

- `.venv`: Python 3.14.7, requests 2.34.2, urllib3 2.8.0, OpenSSL 3.6.4;
  OSMnx 2.1.1. No proxy or CA environment overrides were present.
- Network-restricted agent sandbox cannot resolve public hostnames. Real network
  checks below ran outside that sandbox, with the same `.venv` executable.
- `overpass-api.de` resolved to `162.55.144.139` and `65.109.112.52`.
  Both macOS curl and Python completed certificate-verified TLS 1.3.
  Therefore the reproduced failures were not certificate-verification errors.
- Default requests headers received connection resets; default macOS curl received
  HTTP 406. Kumi's status endpoint explicitly returned HTTP 429 requesting a
  meaningful User-Agent. An identifying project User-Agent received HTTP 200 on
  the main status endpoint, but does not eliminate intermittent query failures.
- Corrected query received HTTP 200 with **205 building ways**, using unchanged
  `.venv` requests POST and GET; macOS curl POST also succeeded. Other attempts
  received connection resets or HTTP 504. Standard-library HTTP also received
  504; Python 3.11 also completed verified TLS and received status HTTP 200.
  No evidence points to a Python 3.14/requests/urllib3/OpenSSL incompatibility.
- Reset origin cannot be proven from client evidence alone: server rejection,
  overload, or an intervening network device may reset a connection. The endpoint
  was intermittently available, not globally unreachable. Encoding variants both
  succeeded; changing `+` to `%20` is not a demonstrated fix.
- Private.coffee and its old Kumi hostname resolved to the same IP and completed
  verified TLS, but queries reset/timed out in this check. They are not two
  independent fallbacks. `overpass.osm.ch` covers Switzerland only, unsuitable
  for Chennai. VK Maps was also tested and reset from this network.

The [OSM public instance list](https://wiki.openstreetmap.org/wiki/Overpass_API#Public_Overpass_API_instances)
lists main and Private.coffee as global-data services. Public endpoints have no
availability guarantee. Keep caches; configure a self-hosted/paid global endpoint
for sustained operational use. See [Overpass service guidance](https://dev.overpass-api.de/overpass-doc/en/preface/commons.html).

## Query correction

The quoted shell heredoc already preserves Python source. Do not escape colons or
parentheses inside the Python query string:

```python
query = '[out:json][timeout:20];way["building"](around:300,12.9830,80.2594);out tags;'
```

Literal `\\:` / `\\(` in the posted Python string send a backslash to Overpass.
The original query was tested: HTTP 400, `':' expected - '\\' found`.
This syntax error is separate from a reset occurring before an HTTP response.
`out tags` is sufficient for the connectivity check but has no route-planning
geometry. The feature downloader uses nodes/ways/relations, downward recursion,
and `out body` to retain building footprints and relation members.

## Project changes

- `config.py`: HTTPS endpoint order, identifying User-Agent, connect/read/server
  timeouts, attempt counts and retry pauses.
- `overpass_client.py`: verified HTTPS POST, at most two attempts per endpoint;
  retry transport failures and temporary HTTP/server failures, then try fallback.
  HTTP 406/429/5xx pause at least 30 seconds; respect Retry-After, stopping if it
  exceeds the configured wait budget. HTTP 400/auth errors and SSL errors surface
  immediately. Partial responses with `remark` are rejected.
- `env_data.py`: both risk-layer and road-overlay downloads use this client;
  cache/fixture behavior remains. OSMnx's public `features_from_xml` parses
  geometry without private API patches or its unbounded recursive HTTP retries.
- `check_overpass.py`: live corrected-query check, optional DNS/TLS/status and
  actual building-feature check. Failures exit nonzero, so cached/offline data
  cannot masquerade as live network success.
- `tests/test_overpass.py`: offline transport, fallback, TLS/error handling,
  backoff, incomplete-response, and geometry regressions.

## Exact verification commands

From any shell:

```bash
cd /Users/meghna/Desktop/fyp-drone
export FYP_PY="$PWD/.venv/bin/python"
"$FYP_PY" -u -B Implementation/check_overpass.py --features
"$FYP_PY" -u -B Implementation/check_overpass.py --status
MPLBACKEND=Agg "$FYP_PY" -B -m unittest discover -s Implementation/tests
```

Success prints `PASS: ... building ways: ...` and, with `--features`,
`FEATURES PASS: ... CRS: EPSG:4326 ...`. Geometry uses a square bounding box,
so its feature count differs from the 300 m circular connectivity query.
Failures print attempted endpoint/errors and raise instead of reporting success.
Busy-server retries can take several minutes; attempts are finite. No automatic
network check runs during ordinary offline unit tests.

Validation: 14 new regression tests pass. Final full discovery ran 208 tests:
207 passed; one module import failed because this existing Homebrew Python lacks
`_tkinter` (`test_dashboard_helpers`). This is unrelated to Overpass; no Python/Tk
installation was changed during network repair.

A separate live XML geometry query also returned HTTP 200, 109,889 bytes and
205 polygons parsed by the same `.venv` OSMnx reader. Some runs of the full
square-bounding-box check still exhausted retries; public-server/network
availability remains intermittent.

To isolate one server:

```bash
"$FYP_PY" -u -B Implementation/check_overpass.py --endpoint https://overpass-api.de/api --features
"$FYP_PY" -u -B Implementation/check_overpass.py --endpoint https://overpass.private.coffee/api
```

For the demo, existing GeoPackage/GeoJSON caches can avoid live queries. The check
script deliberately bypasses those caches. Running a full planner demo is not
required to verify an API connection.
