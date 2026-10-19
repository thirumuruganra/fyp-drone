# Road display reference

`osm_roads_thiruvanmiyur.geojson` contains historical OpenStreetMap driving-network segments extracted from repository commit `5a09428d2d073a1716e657679755e17610d7ce05`, `Review_1/Implementation/pareto_front_storage/airspace_graph_osm.pkl`.

Original downloader: `osmnx.graph_from_address(..., network_type="drive")`. Snapshot date unknown. Duplicate direction edges removed. Curved way geometry was discarded by the original importer; this reference joins original road nodes with straight segments.

Display fallback when the live road fetch/cache is unavailable. Never used for airspace connectivity, costs, obstacles, or safety decisions. Console identifies historical fallback. Live downloads retain full way geometry and take precedence over this reference.

Data © OpenStreetMap contributors, available under the [Open Database License](https://www.openstreetmap.org/copyright).
