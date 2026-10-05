import json
import sys
import tempfile
import unittest
from pathlib import Path

IMPLEMENTATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IMPLEMENTATION_DIR))
FIXTURES = IMPLEMENTATION_DIR / "tests" / "fixtures"

import numpy as np
import networkx as nx

import config as cfg
from env_data import (BuildingIndex, ElevationGrid, ElevationProvider, ElevationUnavailable,
                      FlatProvider, LocalFrame, LocalRasterProvider, OpenMeteoElevationProvider,
                      OpenTopographyProvider, WeatherSnapshot, building_height_m,
                      fetch_elevations)
from module1_graph_builder import LatticeAirspaceBuilder

try:
    import shapely
    from shapely.geometry import box
    HAVE_SHAPELY = True
except ImportError:                                           # pragma: no cover
    HAVE_SHAPELY = False
try:
    import geopandas as gpd
    HAVE_GEOPANDAS = True
except ImportError:                                           # pragma: no cover
    HAVE_GEOPANDAS = False

CALM = WeatherSnapshot.calm()


class FakeResponse:
    def __init__(self, payload=None, text=""):
        self._payload, self.text = payload, text

    def raise_for_status(self):
        pass

    def json(self):
        return self._payload


class SlopeProvider(ElevationProvider):
    """Test DEM: rises 5 m per 0.001° of longitude."""
    name = "slope"

    def fetch(self, coords):
        return [(lon - 80.25) * 5000.0 for _, lon in coords]


# ── building heights ────────────────────────────────────────────────────────

class BuildingHeightTests(unittest.TestCase):
    def test_height_tag_wins_and_parses_units(self):
        self.assertEqual(building_height_m({"height": "24", "building:levels": "3"}), 24.0)
        self.assertEqual(building_height_m({"height": "12.5 m"}), 12.5)
        self.assertEqual(building_height_m({"height": "12;15"}), 12.0)
        self.assertAlmostEqual(building_height_m({"height": "100 ft"}), 30.48, places=2)

    def test_levels_times_three_metres(self):
        self.assertEqual(building_height_m({"building:levels": "5"}), 15.0)
        self.assertEqual(building_height_m({"building:levels": "2.5", "height": "yes"}), 7.5)

    def test_per_type_defaults(self):
        self.assertEqual(building_height_m({"building": "residential"}), 9.0)
        self.assertEqual(building_height_m({"building": "house"}), 9.0)
        self.assertEqual(building_height_m({"building": "commercial"}), 15.0)
        self.assertEqual(building_height_m({"building": "retail"}), 15.0)
        self.assertEqual(building_height_m({"building": "yes"}), 10.0)
        self.assertEqual(building_height_m({}), 10.0)

    def test_nonsense_and_non_positive_heights_fall_through(self):
        self.assertEqual(building_height_m({"height": "tall", "building": "yes"}), 10.0)
        self.assertEqual(building_height_m({"height": "0", "building:levels": "4"}), 12.0)
        self.assertEqual(building_height_m({"height": float("nan")}), 10.0)


# ── elevation grid and providers ────────────────────────────────────────────

class ElevationGridTests(unittest.TestCase):
    def setUp(self):
        self.grid = ElevationGrid(10.0, 20.0, 1.0, 1.0, [[0, 10], [20, 30]])

    def test_bilinear_sampling(self):
        self.assertAlmostEqual(self.grid.sample(10.0, 20.0), 0.0)
        self.assertAlmostEqual(self.grid.sample(10.5, 20.5), 15.0)
        self.assertAlmostEqual(self.grid.sample(11.0, 21.0), 30.0)

    def test_outside_the_grid_is_unavailable(self):
        with self.assertRaises(ElevationUnavailable):
            self.grid.sample(14.0, 20.5)

    def test_aaigrid_rows_run_north_to_south(self):
        text = ("ncols 2\nnrows 2\nxllcorner 20\nyllcorner 10\ncellsize 1\nNODATA_value -9999\n"
                "20 30\n0 10\n")
        grid = ElevationGrid.from_aaigrid(text)
        self.assertAlmostEqual(grid.sample(10.5, 20.5), 0.0)       # south-west cell centre
        self.assertAlmostEqual(grid.sample(11.5, 21.5), 30.0)      # north-east cell centre

    def test_aaigrid_nodata_is_unavailable(self):
        text = "ncols 2\nnrows 2\nxllcorner 0\nyllcorner 0\ncellsize 1\nNODATA_value -9999\n-9999 1\n1 1\n"
        with self.assertRaises(ElevationUnavailable):
            ElevationGrid.from_aaigrid(text).sample(1.0, 0.5)


class ProviderTests(unittest.TestCase):
    COORDS = [(12.98 + 0.001 * k, 80.25 + 0.001 * k) for k in range(250)]

    def test_flat_provider_returns_zeros(self):
        self.assertEqual(FlatProvider().fetch(self.COORDS[:3]), [0.0, 0.0, 0.0])

    def test_local_json_raster_matches_fixture_range(self):
        z = LocalRasterProvider(str(FIXTURES / "dem_thiruvanmiyur.json")).fetch(
            [(cfg.CENTER_LAT, cfg.CENTER_LON), (cfg.CENTER_LAT, cfg.CENTER_LON - 0.03)])
        self.assertTrue(all(0.0 < v < 15.0 for v in z))
        self.assertGreater(z[1], z[0])                              # higher inland (west)

    def test_local_raster_without_file_is_unavailable(self):
        with self.assertRaises(ElevationUnavailable):
            LocalRasterProvider(None).fetch(self.COORDS[:1])
        with self.assertRaises(ElevationUnavailable):
            LocalRasterProvider("/nonexistent/dem.tif").fetch(self.COORDS[:1])

    def test_open_meteo_batches_of_100_and_caches_on_disk(self):
        calls = []

        def fake_get(url, params=None, timeout=None):
            calls.append(params)
            n = len(params["latitude"].split(","))
            return FakeResponse({"elevation": [float(len(calls))] * n})

        provider = OpenMeteoElevationProvider(http_get=fake_get)
        with tempfile.TemporaryDirectory() as tmp:
            z = provider.elevations(self.COORDS, cache_dir=tmp)
            self.assertEqual([len(c["latitude"].split(",")) for c in calls], [100, 100, 50])
            self.assertEqual(z[0], 1.0)
            self.assertEqual(z[-1], 3.0)
            again = provider.elevations(self.COORDS, cache_dir=tmp)
            self.assertEqual(len(calls), 3)                          # served from the cache
            self.assertEqual(again, z)
            self.assertTrue((Path(tmp) / "elevation_open-meteo.json").exists())

    def test_cache_key_uses_rounded_coordinates(self):
        calls = []

        def fake_get(url, params=None, timeout=None):
            calls.append(1)
            return FakeResponse({"elevation": [7.0]})

        provider = OpenMeteoElevationProvider(http_get=fake_get)
        with tempfile.TemporaryDirectory() as tmp:
            provider.elevations([(12.9830001, 80.2594001)], cache_dir=tmp)
            provider.elevations([(12.9830004, 80.2593996)], cache_dir=tmp)
        self.assertEqual(len(calls), 1)

    def test_offline_with_missing_cache_is_unavailable(self):
        provider = OpenMeteoElevationProvider(http_get=lambda *a, **k: self.fail("no network offline"))
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ElevationUnavailable):
                provider.elevations(self.COORDS[:2], offline=True, cache_dir=tmp)

    def test_open_meteo_wrong_length_is_rejected(self):
        provider = OpenMeteoElevationProvider(
            http_get=lambda url, params=None, timeout=None: FakeResponse({"elevation": [1.0]}))
        with self.assertRaises(ElevationUnavailable):
            provider.fetch(self.COORDS[:3])

    def test_opentopography_needs_a_key(self):
        with self.assertRaises(ElevationUnavailable):
            OpenTopographyProvider(api_key="", http_get=lambda *a, **k: None).fetch(self.COORDS[:1])

    def test_opentopography_reads_aaigrid_and_sends_key(self):
        seen = {}
        text = ("ncols 3\nnrows 3\nxllcorner 80.0\nyllcorner 12.0\ncellsize 0.5\nNODATA_value -9999\n"
                "9 9 9\n5 5 5\n1 1 1\n")

        def fake_get(url, params=None, timeout=None):
            seen.update(params)
            return FakeResponse(text=text)

        provider = OpenTopographyProvider(api_key="secret", http_get=fake_get)
        z = provider.fetch([(12.25, 80.75)])                         # southern row centre
        self.assertAlmostEqual(z[0], 1.0)
        self.assertEqual(seen["API_Key"], "secret")
        self.assertEqual(seen["outputFormat"], "AAIGrid")
        self.assertEqual(seen["demtype"], cfg.DEM_PROVIDER_TYPE)

    def test_opentopography_key_comes_from_the_environment(self):
        import os
        old = os.environ.get(cfg.OPENTOPO_ENV_KEY)
        os.environ[cfg.OPENTOPO_ENV_KEY] = "from-env"
        try:
            self.assertEqual(OpenTopographyProvider().api_key, "from-env")
        finally:
            if old is None:
                del os.environ[cfg.OPENTOPO_ENV_KEY]
            else:
                os.environ[cfg.OPENTOPO_ENV_KEY] = old

    def test_chain_falls_through_to_next_provider_then_to_flat(self):
        with tempfile.TemporaryDirectory() as tmp:
            z, name = fetch_elevations(self.COORDS[:2], [LocalRasterProvider(None), SlopeProvider()],
                                       cache_dir=tmp)
            self.assertEqual(name, "slope")
            z, name = fetch_elevations(self.COORDS[:2], [LocalRasterProvider(None)], cache_dir=tmp)
            self.assertEqual(name, "flat")
            self.assertTrue(np.all(z == 0.0))


# ── building index ──────────────────────────────────────────────────────────

@unittest.skipUnless(HAVE_SHAPELY, "shapely not installed")
class BuildingIndexTests(unittest.TestCase):
    def setUp(self):
        self.idx = BuildingIndex([box(-10, -10, 10, 10), box(100, 0, 120, 20)], [40.0, 25.0])

    def test_cell_query_takes_the_tallest_touching_footprint(self):
        h = self.idx.max_height_in_cells([0, 110, 500], [0, 10, 500], half_m=50)
        self.assertEqual(list(h), [40.0, 25.0, 0.0])
        h = self.idx.max_height_in_cells([55], [5], half_m=50)       # touches both
        self.assertEqual(list(h), [40.0])

    def test_under_and_within(self):
        self.assertEqual(list(self.idx.tallest_under([0, 30], [0, 0])), [40.0, 0.0])
        self.assertEqual(list(self.idx.tallest_within([30, 60], [0, 0], 30.0)), [40.0, 0.0])

    def test_empty_index(self):
        empty = BuildingIndex.empty()
        self.assertEqual(len(empty), 0)
        self.assertEqual(list(empty.tallest_within([0], [0], 30.0)), [0.0])


# ── lattice with terrain and buildings ──────────────────────────────────────

def lattice(index=None, providers=None, box_m=1000.0):
    return LatticeAirspaceBuilder(box_m=box_m, weather=CALM, elevation_providers=providers,
                                  building_index=index).fetch_graph()


@unittest.skipUnless(HAVE_SHAPELY, "shapely not installed")
class LatticeTerrainTests(unittest.TestCase):
    CENTRE = 12                                                      # 5 × 5 lattice, node (2, 2)

    def test_default_is_the_flat_phase_one_lattice(self):
        G = lattice()
        for _, d in G.nodes(data=True):
            self.assertEqual((d["ground_elev_m"], d["bldg_h_m"], d["alt_amsl_m"]),
                             (0.0, 0.0, cfg.H_CRUISE_AGL))
        for _, _, d in G.edges(data=True):
            self.assertEqual((d["grade"], d["building"], d["terrain"], d["distance"]),
                             (0.0, 0.0, 0.0, d["d_h"]))

    def test_node_altitude_is_ground_plus_cruise_or_building_clearance(self):
        tall = BuildingIndex([box(-10, -10, 10, 10)], [50.0])        # in the centre node's cell
        G = lattice(index=tall, providers=[SlopeProvider()])
        c = G.nodes[self.CENTRE]
        self.assertEqual(c["bldg_h_m"], 50.0)
        self.assertEqual(c["agl_m"], 50.0 + cfg.CLEARANCE)
        self.assertAlmostEqual(c["alt_amsl_m"], c["ground_elev_m"] + 65.0, places=2)
        far = G.nodes[0]
        self.assertEqual((far["bldg_h_m"], far["agl_m"]), (0.0, cfg.H_CRUISE_AGL))
        self.assertAlmostEqual(far["alt_amsl_m"], far["ground_elev_m"] + cfg.H_CRUISE_AGL, places=2)

    def test_low_building_leaves_cruise_height(self):
        low = BuildingIndex([box(-10, -10, 10, 10)], [20.0])         # 20 + 15 < 60
        self.assertEqual(lattice(index=low).nodes[self.CENTRE]["agl_m"], cfg.H_CRUISE_AGL)

    def test_node_needing_more_than_120_m_is_blocked(self):
        towers = BuildingIndex([box(-10, -10, 10, 10)], [cfg.H_MAX_AGL - cfg.CLEARANCE + 1.0])
        G = lattice(index=towers)
        self.assertNotIn(self.CENTRE, G)
        self.assertEqual(G.number_of_nodes(), 24)
        self.assertEqual(G.graph["blocked_nodes"], 1)
        self.assertTrue(nx.is_strongly_connected(G))
        exactly = BuildingIndex([box(-10, -10, 10, 10)], [cfg.H_MAX_AGL - cfg.CLEARANCE])
        self.assertIn(self.CENTRE, lattice(index=exactly))           # 120 m AGL is allowed

    def test_all_altitudes_stay_within_the_legal_ceiling(self):
        G = LatticeAirspaceBuilder(weather=CALM, elevation_providers=[SlopeProvider()],
                                   building_index=_fixture_index()).fetch_graph()
        for _, d in G.nodes(data=True):
            self.assertLessEqual(d["agl_m"], cfg.H_MAX_AGL + 1e-9)
            self.assertGreaterEqual(d["agl_m"], cfg.H_CRUISE_AGL)

    def test_grade_and_distance_come_from_endpoint_altitudes(self):
        G = lattice(providers=[SlopeProvider()])
        u, v = 11, 12                                                 # eastbound, same row
        e, back = G[u][v], G[v][u]
        dalt = (G.nodes[v]["alt_amsl_m"] - G.nodes[u]["alt_amsl_m"]) / 1000.0
        self.assertGreater(e["grade"], 0.0)
        self.assertAlmostEqual(back["grade"], -e["grade"], places=4)
        self.assertAlmostEqual(e["grade"], dalt / e["d_h"], places=4)
        self.assertAlmostEqual(e["distance"], (e["d_h"] ** 2 + dalt ** 2) ** 0.5, places=4)
        self.assertGreater(e["distance"], e["d_h"])
        self.assertGreater(e["energy"], e["distance"])                # |grade| term
        self.assertAlmostEqual(e["energy"], back["energy"], places=4)  # |grade|: climb = descent

    def test_building_term_is_positive_near_a_footprint_and_zero_far_away(self):
        idx = BuildingIndex([box(-10, -10, 10, 10)], [30.0])
        G = lattice(index=idx)
        near, far = G[self.CENTRE][13], G[0][1]
        self.assertGreater(near["building"], 0.0)
        self.assertEqual(far["building"], 0.0)
        self.assertGreater(near["risk"], far["risk"])
        self.assertGreater(near["risk_components"]["building"], 0.0)

    def test_lower_margin_over_a_roof_means_more_building_risk(self):
        # Node altitude is max(60, h + 15): a 30 m roof leaves a 30 m margin at the
        # 60 m cruise height, a 44 m roof only 16 m.
        high = lattice(index=BuildingIndex([box(-10, -10, 10, 10)], [30.0]))[self.CENTRE][13]["building"]
        low = lattice(index=BuildingIndex([box(-10, -10, 10, 10)], [44.0]))[self.CENTRE][13]["building"]
        self.assertGreater(low, high)

    def test_terrain_term_follows_slope(self):
        flat = lattice()[11][12]["terrain"]
        sloped = lattice(providers=[SlopeProvider()])[11][12]["terrain"]
        self.assertEqual(flat, 0.0)
        self.assertGreater(sloped, 0.0)

    def test_low_margin_mid_edge_raises_terrain_risk(self):
        # Roof inside the far node's cell only: that node is lifted to 85 m, the near node
        # stays at 60 m, and the linear climb passes 70 m roof with < CLEARANCE to spare.
        idx = BuildingIndex([box(105, 105, 125, 125)], [70.0])
        G = lattice(index=idx)
        self.assertEqual(G.nodes[self.CENTRE]["agl_m"], cfg.H_CRUISE_AGL)
        self.assertEqual(G.nodes[18]["agl_m"], 70.0 + cfg.CLEARANCE)
        self.assertGreater(G[self.CENTRE][18]["terrain"], 0.5)
        self.assertEqual(G[self.CENTRE][13]["terrain"], 0.0)       # axis-aligned neighbour: clear


def _fixture_index():
    gdf = gpd.read_file(FIXTURES / "osm_buildings_thiruvanmiyur.geojson")
    return BuildingIndex.from_gdf(gdf, LocalFrame(cfg.CENTER_LAT, cfg.CENTER_LON))


# ── OSM layers and fixtures ─────────────────────────────────────────────────

@unittest.skipUnless(HAVE_SHAPELY and HAVE_GEOPANDAS, "shapely/geopandas not installed")
class OsmLayerTests(unittest.TestCase):
    def test_fixture_buildings_become_a_height_index(self):
        idx = _fixture_index()
        self.assertGreater(len(idx), 100)
        self.assertEqual(idx.heights.max(), 118.0)
        self.assertTrue(set(np.round(idx.heights)) & {9.0, 10.0, 15.0})   # type defaults present

    def test_fixture_lattice_blocks_exactly_the_tall_tower_node(self):
        G = LatticeAirspaceBuilder(weather=CALM, building_index=_fixture_index()).fetch_graph()
        self.assertEqual(G.graph["blocked_nodes"], 1)
        self.assertEqual(G.number_of_nodes(), 899)
        self.assertTrue(any(d["building"] > 0 for *_, d in G.edges(data=True)))

    def test_layers_are_fetched_once_cached_as_geopackage_and_reused(self):
        from env_data import OSM_LAYER_TAGS, fetch_osm_layers
        src = gpd.read_file(FIXTURES / "osm_buildings_thiruvanmiyur.geojson")
        calls = []

        def fake_fetcher(lat, lon, dist, tags):
            calls.append(tags)
            if "building" not in tags:
                err = type("InsufficientResponseError", (Exception,), {})
                raise err("no matching features")
            return src.assign(element="way", id=range(len(src))).set_index(["element", "id"])

        with tempfile.TemporaryDirectory() as tmp:
            first = fetch_osm_layers(cache_dir=tmp, fetcher=fake_fetcher)
            self.assertEqual(len(calls), len(OSM_LAYER_TAGS))
            self.assertEqual(set(first), set(OSM_LAYER_TAGS))
            self.assertEqual(len(first["buildings"]), len(src))
            self.assertEqual(len(first["restricted"]), 0)
            self.assertTrue(list(Path(tmp).glob("osm_*.gpkg")))
            second = fetch_osm_layers(cache_dir=tmp, fetcher=lambda *a: self.fail("should use the cache"))
            self.assertEqual(len(second["buildings"]), len(src))

    def test_offline_without_cache_uses_fixture_only_when_asked(self):
        from env_data import fetch_osm_layers
        with tempfile.TemporaryDirectory() as tmp:
            none = fetch_osm_layers(cache_dir=tmp, offline=True)
            self.assertEqual(len(none["buildings"]), 0)
            fixture = fetch_osm_layers(cache_dir=tmp, offline=True, fixture_dir=str(FIXTURES))
            self.assertGreater(len(fixture["buildings"]), 100)

    def test_failed_live_fetch_falls_back_cleanly(self):
        from env_data import fetch_osm_layers

        def broken(lat, lon, dist, tags):
            raise ConnectionError("blocked")

        with tempfile.TemporaryDirectory() as tmp:
            layers = fetch_osm_layers(cache_dir=tmp, fetcher=broken)
        self.assertEqual(len(layers["buildings"]), 0)


class LocalFrameTests(unittest.TestCase):
    def test_round_trip_and_scale(self):
        f = LocalFrame(12.983, 80.2594)
        lat, lon = f.to_latlon(1000.0, 2000.0)
        x, y = f.to_local(lat, lon)
        self.assertAlmostEqual(float(x), 1000.0, places=6)
        self.assertAlmostEqual(float(y), 2000.0, places=6)
        self.assertLess(f.m_per_deg_lon, f.m_per_deg_lat)           # cos(lat) shrink


if __name__ == "__main__":
    unittest.main()
