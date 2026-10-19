import math
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
from env_data import (BuildingIndex, ClassIndex, LocalFrame, WeatherSnapshot, ZoneIndex,
                      build_layer_indexes, fetch_osm_layers, population_class, sea_polygons)
from module1_graph_builder import (AirspaceGraphBuilder, EdgeFeatures, LatticeAirspaceBuilder,
                                   apply_weather, risk_components)
from module2_tier1_moastar import TierOneEngine
from module4_tier2_payload import TierTwoSelector, WeatherNoGo

try:
    from shapely.geometry import LineString, box
    import geopandas as gpd
    HAVE_GEO = True
except ImportError:                                           # pragma: no cover
    HAVE_GEO = False

CALM = WeatherSnapshot.calm()
FRAME = LocalFrame(cfg.CENTER_LAT, cfg.CENTER_LON)
needs_geo = unittest.skipUnless(HAVE_GEO, "shapely/geopandas not installed")


def lattice(weather=CALM, box_m=1000.0, **layers):
    return LatticeAirspaceBuilder(box_m=box_m, weather=weather, **layers).fetch_graph()


# ── risk components ─────────────────────────────────────────────────────────

class ComponentTests(unittest.TestCase):
    def test_every_weighted_component_is_reported(self):
        comps = risk_components(EdgeFeatures.build(0.2, 0), CALM)
        self.assertEqual(set(comps), set(cfg.RISK_WEIGHTS))
        self.assertEqual(len(comps), 11)

    def test_each_static_term_is_scaled_by_its_weight(self):
        f = EdgeFeatures.build(0.2, 0, obstacle=0.5, building=0.4, terrain=0.3, population=0.2,
                               nfz=0.7, water=0.6, comm=0.1)
        comps = risk_components(f, CALM)
        for key, value in (("obstacle", .5), ("building", .4), ("terrain", .3), ("population", .2),
                           ("nfz", .7), ("water", .6), ("comm", .1)):
            self.assertAlmostEqual(comps[key], cfg.RISK_WEIGHTS[key] * value)

    def test_wind_and_gust_are_separate_terms(self):
        f = EdgeFeatures.build(0.2, 0)                        # wind from the east → crosswind
        w = WeatherSnapshot(wind_speed_kmh=20, wind_dir_from_deg=90, gust_kmh=36)
        comps = risk_components(f, w)
        self.assertAlmostEqual(comps["wind"], cfg.RISK_WEIGHTS["wind"] * 20 / cfg.V_MAX_SAFE_KMH)
        self.assertAlmostEqual(comps["gust"], cfg.RISK_WEIGHTS["gust"] * cfg.K_GUST * 16 / cfg.V_MAX_SAFE_KMH)

    def test_turbulence_is_density_times_relative_wind(self):
        f = EdgeFeatures.build(0.2, 0, density=0.5)
        w = WeatherSnapshot(wind_speed_kmh=20, wind_dir_from_deg=90)
        self.assertAlmostEqual(risk_components(f, w)["turbulence"],
                               cfg.RISK_WEIGHTS["turbulence"] * 0.5 * 20 / cfg.V_MAX_SAFE_KMH)
        self.assertEqual(risk_components(f, CALM)["turbulence"], 0.0)               # no wind
        self.assertEqual(risk_components(EdgeFeatures.build(0.2, 0), w)["turbulence"], 0.0)  # no buildings

    def test_unit_terms_are_clipped(self):
        f = EdgeFeatures.build(0.2, 0, nfz=7.0, water=-1.0)
        self.assertEqual(f.nfz, 1.0)
        self.assertEqual(f.water, 0.0)


# ── length-weighted exposure ────────────────────────────────────────────────

class ExposureTests(unittest.TestCase):
    def test_risk_cost_is_rho_times_length_over_spacing(self):
        w = WeatherSnapshot(wind_speed_kmh=30, wind_dir_from_deg=90, gust_kmh=30)
        G = lattice(weather=w)
        self.assertAlmostEqual(G.graph["spacing_km"], 0.2)
        for u, v, d in G.edges(data=True):
            self.assertAlmostEqual(d["risk"], d["rho"] * d["d_h"] / 0.2, places=4)
            self.assertEqual(d["cost"][2], d["risk"])

    def test_diagonal_hop_counts_sqrt2_axis_hops_for_equal_rho(self):
        w = WeatherSnapshot(precipitation_mm=7.5, visibility_m=10000)     # uniform weather term only
        G = lattice(weather=w)
        axis, diag = G[12][13], G[12][18]
        self.assertAlmostEqual(axis["rho"], diag["rho"], places=5)
        self.assertAlmostEqual(diag["risk"] / axis["risk"], math.sqrt(2), places=2)

    def test_synthetic_graphs_are_length_weighted_too(self):
        G = AirspaceGraphBuilder(seed=3)._build_synthetic_graph(25)
        self.assertGreater(G.graph["spacing_km"], 1.0)
        u, v, d = next(iter(G.edges(data=True)))
        self.assertAlmostEqual(d["risk"], d["rho"] * d["d_h"] / G.graph["spacing_km"], places=4)

    def test_graph_without_spacing_is_left_unscaled(self):
        G = nx.DiGraph()
        G.add_edge(0, 1, **EdgeFeatures.build(0.5, 90, obstacle=0.4).edge_attrs())
        apply_weather(G, CALM)
        self.assertAlmostEqual(G[0][1]["risk"], G[0][1]["rho"] * 0.5, places=4)


# ── weather gate ────────────────────────────────────────────────────────────

class WeatherGateTests(unittest.TestCase):
    def test_clear_weather_is_go(self):
        self.assertEqual(WeatherSnapshot(wind_speed_kmh=20, gust_kmh=30).no_go_reasons(), [])

    def test_each_condition_triggers_no_go(self):
        cases = {
            "thunderstorm": WeatherSnapshot(weather_code=95),
            "heavy rain": WeatherSnapshot(precipitation_mm=cfg.NO_GO_PRECIP_MM_H),
            "low visibility": WeatherSnapshot(visibility_m=cfg.NO_GO_VISIBILITY_M - 1),
            "gusts": WeatherSnapshot(wind_speed_kmh=30, gust_kmh=cfg.NO_GO_GUST_KMH + 1),
        }
        for name, snap in cases.items():
            reasons = snap.no_go_reasons()
            self.assertEqual(len(reasons), 1, name)
            self.assertIn(name.split()[0], reasons[0])

    def test_boundaries_are_go(self):
        snap = WeatherSnapshot(weather_code=94, precipitation_mm=cfg.NO_GO_PRECIP_MM_H - 0.1,
                               visibility_m=cfg.NO_GO_VISIBILITY_M, gust_kmh=cfg.NO_GO_GUST_KMH)
        self.assertEqual(snap.no_go_reasons(), [])

    def test_gate_is_stored_on_the_graph_and_follows_reapply(self):
        G = lattice()
        self.assertFalse(G.graph["no_go"])
        self.assertEqual(G.graph["no_go_reasons"], [])
        apply_weather(G, WeatherSnapshot(weather_code=97, visibility_m=500))
        self.assertTrue(G.graph["no_go"])
        self.assertEqual(len(G.graph["no_go_reasons"]), 2)
        apply_weather(G, CALM)
        self.assertFalse(G.graph["no_go"])

    def test_tier2_holds_when_the_gate_is_no_go(self):
        front = [([1.0, 1.0, 1.0], [0, 1]), ([2.0, 0.5, 0.5], [0, 2, 1])]
        go = TierTwoSelector(front)
        self.assertFalse(go.no_go)
        self.assertEqual(len(go.run_all_presets()), 4)
        hold = TierTwoSelector(front, no_go_reasons=["thunderstorm (WMO code 95)"])
        self.assertTrue(hold.no_go)
        self.assertIn("HOLD", hold.hold_advisory())
        with self.assertRaises(WeatherNoGo):
            hold.select(3, 2, 2, 3)
        self.assertEqual(hold.run_all_presets(), {})


# ── zones, water, population, density on the lattice ────────────────────────

@needs_geo
class ZoneTests(unittest.TestCase):
    def test_zone_index_contains_distance_and_segments(self):
        z = ZoneIndex([box(0, 0, 100, 100)])
        self.assertEqual(list(z.contains([50, 150], [50, 50])), [True, False])
        d = z.distance([50, 150, 1000], [50, 50, 50], max_m=200)
        self.assertEqual(d[0], 0.0)
        self.assertAlmostEqual(d[1], 50.0)
        self.assertTrue(np.isinf(d[2]))
        hit = z.intersects_segments([[-50, 50], [-50, 200]], [[150, 50], [150, 200]])
        self.assertEqual(list(hit), [True, False])

    def test_manual_zone_becomes_a_circle(self):
        z = ZoneIndex.from_restricted(None, FRAME, manual=[("test", cfg.CENTER_LAT, cfg.CENTER_LON, 300.0)])
        self.assertEqual(len(z), 1)
        self.assertTrue(z.contains([0], [250])[0])
        self.assertFalse(z.contains([0], [350])[0])

    def test_manual_zones_are_off_by_default(self):
        self.assertFalse(cfg.USE_MANUAL_NO_FLY_ZONES)
        self.assertEqual(len(ZoneIndex.from_restricted(None, FRAME)), 0)
        layers = {"restricted": None}
        self.assertEqual(len(build_layer_indexes(layers, FRAME)["restricted_zones"]), 0)
        self.assertEqual(len(build_layer_indexes(layers, FRAME, manual_nfz=True)["restricted_zones"]),
                         len(cfg.MANUAL_NO_FLY_ZONES))

    def test_node_inside_a_zone_is_removed(self):
        G = lattice(restricted_zones=ZoneIndex([box(-50, -50, 50, 50)]))     # covers node 12
        self.assertNotIn(12, G)
        self.assertEqual(G.number_of_nodes(), 24)
        self.assertEqual(G.graph["nfz_nodes"], 1)
        self.assertTrue(nx.is_strongly_connected(G))

    def test_edge_crossing_a_zone_is_removed_even_if_both_ends_are_outside(self):
        wall = ZoneIndex([box(95, -300, 105, 300)])           # between columns x = 0 and x = 200
        G = lattice(restricted_zones=wall)
        self.assertEqual(G.graph["nfz_nodes"], 0)
        self.assertEqual(G.number_of_nodes(), 25)
        self.assertFalse(G.has_edge(12, 13) or G.has_edge(13, 12))
        self.assertTrue(G.has_edge(2, 3))                     # row y = −400 passes below the wall
        self.assertGreater(G.graph["nfz_edges"], 0)
        self.assertTrue(nx.is_strongly_connected(G))

    def test_soft_buffer_decays_exponentially_with_distance(self):
        zone = ZoneIndex([box(450, -100, 600, 100)])          # 50 m beyond node 14 (x = 400)
        G = lattice(restricted_zones=zone)
        self.assertAlmostEqual(G[13][14]["nfz"], math.exp(-50 / cfg.NFZ_SOFT_LAMBDA_M), places=4)
        self.assertGreater(G[13][14]["nfz"], G[12][13]["nfz"])           # nearer edge, higher term
        self.assertGreater(G[13][14]["risk_components"]["nfz"], 0.0)
        self.assertEqual(G[0][1]["nfz"], 0.0)                            # beyond 4 λ
        self.assertEqual(G.graph["n_restricted_zones"], 1)

    def test_no_zones_means_no_nfz_risk(self):
        for _, _, d in lattice().edges(data=True):
            self.assertEqual(d["nfz"], 0.0)

    def test_zone_outlines_are_stored_for_plotting(self):
        G = lattice(restricted_zones=ZoneIndex([box(-50, -50, 50, 50)]))
        self.assertEqual(len(G.graph["nfz_outlines"]), 1)
        self.assertEqual(G.graph["nfz_outlines"][0].shape[1], 2)


@needs_geo
class WaterTests(unittest.TestCase):
    def test_water_term_is_the_fraction_of_samples_over_water(self):
        pond = ZoneIndex([box(0, -50, 100, 50)])               # first half of edge 12 → 13
        G = lattice(water_zones=pond)
        self.assertAlmostEqual(G[12][13]["water"], 5 / 9, places=4)      # samples at x = 0 … 100
        self.assertEqual(G[0][1]["water"], 0.0)
        self.assertGreater(G[12][13]["risk_components"]["water"], 0.0)

    def test_coastline_puts_the_sea_on_the_right_of_the_way(self):
        north = sea_polygons(LineString([(0, -1000), (0, 1000)]))[0]     # heading north: sea east
        self.assertTrue(north.contains(__import__("shapely").geometry.Point(500, 0)))
        self.assertFalse(north.contains(__import__("shapely").geometry.Point(-500, 0)))
        south = sea_polygons(LineString([(0, 1000), (0, -1000)]))[0]     # heading south: sea west
        self.assertTrue(south.contains(__import__("shapely").geometry.Point(-500, 0)))

    def test_water_layer_mixes_polygons_waterways_and_coast(self):
        rows = [
            {"natural": "water", "geometry": __import__("shapely").geometry.box(*np.add(
                [-0.0005, -0.0005, 0.0005, 0.0005], [cfg.CENTER_LON, cfg.CENTER_LAT] * 2))},
            {"waterway": "canal", "geometry": LineString([(cfg.CENTER_LON, cfg.CENTER_LAT + 0.01),
                                                          (cfg.CENTER_LON + 0.01, cfg.CENTER_LAT + 0.01)])},
        ]
        z = ZoneIndex.from_water(gpd.GeoDataFrame(rows, crs="EPSG:4326"), FRAME)
        self.assertEqual(len(z), 2)
        self.assertTrue(z.contains([0], [0])[0])                                # the pond
        self.assertTrue(z.contains([500], [FRAME.to_local(cfg.CENTER_LAT + 0.01, 0)[1] + 5])[0])  # canal

    def test_wavy_coastline_handles_a_split_offset_curve(self):
        # Real Chennai coastline triggered a MultiLineString at the 10 km offset.
        line = LineString([(i % 2, i * 100) for i in range(10)])
        self.assertEqual(line.offset_curve(-10000).geom_type, "MultiLineString")
        sea = sea_polygons(line, offset_m=10000)
        point = __import__("shapely").geometry.Point
        self.assertTrue(sea[0].is_valid)
        self.assertTrue(sea[0].contains(point(500, 450)))
        self.assertFalse(sea[0].contains(point(-500, 450)))


class PopulationTests(unittest.TestCase):
    def test_class_lookup_order(self):
        cls = lambda **tags: population_class(tags)
        self.assertEqual(cls(amenity="school"), 1.0)
        self.assertEqual(cls(amenity="hospital", landuse="residential"), 1.0)
        self.assertEqual(cls(landuse="commercial"), 0.7)
        self.assertEqual(cls(landuse="retail"), 0.7)
        self.assertEqual(cls(highway="primary"), 0.6)
        self.assertEqual(cls(highway="residential"), 0.0)           # minor roads are not major roads
        self.assertEqual(cls(landuse="residential"), 0.5)
        self.assertEqual(cls(landuse="industrial"), 0.3)
        self.assertEqual(cls(), 0.0)
        self.assertEqual(cls(amenity=float("nan")), 0.0)

    @needs_geo
    def test_edge_term_is_the_mean_class_along_the_edge(self):
        half = ClassIndex([box(-1000, -1000, 100, 1000)], [0.8])           # covers x ≤ 100
        G = lattice(population_index=half)
        self.assertAlmostEqual(G[12][13]["population"], 0.8 * 5 / 9, places=4)   # 5 of 9 samples
        self.assertAlmostEqual(G[10][11]["population"], 0.8, places=4)           # fully inside
        self.assertAlmostEqual(G[10][11]["risk_components"]["population"], cfg.RISK_WEIGHTS["population"] * 0.8,
                               places=4)
        self.assertEqual(G[13][14]["population"], 0.0)

    @needs_geo
    def test_highest_class_wins_where_features_overlap(self):
        idx = ClassIndex([box(-100, -100, 100, 100), box(-10, -10, 10, 10)], [0.5, 1.0])
        self.assertEqual(list(idx.class_at([0, 50, 500], [0, 50, 500])), [1.0, 0.5, 0.0])

    @needs_geo
    def test_point_and_line_features_are_widened(self):
        rows = [{"amenity": "school", "geometry": __import__("shapely").geometry.Point(cfg.CENTER_LON, cfg.CENTER_LAT)},
                {"highway": "primary", "geometry": LineString([(cfg.CENTER_LON - .01, cfg.CENTER_LAT + .005),
                                                               (cfg.CENTER_LON + .01, cfg.CENTER_LAT + .005)])},
                {"landuse": "meadow", "geometry": __import__("shapely").geometry.Point(0, 0)}]
        idx = ClassIndex.from_gdf(gpd.GeoDataFrame(rows, crs="EPSG:4326"), FRAME)
        self.assertEqual(len(idx), 2)                                       # meadow has no class
        self.assertEqual(idx.class_at([cfg.POI_BUFFER_M - 5], [0])[0], 1.0)
        self.assertEqual(idx.class_at([cfg.POI_BUFFER_M + 5], [0])[0], 0.0)


@needs_geo
class DensityTests(unittest.TestCase):
    def test_cell_density_and_area_weighted_height(self):
        idx = BuildingIndex([box(-50, -50, 50, 50), box(60, -50, 100, 50)], [20.0, 40.0])
        density, mean_h = idx.cell_density_height([0], [0], half_m=100)
        self.assertAlmostEqual(density[0], (10000 + 4000) / 40000)
        self.assertAlmostEqual(mean_h[0], (10000 * 20 + 4000 * 40) / 14000)
        self.assertEqual(float(idx.cell_density_height([1000], [1000], 100)[0][0]), 0.0)

    def test_comm_and_density_terms_follow_the_buildings(self):
        idx = BuildingIndex([box(-50, -50, 50, 50)], [30.0])       # fills 25 % of node 12's cell
        G = lattice(building_index=idx)
        self.assertAlmostEqual(G.nodes[12]["bldg_density"], 0.25, places=4)
        self.assertAlmostEqual(G.nodes[12]["bldg_mean_h_m"], 30.0, places=2)
        edge = G[12][13]                                           # mean of node 12 and (empty) node 13
        dens = min(1.0, 0.125 / cfg.DENSITY_REF)
        self.assertAlmostEqual(edge["density"], dens, places=4)
        self.assertAlmostEqual(edge["comm"], dens * min(1.0, 15.0 / cfg.COMM_HEIGHT_REF_M), places=4)
        self.assertEqual(G[0][1]["comm"], 0.0)

    def test_turbulence_appears_only_with_wind_over_buildings(self):
        idx = BuildingIndex([box(-50, -50, 50, 50)], [30.0])
        windy = WeatherSnapshot(wind_speed_kmh=30, wind_dir_from_deg=90, gust_kmh=30)
        G = lattice(weather=windy, building_index=idx)
        self.assertGreater(G[12][13]["risk_components"]["turbulence"], 0.0)
        self.assertEqual(G[0][1]["risk_components"]["turbulence"], 0.0)
        self.assertEqual(lattice(building_index=idx)[12][13]["risk_components"]["turbulence"], 0.0)


# ── full fixture environment and Tier 1 on it ───────────────────────────────

@needs_geo
class FixtureLatticeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with tempfile.TemporaryDirectory() as tmp:
            layers = fetch_osm_layers(cache_dir=tmp, offline=True, fixture_dir=str(FIXTURES))
        cls.layers = layers
        cls.G = LatticeAirspaceBuilder(weather=WeatherSnapshot(wind_speed_kmh=25, wind_dir_from_deg=225,
                                                               gust_kmh=30, precipitation_mm=2.0,
                                                               visibility_m=8000),
                                       **build_layer_indexes(layers, FRAME)).fetch_graph()

    def test_all_four_fixture_layers_load(self):
        for name in ("buildings", "restricted", "water", "landuse"):
            self.assertGreater(len(self.layers[name]), 0, name)

    def test_fixture_zone_cuts_a_hole_and_every_component_shows_up(self):
        G = self.G
        self.assertGreater(G.graph["nfz_nodes"], 0)
        self.assertTrue(nx.is_strongly_connected(G))
        totals = {k: 0.0 for k in cfg.RISK_WEIGHTS}
        for _, _, d in G.edges(data=True):
            for k, v in d["risk_components"].items():
                totals[k] += v
        # `obstacle` is synthetic-only; the others must contribute on the fixture lattice
        for k in set(cfg.RISK_WEIGHTS) - {"obstacle"}:
            self.assertGreater(totals[k], 0.0, k)
        self.assertEqual(totals["obstacle"], 0.0)

    def test_rho_is_bounded_and_exposure_scales_with_length(self):
        for _, _, d in self.G.edges(data=True):
            self.assertLessEqual(d["rho"], 1.0)
            self.assertAlmostEqual(d["risk"], d["rho"] * d["d_h"] / 0.2, places=4)

    def test_tier1_finds_a_front_on_the_fixture_lattice(self):
        G = self.G
        s, t = min(G.nodes, key=lambda n: G.nodes[n]["x_m"] + G.nodes[n]["y_m"]), \
               max(G.nodes, key=lambda n: G.nodes[n]["x_m"] + G.nodes[n]["y_m"])
        front = TierOneEngine(G, s, t, max_explored=30_000).run()
        self.assertGreater(len(front), 1)
        self.assertEqual(len({tuple(round(x, 6) for x in g) for g, _ in front}), len(front))


if __name__ == "__main__":
    unittest.main()
