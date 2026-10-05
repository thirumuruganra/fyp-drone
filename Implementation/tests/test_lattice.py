import math
import pickle
import sys
import unittest
from pathlib import Path

IMPLEMENTATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IMPLEMENTATION_DIR))

import networkx as nx

import config as cfg
from env_data import WeatherSnapshot
from module1_graph_builder import (LatticeAirspaceBuilder, haversine_km,
                                   initial_bearing_deg)


def build(box_m=1000.0, spacing_m=200.0, **kw):
    kw.setdefault("weather", WeatherSnapshot.calm())
    return LatticeAirspaceBuilder(box_m=box_m, spacing_m=spacing_m, **kw).fetch_graph()


class GeometryTests(unittest.TestCase):
    def test_haversine_one_degree_of_latitude(self):
        self.assertAlmostEqual(haversine_km(12.0, 80.0, 13.0, 80.0), 111.195, delta=0.05)

    def test_bearing_cardinal_directions(self):
        self.assertAlmostEqual(initial_bearing_deg(12.98, 80.25, 12.99, 80.25), 0.0, places=3)
        self.assertAlmostEqual(initial_bearing_deg(12.98, 80.25, 12.98, 80.26), 90.0, delta=0.01)
        self.assertAlmostEqual(initial_bearing_deg(12.98, 80.25, 12.97, 80.25), 180.0, places=3)

    def test_bearing_applies_cos_lat_to_longitude(self):
        # 0.01° east and 0.01° north at 13°N is NOT 45°: east is shorter by cos(lat)
        brg = initial_bearing_deg(13.0, 80.0, 13.01, 80.01)
        expected = math.degrees(math.atan2(0.01 * math.cos(math.radians(13.005)), 0.01))
        self.assertAlmostEqual(brg, expected, delta=0.05)
        self.assertLess(brg, 45.0)


class LatticeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.G = build(box_m=1000.0, spacing_m=200.0)    # 5 × 5

    def test_node_count_and_attributes(self):
        self.assertEqual(self.G.number_of_nodes(), 25)
        for _, d in self.G.nodes(data=True):
            for key in ("x", "y", "lat", "lon", "x_m", "y_m"):
                self.assertIn(key, d)
            self.assertEqual((d["x"], d["y"]), (d["lon"], d["lat"]))

    def test_centred_on_region_centre(self):
        lats = [d["lat"] for _, d in self.G.nodes(data=True)]
        lons = [d["lon"] for _, d in self.G.nodes(data=True)]
        self.assertAlmostEqual((min(lats) + max(lats)) / 2, cfg.CENTER_LAT, places=6)
        self.assertAlmostEqual((min(lons) + max(lons)) / 2, cfg.CENTER_LON, places=6)

    def test_eight_neighbour_edges_both_directions(self):
        self.assertEqual(self.G.number_of_edges(), 2 * (2 * 5 * 4 + 2 * 4 * 4))
        centre = 12
        self.assertEqual(self.G.out_degree(centre), 8)
        self.assertEqual(self.G.in_degree(centre), 8)
        self.assertEqual(self.G.out_degree(0), 3)
        for u, v in self.G.edges():
            self.assertTrue(self.G.has_edge(v, u))

    def test_edge_lengths_match_spacing(self):
        for u, v, d in self.G.edges(data=True):
            dx = self.G.nodes[v]["x_m"] - self.G.nodes[u]["x_m"]
            dy = self.G.nodes[v]["y_m"] - self.G.nodes[u]["y_m"]
            self.assertAlmostEqual(d["d_h"] * 1000.0, math.hypot(dx, dy), delta=0.5)
            self.assertEqual(d["distance"], d["d_h"])          # flat lattice

    def test_bearings_are_compass_headings(self):
        east = self.G[12][13]["bearing"]
        north = self.G[12][17]["bearing"]
        self.assertAlmostEqual(east, 90.0, delta=0.05)
        self.assertAlmostEqual(north, 0.0, delta=0.05)
        self.assertAlmostEqual((self.G[13][12]["bearing"] - east) % 360, 180.0, delta=0.1)

    def test_edges_carry_costs_and_time(self):
        for _, _, d in self.G.edges(data=True):
            self.assertEqual(d["cost"], [d["distance"], d["energy"], d["risk"]])
            self.assertGreater(d["time_s"], 0.0)
            self.assertEqual(set(d["risk_components"]), set(cfg.RISK_WEIGHTS))

    def test_strongly_connected_and_pickleable(self):
        self.assertTrue(nx.is_strongly_connected(self.G))
        clone = pickle.loads(pickle.dumps(self.G))
        self.assertEqual(clone.number_of_edges(), self.G.number_of_edges())

    def test_default_lattice_is_about_900_nodes(self):
        G = LatticeAirspaceBuilder(weather=WeatherSnapshot.calm()).fetch_graph()
        self.assertEqual(G.number_of_nodes(), 900)


class LatticeWindTests(unittest.TestCase):
    def test_eastbound_edges_cost_more_than_westbound_in_easterly_wind(self):
        G = build(wind_override=(30.0, 90.0), offline=True)
        east, west = G[12][13], G[13][12]
        self.assertAlmostEqual(east["distance"], west["distance"], places=4)
        self.assertGreater(east["energy"], west["energy"])
        self.assertAlmostEqual(west["energy"], west["distance"], places=4)   # tailwind: no benefit

    def test_calm_lattice_has_energy_equal_distance_and_zero_risk(self):
        G = build()
        for _, _, d in G.edges(data=True):
            self.assertAlmostEqual(d["energy"], d["distance"], places=4)
            self.assertEqual(d["risk"], 0.0)

    def test_northbound_edges_carry_crosswind_risk_in_easterly_wind(self):
        G = build(wind_override=(30.0, 90.0), offline=True)
        self.assertGreater(G[12][17]["risk"], 0.0)       # crosswind
        self.assertAlmostEqual(G[12][13]["risk"], 0.0, places=4)   # headwind only

    def test_weather_recorded_on_graph(self):
        G = build(wind_override=(30.0, 90.0), offline=True)
        self.assertEqual(G.graph["weather"]["wind_speed_kmh"], 30.0)
        self.assertEqual(G.graph["weather"]["wind_dir_from_deg"], 90.0)


if __name__ == "__main__":
    unittest.main()
