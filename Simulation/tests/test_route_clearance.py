import io
import math
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_test_helpers import CENTER, make_bundle  # noqa: E402

import networkx as nx  # noqa: E402
from shapely.geometry import box  # noqa: E402

from env_data import ElevationGrid, LocalFrame, WeatherSnapshot  # noqa: E402
from route_clearance import (ClearanceChecker, NoFeasibleMission, RoofIndex,  # noqa: E402
                             check_polyline, connect_pad)
from sim_region import RegionBundle  # noqa: E402
import run_sim_mission  # noqa: E402

FRAME = LocalFrame(*CENTER)


def grid(fn, half=600.0, step=10.0):
    """ElevationGrid around CENTER from fn(x, y) in local metres."""
    lat_s, lon_w = FRAME.to_latlon(-half, -half)
    dlat, dlon = step / FRAME.m_per_deg_lat, step / FRAME.m_per_deg_lon
    n = int(2 * half / step) + 1
    vals = [[fn(-half + j * step, -half + i * step) for j in range(n)] for i in range(n)]
    return ElevationGrid(float(lat_s), float(lon_w), dlat, dlon, vals)


FLAT = grid(lambda x, y: 0.0)


def checker(buildings=(), terrain=FLAT, buffer_m=5.0, **kw):
    """buildings: [(xmin, ymin, xmax, ymax, roof_amsl)]."""
    idx = RoofIndex([box(*b[:4]) for b in buildings], [b[4] for b in buildings], buffer_m)
    return ClearanceChecker(idx, terrain, FRAME, **kw)


class SegmentTests(unittest.TestCase):
    def test_narrow_tall_building_between_safe_nodes_is_rejected(self):
        c = checker([(48, -1, 52, 1, 70.0)])           # 4 m × 2 m, roof 70 m — missed by 25 m sampling
        r = c.segment((0, 0), (100, 0), 75.0, 75.0)
        self.assertFalse(r.ok)
        self.assertAlmostEqual(r.min_roof_clearance_m, 5.0)
        self.assertTrue(c.segment((0, 0), (100, 0), 90.0, 90.0).ok)

    def test_building_beside_the_path_outside_the_buffer_is_ignored(self):
        c = checker([(40, 10, 60, 30, 200.0)])
        r = c.segment((0, 0), (100, 0), 60.0, 60.0)
        self.assertTrue(r.ok)
        self.assertEqual(r.min_roof_clearance_m, math.inf)

    def test_buffer_catches_a_near_miss(self):
        c = checker([(40, 3, 60, 30, 200.0)])            # 3 m from the line, buffer 5 m
        self.assertFalse(c.segment((0, 0), (100, 0), 60.0, 60.0).ok)

    def test_diagonal_edge_clipping_a_roof_corner_is_rejected(self):
        c = checker([(53, 43, 59, 49, 70.0)], buffer_m=5.0)
        self.assertFalse(c.segment((0, 0), (100, 100), 60.0, 60.0).ok)
        c0 = checker([(53, 43, 59, 49, 70.0)], buffer_m=0.0)
        self.assertTrue(c0.segment((0, 0), (100, 100), 60.0, 60.0).ok)   # footprint itself is missed

    def test_climbing_segment_uses_altitude_over_the_footprint(self):
        near_start = checker([(5, -2, 15, 2, 60.0)])
        near_end = checker([(85, -2, 95, 2, 60.0)])
        self.assertFalse(near_start.segment((0, 0), (100, 0), 60.0, 120.0).ok)
        self.assertTrue(near_end.segment((0, 0), (100, 0), 60.0, 120.0).ok)

    def test_ridge_between_safe_endpoints_is_rejected(self):
        ridge = grid(lambda x, y: 60.0 * max(0.0, 1.0 - abs(x - 50.0) / 20.0))
        c = checker(terrain=ridge)
        r = c.segment((0, 0), (100, 0), 70.0, 70.0)
        self.assertFalse(r.ok)
        self.assertLess(r.min_terrain_clearance_m, 15.0)

    def test_ceiling_over_a_valley(self):
        valley = grid(lambda x, y: 50.0 - 50.0 * max(0.0, 1.0 - abs(x - 50.0) / 20.0))
        c = checker(terrain=valley)
        r = c.segment((0, 0), (100, 0), 160.0, 160.0)       # 110 m AGL at the ends, 160 m mid-valley
        self.assertFalse(r.ok)
        self.assertGreater(r.max_agl_m, 120.0)

    def test_edge_filter_counts(self):
        c = checker([(48, -1, 52, 1, 70.0)])
        a = {"x_m": 0.0, "y_m": 0.0, "alt_amsl_m": 60.0}
        b = {"x_m": 100.0, "y_m": 0.0, "alt_amsl_m": 60.0}
        d = {"x_m": 0.0, "y_m": 100.0, "alt_amsl_m": 60.0}
        self.assertFalse(c.edge_filter(a, b))
        self.assertTrue(c.edge_filter(a, d))
        self.assertEqual((c.n_checked, c.n_rejected), (2, 1))

    def test_check_polyline(self):
        c = checker([(48, -1, 52, 1, 70.0)])
        legs = check_polyline(c, [(0, 0), (0, 100), (100, 0)], [60.0, 60.0, 60.0])
        self.assertEqual([r.ok for r in legs], [True, True])
        legs = check_polyline(c, [(0, 0), (100, 0)], [60.0, 60.0])
        self.assertFalse(legs[0].ok)


class PadTests(unittest.TestCase):
    def graph(self):
        G = nx.DiGraph()
        for n, (x, y) in enumerate([(0, 0), (100, 0), (0, 100)]):
            G.add_node(n, x_m=float(x), y_m=float(y), alt_amsl_m=60.0)
        return G

    def pad(self, x, y):
        lat, lon = FRAME.to_latlon(x, y)
        return {"name": "p", "lat": float(lat), "lon": float(lon), "ground_amsl_m": 0.0}

    def test_nearest_clear_node(self):
        conn = connect_pad(self.graph(), self.pad(10, 5), checker())
        self.assertEqual(conn.node, 0)
        self.assertAlmostEqual(conn.distance_m, math.hypot(10, 5), places=3)

    def test_blocked_leg_falls_back_to_the_next_node(self):
        c = checker([(3, -20, 4, 20, 70.0)])               # wall between pad (10, 5) and node 0, 6 m away
        conn = connect_pad(self.graph(), self.pad(10, 5), c)
        self.assertEqual(conn.node, 1)

    def test_pad_on_a_building_is_infeasible(self):
        c = checker([(5, 0, 15, 10, 30.0)])
        with self.assertRaises(NoFeasibleMission):
            connect_pad(self.graph(), self.pad(10, 5), c)

    def test_no_node_in_range(self):
        with self.assertRaises(NoFeasibleMission):
            connect_pad(self.graph(), self.pad(500, 500), checker())


class LatticeIntegrationTests(unittest.TestCase):
    """A 70 m tower inside one cell, clipped only by the diagonal between two OTHER cells' nodes."""

    def test_hard_clearance_removes_the_unsafe_diagonal(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_bundle(tmp, "lat", box_m=400.0, depot_xy=(-150, -150), dests_xy=[(150, 150)],
                               buildings=[(3, -7, 9, -1, {"height": "70"})])
            bundle = RegionBundle(root)
            with redirect_stdout(io.StringIO()):
                G, chk, _ = run_sim_mission.build_graph(bundle, 100.0, WeatherSnapshot.calm())
            node = {(round(d["x_m"]), round(d["y_m"])): n for n, d in G.nodes(data=True)}
            a, b = node[(-50, -50)], node[(50, 50)]
            self.assertEqual(G.nodes[a]["agl_m"], 60.0)         # tower is not in either endpoint's cell
            self.assertFalse(G.has_edge(a, b))
            self.assertFalse(G.has_edge(b, a))
            self.assertTrue(G.has_edge(node[(-50, 50)], node[(50, -50)]) or
                            G.nodes[node[(50, -50)]]["agl_m"] > 60.0)
            self.assertGreaterEqual(chk.n_rejected, 2)
            self.assertEqual(G.graph["bundle_hash"], bundle.bundle_hash)
            # every kept edge passes the same check
            for u, v in G.edges:
                self.assertTrue(chk.segment((G.nodes[u]["x_m"], G.nodes[u]["y_m"]),
                                            (G.nodes[v]["x_m"], G.nodes[v]["y_m"]),
                                            G.nodes[u]["alt_amsl_m"], G.nodes[v]["alt_amsl_m"]).ok)


if __name__ == "__main__":
    unittest.main()
