import sys
import unittest
from pathlib import Path

import networkx as nx


IMPLEMENTATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IMPLEMENTATION_DIR))

from dashboard import build_route_figure, route_node_ids, select_payload_route  # noqa: E402


class DashboardHelperTests(unittest.TestCase):
    def test_route_node_ids_from_selection_result_path(self):
        class Result:
            path = [{"node": "s"}, {"node": "a"}, {"node": "t"}]

        self.assertEqual(route_node_ids(Result()), ["s", "a", "t"])

    def test_build_route_figure_without_route(self):
        G = nx.DiGraph()
        G.add_node("s", x=0.0, y=0.0)
        G.add_node("t", x=1.0, y=1.0)
        G.add_edge("s", "t")

        fig = build_route_figure(G)

        self.assertEqual(len(fig.axes), 1)

    def test_select_payload_route_returns_hold_or_route(self):
        G = nx.DiGraph()
        G.add_node("s", x=0.0, y=0.0, lat=12.0, lon=80.0, agl_m=40.0)
        G.add_node("t", x=1.0, y=1.0, lat=12.1, lon=80.1, agl_m=45.0)
        G.add_edge("s", "t", distance=1.0, energy=1.0, risk=1.0)
        front = [([1.0, 1.0, 1.0], ["s", "t"])]

        result = select_payload_route(G, front, 3, 1, 2, 3, write_outputs=False)

        self.assertTrue(result.hold_advisory or result.path)

    def test_select_payload_route_holds_on_weather_no_go(self):
        G = nx.DiGraph(no_go_reasons=["thunderstorm (WMO code 95)"])
        G.add_node("s", lat=12.0, lon=80.0, agl_m=40.0)
        G.add_node("t", lat=12.001, lon=80.001, agl_m=45.0)
        front = [([0.1, 0.1, 0.1], ["s", "t"])]

        result = select_payload_route(G, front, 3, 1, 2, 3, write_outputs=False)

        self.assertIn("HOLD", result.hold_advisory)
        self.assertEqual(result.path, [])


if __name__ == "__main__":
    unittest.main()
