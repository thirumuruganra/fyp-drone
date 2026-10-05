import sys
import unittest
from pathlib import Path

import networkx as nx
import numpy as np


IMPLEMENTATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IMPLEMENTATION_DIR))

from module3_baselines import is_valid_route, route_cost, single_objective_astar, weighted_sum_astar
from module5_evaluation import (
    build_reference_front,
    hypervolume,
    igd,
    payload_alignment_score,
    risk_component_shares,
    nondominated_points,
    spread_delta,
    tier2_sweep,
)


class EvaluationMetricTests(unittest.TestCase):
    def test_nondominated_points_filters_dominated(self):
        points = np.array([
            [1.0, 1.0, 1.0],
            [2.0, 2.0, 2.0],
            [0.5, 2.0, 2.0],
        ])
        front = nondominated_points(points)
        self.assertEqual(len(front), 2)
        self.assertFalse(any(np.allclose(p, [2.0, 2.0, 2.0]) for p in front))

    def test_hypervolume_single_point(self):
        points = np.array([[0.2, 0.3, 0.4]])
        hv = hypervolume(points, np.array([1.0, 1.0, 1.0]))
        self.assertAlmostEqual(hv, 0.8 * 0.7 * 0.6)

    def test_igd_zero_for_same_front(self):
        front = np.array([[0.0, 1.0, 1.0], [1.0, 0.0, 1.0]])
        self.assertAlmostEqual(igd(front, front), 0.0)

    def test_spread_delta_even_spacing_zero(self):
        points = np.array([[0.0, 1.0, 1.0], [0.5, 0.5, 0.5], [1.0, 0.0, 0.0]])
        self.assertAlmostEqual(spread_delta(points), 0.0)

    def test_reference_front_from_method_union(self):
        routes = {
            "a": [([1.0, 1.0, 1.0], ["a"]), ([2.0, 2.0, 2.0], ["b"])],
            "b": [([0.5, 2.0, 1.0], ["c"])],
        }
        ref = build_reference_front(routes)
        self.assertEqual(len(ref), 2)


class BaselineBehaviorTests(unittest.TestCase):
    def _graph(self):
        G = nx.DiGraph()
        G.add_node("s", x=0.0, y=0.0)
        G.add_node("a", x=1.0, y=0.0)
        G.add_node("b", x=0.0, y=1.0)
        G.add_node("t", x=1.0, y=1.0)
        G.add_edge("s", "a", distance=1.0, energy=5.0, risk=5.0)
        G.add_edge("a", "t", distance=1.0, energy=5.0, risk=5.0)
        G.add_edge("s", "b", distance=2.0, energy=1.0, risk=1.0)
        G.add_edge("b", "t", distance=2.0, energy=1.0, risk=1.0)
        return G

    def test_single_objective_astar_distance(self):
        G = self._graph()
        result = single_objective_astar(G, "s", "t", "distance")
        cost, path = result.routes[0]
        self.assertEqual(path, ["s", "a", "t"])
        self.assertEqual(cost, [2.0, 10.0, 10.0])
        self.assertTrue(is_valid_route(G, path, "s", "t"))

    def test_route_cost_rejects_invalid_edge(self):
        G = self._graph()
        with self.assertRaises(ValueError):
            route_cost(G, ["s", "t"])

    def test_weighted_sum_astar_returns_valid_route(self):
        G = self._graph()
        result = weighted_sum_astar(
            G,
            "s",
            "t",
            {"distance": 0.1, "energy": 0.8, "risk": 0.1},
            scales=np.array([1.0, 1.0, 1.0]),
        )
        cost, path = result.routes[0]
        self.assertTrue(is_valid_route(G, path, "s", "t"))
        self.assertEqual(cost, route_cost(G, path))


class Tier2EvidenceTests(unittest.TestCase):
    def test_payload_alignment_score_known_ranks(self):
        score = payload_alignment_score(
            {"distance": 0.5, "energy": 0.25, "risk": 0.25},
            {"distance": 1, "energy": 2, "risk": 3},
            front_size=3,
        )
        self.assertAlmostEqual(score, 0.625)

    def test_tier2_sweep_has_225_rows(self):
        front = [
            ([1.0, 10.0, 10.0], [0, 1]),
            ([10.0, 1.0, 10.0], [0, 2]),
            ([10.0, 10.0, 1.0], [0, 3]),
            ([4.0, 4.0, 4.0], [0, 4]),
        ]
        rows = tier2_sweep(front)
        self.assertEqual(len(rows), 225)
        self.assertTrue(all("payload_alignment" in row for row in rows))

    def test_risk_component_shares_sum_to_one_per_route(self):
        G = nx.DiGraph()
        G.add_edge("s", "a", risk=2.0, risk_components={"building": 1.0, "wind": 1.0})
        G.add_edge("a", "t", risk=1.0, risk_components={"building": 1.0, "wind": 3.0})
        rows = risk_component_shares(G, [("route", ([0.0, 0.0, 3.0], ["s", "a", "t"]))])
        self.assertAlmostEqual(sum(row["share"] for row in rows), 1.0)
        self.assertEqual({row["component"] for row in rows}, {"building", "wind"})


if __name__ == "__main__":
    unittest.main()
