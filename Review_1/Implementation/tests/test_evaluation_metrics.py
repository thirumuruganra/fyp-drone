import sys
import unittest
from pathlib import Path

import networkx as nx
import numpy as np


IMPLEMENTATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IMPLEMENTATION_DIR))

from module3_baselines import is_valid_route, route_cost, single_objective_astar
from module5_evaluation import (
    build_reference_front,
    hypervolume,
    igd,
    nondominated_points,
    spread_delta,
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


if __name__ == "__main__":
    unittest.main()
