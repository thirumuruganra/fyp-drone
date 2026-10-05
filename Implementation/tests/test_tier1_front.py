import sys
import unittest
from pathlib import Path

IMPLEMENTATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IMPLEMENTATION_DIR))

from env_data import WeatherSnapshot
from module1_graph_builder import LatticeAirspaceBuilder
from module2_tier1_moastar import TierOneEngine, pareto_dominates, weakly_dominates


class WeakDominanceTests(unittest.TestCase):
    def test_equal_vectors_weakly_dominate_but_do_not_strictly_dominate(self):
        g = [1.0, 2.0, 3.0]
        self.assertTrue(weakly_dominates(g, list(g)))
        self.assertFalse(pareto_dominates(g, list(g)))

    def test_float_noise_is_tolerated(self):
        self.assertTrue(weakly_dominates([1.0 + 1e-12, 2.0, 3.0], [1.0, 2.0, 3.0]))

    def test_worse_in_one_objective_is_not_dominated(self):
        self.assertFalse(weakly_dominates([1.0, 2.0, 3.5], [1.0, 2.0, 3.0]))


class LatticeFrontTests(unittest.TestCase):
    def test_uniform_wind_lattice_search_terminates_without_duplicates(self):
        """Equal-cost permutations of the same moves used to explode the label set."""
        w = WeatherSnapshot(wind_speed_kmh=30, wind_dir_from_deg=90, gust_kmh=30)
        G = LatticeAirspaceBuilder(box_m=2000, weather=w).fetch_graph()
        engine = TierOneEngine(G, 0, G.number_of_nodes() - 1, max_explored=20_000)
        front = engine.run()
        self.assertGreaterEqual(len(front), 1)
        vectors = [tuple(round(x, 6) for x in g) for g, _ in front]
        self.assertEqual(len(vectors), len(set(vectors)))
        for g, path in front:
            self.assertEqual((path[0], path[-1]), (0, G.number_of_nodes() - 1))
            self.assertEqual(len(path), len(set(path)))


if __name__ == "__main__":
    unittest.main()
