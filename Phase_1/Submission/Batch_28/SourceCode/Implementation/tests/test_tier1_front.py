import itertools
import io
import random
import sys
import unittest
from contextlib import redirect_stdout
from pathlib import Path

IMPLEMENTATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IMPLEMENTATION_DIR))

import networkx as nx

from env_data import WeatherSnapshot
from module1_graph_builder import AirspaceGraphBuilder, LatticeAirspaceBuilder, haversine_km
from module2_tier1_moastar import (TierOneEngine, heuristic_vector, pareto_dominates,
                                   thin_front, weakly_dominates)


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


# ── exactness against brute force ───────────────────────────────────────────

def random_graph(seed: int, n: int = 12, extra_edges: int = 14, zero_risk: bool = False) -> nx.DiGraph:
    """Small geographic graph whose edge costs respect distance ≥ d_h and energy ≥ distance."""
    rng = random.Random(seed)
    G = nx.DiGraph()
    for i in range(n):
        lat, lon = 12.98 + rng.uniform(0, 0.02), 80.25 + rng.uniform(0, 0.02)
        G.add_node(i, x=lon, y=lat, lat=lat, lon=lon)

    def add(u, v):
        d_h = haversine_km(G.nodes[u]["y"], G.nodes[u]["x"], G.nodes[v]["y"], G.nodes[v]["x"])
        dist = d_h * (1.0 + rng.uniform(0.0, 0.5))
        energy = dist * (1.0 + rng.uniform(0.0, 0.6))
        risk = 0.0 if zero_risk else rng.uniform(0.0, 1.0)
        G.add_edge(u, v, distance=dist, energy=energy, risk=risk)

    for i in range(n - 1):                       # a path guarantees reachability
        add(i, i + 1)
        add(i + 1, i)
    for _ in range(extra_edges):
        u, v = rng.sample(range(n), 2)
        if not G.has_edge(u, v):
            add(u, v)
    return G


def brute_force_front(G, s, t) -> set:
    """Non-dominated cost vectors over every simple path s → t."""
    vecs = []
    for path in nx.all_simple_paths(G, s, t):
        vecs.append(tuple(sum(G[u][v][k] for u, v in zip(path[:-1], path[1:]))
                          for k in ("distance", "energy", "risk")))
    front = [v for v in set(vecs) if not any(pareto_dominates(o, v) for o in vecs)]
    return {tuple(round(x, 9) for x in v) for v in front}


def quiet_run(G, s, t, **kw):
    engine = TierOneEngine(G, s, t, **kw)
    with redirect_stdout(io.StringIO()):
        engine.run()
    return engine


class ExactnessTests(unittest.TestCase):
    def test_matches_brute_force_on_random_graphs(self):
        sizes = []
        for seed in range(12):
            for zero_risk in (False, True):
                G = random_graph(seed, extra_edges=45, zero_risk=zero_risk)
                engine = quiet_run(G, 0, 11)
                got = {tuple(round(x, 9) for x in g) for g, _ in engine.pareto_front}
                self.assertEqual(got, brute_force_front(G, 0, 11), f"seed={seed} zero_risk={zero_risk}")
                self.assertFalse(engine.truncated)
                sizes.append(len(got))
        self.assertGreaterEqual(max(sizes), 5)               # the comparison is not vacuous
        self.assertGreaterEqual(sum(n > 1 for n in sizes), 10)

    def test_front_routes_are_mutually_non_dominated_and_unique(self):
        for seed in range(5):
            front = quiet_run(random_graph(seed, extra_edges=45), 0, 11).pareto_front
            for (g1, _), (g2, _) in itertools.permutations(front, 2):
                self.assertFalse(pareto_dominates(g1, g2))
                self.assertFalse(weakly_dominates(g1, g2))
            self.assertGreater(len(front), 0)

    def test_every_route_is_a_valid_simple_path_with_matching_cost(self):
        for G, s, t in ((random_graph(3), 0, 11),
                        (AirspaceGraphBuilder(seed=5)._build_synthetic_graph(36), 0, 35)):
            for g, path in quiet_run(G, s, t).pareto_front:
                self.assertEqual((path[0], path[-1]), (s, t))
                self.assertEqual(len(path), len(set(path)))
                self.assertTrue(all(G.has_edge(u, v) for u, v in zip(path[:-1], path[1:])))
                cost = [sum(G[u][v][k] for u, v in zip(path[:-1], path[1:]))
                        for k in ("distance", "energy", "risk")]
                for a, b in zip(g, cost):
                    self.assertAlmostEqual(a, b, places=9)

    def test_front_is_sorted_and_deterministic(self):
        G = random_graph(2)
        a, b = quiet_run(G, 0, 11).pareto_front, quiet_run(G, 0, 11).pareto_front
        self.assertEqual(a, b)
        self.assertEqual([g for g, _ in a], sorted(g for g, _ in a))

    def test_source_equal_to_target_is_a_trivial_front(self):
        self.assertEqual(quiet_run(random_graph(1), 4, 4).pareto_front, [([0.0, 0.0, 0.0], [4])])

    def test_unreachable_target_raises(self):
        G = random_graph(1)
        G.add_node(99, x=80.3, y=13.0, lat=13.0, lon=80.3)
        with self.assertRaises(ValueError):
            quiet_run(G, 0, 99)


class HeuristicTests(unittest.TestCase):
    def test_vector_is_admissible_in_every_objective(self):
        for seed in range(5):
            G = random_graph(seed)
            t = 11
            for k, idx in (("distance", 0), ("energy", 1), ("risk", 2)):
                best = nx.single_source_dijkstra_path_length(G.reverse(), t, weight=k)
                for u, true_cost in best.items():
                    self.assertLessEqual(heuristic_vector(G, u, t)[idx], true_cost + 1e-9, (seed, k, u))

    def test_vector_form_is_h_h_zero(self):
        G = random_graph(0)
        h = heuristic_vector(G, 0, 5)
        self.assertEqual((h[0], h[2]), (h[1], 0.0))
        self.assertGreater(h[0], 0.0)
        self.assertEqual(heuristic_vector(G, 5, 5), (0.0, 0.0, 0.0))

    def test_heuristic_uses_great_circle_not_degree_scaling(self):
        G = nx.DiGraph()
        G.add_node("a", x=80.0, y=13.0)
        G.add_node("b", x=80.1, y=13.0)                          # 0.1° east at 13°N
        self.assertAlmostEqual(heuristic_vector(G, "a", "b")[0], 10.84, delta=0.05)  # not 11.1


class DiagnosticsTests(unittest.TestCase):
    def test_complete_search_reports_not_truncated(self):
        engine = quiet_run(random_graph(0), 0, 11)
        d = engine.diagnostics()
        self.assertFalse(d["truncated"])
        self.assertGreater(d["expanded"], 0)
        self.assertGreaterEqual(d["generated"], d["expanded"])
        self.assertGreaterEqual(d["runtime_s"], 0.0)
        self.assertEqual(d["front_size"], len(engine.pareto_front))

    def test_small_cap_is_flagged_and_warned(self):
        G = AirspaceGraphBuilder(seed=101)._build_synthetic_graph(50)
        buf = io.StringIO()
        engine = TierOneEngine(G, 0, 49, max_explored=15)
        with redirect_stdout(buf):
            engine.run()
        self.assertTrue(engine.truncated)
        self.assertEqual(engine.expanded, 15)
        self.assertIn("WARNING", buf.getvalue())
        self.assertIn("TRUNCATED", buf.getvalue())

    def test_cap_that_is_not_binding_is_not_truncated(self):
        G = random_graph(0)
        exact = quiet_run(G, 0, 11)
        # a cap equal to the work actually needed must not be reported as truncation
        again = quiet_run(G, 0, 11, max_explored=exact.expanded)
        self.assertFalse(again.truncated)
        self.assertEqual(again.pareto_front, exact.pareto_front)


# ── front thinning ──────────────────────────────────────────────────────────

class ThinFrontTests(unittest.TestCase):
    @staticmethod
    def curved_front(n):
        """n mutually non-dominated points on a convex 3-D surface."""
        pts = []
        for i in range(n):
            a = i / (n - 1)
            pts.append(([1.0 + a, 3.0 - 2.0 * a ** 0.5, 2.0 - a ** 2 + 0.3 * (1 - a)], [i]))
        return pts

    def test_under_the_cap_the_front_is_unchanged(self):
        front = self.curved_front(10)
        self.assertEqual(thin_front(front, 20), front)

    def test_thinned_front_respects_the_cap_keeps_extremes_and_stays_non_dominated(self):
        front = self.curved_front(120)
        for cap in (10, 25, 60):
            thin = thin_front(front, cap)
            self.assertLessEqual(len(thin), cap)
            self.assertGreater(len(thin), 3)
            self.assertTrue(all(r in front for r in thin))
            for k in range(3):
                self.assertEqual(min(g[k] for g, _ in thin), min(g[k] for g, _ in front))
            for (g1, _), (g2, _) in itertools.permutations(thin, 2):
                self.assertFalse(pareto_dominates(g1, g2))

    def test_thinning_is_spread_not_clustered(self):
        front = self.curved_front(200)
        thin = thin_front(front, 20)
        idx = sorted(p[0] for _, p in thin)
        gaps = [b - a for a, b in zip(idx, idx[1:])]
        self.assertLess(max(gaps), 60)                             # no huge hole in the curve

    def test_is_deterministic_and_rejects_tiny_caps(self):
        front = self.curved_front(80)
        self.assertEqual(thin_front(front, 15), thin_front(front, 15))
        with self.assertRaises(ValueError):
            thin_front(front, 2)


if __name__ == "__main__":
    unittest.main()
