import sys
import unittest
from pathlib import Path

import networkx as nx
import numpy as np


IMPLEMENTATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IMPLEMENTATION_DIR))

from module3_baselines import is_valid_route, route_cost, single_objective_astar, weighted_sum_astar
from module5_evaluation import (
    additive_epsilon,
    area_profile,
    build_reference_front,
    evaluate_runs,
    front_robustness,
    gd,
    hypervolume,
    igd,
    igd_plus,
    monotonicity_violations,
    payload_alignment_score,
    reference_share,
    risk_component_shares,
    nondominated_points,
    set_coverage,
    spread_delta,
    summarize_runs,
    tier2_sweep,
    tier2_vs_weighted,
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

    def test_reference_drops_float_noise_ties(self):
        exact = [([7.86329, 9.29768, 16.82034], ["a"])]
        noisy = [([7.86329 - 9e-16, 9.30224, 16.92413], ["b"]),     # beaten, ties on distance
                 ([7.86329 + 1e-15, 9.29768, 16.82034], ["c"])]     # duplicate of the exact point
        ref = build_reference_front({"namoa": exact, "mopso": noisy})
        self.assertEqual(len(ref), 1)
        self.assertAlmostEqual(igd(np.array([exact[0][0]]), ref), 0.0)

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


class ExtendedIndicatorTests(unittest.TestCase):
    REF = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
    MID = np.array([[0.5, 0.5, 0.0]])

    def test_gd_and_igd_plus_known_values(self):
        self.assertAlmostEqual(gd(self.MID, self.REF), np.sqrt(0.5))
        self.assertAlmostEqual(igd(self.MID, self.REF), np.sqrt(0.5))
        self.assertAlmostEqual(igd_plus(self.MID, self.REF), 0.5)

    def test_igd_plus_zero_when_approx_dominates_reference(self):
        self.assertAlmostEqual(igd_plus(np.zeros((1, 3)), self.REF), 0.0)

    def test_additive_epsilon(self):
        self.assertAlmostEqual(additive_epsilon(self.MID, self.REF), 0.5)
        self.assertAlmostEqual(additive_epsilon(self.REF, self.REF), 0.0)

    def test_set_coverage(self):
        self.assertAlmostEqual(set_coverage(self.MID, self.REF), 0.0)
        self.assertAlmostEqual(set_coverage(np.zeros((1, 3)), self.REF), 1.0)
        self.assertAlmostEqual(set_coverage(self.REF, self.REF), 1.0)

    def test_reference_share(self):
        self.assertAlmostEqual(reference_share(self.REF[:1], self.REF), 0.5)

    def test_evaluate_runs_scores_each_run_against_pooled_reference(self):
        run_a = [([1.0, 2.0, 3.0], [0, 1])]
        run_b = [([3.0, 2.0, 1.0], [0, 2])]
        method_routes = {"exact": run_a + run_b, "mopso": run_a + run_b}
        rows = evaluate_runs(method_routes, {"mopso": [run_a, run_b]}, {"mopso": [0.1, 0.3]})
        self.assertEqual([r["run"] for r in rows], [0, 1])
        self.assertTrue(all(r["hv_ratio"] < 1.0 for r in rows))
        self.assertAlmostEqual(rows[1]["runtime_s"], 0.3)

    def test_write_csv_removes_stale_file_when_empty(self):
        import tempfile
        from module5_evaluation import write_csv
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "violations.csv"
            write_csv(path, [{"a": 1}])
            self.assertTrue(path.exists())
            write_csv(path, [])
            self.assertFalse(path.exists())

    def test_summarize_runs_mean_std(self):
        rows = [{"method": "m", "hypervolume": 1.0}, {"method": "m", "hypervolume": 3.0}]
        summary = summarize_runs(rows)[0]
        self.assertEqual(summary["runs"], 2)
        self.assertAlmostEqual(summary["hypervolume_mean"], 2.0)
        self.assertAlmostEqual(summary["hypervolume_std"], np.sqrt(2.0))


class GroupedMonotonicityTests(unittest.TestCase):
    @staticmethod
    def _row(scenario, F, risk):
        return {"scenario": scenario, "F": F, "W": 1, "U": 1, "P": 1,
                "distance": 1.0, "energy": 1.0, "risk": risk}

    def test_no_false_violation_across_scenarios(self):
        rows = [self._row("A", 1, 5.0), self._row("A", 2, 4.0),
                self._row("B", 1, 1.0), self._row("B", 2, 9.0)]
        violations = monotonicity_violations(rows)
        self.assertEqual(len(violations), 1)
        self.assertEqual(violations[0]["scenario"], "B")

    def test_rows_without_scenario_form_one_group(self):
        rows = [{k: v for k, v in self._row("A", F, r).items() if k != "scenario"}
                for F, r in ((1, 1.0), (2, 2.0))]
        self.assertEqual(len(monotonicity_violations(rows)), 1)


class RobustnessAndProfileTests(unittest.TestCase):
    def _graph(self, a_cost, b_cost):
        G = nx.DiGraph(n_buildings=3, elevation_provider="test")
        for node in ("s", "a", "b", "t"):
            G.add_node(node, ground_elev_m=2.0, bldg_h_m=10.0, bldg_density=0.2, agl_m=60.0)
        for path, cost in ((("s", "a", "t"), a_cost), (("s", "b", "t"), b_cost)):
            for u, v in zip(path[:-1], path[1:]):
                G.add_edge(u, v, distance=cost, energy=cost, risk=cost, rho=0.5, water=0.0)
        return G

    def test_front_robustness_counts_routes_dominated_under_new_costs(self):
        calm_front = [([2.0, 2.0, 2.0], ["s", "a", "t"]), ([2.0, 2.0, 2.0], ["s", "b", "t"])]
        G_wind = self._graph(1.0, 3.0)
        wind_front = [([2.0, 2.0, 2.0], ["s", "a", "t"])]
        out = front_robustness(G_wind, calm_front, wind_front)
        self.assertEqual(out["feasible"], 2)
        self.assertAlmostEqual(out["still_nondominated"], 0.5)

    def test_front_robustness_skips_routes_missing_in_new_graph(self):
        G = self._graph(1.0, 1.0)
        out = front_robustness(G, [([1.0, 1.0, 1.0], ["s", "t"])], [([2.0, 2.0, 2.0], ["s", "a", "t"])])
        self.assertEqual(out["feasible"], 0)

    def test_area_profile(self):
        profile = area_profile(self._graph(1.0, 2.0))
        self.assertEqual(profile["nodes"], 4)
        self.assertEqual(profile["n_buildings"], 3)
        self.assertAlmostEqual(profile["rho_mean"], 0.5)
        self.assertAlmostEqual(profile["bldg_h_max_m"], 10.0)

    def test_tier2_vs_weighted_flags_identical_route(self):
        front = [
            ([1.0, 10.0, 10.0], [0, 1]),
            ([10.0, 1.0, 10.0], [0, 2]),
            ([10.0, 10.0, 1.0], [0, 3]),
            ([4.0, 4.0, 4.0], [0, 4]),
        ]
        weighted = {name: [front[3]] for name in ("fragile_medical", "mixed_cargo")}
        rows = tier2_vs_weighted(front, weighted)
        self.assertEqual({r["preset"] for r in rows}, set(weighted))
        self.assertTrue(all(r["weighted_on_front"] for r in rows))
        self.assertTrue(all(r["same_path"] == (r["objective_gap"] == 0.0) for r in rows))


if __name__ == "__main__":
    unittest.main()
