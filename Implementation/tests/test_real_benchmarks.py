import contextlib
import io
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

import networkx as nx
import numpy as np


IMPLEMENTATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IMPLEMENTATION_DIR))

import run_real_benchmarks as rrb
from run_benchmarks import build_scenario


def _grid(n=6, spacing=200.0):
    G = nx.DiGraph()
    for r in range(n):
        for c in range(n):
            G.add_node(r * n + c, x_m=c * spacing, y_m=r * spacing)
    for r in range(n):
        for c in range(n):
            for dr, dc in ((0, 1), (1, 0)):
                rr, cc = r + dr, c + dc
                if rr < n and cc < n:
                    G.add_edge(r * n + c, rr * n + cc)
                    G.add_edge(rr * n + cc, r * n + c)
    return G


class HelperTests(unittest.TestCase):
    def test_od_pairs_fall_inside_their_bands(self):
        G = _grid()
        pairs = rrb.pick_od_pairs(G, {"short": (300.0, 500.0), "long": (900.0, None)}, seed=1)
        for label, (lo, hi) in {"short": (300.0, 500.0), "long": (900.0, np.inf)}.items():
            s, t, d = pairs[label]
            self.assertNotEqual(s, t)
            self.assertTrue(lo <= d <= hi)
        self.assertEqual(pairs, rrb.pick_od_pairs(G, {"short": (300.0, 500.0), "long": (900.0, None)}, seed=1))

    def test_impossible_band_is_skipped(self):
        with contextlib.redirect_stdout(io.StringIO()):
            pairs = rrb.pick_od_pairs(_grid(3), {"huge": (10_000.0, None)}, seed=1, tries=50)
        self.assertEqual(pairs, {})

    def test_farthest_pair_is_corner_to_corner(self):
        s, t, d = rrb.farthest_pair(_grid(4))
        self.assertEqual({s, t}, {0, 15})
        self.assertAlmostEqual(d, np.hypot(600.0, 600.0))

    def test_od_endpoints_stay_on_land(self):
        from shapely.geometry import box
        from env_data import ZoneIndex
        G = _grid()
        sea = ZoneIndex([box(500.0, -50.0, 1100.0, 1100.0)])          # columns 3-5 under water
        land = rrb.land_nodes(G, sea)
        self.assertEqual(land, {n for n in G if G.nodes[n]["x_m"] < 500.0})
        bands = {"short": (200.0, 400.0), "long": (500.0, None)}
        pairs = rrb.pick_od_pairs(G, bands, seed=3, allowed=land)
        for s, t, _ in pairs.values():
            self.assertTrue({s, t} <= land)
        s, t, _ = rrb.farthest_pair(G, land)
        self.assertTrue({s, t} <= land)

    def test_land_filter_keeps_dry_pairs_unchanged(self):
        G = _grid()
        bands = {"short": (300.0, 500.0), "long": (900.0, None)}
        self.assertEqual(rrb.pick_od_pairs(G, bands, seed=1),
                         rrb.pick_od_pairs(G, bands, seed=1, allowed=set(G)))
        self.assertEqual(rrb.land_nodes(G, None), set(G))

    def test_real_data_guard(self):
        area = {"id": "x"}
        for graph_attrs in ({"n_buildings": 0, "elevation_provider": "opentopography"},
                            {"n_buildings": 5, "elevation_provider": "flat"}):
            with self.assertRaises(rrb.AreaDataError):
                rrb.check_real_data(nx.DiGraph(**graph_attrs), area)
        rrb.check_real_data(nx.DiGraph(n_buildings=5, elevation_provider="opentopography"), area)

    def test_synthetic_density_changes_obstacles(self):
        with contextlib.redirect_stdout(io.StringIO()):
            sparse, *_ = build_scenario(50, 101, 0.05)
            dense, *_ = build_scenario(50, 101, 0.40)

        def mean_obstacle(G):
            return np.mean([d["obstacle"] for *_, d in G.edges(data=True)])
        self.assertGreater(mean_obstacle(dense), mean_obstacle(sparse))


class SmokeRunTests(unittest.TestCase):
    """Fixture lattice (hand-made layers, NOT real data) through the whole runner."""

    def test_smoke_run_resume_and_plots_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "out"
            argv = ["--smoke", "--allow-fixtures", "--winds", "calm", "sw_monsoon",
                    "--scaling-boxes", "2000", "--output-dir", str(out)]
            cwd = os.getcwd()
            os.chdir(tmp)                     # no cache/ here -> fixtures, never real caches
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    data = rrb.run_real_benchmarks(rrb.parse_args(argv))
                    case_files = sorted((out / "cases").glob("*.json"))
                    stamps = [f.stat().st_mtime_ns for f in case_files]
                    log = io.StringIO()
                    with contextlib.redirect_stdout(log):
                        rrb.run_real_benchmarks(rrb.parse_args(argv))
                    rrb.run_real_benchmarks(rrb.parse_args(argv + ["--plots-only"]))
                    stamps_cached = [f.stat().st_mtime_ns for f in case_files]

                    # older Tier 2 formula id -> Tier 2 parts refreshed, NAMOA*/MOPSO kept
                    stale = json.loads(case_files[0].read_text())
                    before = stale["tables"]
                    stale["tier2_formula"] = "old_formula"
                    case_files[0].write_text(json.dumps(stale))
                    refresh_log = io.StringIO()
                    with contextlib.redirect_stdout(refresh_log):
                        rrb.run_real_benchmarks(rrb.parse_args(argv))
                    after = json.loads(case_files[0].read_text())
            finally:
                os.chdir(cwd)

            self.assertIn("Tier 2 tables refreshed", refresh_log.getvalue())
            self.assertEqual(after["tier2_formula"], rrb.TIER2_FORMULA)
            self.assertEqual(after["tables"]["mopso_runs"], before["mopso_runs"])
            self.assertEqual([r["selected_index"] for r in after["tables"]["sweep"]],
                             [r["selected_index"] for r in before["sweep"]])
            hv = {r["method"]: r["hypervolume"] for r in before["metrics"]}
            for row in after["tables"]["metrics"]:
                self.assertAlmostEqual(row["hypervolume"], hv[row["method"]])

            self.assertTrue(case_files)
            self.assertEqual(stamps, stamps_cached)
            self.assertIn("cached", log.getvalue())
            for name in ("real_metrics.csv", "real_mopso_runs.csv", "real_tier2_evidence.csv",
                         "real_robustness.csv", "area_profiles.csv", "real_benchmark.json",
                         "real_hv.png", "real_area_maps.png", "real_wind_sensitivity.png",
                         "real_terrain_maps.png", "real_fronts_2d.png", "real_fronts_3d.png"):
                self.assertTrue((out / name).exists(), name)
            methods = {r["method"] for r in data["tables"]["metrics"]}
            self.assertTrue({"namoa", "mopso_pooled", "astar_distance"} <= methods)
            self.assertEqual(data["key_numbers"]["monotonicity_violations"], 0)
            for row in data["tables"]["metrics"]:
                if row["method"] == "namoa" and not row["truncated"]:
                    self.assertAlmostEqual(row["epsilon_add"], 0.0)


if __name__ == "__main__":
    unittest.main()
