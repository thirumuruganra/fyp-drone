import json
import tempfile
import time
import unittest
from pathlib import Path

import networkx as nx
import numpy as np

import sys


IMPLEMENTATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IMPLEMENTATION_DIR))

from module4_tier2_payload import (  # noqa: E402
    TierTwoSelector,
    derive_weights,
    route_energy_wh,
    validate_payload_tags,
)


FIXTURE_PATH = Path(__file__).resolve().parent / "fixtures" / "lattice_front.json"


def load_fixture_front():
    data = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
    return [(row["cost"], row["path"]) for row in data["front"]]


class TierTwoHardeningTests(unittest.TestCase):
    def test_validation_rejects_bad_inputs_and_accepts_numpy_ints(self):
        self.assertEqual(validate_payload_tags(np.int64(5), np.int64(3), np.int64(2), np.int64(4)), (5, 3, 2, 4))

        bad_cases = [
            ("F", (False, 1, 1, 1)),
            ("F", (2.5, 1, 1, 1)),
            ("F", (0, 1, 1, 1)),
            ("F", (6, 1, 1, 1)),
            ("W", (1, True, 1, 1)),
            ("W", (1, 2.5, 1, 1)),
            ("W", (1, 0, 1, 1)),
            ("W", (1, 4, 1, 1)),
            ("U", (1, 1, False, 1)),
            ("U", (1, 1, 2.5, 1)),
            ("U", (1, 1, 0, 1)),
            ("U", (1, 1, 4, 1)),
            ("P", (1, 1, 1, False)),
            ("P", (1, 1, 1, 2.5)),
            ("P", (1, 1, 1, 0)),
            ("P", (1, 1, 1, 6)),
        ]
        for field, args in bad_cases:
            with self.subTest(args=args):
                with self.assertRaisesRegex(ValueError, field):
                    validate_payload_tags(*args)

        with self.assertRaisesRegex(ValueError, "F"):
            derive_weights(False, 1, 1)

    def test_mixed_payload_picks_non_extreme_on_fixture_front(self):
        selector = TierTwoSelector(load_fixture_front())
        selected, *_ = selector.select(F=3, W=2, U=2, P=3)
        idx = selector.selected_index(selected)

        self.assertEqual(idx, 3)
        self.assertEqual(selector.classify_point(idx), "non-extreme")

    def test_monotonicity_on_fixed_front(self):
        front = load_fixture_front()
        selector = TierTwoSelector(front)

        risks = [
            selector.select(F=F, W=2, U=2, P=3)[0][0][2]
            for F in range(1, 6)
        ]
        distances = [
            selector.select(F=3, W=2, U=U, P=3)[0][0][0]
            for U in range(1, 4)
        ]
        energies = [
            selector.select(F=3, W=W, U=2, P=3)[0][0][1]
            for W in range(1, 4)
        ]

        self.assertTrue(all(b <= a for a, b in zip(risks, risks[1:])), risks)
        self.assertTrue(all(b <= a for a, b in zip(distances, distances[1:])), distances)
        self.assertTrue(all(b <= a for a, b in zip(energies, energies[1:])), energies)

    def test_feasibility_filters_more_routes_for_heavier_payload(self):
        front = [
            ([2.0, 1.0, 3.0], [0, 1]),
            ([3.0, 2.0, 2.0], [0, 2]),
            ([4.0, 3.0, 1.0], [0, 3]),
        ]
        selector = TierTwoSelector(front)

        light = selector.select_detailed(
            3, 1, 2, 3,
            write_outputs=False,
            battery_wh=4.0,
            battery_reserve=0.0,
            k_cal_wh_per_km=1.0,
        )
        heavy = selector.select_detailed(
            3, 3, 2, 3,
            write_outputs=False,
            battery_wh=4.0,
            battery_reserve=0.0,
            k_cal_wh_per_km=1.0,
        )

        self.assertGreater(light.feasible_count, heavy.feasible_count)
        self.assertIsNone(light.hold_advisory)
        self.assertIsNone(heavy.hold_advisory)

    def test_hold_advisories_for_no_feasible_route_and_weather_no_go(self):
        selector = TierTwoSelector([([2.0, 10.0, 3.0], [0, 1])])

        infeasible = selector.select_detailed(
            3, 3, 2, 3,
            write_outputs=False,
            battery_wh=1.0,
            battery_reserve=0.0,
            k_cal_wh_per_km=10.0,
        )
        no_go = selector.select_detailed(
            3, 1, 2, 3,
            write_outputs=False,
            weather_no_go=True,
            weather_advisory="HOLD: weather gate is NO-GO.",
        )

        self.assertIsNone(infeasible.selected_index)
        self.assertIn("no feasible route", infeasible.hold_advisory)
        self.assertIsNone(no_go.selected_index)
        self.assertEqual(no_go.hold_advisory, "HOLD: weather gate is NO-GO.")

    def test_detailed_result_exports_json_and_waypoint_csv(self):
        front = [([1.0, 1.0, 1.0], [0, 1])]
        selector = TierTwoSelector(front)
        graph = nx.DiGraph()
        graph.add_node(0, lat=12.0, lon=80.0, alt_amsl_m=30.0, agl_m=20.0)
        graph.add_node(1, lat=12.1, lon=80.1, alt_amsl_m=32.0, agl_m=22.0)

        with tempfile.TemporaryDirectory() as tmpdir:
            result = selector.select_detailed(
                3, 1, 2, 3,
                graph=graph,
                output_name="custom",
                output_dir=tmpdir,
                battery_wh=100.0,
            )

            json_path = Path(tmpdir) / "selection_custom.json"
            csv_path = Path(tmpdir) / "selection_custom.csv"
            self.assertTrue(json_path.exists())
            self.assertTrue(csv_path.exists())
            exported = json.loads(json_path.read_text(encoding="utf-8"))
            self.assertEqual(exported["selected_index"], result.selected_index)
            self.assertEqual(exported["path"][0]["lat"], 12.0)
            self.assertIn("seq,node,lat,lon,alt_amsl_m,agl_m", csv_path.read_text(encoding="utf-8"))

    def test_latency_under_one_second_on_100_route_front(self):
        front = [
            ([float(i + 1), float(101 - i), float(50 + abs(50 - i))], [0, i + 1])
            for i in range(100)
        ]
        selector = TierTwoSelector(front)
        latencies = []
        for _ in range(31):
            t0 = time.perf_counter()
            result = selector.select_detailed(
                3, 2, 2, 3,
                write_outputs=False,
                battery_wh=10_000.0,
            )
            latencies.append(time.perf_counter() - t0)
            self.assertIsNone(result.hold_advisory)

        median = float(np.median(latencies))
        p95 = float(np.percentile(latencies, 95))
        self.assertLess(median, 1.0)
        self.assertLess(p95, 1.0)

    def test_route_energy_uses_payload_mass_factor(self):
        light = route_energy_wh(2.0, 1, k_cal_wh_per_km=1.0)
        heavy = route_energy_wh(2.0, 3, k_cal_wh_per_km=1.0)
        self.assertGreater(heavy, light)


if __name__ == "__main__":
    unittest.main()
