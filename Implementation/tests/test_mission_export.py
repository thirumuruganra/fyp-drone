import sys
import tempfile
import unittest
from pathlib import Path


IMPLEMENTATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IMPLEMENTATION_DIR))

from mission_export import (  # noqa: E402
    MAV_CMD_NAV_RETURN_TO_LAUNCH,
    MAV_CMD_NAV_WAYPOINT,
    MAV_FRAME_GLOBAL_RELATIVE_ALT,
    build_mission_items,
    export_qgc_wpl,
    merge_collinear_waypoints,
    parse_qgc_wpl,
    serialize_qgc_wpl,
)
from module4_tier2_payload import SelectionResult  # noqa: E402


def selection(path, hold=None):
    return SelectionResult(
        tags={"F": 3, "W": 1, "U": 2, "P": 3},
        weights={"risk": 0.3, "distance": 0.4, "energy": 0.3},
        selected_index=None if hold else 0,
        cost_vector=None if hold else [1.0, 1.0, 1.0],
        path=path,
        normalised_scores=[1.0],
        objective_ranks=None if hold else {"distance": 1, "energy": 1, "risk": 1},
        point_type=None if hold else "non-extreme",
        latency_s=0.01,
        feasible_count=0 if hold else 1,
        hold_advisory=hold,
    )


class MissionExportTests(unittest.TestCase):
    def test_qgc_wpl_format_and_round_trip(self):
        result = selection([
            {"node": 1, "lat": 12.0, "lon": 80.0, "agl_m": 40.0},
            {"node": 2, "lat": 12.1, "lon": 80.1, "agl_m": 45.0},
        ])
        with tempfile.TemporaryDirectory() as tmpdir:
            out = export_qgc_wpl(result, Path(tmpdir) / "mission.waypoints", merge_collinear=False)
            text = out.read_text(encoding="utf-8")
        self.assertTrue(text.startswith("QGC WPL 110\n"))
        items = parse_qgc_wpl(text)
        self.assertEqual(len(items), 3)
        self.assertEqual(items[1].frame, MAV_FRAME_GLOBAL_RELATIVE_ALT)
        self.assertEqual(items[1].command, MAV_CMD_NAV_WAYPOINT)
        self.assertEqual(items[-1].command, MAV_CMD_NAV_RETURN_TO_LAUNCH)
        self.assertEqual(serialize_qgc_wpl(items), text)

    def test_merge_collinear_waypoints(self):
        merged = merge_collinear_waypoints([
            {"lat": 12.0, "lon": 80.0, "agl_m": 50.0},
            {"lat": 12.1, "lon": 80.1, "agl_m": 50.0},
            {"lat": 12.2, "lon": 80.2, "agl_m": 50.0},
            {"lat": 12.3, "lon": 80.4, "agl_m": 50.0},
        ])
        self.assertEqual(len(merged), 3)
        self.assertEqual(merged[0]["lat"], 12.0)
        self.assertEqual(merged[-1]["lon"], 80.4)

    def test_altitude_never_above_120m_agl(self):
        with self.assertRaisesRegex(ValueError, "exceeds 120 m AGL"):
            build_mission_items([
                {"lat": 12.0, "lon": 80.0, "agl_m": 30.0},
                {"lat": 12.1, "lon": 80.1, "agl_m": 121.0},
            ])

    def test_export_rejects_hold_advisory(self):
        result = selection([], hold="HOLD: weather gate is NO-GO.")
        with tempfile.TemporaryDirectory() as tmpdir:
            with self.assertRaisesRegex(ValueError, "on hold"):
                export_qgc_wpl(result, Path(tmpdir) / "mission.waypoints")


if __name__ == "__main__":
    unittest.main()
