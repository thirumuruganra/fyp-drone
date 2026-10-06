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


class MissionAltitudeFrameTests(unittest.TestCase):
    """Frame 3 is height above HOME: rel = alt_amsl − home ground, never agl_m (audit #3)."""

    UPHILL = [
        {"node": 1, "lat": 12.0, "lon": 80.0, "agl_m": 60.0, "alt_amsl_m": 70.0},    # ground 10
        {"node": 2, "lat": 12.01, "lon": 80.0, "agl_m": 60.0, "alt_amsl_m": 95.0},   # ground 35
    ]

    def test_uphill_waypoint_is_relative_to_home_not_local_ground(self):
        items = build_mission_items(self.UPHILL, merge_collinear=False, end_with_rtl=False)
        self.assertEqual(items[0].frame, 0)
        self.assertAlmostEqual(items[0].alt, 10.0)          # home row: AMSL
        self.assertAlmostEqual(items[1].alt, 85.0)          # 95 AMSL − 10 home, not 60
        self.assertEqual(items[1].frame, MAV_FRAME_GLOBAL_RELATIVE_ALT)

    def test_explicit_home_altitude(self):
        items = build_mission_items(self.UPHILL, merge_collinear=False, end_with_rtl=False,
                                    home_alt_amsl=5.0)
        self.assertAlmostEqual(items[0].alt, 5.0)
        self.assertAlmostEqual(items[1].alt, 90.0)

    def test_home_altitude_needs_amsl_waypoints(self):
        with self.assertRaises(ValueError):
            build_mission_items([{"lat": 12.0, "lon": 80.0, "agl_m": 60.0},
                                 {"lat": 12.1, "lon": 80.0, "agl_m": 60.0}], home_alt_amsl=5.0)

    def test_rtl_uses_relative_altitude(self):
        items = build_mission_items(self.UPHILL, merge_collinear=False, end_with_rtl=True)
        self.assertEqual(items[-1].command, MAV_CMD_NAV_RETURN_TO_LAUNCH)
        self.assertAlmostEqual(items[-1].alt, 85.0)


if __name__ == "__main__":
    unittest.main()
