import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim_test_helpers  # noqa: E402,F401

from mission_export import MAV_CMD_NAV_RETURN_TO_LAUNCH, parse_qgc_wpl  # noqa: E402
from sim_mission_export import (MAV_CMD_DO_CHANGE_SPEED, MAV_CMD_NAV_LAND, MAV_CMD_NAV_TAKEOFF,  # noqa: E402
                                build_sim_mission, flight_profile, items_from_plan, load_mission,
                                to_qgc_plan, write_mission_files)

DEPOT = {"name": "depot", "lat": 12.97868, "lon": 80.25460, "ground_amsl_m": 10.0}
DEST = {"name": "dest0", "lat": 12.98723, "lon": 80.26420, "ground_amsl_m": 35.0}
UPHILL = [  # ground 10 → 35 m, 60 m AGL everywhere
    {"node": 1, "lat": 12.9787, "lon": 80.2547, "alt_amsl_m": 70.0, "agl_m": 60.0},
    {"node": 2, "lat": 12.9830, "lon": 80.2594, "alt_amsl_m": 82.0, "agl_m": 60.0},
    {"node": 3, "lat": 12.9872, "lon": 80.2641, "alt_amsl_m": 95.0, "agl_m": 60.0},
]


class SimMissionTests(unittest.TestCase):
    def setUp(self):
        self.items = build_sim_mission(UPHILL, DEPOT, DEST, h_home=10.0, speed_ms=5.0)

    def test_sequence_is_takeoff_route_approach_land_without_rtl(self):
        cmds = [it.command for it in self.items]
        self.assertEqual(cmds[:3], [16, MAV_CMD_NAV_TAKEOFF, MAV_CMD_DO_CHANGE_SPEED])
        self.assertEqual(cmds[-1], MAV_CMD_NAV_LAND)
        self.assertNotIn(MAV_CMD_NAV_RETURN_TO_LAUNCH, cmds)
        self.assertEqual(len(self.items), 1 + 1 + 1 + len(UPHILL) + 1 + 1)
        self.assertEqual([it.index for it in self.items], list(range(len(self.items))))

    def test_altitudes_are_relative_to_home_not_local_ground(self):
        self.assertEqual(self.items[0].frame, 0)
        self.assertAlmostEqual(self.items[0].alt, 10.0)                 # home row AMSL
        self.assertAlmostEqual(self.items[1].alt, 60.0)                 # takeoff to first node
        self.assertEqual([round(it.alt, 3) for it in self.items[3:6]], [60.0, 72.0, 85.0])  # not 60/60/60
        approach, land = self.items[-2], self.items[-1]
        self.assertAlmostEqual(approach.alt, 85.0)
        self.assertEqual((approach.lat, approach.lon), (DEST["lat"], DEST["lon"]))
        self.assertEqual((land.lat, land.lon, land.alt), (DEST["lat"], DEST["lon"], 0.0))

    def test_speed_item(self):
        sp = self.items[2]
        self.assertEqual((sp.frame, sp.p1, sp.p2), (2, 1.0, 5.0))

    def test_missing_altitudes_are_rejected(self):
        bad = [dict(UPHILL[0], alt_amsl_m=None)] + UPHILL[1:]
        with self.assertRaises(ValueError):
            build_sim_mission(bad, DEPOT, DEST, 10.0)

    def test_ceiling_is_enforced(self):
        bad = UPHILL[:2] + [dict(UPHILL[2], agl_m=130.0)]
        with self.assertRaises(ValueError):
            build_sim_mission(bad, DEPOT, DEST, 10.0)

    def test_route_below_home_is_rejected(self):
        low = [dict(w, alt_amsl_m=5.0) for w in UPHILL]
        with self.assertRaises(ValueError):
            build_sim_mission(low, DEPOT, DEST, 10.0)

    def test_plan_round_trip(self):
        plan = to_qgc_plan(self.items, 10.0)
        m = plan["mission"]
        self.assertEqual((m["firmwareType"], m["vehicleType"]), (3, 2))
        self.assertEqual(m["plannedHomePosition"], [DEPOT["lat"], DEPOT["lon"], 10.0])
        self.assertEqual(len(m["items"]), len(self.items) - 1)
        self.assertTrue(all(i["AltitudeMode"] == 1 for i in m["items"] if i["frame"] == 3))
        back = items_from_plan(json.loads(json.dumps(plan)))
        for a, b in zip(self.items, back):
            self.assertEqual((a.frame, a.command), (b.frame, b.command))
            self.assertAlmostEqual(a.lat, b.lat)
            self.assertAlmostEqual(a.lon, b.lon)
            self.assertAlmostEqual(a.alt, b.alt)

    def test_files_round_trip(self):
        with tempfile.TemporaryDirectory() as tmp:
            paths = write_mission_files(self.items, 10.0, Path(tmp), {"run_id": "x", "bundle_hash": "h"})
            self.assertEqual(len(parse_qgc_wpl(paths["waypoints"].read_text())), len(self.items))
            meta, items = load_mission(Path(tmp))
        self.assertEqual(meta["bundle_hash"], "h")
        self.assertEqual(meta["h_home_amsl_m"], 10.0)
        self.assertEqual([i.command for i in items], [i.command for i in self.items])

    def test_flight_profile_skips_home_and_speed(self):
        prof = flight_profile(self.items, 10.0)
        self.assertEqual(len(prof), len(self.items) - 2)
        self.assertAlmostEqual(prof[0][2], 70.0)          # takeoff top, AMSL
        self.assertAlmostEqual(prof[-1][2], 10.0)         # land item at rel 0 → home level


if __name__ == "__main__":
    unittest.main()
