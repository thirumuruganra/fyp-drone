import io
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_test_helpers import make_bundle  # noqa: E402
import _paths  # noqa: E402
import run_sim_mission  # noqa: E402
from env_data import WeatherSnapshot  # noqa: E402
from recording_route import build_recording_graph  # noqa: E402
from route_clearance import NoFeasibleMission  # noqa: E402
from sim_mission_export import load_mission, items_from_plan, to_qgc_plan  # noqa: E402
from sim_region import RegionBundle  # noqa: E402


class RecordingRouteTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        make_bundle(self.root / "regions", "turns", box_m=500,
                    depot_xy=(-130, -60), dests_xy=[(130, -60)],
                    buildings=[(-80, -30, -60, -20, {"height": "20"})])
        self.bundle = RegionBundle.load("turns", self.root / "regions")

    def test_export_has_ten_distinct_points_eight_corners_and_clear_legs(self):
        with mock.patch.object(_paths, "REGIONS_DIR", self.root / "regions"), \
                mock.patch.object(_paths, "MISSIONS_DIR", self.root / "missions"), \
                redirect_stdout(io.StringIO()):
            rc = run_sim_mission.main(["--region-id", "turns", "--recording-turns",
                                       "--preset", "mixed_cargo"])
        self.assertEqual(rc, 0)
        meta, items = load_mission(next((self.root / "missions").iterdir()))
        waypoints = [it for it in items[1:] if it.command == 16]
        self.assertEqual(len(waypoints), 10)
        self.assertEqual(len({(it.lat, it.lon) for it in waypoints}), 10)
        xy = np.array([self.bundle.world_frame.to_local(it.lat, it.lon) for it in waypoints])
        legs = np.diff(xy, axis=0)
        angles = [np.degrees(np.arccos(np.clip(a @ b / np.linalg.norm(a) / np.linalg.norm(b), -1, 1)))
                  for a, b in zip(legs, legs[1:])]
        self.assertEqual(sum(a > 45 for a in angles), 8)
        self.assertGreaterEqual(meta["planned_min_roof_clearance_m"], 15)
        self.assertGreaterEqual(meta["planned_min_terrain_clearance_m"], 15)
        self.assertEqual(meta["route_mode"], "ordered_recording_corridor")
        self.assertEqual(meta["route_nodes"], list(range(10)))
        self.assertFalse(meta["front"]["truncated"])
        self.assertEqual((waypoints[-1].lat, waypoints[-1].lon), self.bundle.destinations[0])
        self.assertEqual(items[-1].command, 21)
        back = items_from_plan(to_qgc_plan(items, self.bundle.h_home))
        self.assertEqual([(i.command, i.lat, i.lon) for i in items],
                         [(i.command, i.lat, i.lon) for i in back])

    def test_restricted_crossing_rejected_before_export(self):
        zones = mock.MagicMock()
        zones.contains.return_value = np.zeros(10, dtype=bool)
        zones.intersects_segments.return_value = np.array([True] + [False] * 8)
        with mock.patch("recording_route.build_layer_indexes", return_value={"restricted_zones": zones}), \
                redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(NoFeasibleMission, "restricted zone"):
                build_recording_graph(self.bundle, 100, WeatherSnapshot.calm(*self.bundle.center))


if __name__ == "__main__":
    unittest.main()
