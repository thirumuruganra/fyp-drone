"""End-to-end (offline): bundle → graph → Tier 1 → Tier 2 → mission → synthetic flight → evaluation."""

import csv
import io
import json
import shutil
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_test_helpers import make_bundle  # noqa: E402

import numpy as np  # noqa: E402

import _paths  # noqa: E402
import evaluate_gazebo_run  # noqa: E402
import run_sim_mission  # noqa: E402
from sim_mission_export import load_mission  # noqa: E402
from sim_region import RegionBundle  # noqa: E402
from simulation_logger import parse_pose_message  # noqa: E402
from sitl_mission_runner import (PreflightError, preflight_identity, verify_readback,  # noqa: E402
                                 world_vehicle_model)

BUILDINGS = [(-60, -70, -40, 70, {"height": "40"}),        # a 40 m block between depot and destination
             (20, -30, 40, -10, {"building:levels": "6"}),
             (60, 60, 80, 80, {"building": "house"})]


class EndToEndTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        root = Path(cls.tmp.name)
        cls.patches = [mock.patch.object(_paths, name, root / sub) for name, sub in
                       (("REGIONS_DIR", "regions"), ("MISSIONS_DIR", "missions"),
                        ("WORLDS_DIR", "worlds"), ("RUNS_DIR", "runs"))]
        for p in cls.patches:
            p.start()
        make_bundle(root / "regions", "e", buildings=BUILDINGS, box_m=500.0, terrain=lambda x, y: 5.0,
                    depot_xy=(-200.0, -10.0), dests_xy=[(200.0, 20.0)])
        cls.bundle = RegionBundle.load("e")
        out = io.StringIO()
        with redirect_stdout(out):
            cls.rc1 = run_sim_mission.main(["--region-id", "e", "--all-presets", "--spacing-m", "100"])
            cls.rc2 = run_sim_mission.main(["--region-id", "e", "--preset", "fragile_medical",
                                            "--spacing-m", "100"])
        cls.log = out.getvalue()
        cls.missions = sorted((root / "missions").iterdir())

    @classmethod
    def tearDownClass(cls):
        for p in cls.patches:
            p.stop()
        cls.tmp.cleanup()

    def test_missions_written_and_front_reused(self):
        self.assertEqual((self.rc1, self.rc2), (0, 0), self.log)
        self.assertEqual(len(self.missions), 5)
        self.assertIn("Rebuilding front: no stored front", self.log)
        self.assertIn("Reusing stored front", self.log)                    # tag-only change: Tier 2 only
        self.assertEqual(self.log.count("[Module 2] NAMOA*"), 1)              # Tier 1 ran once
        for d in self.missions:
            for f in ("mission.plan", "mission.waypoints", "mission.json", "route_markers.sdf"):
                self.assertTrue((d / f).exists(), f)

    def test_mission_identity_and_altitudes(self):
        meta, items = load_mission(self.missions[0])
        self.assertEqual(meta["bundle_hash"], self.bundle.bundle_hash)
        self.assertFalse(meta["front"]["truncated"])
        self.assertEqual(meta["front"]["identity"]["bundle_hash"], self.bundle.bundle_hash)
        self.assertGreaterEqual(meta["planned_min_roof_clearance_m"], 15.0)
        self.assertAlmostEqual(items[0].alt, self.bundle.h_home)
        for it in items[3:-2]:                                    # route nodes: AMSL − home
            self.assertGreaterEqual(it.alt, 60.0 - 1e-6)

    def test_route_clears_the_block(self):
        meta, _ = load_mission(self.missions[0])
        xyz = np.asarray(meta["planned_world_xyz"])
        crossing = (xyz[:, 0] > 120) & (xyz[:, 0] < 180)      # block at x −60…−40 (planner) = 140…160 (world)
        if crossing.any():
            self.assertTrue(np.all(xyz[crossing, 2] + self.bundle.h_home >= 5.0 + 40.0 + 15.0 - 1e-6))

    def test_preflight_identity(self):
        meta, _ = load_mission(self.missions[0])
        self.assertTrue(any("no world manifest" in w for w in preflight_identity(meta)))
        with self.assertRaises(PreflightError):
            preflight_identity({**meta, "front": {**meta["front"], "truncated": True}})
        storm = dict(meta["weather"], wind_speed_kmh=60.0, gust_kmh=80.0)
        with self.assertRaises(PreflightError):
            preflight_identity({**meta, "weather": storm})
        wdir = _paths.WORLDS_DIR / "e"
        wdir.mkdir(parents=True, exist_ok=True)
        (wdir / "world_manifest.json").write_text(json.dumps({"bundle_hash": "other"}))
        try:
            with self.assertRaises(PreflightError):
                preflight_identity(meta)
        finally:
            shutil.rmtree(wdir)

    def synthetic_run(self, offset_m: float, status="landed_disarmed"):
        mdir = self.missions[0]
        meta, items = load_mission(mdir)
        run = _paths.RUNS_DIR / f"{meta['run_id']}__{offset_m}"
        run.mkdir(parents=True, exist_ok=True)
        shutil.copy(mdir / "mission.json", run / "mission.json")
        shutil.copy(mdir / "front_world_xyz.json", run / "front_world_xyz.json")
        plan = np.asarray(meta["planned_world_xyz"])
        pts = []
        for a, b in zip(plan[:-1], plan[1:]):
            for t in np.linspace(0, 1, 20, endpoint=False):
                pts.append(a + t * (b - a))
        pts.append(plan[-1])
        pts = np.asarray(pts)
        pts[:, 1] += offset_m                                     # constant north offset
        pts[-1, 1] -= offset_m                                    # lands on the pad
        frame = self.bundle.world_frame
        lat, lon = frame.to_latlon(pts[:, 0], pts[:, 1])
        n_cruise_from, n_cruise_to = 20, len(pts) - 40            # skip the climb and the descent legs
        with open(run / "global_position.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["wall_s", "time_boot_ms", "lat", "lon", "alt_msl_m", "rel_alt_m",
                        "vx_ms", "vy_ms", "vz_ms", "hdg_deg"])
            for k in range(len(pts)):
                w.writerow([1000 + k, k, lat[k], lon[k], pts[k, 2] + self.bundle.h_home, pts[k, 2], 0, 0, 0, 0])
        with open(run / "mission_events.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["wall_s", "event", "seq", "detail"])
            w.writerow([1000 + n_cruise_from, "mission_current", 3, ""])
            w.writerow([1000 + n_cruise_to, "mission_current", len(items) - 1, ""])
        (run / "run_manifest.json").write_text(json.dumps({"status": status, "upload_s": 0.2}))
        return run

    def test_evaluation_of_a_synthetic_flight(self):
        run = self.synthetic_run(2.0)
        m = evaluate_gazebo_run.evaluate(run, plots=True)
        self.assertEqual(m["track_source"], "mavlink")
        self.assertTrue(m["completed"])
        self.assertAlmostEqual(m["xtrack_rmse_m"], 2.0, delta=0.3)
        self.assertLessEqual(m["xtrack_max_m"], 2.3)
        self.assertEqual(m["roof_violations"], 0)
        self.assertLess(m["landing_offset_m"], 0.5)
        self.assertTrue((run / "map_overlay.png").exists())
        self.assertTrue((run / "altitude_profile.png").exists())

    def test_evaluation_flags_an_unfinished_flight(self):
        m = evaluate_gazebo_run.evaluate(self.synthetic_run(0.5, status="timeout"), plots=False)
        self.assertFalse(m["completed"])


class LoggerAndRunnerHelperTests(unittest.TestCase):
    def test_pose_logger_model_comes_from_the_world(self):
        import sim_config as scfg
        with tempfile.TemporaryDirectory() as tmp:
            meta = {"region_id": "r"}
            self.assertEqual(world_vehicle_model(meta, worlds_root=Path(tmp)), scfg.SIM_VEHICLE_MODEL)
            (Path(tmp) / "r").mkdir()
            (Path(tmp) / "r" / "world_manifest.json").write_text(json.dumps({"vehicle_model": "iris_with_gimbal"}))
            self.assertEqual(world_vehicle_model(meta, worlds_root=Path(tmp)), "iris_with_gimbal")

    def test_parse_pose_json(self):
        msg = json.dumps({"header": {"stamp": {"sec": 12, "nsec": 500000000}},
                          "pose": [{"name": "other", "position": {"x": 9}},
                                   {"name": "iris_with_gimbal", "position": {"x": 1.5, "y": -2, "z": 30},
                                    "orientation": {"w": 1}}]})
        p = parse_pose_message(msg, "iris_with_gimbal")
        self.assertEqual((p["sim_s"], p["x"], p["y"], p["z"]), (12.5, 1.5, -2.0, 30.0))
        self.assertIsNone(parse_pose_message(msg, "missing"))

    def test_parse_pose_text_format(self):
        msg = ('header {\n  stamp {\n    sec: 3\n    nsec: 250000000\n  }\n}\n'
               'pose {\n  name: "iris_with_gimbal"\n  id: 9\n  position {\n    x: 4.5\n    y: 1\n    z: 20.25\n  }\n'
               '  orientation {\n    w: 1\n  }\n}\n')
        p = parse_pose_message(msg, "iris_with_gimbal")
        self.assertEqual((p["x"], p["y"], p["z"]), (4.5, 1.0, 20.25))

    def test_verify_readback(self):
        from mission_export import MissionItem
        sent = [MissionItem(i, 0, 3, 16, 0, 0, 0, 0, 12.98 + i * 1e-3, 80.25, 60.0, 1) for i in range(3)]

        def msg(it, alt=None):
            return SimpleNamespace(frame=it.frame, command=it.command, param1=0, param2=0, param3=0,
                                   x=int(round(it.lat * 1e7)), y=int(round(it.lon * 1e7)),
                                   z=it.alt if alt is None else alt)
        self.assertEqual(verify_readback(sent, [msg(i) for i in sent]), [])
        self.assertEqual(len(verify_readback(sent, [msg(sent[0]), msg(sent[1], 70.0), msg(sent[2])])), 1)
        self.assertEqual(len(verify_readback(sent, [msg(sent[0])])), 1)

    def test_readback_accepts_frameless_do_items(self):
        """ArduPilot returns DO_CHANGE_SPEED (frame 2 sent) with frame 0 — seen on real SITL."""
        from mission_export import MissionItem
        home = MissionItem(0, 1, 0, 16, 0, 0, 0, 0, 12.98, 80.25, 7.0, 1)
        speed = MissionItem(1, 0, 2, 178, 1, 5, -1, 0, 0, 0, 0, 1)
        back = SimpleNamespace(frame=0, command=178, param1=1, param2=5, param3=-1, x=0, y=0, z=0)
        self.assertEqual(verify_readback([home, speed], [home, back]), [])
        wrong = SimpleNamespace(frame=0, command=178, param1=1, param2=9, param3=-1, x=0, y=0, z=0)
        self.assertEqual(len(verify_readback([home, speed], [home, wrong])), 1)


if __name__ == "__main__":
    unittest.main()
