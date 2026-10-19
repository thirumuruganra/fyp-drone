import io
import json
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_test_helpers import make_bundle  # noqa: E402

import numpy as np  # noqa: E402
import trimesh  # noqa: E402

from gazebo_world_builder import build_world, route_markers_sdf, world_ground_z  # noqa: E402
from sim_region import RegionBundle  # noqa: E402
import sim_config as scfg  # noqa: E402


class WorldBuilderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        root = Path(cls.tmp.name)
        make_bundle(root / "regions", "w", terrain=lambda x, y: 10.0 + 0.02 * x,
                    buildings=[(-20, -20, 0, 0, {"height": "30"}), (40, 40, 60, 60, {"building:levels": "4"})])
        cls.bundle = RegionBundle.load("w", root / "regions")
        with redirect_stdout(io.StringIO()):
            cls.world = build_world("w", regions_root=root / "regions", worlds_root=root / "worlds",
                                    terrain_res_m=10.0)
        cls.out = cls.world.parent

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_world_sdf_is_valid_and_complete(self):
        w = ET.parse(self.world).getroot().find("world")
        self.assertEqual(w.get("name"), "fyp_w")
        plugins = {p.get("name") for p in w.findall("plugin")}
        self.assertIn("gz::sim::systems::Physics", plugins)
        self.assertIn("gz::sim::systems::NavSat", plugins)
        sc = w.find("spherical_coordinates")
        self.assertAlmostEqual(float(sc.find("latitude_deg").text), self.bundle.depot[0], places=6)
        self.assertAlmostEqual(float(sc.find("elevation").text), self.bundle.h_home, places=2)
        uris = [i.find("uri").text for i in w.findall("include")]
        self.assertIn(f"model://{scfg.SIM_VEHICLE_MODEL}", uris)
        self.assertIn("model://w_terrain", uris)
        self.assertTrue(any(u.startswith("model://w_bldg_") for u in uris))
        for u in uris[:-1]:
            self.assertTrue((self.out / "models" / u[len("model://"):] / "model.sdf").exists(), u)
        for f in (self.out / "models").rglob("model.sdf"):
            ET.parse(f)
        self.assertIn("--custom-location=", (self.out / "sitl_command.txt").read_text())

    def test_gui_config_follows_the_vehicle(self):
        text = (self.out / "gui.config").read_text().split("?>", 1)[1]     # several top-level elements
        root = ET.fromstring(f"<root>{text}</root>")
        tracking = root.find("plugin[@filename='CameraTracking']")
        self.assertEqual(tracking.find("follow_target").text, scfg.SIM_VEHICLE_MODEL)
        offset = [float(v) for v in tracking.find("follow_offset").text.split()]
        self.assertEqual(offset, list(scfg.SIM_CAMERA_VIEWS["follow"]))
        pose = [float(v) for v in root.find("plugin[@filename='MinimalScene']/camera_pose").text.split()]
        self.assertEqual(pose[:3], offset)
        manifest = json.loads((self.out / "world_manifest.json").read_text())
        self.assertEqual(manifest["gui"]["view"], "follow")

    def test_terrain_mesh_matches_the_bundle_surface(self):
        m = trimesh.load(self.out / "models" / "w_terrain" / "meshes" / "terrain.obj", process=False)
        v = np.asarray(m.vertices)
        k = np.argmin(np.hypot(v[:, 0], v[:, 1]))
        self.assertLess(abs(v[k, 2] - float(world_ground_z(self.bundle, [v[k, 0]], [v[k, 1]])[0])), 0.01)
        self.assertLess(abs(float(world_ground_z(self.bundle, [0.0], [0.0])[0])), 1e-6)   # depot = z 0
        idx = np.random.default_rng(0).choice(len(v), 50, replace=False)
        np.testing.assert_allclose(v[idx, 2], world_ground_z(self.bundle, v[idx, 0], v[idx, 1]), atol=1e-3)

    def test_building_roofs_match_the_bundle(self):
        roofs = sorted(float(r) - self.bundle.h_home for r in self.bundle.buildings()["roof_amsl_m"])
        tops = []
        for f in (self.out / "models").glob("w_bldg_*/meshes/*.obj"):
            tops.append(float(trimesh.load(f, process=False).vertices[:, 2].max()))
        self.assertEqual(len(tops), 2)
        np.testing.assert_allclose(sorted(tops), roofs, atol=0.01)

    def test_route_markers(self):
        sel = [(0, 0, 0), (0, 0, 60), (100, 50, 60), (200, 50, 0)]
        front = [[(0, 0, 60), (50, 50, 60)], [(0, 0, 60), (60, 10, 60), (70, 20, 60)]]
        root = ET.fromstring(route_markers_sdf(sel, front))
        visuals = root.findall(".//visual")
        self.assertEqual(len(visuals), 3 + 1 + 2)
        self.assertIsNone(root.find(".//collision"))
        pose = [float(x) for x in root.find(".//visual[@name='sel_0']/pose").text.split()]
        self.assertAlmostEqual(pose[2], 30.0)                  # vertical take-off column midpoint
        self.assertAlmostEqual(abs(pose[4]), np.pi / 2, places=4)


if __name__ == "__main__":
    unittest.main()
