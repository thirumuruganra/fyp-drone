import importlib.util
import io
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_test_helpers import make_bundle  # noqa: E402

import sim_config as scfg  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "camera_director", Path(__file__).resolve().parents[1] / "gazebo" / "camera_director.py")
camera_director = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(camera_director)          # gz bindings are imported inside main() only


class CameraPhaseTests(unittest.TestCase):
    def test_takeoff_tracks_until_switch_altitude_then_follows(self):
        phase, TRACK, FOLLOW = camera_director.phase, camera_director.TRACK, camera_director.FOLLOW
        self.assertEqual(phase("takeoff", None, False, 6.0), TRACK)        # no pose yet
        self.assertEqual(phase("takeoff", 0.2, False, 6.0), TRACK)         # on the pad
        self.assertEqual(phase("takeoff", 6.0, False, 6.0), FOLLOW)
        self.assertEqual(phase("takeoff", 0.2, True, 6.0), FOLLOW)         # landing: stays on FOLLOW

    def test_follow_and_wide_follow_from_the_start(self):
        for view in ("follow", "wide"):
            self.assertEqual(camera_director.phase(view, None, False, 6.0), camera_director.FOLLOW)


class TakeoffGuiConfigTests(unittest.TestCase):
    def test_takeoff_view_is_a_fixed_pad_shot(self):
        import trimesh  # noqa: F401  (world builder needs it)
        from gazebo_world_builder import build_world
        import json
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            make_bundle(root / "regions", "t", terrain=lambda x, y: 5.0)
            with redirect_stdout(io.StringIO()):
                world = build_world("t", regions_root=root / "regions", worlds_root=root / "worlds",
                                    terrain_res_m=20.0, gui_view="takeoff")
            text = (world.parent / "gui.config").read_text().split("?>", 1)[1]
            gui = ET.fromstring(f"<root>{text}</root>")
            self.assertIsNone(gui.find("plugin[@filename='CameraTracking']/follow_target"))
            pose = [float(v) for v in gui.find("plugin[@filename='MinimalScene']/camera_pose").text.split()]
            self.assertEqual(pose[:3], list(scfg.SIM_CAMERA_VIEWS["takeoff"]))
            manifest = json.loads((world.parent / "world_manifest.json").read_text())
            self.assertEqual(manifest["gui"]["mode"], "fixed")


if __name__ == "__main__":
    unittest.main()
