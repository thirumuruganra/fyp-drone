import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_test_helpers import latlon, make_bundle  # noqa: E402

import numpy as np  # noqa: E402

from env_data import FlatProvider  # noqa: E402
from prepare_sim_region import dsm_filter, height_source  # noqa: E402
from sim_region import BundleError, RegionBundle  # noqa: E402

BUILDINGS = [(-20, -20, 0, 0, {"height": "30"}),
             (40, 40, 60, 60, {"building:levels": "4"}),
             (100, -80, 120, -60, {"building": "house"})]


class HeightSourceTests(unittest.TestCase):
    def test_rules_follow_building_height_m(self):
        self.assertEqual(height_source({"height": "12 m"}), "tagged")
        self.assertEqual(height_source({"height": None, "building:levels": "3"}), "levels")
        self.assertEqual(height_source({"building": "house"}), "default")


class DsmFilterTests(unittest.TestCase):
    def test_p10_removes_an_isolated_spike(self):
        v = np.full((5, 5), 5.0)
        v[2, 2] = 40.0                                     # one DSM cell on a roof
        self.assertAlmostEqual(dsm_filter(v, "p10")[2, 2], 5.0)
        np.testing.assert_array_equal(dsm_filter(v, "none"), v)


class BundleTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def test_contents_and_manifest(self):
        root = make_bundle(self.tmp.name, "b", buildings=BUILDINGS, terrain=lambda x, y: 10.0 + 0.01 * x)
        b = RegionBundle(root)
        m = b.manifest
        self.assertEqual(m["buildings"]["count"], 3)
        self.assertEqual(m["buildings"]["height_source_counts"], {"tagged": 1, "levels": 1, "default": 1})
        self.assertEqual(m["terrain"]["provider"], "test-dem")
        self.assertFalse(m["data_mode"]["fixture"])
        self.assertEqual(len(m["bundle_hash"]), 16)
        for f in m["files"].values():
            self.assertTrue((root / f["path"]).exists())
        # H_home = DEM at the depot (x = −150 → 8.5 m)
        self.assertAlmostEqual(b.h_home, 8.5, delta=0.05)
        self.assertAlmostEqual(b.pads["destinations"][0]["ground_amsl_m"], 11.5, delta=0.05)

    def test_building_base_is_lowest_ground_under_the_footprint(self):
        root = make_bundle(self.tmp.name, "slope", buildings=[(0, 0, 40, 10, {"height": "20"})],
                           terrain=lambda x, y: 0.1 * x)          # 0 … 4 m across the footprint
        row = RegionBundle(root).buildings().iloc[0]
        self.assertAlmostEqual(row["base_amsl_m"], 0.0 - 0.5, delta=0.1)
        self.assertAlmostEqual(row["roof_amsl_m"] - row["base_amsl_m"], 20.0, places=2)

    def test_hash_is_reproducible_and_data_sensitive(self):
        h1 = RegionBundle(make_bundle(self.tmp.name, "h", buildings=BUILDINGS)).bundle_hash
        h2 = RegionBundle(make_bundle(self.tmp.name, "h", buildings=BUILDINGS)).bundle_hash
        self.assertEqual(h1, h2)
        taller = [(-20, -20, 0, 0, {"height": "31"})] + BUILDINGS[1:]
        self.assertNotEqual(h1, RegionBundle(make_bundle(self.tmp.name, "h", buildings=taller)).bundle_hash)

    def test_strict_rejects_flat_ground_and_empty_buildings(self):
        with self.assertRaises(BundleError):
            make_bundle(self.tmp.name, "s1", buildings=BUILDINGS, strict=True, provider=FlatProvider())
        with self.assertRaises(BundleError):
            make_bundle(self.tmp.name, "s2", buildings=(), strict=True)
        make_bundle(self.tmp.name, "s3", buildings=BUILDINGS, strict=True)    # real-looking data passes

    def test_pads_must_be_inside_the_box(self):
        with self.assertRaises(BundleError):
            make_bundle(self.tmp.name, "p", buildings=BUILDINGS, dests_xy=[(500.0, 0.0)])

    def test_frames(self):
        b = RegionBundle(make_bundle(self.tmp.name, "f", buildings=BUILDINGS, terrain=lambda x, y: 7.0))
        x, y, z = b.to_world(*b.depot, b.h_home)
        self.assertAlmostEqual(float(x), 0.0, places=6)
        self.assertAlmostEqual(float(y), 0.0, places=6)
        self.assertAlmostEqual(float(z), 0.0, places=6)
        lat, lon = latlon(150.0, 150.0)
        px, py = b.planner_frame.to_local(lat, lon)
        self.assertAlmostEqual(float(px), 150.0, places=3)
        wx, wy = b.to_world(lat, lon)
        self.assertAlmostEqual(float(wx), 300.0, delta=0.05)     # depot at (−150, −150)
        self.assertAlmostEqual(float(wy), 300.0, delta=0.05)
        # round trip to < 0.1 m
        la2, lo2 = b.world_frame.to_latlon(wx, wy)
        bx, by = b.planner_frame.to_local(la2, lo2)
        self.assertLess(abs(float(bx) - 150.0) + abs(float(by) - 150.0), 0.1)


if __name__ == "__main__":
    unittest.main()
