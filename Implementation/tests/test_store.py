import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from datetime import datetime, timedelta, timezone
from pathlib import Path
from unittest import mock

IMPLEMENTATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IMPLEMENTATION_DIR))
sys.path.insert(0, str(IMPLEMENTATION_DIR / "tests"))

import config as cfg
import pareto_store
from env_data import WeatherSnapshot
from module1_graph_builder import LatticeAirspaceBuilder, apply_weather
from module2_tier1_moastar import TierOneEngine
from pareto_store import (ParetoStore, front_from_record, graph_hash, run_region, snap_to_node,
                          weather_drift)
from test_tier1_front import random_graph

T0 = datetime(2026, 10, 5, 9, 0, tzinfo=timezone.utc)
WIND = WeatherSnapshot(wind_speed_kmh=20, wind_dir_from_deg=225, gust_kmh=28, time="2026-10-05T14:30")


def snap(speed=20, direction=225, gust=28, **kw):
    return WeatherSnapshot(wind_speed_kmh=speed, wind_dir_from_deg=direction, gust_kmh=gust, **kw)


def lattice(weather=WIND, box_m=1400.0):
    with redirect_stdout(io.StringIO()):
        return LatticeAirspaceBuilder(box_m=box_m, weather=weather).fetch_graph()


def quiet_get(store, G, s, t, **kw):
    with redirect_stdout(io.StringIO()):
        return store.get_or_build(G, s, t, **kw)


class DriftTests(unittest.TestCase):
    def drift(self, old, new):
        return weather_drift(old.to_dict(), new)

    def test_identical_weather_has_no_drift(self):
        self.assertEqual(self.drift(WIND, WIND), [])

    def test_wind_speed_threshold(self):
        self.assertEqual(self.drift(snap(20, gust=20), snap(29.9, gust=29.9)), [])
        self.assertIn("wind speed", self.drift(snap(20, gust=20), snap(30.1, gust=30.1))[0])
        self.assertIn("wind speed", self.drift(snap(30, gust=30), snap(19.9, gust=19.9))[0])   # drops too

    def test_direction_threshold_applies_only_above_10_kmh(self):
        self.assertEqual(self.drift(snap(20, 100), snap(20, 140)), [])                          # 40° < 45°
        self.assertIn("direction", self.drift(snap(20, 100), snap(20, 150))[0])                 # 50°
        self.assertEqual(self.drift(snap(5, 100, 5), snap(6, 280, 6)), [])                      # light wind
        self.assertIn("direction", self.drift(snap(5, 100, 5), snap(12, 190, 12))[0])           # now > 10 km/h

    def test_direction_wraps_around_north(self):
        self.assertEqual(self.drift(snap(20, 350), snap(20, 10)), [])                           # 20° apart
        self.assertIn("direction", self.drift(snap(20, 350), snap(20, 60))[0])                  # 70°

    def test_gust_jump_threshold(self):
        self.assertEqual(self.drift(snap(20, gust=20), snap(20, gust=34)), [])
        self.assertIn("gusts", self.drift(snap(20, gust=20), snap(20, gust=36))[0])

    def test_go_no_go_flip_in_either_direction(self):
        storm = snap(20, 225, 28, weather_code=95)
        self.assertIn("go/no-go", self.drift(WIND, storm)[-1])
        self.assertIn("go/no-go", self.drift(storm, WIND)[-1])
        self.assertEqual(self.drift(storm, snap(20, 225, 28, weather_code=96)), [])             # still no-go

    def test_several_reasons_are_all_reported(self):
        reasons = self.drift(snap(10, 90, 10), snap(40, 200, 60))
        self.assertGreaterEqual(len(reasons), 3)


class GraphHashTests(unittest.TestCase):
    def test_weather_changes_do_not_change_the_hash(self):
        G = lattice()
        before = graph_hash(G)
        apply_weather(G, snap(35, 45, 50))
        self.assertEqual(graph_hash(G), before)

    def test_static_changes_do_change_the_hash(self):
        G = lattice()
        before = graph_hash(G)
        u, v = next(iter(G.edges))
        G[u][v]["building"] = 0.9
        self.assertNotEqual(graph_hash(G), before)
        G2 = lattice()
        G2.remove_node(next(iter(G2.nodes)))
        self.assertNotEqual(graph_hash(G2), before)

    def test_same_build_gives_the_same_hash(self):
        self.assertEqual(graph_hash(lattice()), graph_hash(lattice()))

    def test_hand_made_graphs_fall_back_to_costs(self):
        G = random_graph(1)
        h = graph_hash(G)
        self.assertEqual(h, graph_hash(random_graph(1)))
        u, v = next(iter(G.edges))
        G[u][v]["risk"] += 0.5
        self.assertNotEqual(graph_hash(G), h)


class StoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.store = ParetoStore(self.tmp.name)
        self.G = lattice()
        self.s, self.t = 0, self.G.number_of_nodes() - 1

    def tearDown(self):
        self.tmp.cleanup()

    def build(self, **kw):
        kw.setdefault("now", T0)
        return quiet_get(self.store, self.G, self.s, self.t, **kw)

    def test_missing_record_is_built_and_saved_with_full_metadata(self):
        record, rebuilt, reasons = self.build()
        self.assertTrue(rebuilt)
        self.assertEqual(reasons, ["no stored front"])
        for key in ("location", "graph_hash", "graph_timestamp", "weather_timestamp", "weather_snapshot",
                    "source", "target", "created_at", "valid_until", "truncated", "front"):
            self.assertIn(key, record)
        self.assertEqual(record["graph_hash"], graph_hash(self.G))
        self.assertEqual(record["weather_timestamp"], "2026-10-05T14:30")
        self.assertEqual(record["weather_snapshot"]["wind_speed_kmh"], 20)
        self.assertEqual((record["source"], record["target"]), (self.s, self.t))
        self.assertFalse(record["truncated"])
        self.assertEqual(record["graph_timestamp"], self.G.graph["built_at"])
        self.assertEqual(datetime.fromisoformat(record["valid_until"]) - datetime.fromisoformat(record["created_at"]),
                         timedelta(hours=cfg.FRONT_VALID_HOURS))
        self.assertTrue(Path(self.store.path(record["location"], self.s, self.t)).exists())

    def test_record_round_trips_through_json_and_matches_the_engine(self):
        record, *_ = self.build()
        loaded = self.store.load(record["location"], self.s, self.t)
        self.assertEqual(loaded, json.loads(json.dumps(record)))
        with redirect_stdout(io.StringIO()):
            exact = TierOneEngine(self.G, self.s, self.t).run()
        self.assertEqual(front_from_record(loaded), exact)

    def test_second_call_reuses_without_running_tier1(self):
        first, *_ = self.build()
        with mock.patch.object(pareto_store.TierOneEngine, "run", side_effect=AssertionError("Tier 1 ran")):
            second, rebuilt, reasons = self.build(now=T0 + timedelta(hours=1))
        self.assertFalse(rebuilt)
        self.assertEqual(reasons, [])
        self.assertEqual(second, first)

    def test_expiry_triggers_a_rebuild(self):
        self.build()
        _, rebuilt, reasons = self.build(now=T0 + timedelta(hours=cfg.FRONT_VALID_HOURS, minutes=1))
        self.assertTrue(rebuilt)
        self.assertIn("expired", reasons[0])
        _, rebuilt, _ = self.build(now=T0 + timedelta(hours=cfg.FRONT_VALID_HOURS - 0.01))
        self.assertFalse(rebuilt)                                    # (it was re-saved fresh above)

    def test_custom_validity_window(self):
        record, *_ = self.build(valid_hours=0.5)
        self.assertEqual(datetime.fromisoformat(record["valid_until"]), T0 + timedelta(minutes=30))

    def test_small_weather_change_reuses_and_leaves_the_graph_alone(self):
        self.build()
        before = {(u, v): d["energy"] for u, v, d in self.G.edges(data=True)}
        _, rebuilt, _ = self.build(weather=snap(24, 235, 33))
        self.assertFalse(rebuilt)
        self.assertEqual(before, {(u, v): d["energy"] for u, v, d in self.G.edges(data=True)})

    def test_drift_rebuilds_and_only_reapplies_weather(self):
        first, *_ = self.build()
        static_before = graph_hash(self.G)
        built_at = self.G.graph["built_at"]
        new_weather = snap(40, 45, 50, time="2026-10-05T16:00")
        record, rebuilt, reasons = self.build(weather=new_weather)
        self.assertTrue(rebuilt)
        self.assertGreaterEqual(len(reasons), 2)
        self.assertEqual(self.G.graph["weather"]["wind_speed_kmh"], 40)            # apply_weather ran
        self.assertEqual(graph_hash(self.G), static_before)                        # static layers untouched
        self.assertEqual(self.G.graph["built_at"], built_at)
        self.assertEqual(record["weather_snapshot"]["wind_speed_kmh"], 40)
        self.assertEqual(record["weather_timestamp"], "2026-10-05T16:00")
        self.assertNotEqual(front_from_record(record), front_from_record(first))

    def test_go_no_go_flip_rebuilds(self):
        self.build()
        _, rebuilt, reasons = self.build(weather=snap(20, 225, 28, weather_code=95))
        self.assertTrue(rebuilt)
        self.assertTrue(any("go/no-go" in r for r in reasons))

    def test_refresh_forces_a_rebuild(self):
        self.build()
        _, rebuilt, reasons = self.build(refresh=True)
        self.assertTrue(rebuilt)
        self.assertEqual(reasons, ["refresh requested"])

    def test_a_changed_static_graph_invalidates_the_record(self):
        self.build()
        u, v = next(iter(self.G.edges))
        self.G[u][v]["nfz"] = 0.5
        _, rebuilt, reasons = self.build()
        self.assertTrue(rebuilt)
        self.assertIn("static graph changed", reasons)

    def test_corrupt_file_is_rebuilt(self):
        record, *_ = self.build()
        Path(self.store.path(record["location"], self.s, self.t)).write_text("{not json")
        _, rebuilt, reasons = self.build()
        self.assertTrue(rebuilt)
        self.assertEqual(reasons, ["no stored front"])

    def test_default_weather_is_the_one_applied_to_the_graph(self):
        record, *_ = self.build()
        self.assertEqual(record["weather_snapshot"], self.G.graph["weather"])

    def test_truncation_is_recorded(self):
        G = lattice(box_m=2200.0)
        record, *_ = quiet_get(self.store, G, 0, G.number_of_nodes() - 1, max_explored=5, now=T0)
        self.assertTrue(record["truncated"])
        self.assertEqual(record["diagnostics"]["max_explored"], 5)

    def test_raised_cap_or_changed_thinning_rebuilds(self):
        G = lattice(box_m=2200.0)
        t = G.number_of_nodes() - 1
        quiet_get(self.store, G, 0, t, max_explored=5, now=T0)
        _, rebuilt, _ = quiet_get(self.store, G, 0, t, max_explored=5, now=T0)
        self.assertFalse(rebuilt)                                    # same cap: reuse
        record, rebuilt, reasons = quiet_get(self.store, G, 0, t, now=T0)
        self.assertTrue(rebuilt)
        self.assertTrue(any("search cap raised" in r for r in reasons))
        self.assertFalse(record["truncated"])
        _, rebuilt, reasons = quiet_get(self.store, G, 0, t, now=T0, front_max=3)
        self.assertTrue(rebuilt)
        self.assertTrue(any("front_max changed" in r for r in reasons))

    def test_thinning_is_off_by_default_and_opt_in(self):
        G = random_graph(1, extra_edges=45)
        full, *_ = quiet_get(self.store, G, 0, 11, now=T0, location="r1")
        self.assertIsNone(full["front_max"])
        self.assertEqual(full["front_size_exact"], len(full["front"]))
        thin, *_ = quiet_get(self.store, G, 0, 11, now=T0, location="r2", front_max=3)
        self.assertEqual(thin["front_max"], 3)
        self.assertLessEqual(len(thin["front"]), 3)
        self.assertEqual(thin["front_size_exact"], full["front_size_exact"])

    def test_locations_and_pairs_are_stored_separately(self):
        a, *_ = self.build(location="Region A")
        b, *_ = self.build(location="Region B")
        self.assertNotEqual(self.store.path("Region A", 0, 1), self.store.path("Region B", 0, 1))
        self.assertNotEqual(self.store.path("A", 0, 1), self.store.path("A", 0, 2))
        self.assertEqual(self.store.load("Region A", self.s, self.t)["location"], "Region A")
        self.assertEqual(self.store.load("Region B", self.s, self.t)["location"], "Region B")
        self.assertIsNone(self.store.load("Region C", self.s, self.t))


class IdentityTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.store = ParetoStore(self.tmp.name)
        self.G = lattice()
        self.s, self.t = 0, self.G.number_of_nodes() - 1

    def tearDown(self):
        self.tmp.cleanup()

    def get(self, **kw):
        return quiet_get(self.store, self.G, self.s, self.t, now=T0, **kw)

    def test_identity_is_stored_and_reused(self):
        ident = {"bundle_hash": "abc", "hard_clearance": True}
        record, rebuilt, _ = self.get(extra_identity=ident)
        self.assertTrue(rebuilt)
        self.assertEqual(record["identity"], ident)
        _, rebuilt, _ = self.get(extra_identity=dict(ident))
        self.assertFalse(rebuilt)

    def test_changed_identity_rebuilds(self):
        self.get(extra_identity={"bundle_hash": "abc"})
        _, rebuilt, reasons = self.get(extra_identity={"bundle_hash": "def"})
        self.assertTrue(rebuilt)
        self.assertIn("identity changed", reasons)

    def test_identity_is_ignored_when_not_given(self):
        self.get(extra_identity={"bundle_hash": "abc"})
        _, rebuilt, _ = self.get()
        self.assertFalse(rebuilt)


class RegionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.store = ParetoStore(self.tmp.name)
        self.G = lattice()

    def tearDown(self):
        self.tmp.cleanup()

    def region(self, depot, points, **kw):
        with redirect_stdout(io.StringIO()):
            return run_region(self.G, depot, points, store=self.store, now=T0, **kw)

    def test_snap_to_node_picks_the_nearest_node(self):
        n = self.G.nodes[24]
        self.assertEqual(snap_to_node(self.G, n["lat"] + 1e-5, n["lon"] - 1e-5), 24)

    def test_one_front_per_delivery_point(self):
        pts = [self.G.nodes[n] for n in (48, 42, 6)]
        out = self.region(0, [(p["lat"], p["lon"]) for p in pts])
        self.assertEqual(set(out), {48, 42, 6})
        for target, record in out.items():
            self.assertEqual((record["source"], record["target"]), (0, target))
            self.assertTrue(record["front"])
            for route in front_from_record(record):
                self.assertEqual((route[1][0], route[1][-1]), (0, target))

    def test_points_may_be_nodes_or_coordinates_and_are_deduplicated(self):
        near = self.G.nodes[30]
        out = self.region(0, [30, (near["lat"] + 1e-6, near["lon"]), 0, 12])
        self.assertEqual(set(out), {30, 12})                          # duplicate and depot skipped

    def test_depot_may_be_a_coordinate(self):
        d = self.G.nodes[0]
        out = self.region((d["lat"], d["lon"]), [48])
        self.assertEqual(out[48]["source"], 0)

    def test_second_run_reuses_every_front(self):
        self.region(0, [48, 42])
        with mock.patch.object(pareto_store.TierOneEngine, "run", side_effect=AssertionError("Tier 1 ran")):
            out = self.region(0, [48, 42])
        self.assertEqual(set(out), {48, 42})

    def test_kwargs_reach_get_or_build(self):
        out = self.region(0, [48], front_max=4, valid_hours=1.0)
        self.assertEqual(out[48]["front_max"], 4)
        self.assertEqual(datetime.fromisoformat(out[48]["valid_until"]), T0 + timedelta(hours=1))

    def test_unknown_node_raises(self):
        with self.assertRaises(ValueError):
            self.region(9999, [48])
        with self.assertRaises(ValueError):
            self.region(0, [9999])


if __name__ == "__main__":
    unittest.main()
