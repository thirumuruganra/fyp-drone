import json
import sys
import tempfile
import unittest
from pathlib import Path

IMPLEMENTATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IMPLEMENTATION_DIR))

import config as cfg
from env_data import (WeatherSnapshot, fetch_weather, parse_open_meteo,
                      power_law_wind, seeded_weather)
from module1_graph_builder import (AirspaceGraphBuilder, EdgeFeatures,
                                   apply_weather, energy_cost, risk_components,
                                   wind_components)

FIXTURE = IMPLEMENTATION_DIR / "tests" / "fixtures" / "open_meteo_chennai.json"


def load_fixture() -> dict:
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


class FakeResponse:
    def __init__(self, payload):
        self._payload = payload

    def raise_for_status(self):
        pass

    def json(self):
        return self._payload


class WindComponentTests(unittest.TestCase):
    def test_wind_from_east_is_headwind_for_eastbound_edge(self):
        hw, cw = wind_components(bearing_deg=90, speed_kmh=30, dir_from_deg=90)
        self.assertAlmostEqual(hw, 30.0)
        self.assertAlmostEqual(cw, 0.0, places=9)

    def test_wind_from_east_is_tailwind_for_westbound_edge(self):
        hw, cw = wind_components(bearing_deg=270, speed_kmh=30, dir_from_deg=90)
        self.assertAlmostEqual(hw, -30.0)
        self.assertAlmostEqual(cw, 0.0, places=9)

    def test_crosswind_is_perpendicular_and_non_negative(self):
        for bearing in (0, 180):
            hw, cw = wind_components(bearing, 30, 90)
            self.assertAlmostEqual(hw, 0.0, places=9)
            self.assertAlmostEqual(cw, 30.0)

    def test_bearing_wraps_around_north(self):
        hw, _ = wind_components(bearing_deg=350, speed_kmh=20, dir_from_deg=10)
        self.assertAlmostEqual(hw, 20 * 0.9396926, places=5)   # cos(20°)


class EnergyAndRiskTests(unittest.TestCase):
    def setUp(self):
        self.east = EdgeFeatures.build(d_h_km=0.2, bearing_deg=90)
        self.west = EdgeFeatures.build(d_h_km=0.2, bearing_deg=270)
        self.wind = WeatherSnapshot(wind_speed_kmh=30, wind_dir_from_deg=90, gust_kmh=30)

    def test_calm_energy_equals_distance(self):
        calm = WeatherSnapshot.calm()
        self.assertAlmostEqual(energy_cost(self.east, calm), self.east.distance_km)

    def test_headwind_edge_costs_more_than_tailwind_edge(self):
        self.assertGreater(energy_cost(self.east, self.wind), energy_cost(self.west, self.wind))

    def test_energy_never_below_distance(self):
        for bearing in range(0, 360, 15):
            f = EdgeFeatures.build(0.2, bearing, dalt_m=-8.0)
            for speed in (0, 10, 40):
                for direction in (0, 90, 225):
                    w = WeatherSnapshot(wind_speed_kmh=speed, wind_dir_from_deg=direction)
                    self.assertGreaterEqual(energy_cost(f, w) + 1e-12, f.distance_km)

    def test_energy_uses_absolute_grade_and_3d_distance(self):
        f = EdgeFeatures.build(d_h_km=0.2, bearing_deg=0, dalt_m=20.0)
        self.assertAlmostEqual(f.grade, 0.1)
        self.assertAlmostEqual(f.distance_km, (0.2 ** 2 + 0.02 ** 2) ** 0.5)
        self.assertAlmostEqual(energy_cost(f, WeatherSnapshot.calm()), f.distance_km * 1.1)

    def test_wind_risk_is_crosswind_driven(self):
        north = EdgeFeatures.build(0.2, 0)     # wind from east → pure crosswind
        along = self.east                       # pure headwind
        r_cross = risk_components(north, self.wind)["wind"]
        r_head = risk_components(along, self.wind)["wind"]
        self.assertAlmostEqual(r_cross, cfg.RISK_WEIGHTS["wind"] * 30 / cfg.V_MAX_SAFE_KMH)
        self.assertAlmostEqual(r_head, 0.0, places=9)

    def test_gust_excess_raises_wind_risk(self):
        steady = WeatherSnapshot(wind_speed_kmh=20, wind_dir_from_deg=90, gust_kmh=20)
        gusty = WeatherSnapshot(wind_speed_kmh=20, wind_dir_from_deg=90, gust_kmh=36)
        f = EdgeFeatures.build(0.2, 90)
        self.assertGreater(risk_components(f, gusty)["wind"], risk_components(f, steady)["wind"])

    def test_weather_term_is_uniform_and_bounded(self):
        stormy = WeatherSnapshot(precipitation_mm=50, visibility_m=200)
        a = risk_components(EdgeFeatures.build(0.2, 0), stormy)["weather"]
        b = risk_components(EdgeFeatures.build(0.5, 137), stormy)["weather"]
        self.assertAlmostEqual(a, b)
        self.assertLessEqual(a, cfg.RISK_WEIGHTS["weather"] + 1e-12)

    def test_risk_clipped_to_unit_interval(self):
        f = EdgeFeatures.build(0.2, 0, obstacle=1.0)
        w = WeatherSnapshot(wind_speed_kmh=80, wind_dir_from_deg=90, gust_kmh=120,
                            precipitation_mm=50, visibility_m=0)
        G = AirspaceGraphBuilder(seed=1, weather=w)._build_synthetic_graph(16)
        self.assertTrue(all(0.0 <= d["risk"] <= 1.0 for _, _, d in G.edges(data=True)))
        self.assertGreater(sum(risk_components(f, w).values()), 1.0)


class ApplyWeatherTests(unittest.TestCase):
    def test_reapplying_weather_changes_costs_and_is_idempotent(self):
        G = AirspaceGraphBuilder(seed=7, weather=WeatherSnapshot.calm())._build_synthetic_graph(25)
        calm_energy = {(u, v): d["energy"] for u, v, d in G.edges(data=True)}
        windy = WeatherSnapshot(wind_speed_kmh=30, wind_dir_from_deg=90, gust_kmh=30)
        apply_weather(G, windy)
        first = {(u, v): d["energy"] for u, v, d in G.edges(data=True)}
        apply_weather(G, windy)
        second = {(u, v): d["energy"] for u, v, d in G.edges(data=True)}
        self.assertEqual(first, second)
        self.assertTrue(any(first[e] > calm_energy[e] for e in first))
        apply_weather(G, WeatherSnapshot.calm())
        self.assertEqual({(u, v): d["energy"] for u, v, d in G.edges(data=True)}, calm_energy)

    def test_cost_vector_matches_attributes(self):
        G = AirspaceGraphBuilder(seed=3)._build_synthetic_graph(25)
        for _, _, d in G.edges(data=True):
            self.assertEqual(d["cost"], [d["distance"], d["energy"], d["risk"]])
            if d["risk"] < 1.0:      # below the clip, components add up to the risk
                self.assertAlmostEqual(sum(d["risk_components"].values()), d["risk"], places=4)

    def test_synthetic_graph_is_deterministic(self):
        a = AirspaceGraphBuilder(seed=11)._build_synthetic_graph(36)
        b = AirspaceGraphBuilder(seed=11)._build_synthetic_graph(36)
        self.assertEqual([d["cost"] for *_, d in a.edges(data=True)],
                         [d["cost"] for *_, d in b.edges(data=True)])
        self.assertEqual(seeded_weather(11), seeded_weather(11))

    def test_obstacle_density_controls_obstacle_coverage(self):
        def covered(density):
            G = AirspaceGraphBuilder(seed=5, obstacle_density=density)._build_synthetic_graph(100)
            return sum(d["obstacle"] > 0 for *_, d in G.edges(data=True))
        self.assertEqual(covered(0.0), 0)
        self.assertGreater(covered(0.40), covered(0.05))


class WeatherFetchTests(unittest.TestCase):
    def test_hourly_row_matched_by_response_time_string(self):
        snap = parse_open_meteo(load_fixture())
        # current.time = 2026-10-05T14:15 (IST) → hourly row "…T14:00" (index 14), not UTC hour 8
        self.assertEqual(snap.wind_speed_kmh, 27.0)
        self.assertEqual(snap.wind_dir_from_deg, 228.0)
        self.assertEqual(snap.source, "open-meteo:80m")
        self.assertEqual(snap.wind_height_m, 80.0)

    def test_gust_never_below_mean_wind_and_uses_10m_gust_factor(self):
        snap = parse_open_meteo(load_fixture())
        self.assertAlmostEqual(snap.gust_kmh, 27.0 * 21.6 / 14.4, places=2)
        self.assertGreaterEqual(snap.gust_kmh, snap.wind_speed_kmh)

    def test_power_law_fallback_when_80m_row_missing(self):
        payload = load_fixture()
        payload["hourly"]["time"] = ["2000-01-01T00:00"]
        snap = parse_open_meteo(payload)
        self.assertEqual(snap.source, "open-meteo:10m-power-law")
        self.assertAlmostEqual(snap.wind_speed_kmh,
                               power_law_wind(14.4, 10.0, cfg.H_CRUISE_AGL), places=2)
        self.assertEqual(snap.wind_dir_from_deg, 228.0)

    def test_live_fetch_is_cached_and_reused_offline(self):
        payload = load_fixture()
        calls = []

        def fake_get(url, params=None, timeout=None):
            calls.append(params)
            return FakeResponse(payload)

        with tempfile.TemporaryDirectory() as tmp:
            live = fetch_weather(offline=False, cache_dir=tmp, http_get=fake_get)
            self.assertEqual(live.wind_speed_kmh, 27.0)
            self.assertEqual(len(calls), 1)
            self.assertIn("wind_speed_80m", calls[0]["hourly"])
            self.assertEqual(calls[0]["timezone"], "Asia/Kolkata")
            offline = fetch_weather(offline=True, cache_dir=tmp, http_get=fake_get)
            self.assertEqual(len(calls), 1)                      # no new request
            self.assertEqual(offline.wind_speed_kmh, 27.0)
            self.assertTrue(offline.source.startswith("cache"))

    def test_failed_fetch_falls_back_to_cache_then_calm(self):
        def broken_get(url, params=None, timeout=None):
            raise ConnectionError("blocked")

        with tempfile.TemporaryDirectory() as tmp:
            calm = fetch_weather(cache_dir=tmp, http_get=broken_get)
            self.assertEqual((calm.wind_speed_kmh, calm.source), (0.0, "calm"))
            good = lambda url, params=None, timeout=None: FakeResponse(load_fixture())
            fetch_weather(cache_dir=tmp, http_get=good)
            cached = fetch_weather(cache_dir=tmp, http_get=broken_get)
            self.assertEqual(cached.wind_speed_kmh, 27.0)

    def test_offline_without_cache_is_calm_and_deterministic(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = fetch_weather(offline=True, cache_dir=tmp)
            b = fetch_weather(offline=True, cache_dir=tmp)
        self.assertEqual(a, b)
        self.assertEqual(a.wind_speed_kmh, 0.0)

    def test_wind_override_replaces_wind_only(self):
        base = parse_open_meteo(load_fixture())
        w = base.with_wind(30, 90)
        self.assertEqual((w.wind_speed_kmh, w.wind_dir_from_deg, w.gust_kmh, w.source),
                         (30.0, 90.0, 30.0, "override"))
        self.assertEqual(w.visibility_m, base.visibility_m)


if __name__ == "__main__":
    unittest.main()
