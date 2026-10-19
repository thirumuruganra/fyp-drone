"""Offline regression tests for real Overpass failure modes and geometry parsing."""
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import requests
import config as cfg
from overpass_client import (OverpassBackoff, OverpassUnavailable, endpoint_base,
                             fetch_osm_features, request_overpass)

BASES = ("https://primary.example/api", "https://fallback.example/api")
QUERY = '[out:json][timeout:20];way["building"](around:300,12.9830,80.2594);out tags;'


def response(status=200, payload=None, content=b"", headers=None):
    r = Mock(status_code=status, text=content.decode(), content=content, headers=headers or {})
    r.json.return_value = {"elements": []} if payload is None else payload
    if status >= 400:
        r.raise_for_status.side_effect = requests.HTTPError(str(status), response=r)
    return r


class TransportTests(unittest.TestCase):
    def run_request(self, post, **kwargs):
        return request_overpass(QUERY, endpoints=BASES, post=post,
                                sleep=kwargs.pop("sleep", Mock()), report=Mock(), **kwargs)

    def test_reset_retries_then_falls_back_with_identifying_header_and_verified_tls(self):
        post = Mock(side_effect=[requests.ConnectionError("reset"), requests.ConnectionError("reset"),
                                 response(payload={"elements": [{"type": "way", "id": 1}]})])
        sleep = Mock()
        payload, base = self.run_request(post, sleep=sleep)
        self.assertEqual(base, BASES[1])
        self.assertEqual(len(payload["elements"]), 1)
        self.assertEqual([c.args[0] for c in post.call_args_list],
                         [BASES[0]+"/interpreter", BASES[0]+"/interpreter", BASES[1]+"/interpreter"])
        for call in post.call_args_list:
            self.assertTrue(call.kwargs["verify"])
            self.assertEqual(call.kwargs["headers"]["User-Agent"], cfg.OVERPASS_USER_AGENT)
            self.assertEqual(call.kwargs["data"], {"data": QUERY})
            self.assertEqual(call.kwargs["timeout"],
                             (cfg.OVERPASS_CONNECT_TIMEOUT_S, cfg.OVERPASS_READ_TIMEOUT_S))
        self.assertEqual(sleep.call_count, 2)

    def test_timeouts_stop_after_fixed_attempt_budget(self):
        post = Mock(side_effect=requests.Timeout("timeout"))
        with self.assertRaises(OverpassUnavailable):
            self.run_request(post)
        self.assertEqual(post.call_count, len(BASES)*cfg.OVERPASS_ATTEMPTS_PER_ENDPOINT)

    def test_certificate_failure_never_disables_verification(self):
        post = Mock(side_effect=requests.exceptions.SSLError("certificate verify failed"))
        with self.assertRaises(requests.exceptions.SSLError):
            self.run_request(post)
        self.assertEqual(post.call_count, 1)
        self.assertTrue(post.call_args.kwargs["verify"])

    def test_bad_query_stops_without_retry_or_failover(self):
        post = Mock(return_value=response(status=400))
        with self.assertRaises(requests.HTTPError):
            self.run_request(post)
        self.assertEqual(post.call_count, 1)

    def test_busy_http_respects_minimum_pause_and_retry_after(self):
        for status in (406, 429, 504):
            with self.subTest(status=status):
                post = Mock(side_effect=[response(status=status, headers={"Retry-After": "40"}), response()])
                sleep = Mock()
                self.run_request(post, sleep=sleep)
                sleep.assert_called_once_with(40)

    def test_long_retry_after_stops_without_early_retry(self):
        post = Mock(return_value=response(status=429, headers={"Retry-After": "120"}))
        with self.assertRaises(OverpassBackoff):
            self.run_request(post)
        self.assertEqual(post.call_count, 1)

    def test_partial_json_never_accepted_as_success(self):
        post = Mock(side_effect=[response(payload={"elements": [], "remark": "runtime error: timed out"}), response()])
        self.run_request(post)
        self.assertEqual(post.call_count, 2)

    def test_partial_xml_never_accepted_as_empty_layer(self):
        post = Mock(side_effect=[response(content=b'<osm><remark>timed out</remark></osm>'),
                                 response(content=b'<osm version="0.6"/>')])
        payload, _ = self.run_request(post, output="xml")
        self.assertEqual(payload, b'<osm version="0.6"/>')
        self.assertEqual(post.call_count, 2)

    def test_invalid_response_never_accepted(self):
        post = Mock(side_effect=[response(payload={"error": "proxy"}), response()])
        self.run_request(post)
        self.assertEqual(post.call_count, 2)

    def test_endpoint_validation_and_interpreter_normalization(self):
        self.assertEqual(endpoint_base(BASES[0]+"/interpreter/"), BASES[0])
        for url in ("http://example.org/api", "https://user:pass@example.org/api", "https://example.org/api?token=x"):
            with self.assertRaises(ValueError):
                endpoint_base(url)


class GeometryTests(unittest.TestCase):
    def test_union_query_recursion_and_public_parser_preserve_polygon_and_height(self):
        xml = b'''<osm version="0.6" generator="Overpass API">
        <node id="1" lat="12.9830" lon="80.2594"/>
        <node id="2" lat="12.9831" lon="80.2594"/>
        <node id="3" lat="12.9831" lon="80.2595"/>
        <node id="4" lat="12.9830" lon="80.2595"/>
        <way id="10"><nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/>
        <tag k="building" v="yes"/><tag k="height" v="12"/></way>
        </osm>'''
        with patch("overpass_client.request_overpass", return_value=(xml, BASES[0])) as request:
            gdf = fetch_osm_features(12.9830, 80.2594, 300, {"building": True, "amenity": ["hospital"]})
        self.assertEqual(len(gdf), 1)
        self.assertEqual(gdf.iloc[0].geometry.geom_type, "Polygon")
        self.assertEqual(gdf.iloc[0]["height"], "12")
        self.assertEqual(gdf.crs.to_epsg(), 4326)
        self.assertEqual(gdf.attrs["overpass_endpoint"], BASES[0])
        query = request.call_args.args[0]
        self.assertIn('nwr["building"](', query)
        self.assertIn('nwr["amenity"="hospital"](', query)
        self.assertIn('(._;>;);out body;', query)
        self.assertEqual(request.call_args.kwargs["output"], "xml")

    def test_multipolygon_relation_keeps_member_geometry_and_inner_hole(self):
        xml = b'''<osm version="0.6" generator="Overpass API">
        <node id="1" lat="12.9830" lon="80.2594"/>
        <node id="2" lat="12.9832" lon="80.2594"/>
        <node id="3" lat="12.9832" lon="80.2596"/>
        <node id="4" lat="12.9830" lon="80.2596"/>
        <node id="5" lat="12.98305" lon="80.25945"/>
        <node id="6" lat="12.98315" lon="80.25945"/>
        <node id="7" lat="12.98315" lon="80.25955"/>
        <node id="8" lat="12.98305" lon="80.25955"/>
        <way id="10"><nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/></way>
        <way id="11"><nd ref="5"/><nd ref="6"/><nd ref="7"/><nd ref="8"/><nd ref="5"/></way>
        <relation id="20"><member type="way" ref="10" role="outer"/>
        <member type="way" ref="11" role="inner"/>
        <tag k="type" v="multipolygon"/><tag k="building" v="yes"/>
        <tag k="height" v="24"/></relation></osm>'''
        with patch("overpass_client.request_overpass", return_value=(xml, BASES[0])):
            gdf = fetch_osm_features(12.9830, 80.2594, 300, {"building": True})
        self.assertEqual(list(gdf.index), [("relation", 20)])
        self.assertEqual(len(gdf.iloc[0].geometry.interiors), 1)
        self.assertEqual(gdf.iloc[0]["height"], "24")

    def test_layer_and_road_live_branches_use_verified_client(self):
        from env_data import _empty_layer, fetch_osm_layers, fetch_osm_roads, OSM_LAYER_TAGS
        with tempfile.TemporaryDirectory() as folder:
            with patch("overpass_client.fetch_osm_features", return_value=_empty_layer()) as fetch:
                layers = fetch_osm_layers(cache_dir=folder)
                self.assertEqual(set(layers), set(OSM_LAYER_TAGS))
                self.assertEqual(fetch.call_count, len(OSM_LAYER_TAGS))
                fetch.reset_mock()
                self.assertEqual(len(fetch_osm_roads(cache_dir=folder)), 0)
                fetch.assert_called_once_with(cfg.CENTER_LAT, cfg.CENTER_LON,
                                              cfg.OSM_FETCH_RADIUS_M, {"highway": True})

    def test_invalid_tags_or_coordinates_fail_before_network(self):
        with patch("overpass_client.request_overpass") as request:
            for args in ((91, 80, 300, {"building": True}), (12, 80, -1, {"building": True}),
                         (12, 80, 300, {}), (12, 80, 300, {"building": 1})):
                with self.assertRaises((ValueError, TypeError)):
                    fetch_osm_features(*args)
            request.assert_not_called()


if __name__ == "__main__":
    unittest.main()
