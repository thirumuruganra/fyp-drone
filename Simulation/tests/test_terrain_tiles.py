import io
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
from PIL import Image

from terrain_tiles import TerrariumElevationProvider, decode_terrarium, global_pixel
import prepare_sim_region


def tile_png(height):
    value = int(round((height + 32768) * 256))
    rgb = np.empty((256, 256, 3), dtype=np.uint8)
    rgb[:] = (value >> 16, (value >> 8) & 255, value & 255)
    buffer = io.BytesIO()
    Image.fromarray(rgb).save(buffer, format="PNG")
    return buffer.getvalue()


class TerrainTileTests(unittest.TestCase):
    def test_encoding_preserves_negative_and_fractional_heights(self):
        for height in (-12.25, 0, 38.5, 1000.75):
            np.testing.assert_allclose(decode_terrarium(tile_png(height)), height)

    def test_projection_orientation_and_coverage(self):
        np.testing.assert_allclose(global_pixel(0, 0, 1), (255.5, 255.5))
        self.assertLess(global_pixel(10, 0, 1)[1], global_pixel(0, 0, 1)[1])
        with self.assertRaises(ValueError):
            global_pixel(90, 0, 1)

    def test_bilinear_sampling_crosses_tile_boundaries_and_reuses_disk_cache(self):
        def get(url, **kwargs):
            x, y = [int(v.split(".")[0]) for v in url.split("/")[-2:]]
            response = Mock(content=tile_png(10 + 10 * x + 20 * y))
            return response
        with tempfile.TemporaryDirectory() as cache:
            getter = Mock(side_effect=get)
            provider = TerrariumElevationProvider(cache, zoom=1, http_get=getter)
            self.assertEqual(provider.fetch([(0, 0)]), [25])
            self.assertEqual(getter.call_count, 4)
            cached = TerrariumElevationProvider(cache, zoom=1,
                       http_get=Mock(side_effect=AssertionError("unexpected network")))
            self.assertEqual(cached.fetch([(0, 0)]), [25])

    def test_rejects_non_tile_images(self):
        data = io.BytesIO()
        Image.new("RGB", (8, 8)).save(data, format="PNG")
        with self.assertRaises(ValueError):
            decode_terrarium(data.getvalue())

    def test_keyless_cli_never_loads_dotenv_or_opentopography(self):
        with patch.object(prepare_sim_region, "load_dotenv") as dotenv, \
             patch.object(prepare_sim_region, "build_bundle") as build:
            prepare_sim_region.main(["--keyless", "--strict"])
            dotenv.assert_not_called()
            providers = build.call_args.kwargs["elevation_providers"]
            self.assertEqual([p.name for p in providers], ["aws-terrarium-z12", "open-meteo"])


if __name__ == "__main__":
    unittest.main()
