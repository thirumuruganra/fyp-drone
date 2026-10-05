import csv
import sys
import tempfile
import unittest
from pathlib import Path

IMPLEMENTATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IMPLEMENTATION_DIR))

try:
    import geopandas  # noqa: F401
    import shapely    # noqa: F401
    HAVE_GEO = True
except ImportError:                                           # pragma: no cover
    HAVE_GEO = False


@unittest.skipUnless(HAVE_GEO, "shapely/geopandas not installed")
class AblationSmokeTest(unittest.TestCase):
    def test_small_offline_run_writes_every_output(self):
        from run_ablation import VARIANTS, parse_args, run_ablation
        with tempfile.TemporaryDirectory() as tmp:
            rows = run_ablation(parse_args(["--offline", "--box-km", "2.4", "--max-explored", "20000",
                                            "--output-dir", tmp]))
            names = {p.name for p in Path(tmp).iterdir()}
            self.assertEqual(names, {"ablation_fronts.png", "edge_risk_maps.png",
                                     "edge_risk_components.png", "ablation_summary.csv",
                                     "ablation_summary.json"})
            with open(Path(tmp) / "ablation_summary.csv", newline="") as fh:
                csv_rows = list(csv.DictReader(fh))
        self.assertEqual([r["variant"] for r in csv_rows], list(VARIANTS))
        by_name = {r["variant"]: r for r in rows}
        for r in rows:
            self.assertGreaterEqual(r["front_size"], 1)
        # calm air removes the wind and gust terms, so mean edge risk must drop
        self.assertLess(by_name["no_wind"]["mean_rho"], by_name["all"]["mean_rho"])


if __name__ == "__main__":
    unittest.main()
