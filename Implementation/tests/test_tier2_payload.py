import sys
import unittest
from pathlib import Path


IMPLEMENTATION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IMPLEMENTATION_DIR))

from module4_tier2_payload import TierTwoSelector, derive_weights


class TierTwoPayloadTests(unittest.TestCase):
    def test_derive_weights_sums_to_one(self):
        w_risk, w_dist, w_energy = derive_weights(F=5, W=1, U=1)

        self.assertAlmostEqual(w_risk, 10 / 15)
        self.assertAlmostEqual(w_dist, 3 / 15)
        self.assertAlmostEqual(w_energy, 2 / 15)
        self.assertAlmostEqual(w_risk + w_dist + w_energy, 1.0)

    def test_normalisation_handles_objective_ranges(self):
        selector = TierTwoSelector([
            ([10.0, 20.0, 30.0], ["low"]),
            ([20.0, 40.0, 50.0], ["high"]),
        ])

        self.assertEqual(selector._normalise([10.0, 20.0, 30.0]), (0.0, 0.0, 0.0))
        self.assertEqual(selector._normalise([20.0, 40.0, 50.0]), (1.0, 1.0, 1.0))

    def test_priority_sharpens_weights(self):
        selector = TierTwoSelector([
            ([0.0, 8.0, 8.0], ["distance"]),
            ([8.0, 0.0, 8.0], ["energy"]),
            ([8.0, 8.0, 0.0], ["risk"]),
        ])

        _, w_risk, w_dist, w_energy, _ = selector.select(F=1, W=1, U=3, P=5)

        g = 1.07
        raw = (2 / 13) ** g, (9 / 13) ** g, (2 / 13) ** g
        self.assertAlmostEqual(w_risk, raw[0] / sum(raw))
        self.assertAlmostEqual(w_dist, raw[1] / sum(raw))
        self.assertAlmostEqual(w_energy, raw[2] / sum(raw))
        self.assertGreater(w_dist, 9 / 13)                      # dominant weight grows
        self.assertAlmostEqual(w_risk + w_dist + w_energy, 1.0)
        _, *w_low, _ = selector.select(F=1, W=1, U=3, P=3)       # P <= 3: plain weights
        self.assertAlmostEqual(w_low[1], 9 / 13)

    def test_priority_keeps_tag_monotonicity_at_weight_ties(self):
        # "a" = 0.05 shorter + lower energy, "b" = safer.  F=1 W=3: raw (2,3,6) at U=1,
        # (2,6,6) at U=2.  The old +0.1*w_max bonus switched from energy to distance at
        # the U=2 tie, shifted energy:risk 3.3 -> 3.0 and picked the LONGER "b" when
        # urgency rose (seen on real Thiruvanmiyur / Guindy fronts, 2 cm apart).
        front = [
            ([5.41, 4.25, 2.57], ["a"]),
            ([5.46, 5.16, 0.71], ["b"]),
            ([0.72, 8.48, 3.32], ["c"]),
        ]
        selector = TierTwoSelector(front)
        for P in (4, 5):
            u1, *_ = selector.select(F=1, W=3, U=1, P=P)
            u2, *_ = selector.select(F=1, W=3, U=2, P=P)
            self.assertLessEqual(u2[0][0], u1[0][0] + 1e-9)

    def test_selection_on_controlled_front(self):
        front = [
            ([0.0, 8.0, 8.0], ["distance"]),
            ([8.0, 0.0, 8.0], ["energy"]),
            ([8.0, 8.0, 0.0], ["risk"]),
            ([4.0, 4.0, 4.0], ["balanced"]),
        ]
        selector = TierTwoSelector(front)

        selected, *_ = selector.select(F=1, W=1, U=3, P=3)
        self.assertEqual(selected[1], ["distance"])

        selected, *_ = selector.select(F=1, W=3, U=1, P=3)
        self.assertEqual(selected[1], ["energy"])

        selected, *_ = selector.select(F=5, W=1, U=1, P=3)
        self.assertEqual(selected[1], ["risk"])

        selected, *_ = selector.select(F=3, W=2, U=2, P=3)
        self.assertEqual(selected[1], ["balanced"])


if __name__ == "__main__":
    unittest.main()
