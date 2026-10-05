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

    def test_priority_amplifies_dominant_weight(self):
        selector = TierTwoSelector([
            ([0.0, 8.0, 8.0], ["distance"]),
            ([8.0, 0.0, 8.0], ["energy"]),
            ([8.0, 8.0, 0.0], ["risk"]),
        ])

        _, w_risk, w_dist, w_energy, _ = selector.select(F=1, W=1, U=3, P=5)

        self.assertAlmostEqual(w_risk, 0.14388489208633093)
        self.assertAlmostEqual(w_dist, 0.7122302158273381)
        self.assertAlmostEqual(w_energy, 0.14388489208633093)
        self.assertAlmostEqual(w_risk + w_dist + w_energy, 1.0)

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
