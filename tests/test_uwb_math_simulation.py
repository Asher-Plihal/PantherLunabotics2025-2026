import math
import unittest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from library.uwb_math import compute_distance_to_anchor, compute_heading_offset


class TestComputeDistanceToAnchor(unittest.TestCase):

    def test_symmetric_robot(self):
        # Robot directly facing anchor, tags symmetric. 3-4-5 triangle: d_L=d_R=5, W=6 → d_center=4
        self.assertAlmostEqual(compute_distance_to_anchor(5.0, 5.0, 6.0), 4.0, places=6)

    def test_broadside_right_tag_closer(self):
        # Robot broadside, left tag at 2m, right tag at 3m, sep=1m → d_center=2.5
        self.assertAlmostEqual(compute_distance_to_anchor(2.0, 3.0, 1.0), 2.5, places=6)

    def test_broadside_left_tag_closer(self):
        # Symmetric to above with tags swapped
        self.assertAlmostEqual(compute_distance_to_anchor(3.0, 2.0, 1.0), 2.5, places=6)

    def test_30_degree_angle(self):
        # d_L=sqrt(7), d_R=sqrt(13), W=2 → d_center=3 (exact)
        self.assertAlmostEqual(
            compute_distance_to_anchor(math.sqrt(7), math.sqrt(13), 2.0), 3.0, places=6
        )

    def test_distance_is_positive(self):
        self.assertGreater(compute_distance_to_anchor(5.0, 5.0, 6.0), 0)

    def test_distance_is_symmetric(self):
        # Swapping left/right does not change d_center
        self.assertAlmostEqual(
            compute_distance_to_anchor(3.0, 4.0, 2.0),
            compute_distance_to_anchor(4.0, 3.0, 2.0),
            places=6,
        )


class TestComputeHeadingOffset(unittest.TestCase):

    def test_symmetric_gives_zero(self):
        self.assertAlmostEqual(compute_heading_offset(5.0, 5.0, 6.0), 0.0, places=6)

    def test_broadside_right_tag_farther_gives_positive_90(self):
        self.assertAlmostEqual(compute_heading_offset(2.0, 3.0, 1.0), 90.0, places=6)

    def test_broadside_left_tag_farther_gives_negative_90(self):
        self.assertAlmostEqual(compute_heading_offset(3.0, 2.0, 1.0), -90.0, places=6)

    def test_30_degree_angle(self):
        # d_L=sqrt(7), d_R=sqrt(13), W=2, d_center=3 → sin(offset)=6/12=0.5 → 30°
        self.assertAlmostEqual(
            compute_heading_offset(math.sqrt(7), math.sqrt(13), 2.0), 30.0, places=6
        )

    def test_negative_30_degree_angle(self):
        # Swap left/right from above → -30°
        self.assertAlmostEqual(
            compute_heading_offset(math.sqrt(13), math.sqrt(7), 2.0), -30.0, places=6
        )

    def test_offset_is_antisymmetric(self):
        # Swapping left/right negates the offset
        self.assertAlmostEqual(
            compute_heading_offset(3.0, 4.0, 2.0),
            -compute_heading_offset(4.0, 3.0, 2.0),
            places=6,
        )


if __name__ == "__main__":
    unittest.main()
