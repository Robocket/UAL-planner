#!/usr/bin/env python3
import math
import sys
import unittest
from pathlib import Path

from geometry_msgs.msg import Quaternion


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from odom_gravity_adapter import body_up_from_orientation  # noqa: E402


def quaternion(x, y, z, w):
    return Quaternion(x=x, y=y, z=z, w=w)


class OdomGravityAdapterTest(unittest.TestCase):
    def assert_vector_almost_equal(self, actual, expected, places=11):
        self.assertIsNotNone(actual)
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, places=places)

    def test_body_up_known_orientations(self):
        cases = [
            (quaternion(0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 1.0)),
            (quaternion(0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)),
             (0.0, 0.0, 1.0)),
            (quaternion(math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)),
             (0.0, 1.0, 0.0)),
            (quaternion(0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)),
             (-1.0, 0.0, 0.0)),
        ]
        for orientation, expected in cases:
            with self.subTest(orientation=orientation):
                self.assert_vector_almost_equal(
                    body_up_from_orientation(orientation, 1.0), expected)

    def test_body_up_from_provided_gazebo_sample(self):
        orientation = quaternion(
            0.010435249024591206,
            0.00010068152298617687,
            0.0006702995907206518,
            0.999945321574877,
        )
        self.assert_vector_almost_equal(
            body_up_from_orientation(orientation, 1.0),
            (-0.000187362549, 0.020869491857, 0.999782190882),
        )

    def test_body_up_rejects_zero_quaternion(self):
        self.assertIsNone(body_up_from_orientation(
            quaternion(0.0, 0.0, 0.0, 0.0), 1.0))


if __name__ == "__main__":
    unittest.main()
