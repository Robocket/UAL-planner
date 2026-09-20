import math
import sys
from pathlib import Path

import pytest
from geometry_msgs.msg import Quaternion


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from odom_gravity_adapter import body_up_from_orientation  # noqa: E402


def quaternion(x, y, z, w):
    return Quaternion(x=x, y=y, z=z, w=w)


@pytest.mark.parametrize("orientation, expected", [
    (quaternion(0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 1.0)),
    (quaternion(0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)),
     (0.0, 0.0, 1.0)),
    (quaternion(math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)),
     (0.0, 1.0, 0.0)),
    (quaternion(0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)),
     (-1.0, 0.0, 0.0)),
])
def test_body_up_known_orientations(orientation, expected):
    actual = body_up_from_orientation(orientation, 1.0)
    assert actual == pytest.approx(expected, abs=1.0e-12)


def test_body_up_from_provided_gazebo_sample():
    orientation = quaternion(
        0.010435249024591206,
        0.00010068152298617687,
        0.0006702995907206518,
        0.999945321574877,
    )
    assert body_up_from_orientation(orientation, 1.0) == pytest.approx(
        (-0.000187362549, 0.020869491857, 0.999782190882),
        abs=1.0e-12,
    )


def test_body_up_rejects_zero_quaternion():
    assert body_up_from_orientation(
        quaternion(0.0, 0.0, 0.0, 0.0), 1.0) is None
