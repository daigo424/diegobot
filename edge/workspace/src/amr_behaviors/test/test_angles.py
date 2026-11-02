import math

import pytest
from geometry_msgs.msg import Quaternion

from amr_behaviors.behavior_server import wrap, yaw_of


@pytest.mark.parametrize('angle, expected', [(0.0, 0.0), (math.pi + 0.1, -math.pi + 0.1), (-3 * math.pi / 2, math.pi / 2)])
def test_wrap(angle, expected):
    assert wrap(angle) == pytest.approx(expected)


@pytest.mark.parametrize('yaw', [0.0, 1.0, -2.5, 3.0])
def test_yaw_of(yaw):
    q = Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    assert yaw_of(q) == pytest.approx(yaw)
