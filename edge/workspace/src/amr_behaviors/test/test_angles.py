import math

import pytest
from geometry_msgs.msg import Quaternion

from amr_behaviors.behavior_server import yaw_of


@pytest.mark.parametrize('yaw', [0.0, 1.0, -2.5, 3.0])
def test_yaw_of(yaw):
    q = Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    assert yaw_of(q) == pytest.approx(yaw)
