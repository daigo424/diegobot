import pytest
from amr_interfaces.action import PickObject
from amr_interfaces.msg import DetectedObject

from amr_robot_tools.robot_tools import code_name, create_app, object_info

TestClient = pytest.importorskip('fastapi.testclient').TestClient


def detected(x, y):
    obj = DetectedObject(label='blue block', color='blue', score=0.734, position_valid=True)
    obj.position.point.x, obj.position.point.y = x, y
    return obj


class FakeNode:
    def __init__(self, status='SUCCESS'):
        self.status = status
        self.calls = []
        self.holding, self.held = False, None
        self.last_action, self.last_status = 'none', 'none'

    def run(self, name, action_type, goal):
        self.calls.append((name, goal))
        self.last_action, self.last_status = name, self.status
        return self.status, 'done', detected(0.18, 0.0)


def test_code_name():
    assert code_name(PickObject.Result, PickObject.Result.OUT_OF_REACH) == 'OUT_OF_REACH'
    assert code_name(PickObject.Result, 99) == '99'


def test_object_info_is_distance_and_direction():
    info = object_info(detected(0.3, 0.3))
    assert info.distance_m == pytest.approx(0.42)
    assert info.direction_deg == 45
    assert object_info(DetectedObject()) is None


def test_pick_then_place_tracks_holding():
    node = FakeNode()
    client = TestClient(create_app(node, ''))
    res = client.post('/pick', json={'queries': ['block', 'cube'], 'color': 'blue'}).json()
    assert res['success'] and res['object']['label'] == 'blue block'
    assert list(node.calls[0][1].queries) == ['block', 'cube']
    assert client.get('/state').json()['holding'] is True
    client.post('/place', json={})
    assert client.get('/state').json()['holding'] is False


def test_failed_pick_is_not_success():
    client = TestClient(create_app(FakeNode('OUT_OF_REACH'), ''))
    res = client.post('/pick', json={'queries': ['block']}).json()
    assert res['success'] is False and res['status'] == 'OUT_OF_REACH'


def test_api_key_is_required_when_set():
    client = TestClient(create_app(FakeNode(), 'secret'))
    assert client.get('/state').status_code == 401
    assert client.get('/state', headers={'Authorization': 'Bearer secret'}).status_code == 200
