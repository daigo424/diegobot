import pytest
from amr_interfaces.action import ApproachObject, PickObject, SearchObject
from amr_interfaces.msg import DetectedObject

from amr_robot_tools.robot_tools import TargetMemory, code_name, create_app, object_info

TestClient = pytest.importorskip('fastapi.testclient').TestClient


def detected(x, y, frame='base_link'):
    obj = DetectedObject(label='blue block', color='blue', score=0.734, position_valid=True)
    obj.position.header.frame_id = frame
    obj.position.point.x, obj.position.point.y = x, y
    return obj


class FakeNode:
    def __init__(self, status='SUCCESS'):
        self.status = status
        self.calls = []
        self.holding, self.held = False, None
        self.last_action, self.last_status = 'none', 'none'
        self.target = TargetMemory(120.0)

    def run(self, name, action_type, goal):
        self.calls.append((name, goal))
        self.last_action, self.last_status = name, self.status
        result = action_type.Result(message='done', object=detected(0.18, 0.0))
        if action_type is ApproachObject:
            result.object_odom = detected(1.2, 0.3, 'odom')
        if action_type is SearchObject and self.status == 'FOUND_UNIDENTIFIED':
            result.candidates = [detected(0.5, 0.1), detected(0.6, -0.1)]
        return self.status, 'done', result


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


def test_approached_target_is_passed_to_pick_once():
    node = FakeNode()
    client = TestClient(create_app(node, ''))
    body = {'queries': ['figurine'], 'target_name': 'ハチワレ'}
    client.post('/approach', json=body)
    assert node.calls[0][1].target_name == 'ハチワレ'
    client.post('/pick', json=body)
    passed = node.calls[1][1].target_object
    assert passed.position.header.frame_id == 'odom' and passed.position.point.x == pytest.approx(1.2)
    client.post('/pick', json=body)
    assert node.calls[2][1].target_object.position.header.frame_id == ''


@pytest.mark.parametrize('between', ['/search', '/place'])
def test_other_actions_drop_the_kept_target(between):
    node = FakeNode()
    client = TestClient(create_app(node, ''))
    body = {'queries': ['figurine'], 'target_name': 'ハチワレ'}
    client.post('/approach', json=body)
    client.post(between, json=body if between == '/search' else {})
    client.post('/pick', json=body)
    assert node.calls[-1][1].target_object.position.header.frame_id == ''


def test_target_is_kept_without_a_name_but_not_after_failure():
    node = FakeNode()
    client = TestClient(create_app(node, ''))
    client.post('/approach', json={'queries': ['block']})
    client.post('/pick', json={'queries': ['block']})
    assert node.calls[-1][1].target_object.position.header.frame_id == 'odom'
    node.status = 'NO_MATCH'
    client.post('/approach', json={'queries': ['figurine'], 'target_name': 'ハチワレ'})
    node.status = 'SUCCESS'
    client.post('/pick', json={'queries': ['figurine'], 'target_name': 'ハチワレ'})
    assert node.calls[-1][1].target_object.position.header.frame_id == ''


def test_kept_target_expires():
    memory = TargetMemory(0.0)
    memory.keep(detected(1.0, 0.0, 'odom'))
    assert memory.take() is None


def test_unidentified_candidates_are_counted():
    client = TestClient(create_app(FakeNode('FOUND_UNIDENTIFIED'), ''))
    res = client.post('/search', json={'queries': ['figurine'], 'target_name': 'ハチワレ'}).json()
    assert res['success'] is False and res['candidate_count'] == 2
