import cv2
import numpy as np
import pytest
import rclpy
import rclpy.parameter
from amr_interfaces.msg import DetectedObject
from amr_interfaces.srv import FindObjects, ResolveObject

object_resolver = pytest.importorskip('amr_perception.object_resolver')


def candidate(x, y, size_px, score=0.5, width=0.05):
    obj = DetectedObject(label='figurine', score=score, position_valid=True, width=width, height=width,
                         crop_px=float(size_px))
    obj.bbox.center.position.x, obj.bbox.center.position.y = x, y
    obj.bbox.size_x = obj.bbox.size_y = float(size_px)
    obj.position.header.frame_id = 'base_link'
    return obj


def found(objects):
    image = np.full((480, 640, 3), 200, np.uint8)
    ok, jpeg = cv2.imencode('.jpg', image)
    response = FindObjects.Response(success=True, objects=objects)
    response.image.format, response.image.data = 'jpeg', jpeg.tobytes()
    return response


@pytest.fixture
def resolver():
    rclpy.init()
    answers = []
    node = object_resolver.ObjectResolver(chooser=lambda jpeg, text, effort: answers.pop(0))
    yield node, answers
    node.destroy_node()
    rclpy.shutdown()


def run(node, objects):
    node.find = lambda queries, color: found(objects)
    return node.resolve(ResolveObject.Request(target_name='ハチワレ'), ResolveObject.Response())


def test_parse_answer():
    assert object_resolver.parse_answer('2', 3) == 2
    assert object_resolver.parse_answer('1は髪が合う。3は服が違う。\n答え: 1', 3) == 1
    assert object_resolver.parse_answer('答え：0', 3) == 0
    assert object_resolver.parse_answer('答え: -1', 3) == -1
    assert object_resolver.parse_answer('答え: 5', 3) is None
    assert object_resolver.parse_answer('1番と2番を比べると', 3) is None


def test_parse_verdict():
    assert object_resolver.parse_verdict('髪: 写っている\n判定: 合う') == '合う'
    assert object_resolver.parse_verdict('判定: 合わない') == '合わない'
    assert object_resolver.parse_verdict('分からない') is None


def test_grid_has_one_tile_per_candidate():
    image = np.zeros((480, 640, 3), np.uint8)
    grid = object_resolver.make_grid(image, [candidate(100, 100, 50), candidate(600, 450, 80)], 64, 0.15)
    assert grid.shape == (64, 128, 3)


def test_picks_the_numbered_candidate(resolver):
    node, answers = resolver
    answers.append('2')
    res = run(node, [candidate(200, 240, 150, score=0.9), candidate(420, 240, 150, score=0.4)])
    assert res.error_code == ResolveObject.Response.SUCCESS
    assert res.object.bbox.center.position.x == 420


def test_no_match_returns_only_unseen_candidates(resolver):
    node, answers = resolver
    answers.append('0')
    res = run(node, [candidate(200, 240, 150), candidate(500, 100, 40)])
    assert res.error_code == ResolveObject.Response.NO_MATCH
    assert [o.bbox.center.position.x for o in res.candidates] == [500]


def test_too_small_candidates_are_not_shown_to_the_model(resolver):
    node, answers = resolver
    res = run(node, [candidate(200, 240, 60)])
    assert res.error_code == ResolveObject.Response.TOO_FAR and len(res.candidates) == 1
    assert answers == []


def test_retries_once_on_an_unreadable_answer(resolver):
    node, answers = resolver
    answers.extend(['わかりません', '-1'])
    res = run(node, [candidate(200, 240, 150)])
    assert res.error_code == ResolveObject.Response.AMBIGUOUS


def test_large_objects_and_no_candidates(resolver):
    node, answers = resolver
    res = run(node, [candidate(200, 240, 150, width=0.4)])
    assert res.error_code == ResolveObject.Response.NOT_FOUND


def test_verification_can_reject_the_choice(resolver):
    node, answers = resolver
    node.set_parameters([rclpy.parameter.Parameter('verify', value=True)])
    answers.extend(['答え: 1', '帽子: 写っていない\n判定: 合わない'])
    res = run(node, [candidate(200, 240, 150)])
    assert res.error_code == ResolveObject.Response.NO_MATCH
    answers.extend(['答え: 1', '判定: 合う'])
    assert run(node, [candidate(200, 240, 150)]).error_code == ResolveObject.Response.SUCCESS
