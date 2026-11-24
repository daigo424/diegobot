import math
import os
import threading
import time

import rclpy
import uvicorn
from amr_interfaces.action import ApproachObject, PickObject, PlaceObject, SearchObject
from amr_interfaces.msg import DetectedObject
from amr_manipulation.ros_util import send
from fastapi import Depends, FastAPI, HTTPException
from fastapi.openapi.docs import get_redoc_html
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer
from pydantic import BaseModel, Field
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

QUERIES_DESCRIPTION = (
    'English short noun phrases for the object, including paraphrases because the detector recognizes '
    'some wordings better than others (e.g. ["block", "cube", "box"]). Do not put the color here.')
COLOR_DESCRIPTION = (
    'Color name to filter by, or empty for any color. One of: red, orange, yellow, green, cyan, blue, purple, '
    'pink, white, gray, black.')
TARGET_NAME_DESCRIPTION = (
    "A specific object named in the user's own words, without translating (e.g. a character name). Leave empty "
    'for a generic object. When set, put broad categories in queries (e.g. ["figurine", "toy figure"]); the robot '
    'identifies the object by appearance and ignores color.')


class Target(BaseModel):
    queries: list[str] = Field(..., description=QUERIES_DESCRIPTION)
    color: str = Field('', description=COLOR_DESCRIPTION)
    target_name: str = Field('', description=TARGET_NAME_DESCRIPTION)


class PlaceRequest(BaseModel):
    release_here: bool = Field(False, description='Just open the gripper where the arm is, instead of putting the '
                                                  'object down on the floor in front of the robot.')


class ObjectInfo(BaseModel):
    label: str
    color: str
    score: float
    distance_m: float = Field(..., description='Horizontal distance from the robot.')
    direction_deg: float = Field(..., description='Direction from the robot front; positive is left.')


class ActionResult(BaseModel):
    success: bool
    status: str = Field(..., description='Result code, e.g. SUCCESS, NOT_FOUND, OUT_OF_REACH.')
    message: str
    object: ObjectInfo | None = None
    candidate_count: int | None = Field(None, description='Number of candidates not identified yet (with '
                                                          'FOUND_UNIDENTIFIED or NO_MATCH).')


class State(BaseModel):
    holding: bool = Field(..., description='Whether the robot thinks it is holding an object.')
    held_object: ObjectInfo | None = None
    last_action: str
    last_status: str


def code_name(result_type, code):
    """アクションの結果の型の定数(SUCCESS=0等)から、値に対応する名前を引く。"""
    names = {getattr(result_type, n): n for n in dir(result_type) if n.isupper() and isinstance(getattr(result_type, n), int)}
    return names.get(code, str(code))


def object_info(obj, with_color=True):
    if obj is None or not obj.position_valid:
        return None
    p = obj.position.point
    # 色は物の画素の多数決で、名前で指定した物の見た目の説明には使えないため、色で探したときだけ返す。
    return ObjectInfo(label=obj.label, color=obj.color if with_color else '', score=round(obj.score, 2),
                      distance_m=round(math.hypot(p.x, p.y), 2), direction_deg=round(math.degrees(math.atan2(p.y, p.x))))


class TargetMemory:
    """近づく動作が見分けて測った対象(odom基準)を、次のつかむ動作まで持っておく。
    別の物へ向かった後で前の物を渡さないよう、つかむ以外の動作を始めたときと、時間が経ったときに捨てる。"""

    def __init__(self, lifetime):
        self.lifetime = lifetime
        self.kept = None

    def keep(self, obj):
        self.kept = (obj, time.monotonic())

    def drop(self):
        self.kept = None

    def take(self):
        kept, self.kept = self.kept, None
        if kept is None or time.monotonic() - kept[1] > self.lifetime:
            return None
        return kept[0]


class RobotToolsNode(Node):
    def __init__(self):
        super().__init__('robot_tools')
        self.timeout = self.declare_parameter('action_timeout', 300.0).value
        self.target = TargetMemory(self.declare_parameter('target_lifetime', 120.0).value)
        group = ReentrantCallbackGroup()
        self.action_clients = {
            'search': ActionClient(self, SearchObject, '/behavior_server/search', callback_group=group),
            'approach': ActionClient(self, ApproachObject, '/behavior_server/approach', callback_group=group),
            'pick': ActionClient(self, PickObject, '/manipulator/pick', callback_group=group),
            'place': ActionClient(self, PlaceObject, '/manipulator/place', callback_group=group),
        }
        self.lock = threading.Lock()
        self.held = None
        self.holding = False
        self.last_action, self.last_status = 'none', 'none'

    def run(self, name, action_type, goal):
        # 車体とアームは1つなので、HTTPの呼び出しが重なっても動作は1つずつにする。
        if not self.lock.acquire(blocking=False):
            raise HTTPException(status_code=409, detail='another action is running')
        try:
            result = send(self.action_clients[name], goal, self.timeout)
            if result is None:
                status, message = 'NO_RESPONSE', f'{name} did not answer (is the server running?)'
            else:
                status, message = code_name(action_type.Result, result.error_code), result.message
            self.last_action, self.last_status = name, status
            return status, message, result
        finally:
            self.lock.release()


def create_app(node, api_key):
    app = FastAPI(
        title='diegobot robot tools',
        version='0.1.0',
        description='Tools to control a small mobile robot with an arm. Typical flow to fetch an object: '
                    'search -> approach -> pick, then approach a place and place. Each call blocks until the '
                    'robot finishes (it can take tens of seconds).',
        servers=[{'url': os.environ.get('ROBOT_TOOLS_PUBLIC_URL', 'http://localhost:8090')}],
        redoc_url=None,
    )

    # FastAPIの既定のReDocの画面は試作版(redoc@next)を読み込み、版が上がると真っ白になるため、版を固定する。
    @app.get('/redoc', include_in_schema=False)
    def redoc():
        return get_redoc_html(openapi_url=app.openapi_url, title=f'{app.title} - ReDoc',
                              redoc_js_url='https://cdn.jsdelivr.net/npm/redoc@2.5.4/bundles/redoc.standalone.js')

    # スキーマにBearer認証を載せ、Swagger UIのAuthorizeやDifyの取り込みで使えるようにする。キー未設定なら認証なし。
    bearer = HTTPBearer(auto_error=False)

    def authorize(credentials: HTTPAuthorizationCredentials | None = Depends(bearer)):
        if api_key and (credentials is None or credentials.credentials != api_key):
            raise HTTPException(status_code=401, detail='invalid api key')

    def reply(status, message, result, with_color=True):
        candidates = getattr(result, 'candidates', None)
        return ActionResult(success=status == 'SUCCESS', status=status, message=message,
                            object=object_info(getattr(result, 'object', None), with_color),
                            candidate_count=len(candidates) if candidates else None)

    @app.post('/search', response_model=ActionResult, operation_id='search', dependencies=[Depends(authorize)],
              summary='Look around for an object without driving toward it. With target_name, returns '
                      'FOUND_UNIDENTIFIED when there are candidates too far to identify; call approach next.')
    def search(req: Target):
        node.target.drop()
        goal = SearchObject.Goal(queries=req.queries, color=req.color, target_name=req.target_name)
        return reply(*node.run('search', SearchObject, goal), with_color=not req.target_name)

    @app.post('/approach', response_model=ActionResult, operation_id='approach', dependencies=[Depends(authorize)],
              summary='Find an object and drive to it, stopping where the arm can reach it. With target_name, '
                      'it visits the candidates one by one; NO_MATCH means none of them was the named object.')
    def approach(req: Target):
        node.target.drop()
        goal = ApproachObject.Goal(queries=req.queries, color=req.color, standoff=0.0, target_name=req.target_name)
        status, message, result = node.run('approach', ApproachObject, goal)
        if status == 'SUCCESS' and result.object_odom.position.header.frame_id:
            node.target.keep(result.object_odom)
        return reply(status, message, result, with_color=not req.target_name)

    @app.post('/pick', response_model=ActionResult, operation_id='pick', dependencies=[Depends(authorize)],
              summary='Pick up an object that is within reach in front of the robot. If it returns OUT_OF_REACH, '
                      'call approach first.')
    def pick(req: Target):
        target = node.target.take()
        goal = PickObject.Goal(queries=req.queries, color=req.color, target_name=req.target_name,
                               target_object=target or DetectedObject())
        status, message, result = node.run('pick', PickObject, goal)
        if status == 'SUCCESS':
            node.holding, node.held = True, object_info(result.object, not req.target_name)
        return reply(status, message, result, with_color=not req.target_name)

    @app.post('/place', response_model=ActionResult, operation_id='place', dependencies=[Depends(authorize)],
              summary='Put down the object being held on the floor in front of the robot, or just release it.')
    def place(req: PlaceRequest):
        node.target.drop()
        status, message, result = node.run('place', PlaceObject, PlaceObject.Goal(release_here=req.release_here))
        if status in ('SUCCESS', 'NOT_HOLDING'):
            node.holding, node.held = False, None
        return reply(status, message, result)

    @app.get('/state', response_model=State, operation_id='get_state', dependencies=[Depends(authorize)],
             summary='Get what the robot is holding and the result of the last action.')
    def state():
        return State(holding=node.holding, held_object=node.held, last_action=node.last_action,
                     last_status=node.last_status)

    return app


def main():
    rclpy.init()
    node = RobotToolsNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    host = node.declare_parameter('host', '0.0.0.0').value
    port = node.declare_parameter('port', 8090).value
    try:
        uvicorn.run(create_app(node, os.environ.get('ROBOT_TOOLS_API_KEY', '')), host=host, port=port)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
