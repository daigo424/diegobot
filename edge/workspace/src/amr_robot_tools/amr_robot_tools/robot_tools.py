import math
import os
import threading

import rclpy
import uvicorn
from amr_interfaces.action import ApproachObject, PickObject, PlaceObject, SearchObject
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


class Target(BaseModel):
    queries: list[str] = Field(..., description=QUERIES_DESCRIPTION)
    color: str = Field('', description=COLOR_DESCRIPTION)


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


class State(BaseModel):
    holding: bool = Field(..., description='Whether the robot thinks it is holding an object.')
    held_object: ObjectInfo | None = None
    last_action: str
    last_status: str


def code_name(result_type, code):
    """アクションの結果の型の定数(SUCCESS=0等)から、値に対応する名前を引く。"""
    names = {getattr(result_type, n): n for n in dir(result_type) if n.isupper() and isinstance(getattr(result_type, n), int)}
    return names.get(code, str(code))


def object_info(obj):
    if obj is None or not obj.position_valid:
        return None
    p = obj.position.point
    return ObjectInfo(label=obj.label, color=obj.color, score=round(obj.score, 2),
                      distance_m=round(math.hypot(p.x, p.y), 2), direction_deg=round(math.degrees(math.atan2(p.y, p.x))))


class RobotToolsNode(Node):
    def __init__(self):
        super().__init__('robot_tools')
        self.timeout = self.declare_parameter('action_timeout', 300.0).value
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
                status, message, obj = 'NO_RESPONSE', f'{name} did not answer (is the server running?)', None
            else:
                status = code_name(action_type.Result, result.error_code)
                message, obj = result.message, getattr(result, 'object', None)
            self.last_action, self.last_status = name, status
            return status, message, obj
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

    def reply(status, message, obj):
        return ActionResult(success=status == 'SUCCESS', status=status, message=message, object=object_info(obj))

    @app.post('/search', response_model=ActionResult, operation_id='search', dependencies=[Depends(authorize)],
              summary='Look around for an object without driving toward it.')
    def search(req: Target):
        goal = SearchObject.Goal(queries=req.queries, color=req.color)
        return reply(*node.run('search', SearchObject, goal))

    @app.post('/approach', response_model=ActionResult, operation_id='approach', dependencies=[Depends(authorize)],
              summary='Find an object and drive to it, stopping where the arm can reach it.')
    def approach(req: Target):
        goal = ApproachObject.Goal(queries=req.queries, color=req.color, standoff=0.0)
        return reply(*node.run('approach', ApproachObject, goal))

    @app.post('/pick', response_model=ActionResult, operation_id='pick', dependencies=[Depends(authorize)],
              summary='Pick up an object that is within reach in front of the robot. If it returns OUT_OF_REACH, '
                      'call approach first.')
    def pick(req: Target):
        status, message, obj = node.run('pick', PickObject, PickObject.Goal(queries=req.queries, color=req.color))
        if status == 'SUCCESS':
            node.holding, node.held = True, object_info(obj)
        return reply(status, message, obj)

    @app.post('/place', response_model=ActionResult, operation_id='place', dependencies=[Depends(authorize)],
              summary='Put down the object being held on the floor in front of the robot, or just release it.')
    def place(req: PlaceRequest):
        status, message, obj = node.run('place', PlaceObject, PlaceObject.Goal(release_here=req.release_here))
        if status in ('SUCCESS', 'NOT_HOLDING'):
            node.holding, node.held = False, None
        return reply(status, message, obj)

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
