import threading
import time

from amr_interfaces.srv import FindObjects
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes

# 実行中に止まったと判断するMoveItの結果。これ以外の失敗は計画できなかったものとして扱う。
EXECUTION_ERRORS = {MoveItErrorCodes.CONTROL_FAILED, MoveItErrorCodes.TIMED_OUT, MoveItErrorCodes.PREEMPTED,
                    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE}


def wait(future, timeout):
    """MultiThreadedExecutorで回っているノードから、futureの完了をspinせずに待つ。"""
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    return future.result() if done.wait(timeout) else None


def call(client, request, timeout):
    if not client.wait_for_service(timeout_sec=2.0):
        return None
    return wait(client.call_async(request), timeout)


def send(client, goal, timeout):
    if not client.wait_for_server(timeout_sec=2.0):
        return None
    handle = wait(client.send_goal_async(goal), 5.0)
    if handle is None or not handle.accepted:
        return None
    res = wait(handle.get_result_async(), timeout)
    return res.result if res else None


def move_joints(move_client, group, names, joints, velocity_scaling, timeout=60.0):
    """MoveItで関節の目標へ動かし、MoveItErrorCodesの値を返す。"""
    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = group
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = velocity_scaling
    req.max_acceleration_scaling_factor = velocity_scaling
    req.start_state.is_diff = True
    req.goal_constraints = [Constraints(joint_constraints=[
        JointConstraint(joint_name=n, position=float(joints[n]), tolerance_above=0.01, tolerance_below=0.01,
                        weight=1.0) for n in names])]
    res = send(move_client, goal, timeout)
    return res.error_code.val if res else MoveItErrorCodes.FAILURE


def find_objects(find_client, queries, color, attempts, min_score=0.0):
    """object_finderで探し、深度のある物を色で絞ってスコア順に返す。(objects, message)。"""
    # YOLOEは色の語があると物の種類の確信度が上がる一方、色の違う物も拾うため、色は判定結果でも絞る。
    targets = [f'{color} {q}'.strip() for q in queries if q.strip()]
    message = 'object_finder did not answer'
    # ラズパイの負荷で画像が遅れて届くとobject_finderは古いとして断り、アームを動かした直後は写っていても
    # 検出されない画像があるため、別の画像で探し直す。
    for i in range(attempts):
        if i:
            time.sleep(0.5)
        response = call(find_client, FindObjects.Request(queries=targets, min_score=float(min_score)), 15.0)
        if response is None:
            continue
        if not response.success:
            message = response.message
            continue
        objects = [o for o in response.objects if o.position_valid and (not color or o.color == color)]
        objects.sort(key=lambda o: o.score, reverse=True)
        message = f'no {" / ".join(targets)} with depth'
        if objects:
            return objects, ''
    return [], message
