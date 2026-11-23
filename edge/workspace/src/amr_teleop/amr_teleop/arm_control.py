"""テレオペ・記録・ポリシー実行で共通に使う、実機のアームと頭部の操作。"""
import threading
import xml.etree.ElementTree as ET

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from trajectory_msgs.msg import JointTrajectoryPoint


def switch_request(activate, deactivate):
    req = SwitchController.Request(activate_controllers=activate, deactivate_controllers=deactivate,
                                   strictness=SwitchController.Request.STRICT)
    req.timeout.sec = 3
    return req


def joint_limits(urdf, joints):
    """URDFから関節の(下限, 上限)を関節の順に返す。足りない関節があればNone。"""
    limits = {}
    for j in ET.fromstring(urdf).iter('joint'):
        lim = j.find('limit')
        if j.get('name') in joints and lim is not None:
            limits[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
    return [limits[n] for n in joints] if all(n in limits for n in joints) else None


def move_head(client, joints, pose, seconds=1.5, timeout=10.0):
    """FollowJointTrajectoryで頭部を動かし、着くまで待つ。別スレッドでexecutorが回っている前提。"""
    if not client.wait_for_server(timeout_sec=3.0):
        return False
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(joints)
    sec = int(seconds)
    goal.trajectory.points = [JointTrajectoryPoint(positions=list(pose),
                                                   time_from_start=Duration(sec=sec, nanosec=int((seconds - sec) * 1e9)))]
    done = threading.Event()
    result = {}

    def on_goal(future):
        handle = future.result()
        if not handle.accepted:
            done.set()
            return
        handle.get_result_async().add_done_callback(lambda f: (result.update(r=f.result()), done.set()))

    client.send_goal_async(goal).add_done_callback(on_goal)
    done.wait(timeout)
    return 'r' in result and result['r'].result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
