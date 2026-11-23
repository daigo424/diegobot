"""テレオペの前に実機のアームを確かめ、ハードウェアやコントローラが止まっていれば動く状態に戻す。"""
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents, SetHardwareComponentState, SwitchController
from lifecycle_msgs.msg import State
from rclpy.node import Node
from sensor_msgs.msg import JointState

CM = '/controller_manager'


class ArmCheck(Node):
    def __init__(self):
        super().__init__('check_arm')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.joints = p('joints', ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'])
        self.active = p('active_controllers', ['joint_state_broadcaster', 'arm_group_controller', 'grip_group_controller', 'head_controller'])
        self.teleop_controller = p('teleop_controller', 'arm_teleop_controller')
        self.timeout = p('timeout', 5.0)
        self.joint_states = None
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)

    def on_joint_states(self, msg):
        if all(j in msg.name for j in self.joints):
            self.joint_states = msg

    def call(self, srv_type, name, request):
        client = self.create_client(srv_type, f'{CM}/{name}')
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError(f'{CM}/{name} が見つからない。ラズパイの diegobot_arm が起動しているか確かめてください')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        if future.result() is None:
            raise RuntimeError(f'{CM}/{name} が応答しない')
        return future.result()

    def arm_hardware(self):
        for c in self.call(ListHardwareComponents, 'list_hardware_components', ListHardwareComponents.Request()).component:
            if any(i.name.split('/')[0] in self.joints for i in c.command_interfaces):
                return c
        raise RuntimeError('アームのハードウェアが controller_manager に無い')

    def ensure_hardware(self):
        hw = self.arm_hardware()
        if hw.state.id == State.PRIMARY_STATE_ACTIVE:
            return
        print(f'アームのハードウェア {hw.name} が {hw.state.label} のため有効にし直す(トルクが入る)', flush=True)
        # 読み出しエラーで止まると unconfigured まで戻るため、configure してから activate する。
        for target in (State(id=State.PRIMARY_STATE_INACTIVE, label='inactive'), State(id=State.PRIMARY_STATE_ACTIVE, label='active')):
            if self.arm_hardware().state.id >= target.id:
                continue
            res = self.call(SetHardwareComponentState, 'set_hardware_component_state',
                            SetHardwareComponentState.Request(name=hw.name, target_state=target))
            if not res.ok:
                raise RuntimeError(f'{hw.name} を {target.label} にできない(電源・配線を確かめてください)')

    def ensure_controllers(self):
        states = {c.name: c.state for c in self.call(ListControllers, 'list_controllers', ListControllers.Request()).controller}
        missing = [n for n in self.active + [self.teleop_controller] if n not in states]
        if missing:
            raise RuntimeError(f'コントローラが読み込まれていない: {missing}')
        # 前回のテレオペが元に戻さず終わっていると、テレオペ用のコントローラが有効のまま残る。
        deactivate = [self.teleop_controller] if states[self.teleop_controller] == 'active' else []
        activate = [n for n in self.active if states[n] != 'active']
        if not activate and not deactivate:
            return
        print(f'コントローラを切り替える: 有効化 {activate} 無効化 {deactivate}', flush=True)
        req = SwitchController.Request(activate_controllers=activate, deactivate_controllers=deactivate,
                                       strictness=SwitchController.Request.STRICT)
        req.timeout.sec = 3
        if not self.call(SwitchController, 'switch_controller', req).ok:
            raise RuntimeError('コントローラを切り替えられない')

    def wait_joint_states(self):
        self.joint_states = None
        end = time.monotonic() + self.timeout
        while self.joint_states is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.joint_states is None:
            raise RuntimeError('/joint_states にアームの関節角が届かない')


def main():
    rclpy.init()
    node = ArmCheck()
    try:
        node.ensure_hardware()
        node.ensure_controllers()
        node.wait_joint_states()
        print('実機のアームは動ける状態', flush=True)
        code = 0
    except RuntimeError as e:
        print(f'実機のアームの確認に失敗: {e}', file=sys.stderr, flush=True)
        code = 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
