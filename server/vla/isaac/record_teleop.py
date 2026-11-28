"""Drive the simulated arm with the SO-101 leader arm and record the successful picks as an Isaac Lab HDF5 dataset.

The leader is read straight from its servo bus with the same calibration as the real teleop (amr_teleop), so no ROS
is needed. Press Enter to start an episode: the simulated arm jumps to the leader's pose and the block is placed anew.
An episode is saved when the block has been held lifted for --success_steps steps in a row; r + Enter throws the
current one away, q + Enter quits. Run with the Isaac Lab Python environment and a viewer, e.g.
    python server/vla/isaac/record_teleop.py --viz kit --dataset data/vla/isaac/demos/pick_block.hdf5
"""

import argparse
import math
import os
import select
import sys
import time
from pathlib import Path

from isaaclab.app import add_launcher_args, launch_simulation

REPO = Path(__file__).resolve().parents[3]
TELEOP_PKG = REPO / 'edge/workspace/src/amr_teleop'

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument('--dataset', required=True, help='HDF5 file to write the successful episodes into')
parser.add_argument('--port', default='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79033560-if00',
                    help="leader arm's servo bus")
parser.add_argument('--num_demos', type=int, default=0, help='stop after this many saved episodes (0: no limit)')
parser.add_argument('--success_steps', type=int, default=15, help='steps the block has to stay lifted')
parser.add_argument('--max_velocity', type=float, default=3.0, help='joint target speed limit [rad/s], as on the robot')
parser.add_argument('--cameras', action='store_true', help='also render the head and wrist cameras (slower)')
add_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = args.cameras

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
import yaml  # noqa: E402
from isaaclab.envs.utils._manual_success import _prepare_success_term  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, str(TELEOP_PKG))
from amr_teleop.feetech_bus import FeetechBus  # noqa: E402
from pick_block_env import TASK_ID, PickBlockEnvCfg, joint_ids, record_successes, save_success  # noqa: E402

STEP_RAD = 2 * math.pi / 4096


class Leader:
    def __init__(self, port):
        cfg = yaml.safe_load((TELEOP_PKG / 'config/leader_teleop.yaml').read_text())['leader_teleop']['ros__parameters']
        self.ids, self.zero_raw, self.zero_angle = cfg['servo_ids'], cfg['zero_raw'], cfg['zero_angle']
        self.bus = FeetechBus(port)
        self.previous = None

    def read(self):
        """Joint angles [rad] in URDF terms, or None when the servos did not all answer."""
        raw = self.bus.read_positions(self.ids)
        if raw is None:
            return None
        q = [(r - z) * STEP_RAD + a for r, z, a in zip(raw, self.zero_raw, self.zero_angle)]
        # A servo reports one turn as 0-4095, so crossing the seam jumps a full turn; keep the angle continuous.
        previous = self.previous or [0.0] * len(q)
        q = [p + math.remainder(x - p, 2 * math.pi) for x, p in zip(q, previous)]
        self.previous = q
        return q

    def wait(self):
        while (q := self.read()) is None:
            print('waiting for the leader arm', flush=True)
            time.sleep(0.5)
        return q


def typed_command():
    if not select.select([sys.stdin], [], [], 0)[0]:
        return None
    return sys.stdin.readline().strip().lower()


def main():
    env_cfg = PickBlockEnvCfg()
    if not args.cameras:
        env_cfg.without_cameras()
    env_cfg.sim.device = args.device
    env_cfg.env_name = TASK_ID
    # The recorder decides success itself; the success term would otherwise reset the episode on its first step.
    success_term = _prepare_success_term(env_cfg)
    env_cfg.terminations.time_out = None
    record_successes(env_cfg, args.dataset)

    leader = Leader(args.port)
    with launch_simulation(env_cfg, args):
        env = gym.make(TASK_ID, cfg=env_cfg).unwrapped
        robot = env.scene['robot']
        ids = joint_ids(robot, env_cfg.actions.arm_action.joint_names + env_cfg.actions.gripper_action.joint_names)
        limits = robot.data.soft_joint_pos_limits.torch[0, ids]
        step_dt = env.step_dt
        max_step = args.max_velocity * step_dt

        def start_episode():
            env.arm_start = torch.tensor(leader.wait(), device=env.device).clamp(limits[:, 0], limits[:, 1])
            env.recorder_manager.reset()
            env.reset()
            return env.arm_start.clone()

        saved = 0
        recording = False
        target, lifted = None, 0
        print(f'saving successful episodes into {args.dataset}', flush=True)
        print('Enter: start an episode, r + Enter: discard it, q + Enter: quit', flush=True)
        next_time = time.monotonic()
        with torch.inference_mode():
            while env.sim.is_running():
                command = typed_command()
                if command == 'q':
                    break
                if command == 'r' and recording:
                    recording = False
                    print('discarded; Enter to start again', flush=True)
                elif command == '' and not recording:
                    target, lifted, recording = start_episode(), 0, True
                    next_time = time.monotonic()
                    print('recording', flush=True)
                if not recording:
                    env.sim.render()
                    time.sleep(step_dt)
                    continue
                q = leader.read()
                if q is not None:
                    goal = torch.tensor(q, device=env.device).clamp(limits[:, 0], limits[:, 1])
                    target = target + (goal - target).clamp(-max_step, max_step)
                env.step(target.unsqueeze(0))
                lifted = lifted + 1 if bool(success_term.func(env, **success_term.params)[0]) else 0
                if lifted >= args.success_steps:
                    save_success(env)
                    saved += 1
                    recording = False
                    print(f'saved episode {saved}; Enter to start the next one', flush=True)
                    if args.num_demos and saved >= args.num_demos:
                        break
                    continue
                next_time += step_dt
                time.sleep(max(0.0, next_time - time.monotonic()))
                next_time = max(next_time, time.monotonic() - step_dt)
        print(f'saved {saved} episodes', flush=True)
        env.close()
    leader.bus.close()


if __name__ == '__main__':
    main()
