"""Render the head camera at the start of each real episode, with the block where estimate_block_poses.py put it.

With --wrist, also render the wrist camera at the grasp frame next to the real one.

Saves the render next to the real first frame and a side-by-side image (real | simulated | blend) per episode, to
check the estimated block poses by eye. Run with the Isaac Lab Python environment, e.g.
    python server/vla/isaac/render_episode_starts.py data/vla/isaac/replay/pick_block_v2
"""

import argparse
import json
import os
import sys
from pathlib import Path

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('replay', type=Path, help='folder with episodes.json and real_<episode>.png')
parser.add_argument('--steps', type=int, default=30, help='physics steps to settle each pose before rendering')
parser.add_argument('--episodes', type=int, nargs='*', help='only these episodes (default: all)')
parser.add_argument('--head_offset', type=float, nargs=2, default=(0.0, 0.0), metavar=('PAN', 'TILT'),
                    help='added to the recorded head pose [rad], to look for the real camera direction')
parser.add_argument('--wrist', action='store_true', help='also compare the wrist camera at the grasp frame')
parser.add_argument('--tag', default='', help='suffix for the saved images, to compare look settings')
parser.add_argument('--dome', type=float, help='dome light intensity')
parser.add_argument('--ceiling', type=float, help='ceiling (distant) light intensity')
parser.add_argument('--black', type=float, nargs=3, metavar=('COLOUR', 'ROUGHNESS', 'SPECULAR'),
                    help='look of the black parts')
add_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True

import isaaclab.sim as sim_utils  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from isaaclab.utils import instantiate  # noqa: E402
from PIL import Image  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import diegobot_scene  # noqa: E402
from diegobot_scene import (  # noqa: E402
    ARM_JOINTS, BLOCK_SIZE, DiegobotSceneCfg, block_cfg, camera_rgb, dress_scene, floor_texture, robot_cfg, yaw_quat)
from pick_block_env import ROBOT_USD, TEXTURES  # noqa: E402


def main():
    episodes = json.loads((args.replay / 'episodes.json').read_text())['episodes']
    sim_cfg = sim_utils.SimulationCfg(dt=1.0 / 120.0, device=args.device)
    with launch_simulation(sim_cfg, args):
        sim = sim_utils.SimulationContext(sim_cfg)
        scene_cfg = DiegobotSceneCfg(num_envs=1, env_spacing=2.0)
        scene_cfg.robot = robot_cfg(ROBOT_USD)
        # Kinematic so the arm passing through it while jumping between poses cannot knock it away.
        scene_cfg.block = block_cfg(ghost=True)
        if not args.wrist:
            scene_cfg.wrist_camera = None
        if args.dome is not None:
            scene_cfg.dome_light.spawn.intensity = args.dome
        if args.ceiling is not None:
            scene_cfg.ceiling_light.spawn.intensity = args.ceiling
        if args.black:
            diegobot_scene.LOOKS['part_black'] = ((args.black[0],) * 3, args.black[1], args.black[2])
        scene = instantiate(scene_cfg)
        dress_scene(sim.stage, 1, floor_texture(TEXTURES, 'Ash_Planks'), 1.2, 1.5708)
        sim.reset()
        robot, block = scene['robot'], scene['block']
        ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
        # Without targets the head drives towards 0 and looks at the horizon instead of the recording tilt.
        head = robot.data.default_joint_pos.torch.clone()
        head_ids = robot.find_joints(['head_pan', 'head_tilt'], preserve_order=True)[0]
        head[:, head_ids] += torch.tensor(args.head_offset, device=sim.device)
        robot.write_joint_position_to_sim_index(position=head[:, head_ids], joint_ids=head_ids)
        robot.set_joint_position_target_index(target=head)

        def settle(robot, ids, joints):
            q = torch.tensor([joints], device=sim.device)
            robot.write_joint_position_to_sim_index(position=q, joint_ids=ids)
            robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), joint_ids=ids)
            robot.set_joint_position_target_index(target=q, joint_ids=ids)
            for _ in range(args.steps):
                scene.write_data_to_sim()
                sim.step()
                scene.update(sim.get_physics_dt())

        def compare(rendered, real_path, name):
            real = np.asarray(Image.open(real_path).convert('RGB'))
            blend = (real.astype(np.float32) * 0.5 + rendered.astype(np.float32) * 0.5).astype(np.uint8)
            Image.fromarray(rendered).save(args.replay / f'sim_{name}.png')
            Image.fromarray(np.concatenate([real, rendered, blend], axis=1)).save(args.replay / f'compare_{name}.png')

        for entry in episodes:
            if 'block' not in entry or (args.episodes and entry['episode'] not in args.episodes):
                continue
            ep = entry['episode']
            x, y, yaw = entry['block']
            pose = torch.tensor([[x, y, BLOCK_SIZE[2] / 2, *yaw_quat(yaw)]], device=sim.device)
            block.write_root_pose_to_sim_index(root_pose=pose)
            settle(robot, ids, entry['joints'][0])
            rendered = camera_rgb(scene, 'head_camera')[0].cpu().numpy().astype(np.uint8)
            compare(rendered, args.replay / f'real_{ep:03d}.png', f'{ep:03d}{args.tag}')
            if args.wrist:
                settle(robot, ids, entry['joints'][entry['grasp_frame']])
                rendered = camera_rgb(scene, 'wrist_camera')[0].cpu().numpy().astype(np.uint8)
                compare(rendered, args.replay / f'real_wrist_grasp_{ep:03d}.png', f'wrist_grasp_{ep:03d}{args.tag}')
            print('rendered episode', ep, flush=True)


if __name__ == '__main__':
    main()
