"""Render the head camera at the start of each real episode, with the block where estimate_block_poses.py put it.

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
    ARM_JOINTS, BLOCK_SIZE, DiegobotSceneCfg, block_cfg, dress_scene, floor_texture, robot_cfg, yaw_quat)
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
        robot, block, camera = scene['robot'], scene['block'], scene['head_camera']
        ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
        # Without targets the head drives towards 0 and looks at the horizon instead of the recording tilt.
        robot.set_joint_position_target_index(target=robot.data.default_joint_pos.torch)
        for entry in episodes:
            if 'block' not in entry or (args.episodes and entry['episode'] not in args.episodes):
                continue
            ep = entry['episode']
            q = torch.tensor([entry['joints'][0]], device=sim.device)
            x, y, yaw = entry['block']
            pose = torch.tensor([[x, y, BLOCK_SIZE[2] / 2, *yaw_quat(yaw)]], device=sim.device)
            robot.write_joint_position_to_sim_index(position=q, joint_ids=ids)
            robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), joint_ids=ids)
            robot.set_joint_position_target_index(target=q, joint_ids=ids)
            block.write_root_pose_to_sim_index(root_pose=pose)
            for _ in range(args.steps):
                scene.write_data_to_sim()
                sim.step()
                scene.update(sim.get_physics_dt())
            rendered = camera.data.output['rgb'].torch[0].cpu().numpy().astype(np.uint8)[..., :3]
            real = np.asarray(Image.open(args.replay / f'real_{ep:03d}.png').convert('RGB'))
            blend = (real.astype(np.float32) * 0.5 + rendered.astype(np.float32) * 0.5).astype(np.uint8)
            Image.fromarray(rendered).save(args.replay / f'sim_{ep:03d}{args.tag}.png')
            Image.fromarray(np.concatenate([real, rendered, blend], axis=1)).save(args.replay / f'compare_{ep:03d}{args.tag}.png')
            print('rendered episode', ep, flush=True)


if __name__ == '__main__':
    main()
