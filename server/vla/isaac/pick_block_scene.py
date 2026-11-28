"""Isaac Lab scene of diegobot picking a block, with the head and wrist cameras placed as on the real robot.

Run with the Isaac Lab Python environment, e.g.
    python server/vla/isaac/pick_block_scene.py --usd data/vla/isaac/diegobot/diegobot.usd/robot/robot.usda \
        --out data/vla/isaac/render_check
"""

import argparse
import os
import sys

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--usd', required=True, help='robot USD converted from the URDF on /robot_description')
parser.add_argument('--out', required=True, help='folder to save the camera images into')
parser.add_argument('--steps', type=int, default=60)
parser.add_argument('--block', type=float, nargs=3, default=(0.25, 0.10, 0.015), metavar=('X', 'Y', 'YAW'),
                    help='block centre in base_footprint [m] and yaw [rad]')
parser.add_argument('--joints', type=float, nargs=6, metavar='Q',
                    default=(-0.215, -1.741, 1.612, 1.336, -1.593, 0.411),
                    help='shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll gripper [rad]')
parser.add_argument('--textures', default='data/vla/isaac/textures/nvidia', help='folder of floor texture images')
parser.add_argument('--floor', default='Ash_Planks', help='floor texture file name without extension')
parser.add_argument('--floor_yaw', type=float, default=0.0, help='floor texture rotation [rad]')
parser.add_argument('--floor_tile', type=float, default=1.0, help='floor length one texture image covers [m]')
parser.add_argument('--light', type=float, default=600.0, help='dome light intensity')
parser.add_argument('--block_z', type=float, default=0.015, help='block centre height [m]')
parser.add_argument('--ghost_block', action='store_true',
                    help='fixed block without collision, to compare a grasp frame against the real image')
add_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True

import isaaclab.sim as sim_utils  # noqa: E402
import numpy as np  # noqa: E402
from isaaclab.utils import instantiate  # noqa: E402
from PIL import Image  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from diegobot_scene import (  # noqa: E402
    DiegobotSceneCfg, block_cfg, dress_scene, floor_texture, robot_cfg)


def main():
    sim_cfg = sim_utils.SimulationCfg(dt=1.0 / 120.0, device=args.device)
    with launch_simulation(sim_cfg, args):
        sim = sim_utils.SimulationContext(sim_cfg)
        scene_cfg = DiegobotSceneCfg(num_envs=1, env_spacing=2.0)
        scene_cfg.robot = robot_cfg(args.usd, args.joints)
        scene_cfg.block = block_cfg(*args.block, ghost=args.ghost_block)
        scene_cfg.block.init_state.pos = (args.block[0], args.block[1], args.block_z)
        scene_cfg.dome_light.spawn.intensity = args.light
        scene = instantiate(scene_cfg)
        dress_scene(sim.stage, 1, floor_texture(args.textures, args.floor), args.floor_tile, args.floor_yaw)
        sim.reset()
        robot = scene['robot']
        for _ in range(args.steps):
            robot.set_joint_position_target_index(target=robot.data.default_joint_pos.torch)
            scene.write_data_to_sim()
            sim.step()
            scene.update(sim.get_physics_dt())
        os.makedirs(args.out, exist_ok=True)
        for name in ('head_camera', 'wrist_camera'):
            rgb = scene[name].data.output['rgb'].torch[0].cpu().numpy().astype(np.uint8)
            Image.fromarray(rgb[..., :3]).save(os.path.join(args.out, f'{name}.png'))
        print('saved', args.out, flush=True)


if __name__ == '__main__':
    main()
