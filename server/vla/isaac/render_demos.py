"""Replay episodes from an Isaac Lab HDF5 dataset with the cameras on and save head | wrist videos to look at.

Add ``--viz kit --realtime`` to watch the replay in the simulator window. Run with the Isaac Lab Python environment,
e.g. (``make sim-replay`` wraps this)
    python server/vla/isaac/render_demos.py data/vla/isaac/demos/pick_block_v2_generated_test.hdf5 \
        --out data/vla/isaac/videos --demos demo_0 demo_1
"""

import argparse
import os
import sys
import time
from pathlib import Path

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument('dataset')
parser.add_argument('--out', type=Path, required=True)
parser.add_argument('--demos', nargs='*', help='episode names (default: all)')
parser.add_argument('--realtime', action='store_true', help='step no faster than the recording ran')
add_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True

import gymnasium as gym  # noqa: E402
import imageio  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from isaaclab.envs.utils._manual_success import _prepare_success_term  # noqa: E402
from isaaclab.utils.datasets import HDF5DatasetFileHandler  # noqa: E402
from PIL import Image  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from diegobot_scene import camera_rgb  # noqa: E402
from pick_block_env import TASK_ID, PickBlockEnvCfg  # noqa: E402


def frame(env):
    head = camera_rgb(env.scene, 'head_camera')[0].cpu().numpy().astype(np.uint8)
    wrist = camera_rgb(env.scene, 'wrist_camera')[0].cpu().numpy().astype(np.uint8)
    wrist = np.asarray(Image.fromarray(wrist).resize((head.shape[0] * 4 // 3, head.shape[0])))
    return np.concatenate([head, wrist], axis=1)


def main():
    data = HDF5DatasetFileHandler()
    data.open(args.dataset)
    names = args.demos or sorted(data.get_episode_names(), key=lambda n: int(n.split('_')[-1]))
    env_cfg = PickBlockEnvCfg()
    env_cfg.sim.device = args.device
    success_term = _prepare_success_term(env_cfg, disable_terminations=True)
    args.out.mkdir(parents=True, exist_ok=True)
    with launch_simulation(env_cfg, args):
        env = gym.make(TASK_ID, cfg=env_cfg).unwrapped
        env.reset()
        with torch.inference_mode():
            for name in names:
                episode = data.load_episode(name, env.device)
                env.reset_to(episode.data['initial_state'], None, is_relative=True)
                frames = []
                next_time = time.monotonic()
                for action in episode.data['actions']:
                    env.step(action.reshape(1, -1))
                    frames.append(frame(env))
                    if args.realtime:
                        next_time += env.step_dt
                        time.sleep(max(0.0, next_time - time.monotonic()))
                ok = bool(success_term.func(env, **success_term.params)[0])
                path = args.out / f'{Path(args.dataset).stem}_{name}.mp4'
                imageio.mimwrite(path, frames, fps=round(1.0 / env.step_dt), macro_block_size=8)
                print(f'{name}: {len(frames)} frames, lifted at the end: {ok} -> {path}', flush=True)
        if getattr(args, 'visualizer', None):
            print('done; close the simulator window to quit', flush=True)
            while env.sim.is_running():
                env.sim.render()
        env.close()


if __name__ == '__main__':
    main()
