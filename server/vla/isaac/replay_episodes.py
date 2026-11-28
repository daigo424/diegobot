"""Replay the real episodes in the pick-block task and save the ones that also lift the block in simulation.

Each episode starts with the block where estimate_block_poses.py put it and the arm at the first recorded angles.
The arm follows the recorded follower angles; the gripper follows the leader, since the follower only reports
where the jaw stopped against the block and aiming there would barely squeeze it. The saved HDF5 is the
source dataset for Isaac Lab Mimic. Run with the Isaac Lab Python environment, e.g.
    python server/vla/isaac/replay_episodes.py data/vla/isaac/replay/pick_block_v2 \
        --dataset data/vla/isaac/demos/pick_block_v2_replay.hdf5
"""

import argparse
import json
import os
import sys
from pathlib import Path

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument('replay', type=Path, help='folder with episodes.json from estimate_block_poses.py')
parser.add_argument('--dataset', required=True, help='HDF5 file to write the episodes that succeed into')
parser.add_argument('--episodes', type=int, nargs='*', help='only these episodes (default: all)')
parser.add_argument('--success_steps', type=int, default=15, help='steps the block has to stay lifted')
parser.add_argument('--hold', type=float, default=1.0, help='seconds to keep the last pose after the recording ends')
add_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = False

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from isaaclab.envs.utils._manual_success import _prepare_success_term  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pick_block_env import (  # noqa: E402
    TASK_ID, PickBlockEnvCfg, block_grasped, record_successes, save_success)


def resample(frames, fps, rate):
    """Linearly interpolate the recorded frames to the control rate."""
    frames = np.asarray(frames)
    t = np.arange(len(frames)) / fps
    tq = np.arange(0.0, t[-1] + 1e-9, 1.0 / rate)
    return np.stack([np.interp(tq, t, frames[:, j]) for j in range(frames.shape[1])], axis=1)


def main():
    data = json.loads((args.replay / 'episodes.json').read_text())
    episodes = [e for e in data['episodes'] if 'block' in e and (not args.episodes or e['episode'] in args.episodes)]
    env_cfg = PickBlockEnvCfg().without_cameras()
    env_cfg.sim.device = args.device
    env_cfg.env_name = TASK_ID
    success_term = _prepare_success_term(env_cfg)
    env_cfg.terminations.time_out = None
    record_successes(env_cfg, args.dataset)

    with launch_simulation(env_cfg, args):
        env = gym.make(TASK_ID, cfg=env_cfg).unwrapped
        rate = 1.0 / env.step_dt
        results = []
        with torch.inference_mode():
            for entry in episodes:
                plan = resample(entry['joints'], data['fps'], rate)
                plan[:, 5] = resample(entry['leader'], data['fps'], rate)[:, 5]
                plan = np.concatenate([plan, np.repeat(plan[-1:], int(args.hold * rate), axis=0)])
                env.arm_start = torch.tensor(plan[0], dtype=torch.float32, device=env.device)
                env.block_start = entry['block']
                env.recorder_manager.reset()
                env.reset()
                lifted, grasped_at, saved, peak = 0, None, False, 0.0
                for i, target in enumerate(plan):
                    env.step(torch.tensor(target, dtype=torch.float32, device=env.device).unsqueeze(0))
                    if grasped_at is None and bool(block_grasped(env)[0]):
                        grasped_at = i
                    z = float(env.scene['block'].data.root_pos_w.torch[0, 2] - env.scene.env_origins[0, 2])
                    peak = max(peak, z)
                    lifted = lifted + 1 if bool(success_term.func(env, **success_term.params)[0]) else 0
                    if lifted >= args.success_steps:
                        save_success(env)
                        saved = True
                        break
                pos = env.scene['block'].data.root_pos_w.torch[0] - env.scene.env_origins[0]
                results.append((entry['episode'], saved))
                print(f"episode {entry['episode']:3d}: {'saved ' if saved else 'failed'}  grasp step {grasped_at}  "
                      f"block peak height {peak:.3f} m  block now ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})",
                      flush=True)
        print(f'saved {sum(s for _, s in results)} of {len(results)} episodes into {args.dataset}', flush=True)
        env.close()


if __name__ == '__main__':
    main()
