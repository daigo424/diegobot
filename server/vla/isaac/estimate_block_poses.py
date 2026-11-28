"""Work out where the block was in each real episode from the arm's joint angles at the moment it was grasped.

The real recordings keep only joint angles and images, so the block pose is taken as the pose between the closed
fingertips (URDF forward kinematics) at the grasp. Writes a JSON with each episode's block pose, joint trajectory
leader angles and grasp frame, plus the first head camera frame and the wrist camera frame at the grasp as PNG for comparing
against a simulated render.
Run with the LeRobot Python environment, e.g.
    python server/vla/isaac/estimate_block_poses.py data/vla/lerobot/pick_block_v2 data/vla/isaac/replay/pick_block_v2
"""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from PIL import Image
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[3]
URDF = REPO / '.claude/report/isaac-scene-inputs/robot.urdf'
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
# Block centre in gripper_frame_link: midway between the fixed and moving fingertips, which close along x and reach
# the frame's origin plane; the block's half height lies back towards the wrist (-z).
BLOCK_IN_GRIPPER = np.array([-0.016, 0.0, -0.015])
BLOCK_HALF_HEIGHT = 0.015
LIFT = 0.03
SETTLED = 0.03
OPEN_AGAIN = 0.15


class Kinematics:
    def __init__(self, urdf):
        self.joints = {j.find('child').get('link'): j for j in ET.parse(urdf).getroot().findall('joint')}

    def pose(self, link, q):
        if link not in self.joints:
            return np.eye(4)
        j = self.joints[link]
        o = j.find('origin')
        m = np.eye(4)
        m[:3, :3] = Rotation.from_euler('xyz', [float(v) for v in o.get('rpy', '0 0 0').split()]).as_matrix()
        m[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
        if j.get('type') != 'fixed':
            axis = np.array([float(v) for v in j.find('axis').get('xyz').split()])
            turn = np.eye(4)
            turn[:3, :3] = Rotation.from_rotvec(axis * q.get(j.get('name'), 0.0)).as_matrix()
            m = m @ turn
        return self.pose(j.find('parent').get('link'), q) @ m


def find_grasp(z, grip):
    """Frame the jaw settled on the block before the lift, and how many times the jaw closed and opened again."""
    low = int(np.argmin(z))
    lifted = next((i for i in range(low, len(z)) if z[i] > z[low] + LIFT), None)
    if lifted is None:
        return None, 0
    held = grip[lifted]
    start = lifted
    while start > 0 and abs(grip[start - 1] - held) < SETTLED:
        start -= 1
    # Opening again after closing below the held angle means the operator let go and grasped again.
    regrasps, closed = 0, False
    for g in grip[:lifted]:
        if g < held + SETTLED:
            closed = True
        elif closed and g > held + OPEN_AGAIN:
            regrasps, closed = regrasps + 1, False
    return start, regrasps


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('out', type=Path)
    args = parser.parse_args()

    kin = Kinematics(URDF)
    dataset = LeRobotDataset(repo_id=args.dataset.name, root=args.dataset)
    states = np.stack([np.asarray(s) for s in dataset.hf_dataset['observation.state']])
    actions = np.stack([np.asarray(a) for a in dataset.hf_dataset['action']])
    episodes = np.asarray(dataset.hf_dataset['episode_index'])
    args.out.mkdir(parents=True, exist_ok=True)
    result = {'fps': dataset.fps, 'episodes': []}
    for ep in np.unique(episodes):
        index = np.flatnonzero(episodes == ep)
        q = states[index]
        tips = [kin.pose('gripper_frame_link', dict(zip(JOINTS, row))) for row in q]
        z = np.array([t[2, 3] for t in tips])
        grasp, regrasps = find_grasp(z, q[:, 5])
        entry = {'episode': int(ep), 'frames': len(q), 'joints': q.round(5).tolist(),
                 'leader': actions[index].round(5).tolist(), 'grasp_frame': grasp,
                 'regrasps': regrasps}
        if grasp is not None:
            tip = tips[grasp]
            centre = tip[:3, :3] @ BLOCK_IN_GRIPPER + tip[:3, 3]
            # The jaws squeeze the block's short side, so its long side (block x) runs along the fingertip y axis.
            long_side = tip[:3, 1]
            yaw = float(np.arctan2(long_side[1], long_side[0]))
            yaw = (yaw + np.pi / 2) % np.pi - np.pi / 2
            entry.update(block=[float(centre[0]), float(centre[1]), yaw], tip_height=float(tip[2, 3]),
                         grip_at_grasp=float(q[grasp, 5]), tilt=float(-tip[2, 2]))
        frame = dataset[int(index[0])]['observation.images.head']
        Image.fromarray((frame.permute(1, 2, 0).numpy() * 255).astype(np.uint8)).save(args.out / f'real_{ep:03d}.png')
        if grasp is not None:
            frame = dataset[int(index[grasp])]['observation.images.wrist']
            Image.fromarray((frame.permute(1, 2, 0).numpy() * 255).astype(np.uint8)).save(
                args.out / f'real_wrist_grasp_{ep:03d}.png')
        result['episodes'].append(entry)
        block = entry.get('block')
        print(f'ep{ep:3d} {len(q):3d} frames  grasp@{grasp}  regrasps {regrasps}  '
              + (f'block ({block[0]:.3f}, {block[1]:.3f}) yaw {np.degrees(block[2]):6.1f}deg  '
                 f'tip z {entry["tip_height"]:.3f}  grip {entry["grip_at_grasp"]:.2f}  down {entry["tilt"]:.2f}'
                 if block else 'no lift found'), flush=True)
    (args.out / 'episodes.json').write_text(json.dumps(result))


if __name__ == '__main__':
    main()
