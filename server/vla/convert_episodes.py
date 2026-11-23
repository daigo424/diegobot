"""record_episodesで記録したrosbag(mcap)を、LeRobotデータセットに変換する。

LeRobotとrosbagsが入ったPython環境で動かす(ROSは要らない)。
    python convert_episodes.py data/vla/raw/<dataset> data/vla/lerobot/<dataset>
"""
import argparse
import io
import json
import shutil
from pathlib import Path

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from PIL import Image
from rosbags.highlevel import AnyReader, AnyReaderError
from rosbags.typesys import Stores, get_typestore

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
CAMERAS = {
    'head': '/head_camera/color/image_raw/compressed',
    'wrist': '/camera/camera/color/image_raw/compressed',
}
STATE_TOPIC = '/joint_states'
ACTION_TOPIC = '/leader/joint_states'


def read_episode(path, typestore):
    images = {name: [] for name in CAMERAS}
    joints = {STATE_TOPIC: [], ACTION_TOPIC: []}
    topic_to_camera = {topic: name for name, topic in CAMERAS.items()}
    with AnyReader([path], default_typestore=typestore) as reader:
        for conn, t, raw in reader.messages():
            if conn.topic in topic_to_camera:
                msg = reader.deserialize(raw, conn.msgtype)
                images[topic_to_camera[conn.topic]].append((t, bytes(msg.data)))
            elif conn.topic in joints:
                try:
                    msg = reader.deserialize(raw, conn.msgtype)
                except AnyReaderError:
                    # ESP32(micro-ROS)の車輪の関節角は空文字列を長さ0で書き、rosbagsが読めない。腕の関節は入っていない。
                    continue
                pos = dict(zip(msg.name, msg.position))
                # /joint_statesには車輪だけのメッセージも混ざるため、腕の関節がそろったものだけ使う。
                if all(j in pos for j in JOINTS):
                    joints[conn.topic].append((t, [pos[j] for j in JOINTS]))
    return images, joints


def frames(images, joints, fps):
    streams = list(images.values()) + list(joints.values())
    if any(not s for s in streams):
        return None
    # 全部の流れがそろっている区間だけを、記録した時刻(開発機で受け取った時刻)の一定間隔で切り出す。
    start = max(s[0][0] for s in streams)
    end = min(s[-1][0] for s in streams)
    grid = np.arange(start, end, int(1e9 / fps))
    out = {}
    for name, seq in images.items():
        t = np.array([x[0] for x in seq])
        idx = np.searchsorted(t, grid, side='right') - 1
        out[name] = [seq[i][1] for i in idx]
    for topic, seq in joints.items():
        t = np.array([x[0] for x in seq], dtype=np.float64)
        q = np.array([x[1] for x in seq])
        out[topic] = np.stack([np.interp(grid.astype(np.float64), t, q[:, k]) for k in range(q.shape[1])], 1)
    return grid, out


def decode(jpeg):
    return np.asarray(Image.open(io.BytesIO(jpeg)).convert('RGB'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('raw', type=Path, help='record_episodesの保存先(episode_*が並ぶフォルダ)')
    ap.add_argument('out', type=Path, help='作るLeRobotデータセットのフォルダ')
    ap.add_argument('--repo-id', default=None, help='既定はdiegobot/<rawのフォルダ名>')
    ap.add_argument('--fps', type=int, default=15)
    ap.add_argument('--overwrite', action='store_true')
    args = ap.parse_args()

    episodes = sorted(p for p in args.raw.glob('episode_*') if (p / 'episode.json').exists())
    if not episodes:
        raise SystemExit(f'{args.raw} に保存済みのエピソードがありません')
    if args.out.exists():
        if not args.overwrite:
            raise SystemExit(f'{args.out} は既にあります(作り直すなら --overwrite)')
        shutil.rmtree(args.out)

    typestore = get_typestore(Stores.ROS2_JAZZY)
    first_images, _ = read_episode(episodes[0], typestore)
    shapes = {name: decode(seq[0][1]).shape for name, seq in first_images.items()}
    features = {
        'observation.state': {'dtype': 'float32', 'shape': (len(JOINTS),), 'names': JOINTS},
        'action': {'dtype': 'float32', 'shape': (len(JOINTS),), 'names': JOINTS},
    }
    for name, shape in shapes.items():
        features[f'observation.images.{name}'] = {'dtype': 'video', 'shape': shape, 'names': ['height', 'width', 'channels']}
    dataset = LeRobotDataset.create(args.repo_id or f'diegobot/{args.raw.name}', args.fps, features,
                                    root=args.out, robot_type='diegobot_so101', use_videos=True)

    for path in episodes:
        info = json.loads((path / 'episode.json').read_text())
        task = info['task']
        images, joints = read_episode(path, typestore)
        result = frames(images, joints, args.fps)
        if result is None:
            print(f'{path.name}: 届いていないトピックがあるため飛ばす')
            continue
        grid, data = result
        for k in range(len(grid)):
            frame = {
                'observation.state': data[STATE_TOPIC][k].astype(np.float32),
                'action': data[ACTION_TOPIC][k].astype(np.float32),
                'task': task,
            }
            for name in CAMERAS:
                frame[f'observation.images.{name}'] = decode(data[name][k])
            dataset.add_frame(frame)
        dataset.save_episode()
        cut = info['duration'] - len(grid) / args.fps
        # 記録の途中から届き始めたカメラがあると、そろっている区間だけになり始めの動きが抜ける。
        note = f'  注意: 全トピックがそろわない{cut:.1f}秒を削った' if cut > 1.0 else ''
        print(f'{path.name}: {len(grid)}コマ({len(grid) / args.fps:.1f}秒){note}')
    dataset.finalize()
    print(f'保存: {args.out}')


if __name__ == '__main__':
    main()
