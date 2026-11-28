"""record_episodesで記録したエピソードを、頭部|手首カメラを横に並べたmp4にする。

LeRobotとrosbagsが入ったPython環境で動かす(ROSは要らない。make vla-videoから呼ぶ)。
    python episode_video.py data/vla/raw/<dataset> --out data/vla/videos/<dataset> [--episodes 0 3]
"""
import argparse
import json
from pathlib import Path

import av
import cv2
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

CAMERAS = ['/head_camera/color/image_raw/compressed', '/camera/camera/color/image_raw/compressed']
HEIGHT = 480


def read_images(path, typestore):
    images = {topic: [] for topic in CAMERAS}
    with AnyReader([path], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic in images]
        for conn, t, raw in reader.messages(connections=connections):
            images[conn.topic].append((t, bytes(reader.deserialize(raw, conn.msgtype).data)))
    return images


def blank(width):
    frame = np.zeros((HEIGHT, width, 3), np.uint8)
    cv2.putText(frame, 'no image', (20, HEIGHT // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)
    return frame


def decode(jpeg):
    frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    return cv2.resize(frame, (round(frame.shape[1] * HEIGHT / frame.shape[0]), HEIGHT))


def make_video(episode_dir, out_path, fps, typestore):
    images = read_images(episode_dir, typestore)
    present = [topic for topic in CAMERAS if images[topic]]
    if not present:
        return None
    start = min(images[topic][0][0] for topic in present)
    end = max(images[topic][-1][0] for topic in present)
    info_path = episode_dir / 'episode.json'
    info = json.loads(info_path.read_text()) if info_path.exists() else {}
    title = f'{episode_dir.name}  {info.get("task", "")}' + ('' if info.get('success', True) else '  (failed)')
    widths = {topic: decode(images[topic][0][1]).shape[1] if images[topic] else 640 for topic in CAMERAS}
    width = sum(widths.values())
    width += width % 2
    with av.open(str(out_path), 'w') as container:
        stream = container.add_stream('libx264', rate=round(fps))
        stream.width, stream.height, stream.pix_fmt = width, HEIGHT, 'yuv420p'
        index = {topic: 0 for topic in CAMERAS}
        last = {}
        for k in range(int((end - start) * 1e-9 * fps) + 1):
            t = start + int(k * 1e9 / fps)
            row = []
            for topic in CAMERAS:
                frames = images[topic]
                # その時刻以前で最後に届いた画像を使う。
                while index[topic] + 1 < len(frames) and frames[index[topic] + 1][0] <= t:
                    index[topic] += 1
                if not frames or frames[0][0] > t:
                    row.append(blank(widths[topic]))
                    continue
                if last.get(topic, (None,))[0] != index[topic]:
                    last[topic] = (index[topic], decode(frames[index[topic]][1]))
                row.append(last[topic][1])
            frame = np.concatenate(row, axis=1)
            if frame.shape[1] < width:
                frame = np.pad(frame, ((0, 0), (0, width - frame.shape[1]), (0, 0)))
            cv2.putText(frame, f'{title}  {k / fps:5.1f}s', (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            for packet in stream.encode(av.VideoFrame.from_ndarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), 'rgb24')):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)
    return out_path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--episodes', type=int, nargs='*', help='episode numbers (default: all)')
    parser.add_argument('--fps', type=int, default=15)
    args = parser.parse_args()

    typestore = get_typestore(Stores.ROS2_JAZZY)
    episodes = sorted(d for d in args.dataset.glob('episode_*') if d.is_dir())
    if args.episodes:
        episodes = [d for d in episodes if int(d.name.split('_')[1]) in args.episodes]
    if not episodes:
        raise SystemExit(f'no episodes in {args.dataset}')
    args.out.mkdir(parents=True, exist_ok=True)
    for episode_dir in episodes:
        out_path = args.out / f'{episode_dir.name}.mp4'
        bags = list(episode_dir.glob('*.mcap'))
        # 作った後に記録が変わっていなければ作り直さない。
        if out_path.exists() and bags and out_path.stat().st_mtime > max(b.stat().st_mtime for b in bags):
            print(out_path, flush=True)
            continue
        print(make_video(episode_dir, out_path, args.fps, typestore) or f'{episode_dir.name}: no camera images',
              flush=True)


if __name__ == '__main__':
    main()
