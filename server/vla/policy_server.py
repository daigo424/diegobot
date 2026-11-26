"""学習したLeRobotのポリシーを開発機のGPUで動かし、ROS側のpolicy_runnerからTCPで観測を受けて動作を返す。

LeRobot(torch)はROSのコンテナに無いため、推論だけをコンテナの外で動かす。
    python policy_server.py data/vla/train/<job>/checkpoints/last/pretrained_model [--port 5555]

1回のやり取りは、4バイト(ビッグエンディアン)の長さ＋JSONのヘッダー＋JPEGの本体。
ヘッダーは {"reset": bool, "state": [...], "images": {"head": <バイト数>, "wrist": <バイト数>}, "task": str}。
返事は4バイトの長さ＋JSON {"action": [...]}(resetだけの時は {"ok": true})。
"""
import argparse
import io
import json
import socket
import struct
import time
from pathlib import Path

import numpy as np
import torch
from lerobot.common.control_utils import predict_action
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from PIL import Image


def recv_exact(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError('closed')
        buf += chunk
    return bytes(buf)


def recv_message(conn):
    header = json.loads(recv_exact(conn, struct.unpack('>I', recv_exact(conn, 4))[0]))
    images = {name: recv_exact(conn, size) for name, size in header.get('images', {}).items()}
    return header, images


def send_json(conn, obj):
    data = json.dumps(obj).encode()
    conn.sendall(struct.pack('>I', len(data)) + data)


def decode(jpeg):
    # 変換(convert_episodes.py)と同じくPILでRGBに戻す。学習と推論で画像の作り方をそろえるため。
    return np.asarray(Image.open(io.BytesIO(jpeg)).convert('RGB'))


def read_rename_map(checkpoint):
    path = Path(checkpoint) / 'train_config.json'
    if not path.exists():
        return {}
    return json.loads(path.read_text()).get('rename_map') or {}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('checkpoint', help='pretrained_modelのフォルダ')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5555)
    ap.add_argument('--device', default='cuda')
    ap.add_argument('--n-action-steps', type=int, help='1回の推論で出した動作を何コマ続けて使うか')
    ap.add_argument('--temporal-ensemble', type=float, metavar='COEFF',
                    help='ACTで毎コマ推論し、重なった予測を重み exp(-COEFF*i) で平均する')
    args = ap.parse_args()

    cfg = PreTrainedConfig.from_pretrained(args.checkpoint)
    cfg.device = args.device
    if args.temporal_ensemble is not None:
        cfg.temporal_ensemble_coeff = args.temporal_ensemble
        cfg.n_action_steps = 1
    elif args.n_action_steps:
        cfg.n_action_steps = args.n_action_steps
    policy = get_policy_class(cfg.type).from_pretrained(args.checkpoint, config=cfg)
    policy.to(args.device).eval()
    preprocessor, postprocessor = make_pre_post_processors(cfg, pretrained_path=args.checkpoint)
    device = torch.device(args.device)
    # SmolVLAは学習時にカメラ名をcamera1等へ読み替えている。読み替え先が無いキー(空のカメラ)は渡さず、モデル側で空にさせる。
    renamed = {v: k for k, v in read_rename_map(args.checkpoint).items()}
    image_keys = {k: renamed.get(k, k).rsplit('.', 1)[1] for k in cfg.input_features
                  if k.startswith('observation.images.') and (k in renamed or not renamed)}
    print(f'{cfg.type} を読み込んだ。カメラ {image_keys}、n_action_steps={cfg.n_action_steps}、'
          f'temporal_ensemble_coeff={getattr(cfg, "temporal_ensemble_coeff", None)}', flush=True)

    with socket.create_server((args.host, args.port)) as server:
        print(f'{args.host}:{args.port} で待っている', flush=True)
        while True:
            conn, _ = server.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print('つながった', flush=True)
            with conn:
                try:
                    while True:
                        header, images = recv_message(conn)
                        if header.get('reset'):
                            policy.reset()
                            preprocessor.reset()
                            postprocessor.reset()
                            send_json(conn, {'ok': True})
                            continue
                        start = time.perf_counter()
                        obs = {'observation.state': np.asarray(header['state'], dtype=np.float32)}
                        for key, camera in image_keys.items():
                            obs[key] = decode(images[camera])
                        action = predict_action(obs, policy, device, preprocessor, postprocessor,
                                                use_amp=False, task=header.get('task'))
                        send_json(conn, {'action': action.squeeze(0).cpu().numpy().astype(float).tolist(),
                                         'time': time.perf_counter() - start})
                except ConnectionError:
                    print('切れた', flush=True)


if __name__ == '__main__':
    main()
