"""アームが別のカメラ(頭部カメラ)の画像のどこを隠すかを、URDFの見た目の形と関節角度から求める。"""
import os
import struct
import xml.etree.ElementTree as ET

import cv2
import numpy as np

from amr_manipulation.top_down_ik import KinematicChain, _floats, _rpy


def load_stl(path):
    """STL(バイナリかASCII)の頂点を(N, 3)で返す。"""
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) >= 84:
        count = struct.unpack_from('<I', data, 80)[0]
        if 84 + 50 * count == len(data):
            tris = np.frombuffer(data, dtype=np.dtype([('n', '<3f4'), ('v', '<3f4', (3,)), ('a', '<u2')]),
                                 count=count, offset=84)
            return tris['v'].reshape(-1, 3).astype(float)
    words = data.decode('ascii', errors='ignore').split()
    return np.array([[float(w) for w in words[i + 1:i + 4]] for i, w in enumerate(words) if w == 'vertex'])


def resolve_package_path(url):
    from ament_index_python.packages import get_package_share_directory
    package, _, rest = url[len('package://'):].partition('/')
    return os.path.join(get_package_share_directory(package), rest)


class ArmShadow:
    """linksの見た目の形を、関節角度とカメラの位置から画像に写した範囲(マスク)を返す。"""

    def __init__(self, urdf, base, links, max_points=800, resolve=resolve_package_path):
        root = ET.fromstring(urdf)
        self.parts = []
        for link in root.findall('link'):
            name = link.get('name')
            if name not in links:
                continue
            points = []
            for visual in link.findall('visual'):
                local = self.shape_points(visual, resolve)
                if local is None:
                    continue
                origin, m = visual.find('origin'), np.eye(4)
                if origin is not None:
                    m[:3, :3] = _rpy(*_floats(origin.get('rpy'), '0 0 0'))
                    m[:3, 3] = _floats(origin.get('xyz'), '0 0 0')
                if len(local) > max_points:
                    local = local[np.random.default_rng(0).choice(len(local), max_points, replace=False)]
                points.append((m[:3, :3] @ local.T).T + m[:3, 3])
            if points:
                self.parts.append((KinematicChain(urdf, base, name), np.vstack(points)))

    @staticmethod
    def shape_points(visual, resolve):
        shape = visual.find('geometry')[0]
        if shape.tag == 'mesh':
            try:
                vertices = load_stl(resolve(shape.get('filename')))
            except (OSError, ValueError):
                return None  # 配布していないメッシュ(手元にだけある物)は無視する
            return vertices * np.array(_floats(shape.get('scale'), '1 1 1'))
        if shape.tag == 'box':
            half = np.array(_floats(shape.get('size'), '0 0 0')) / 2.0
            return np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * half
        return None

    def mask(self, joints, camera_from_base, k, size):
        """joints(関節名→角度)の姿勢で、base→カメラ(光学座標系)の変換camera_from_base・内部パラメータk(3x3)・
        画像の大きさsize(幅, 高さ)のカメラに写る範囲をuint8のマスク(255が隠れる)で返す。"""
        width, height = size
        out = np.zeros((height, width), np.uint8)
        for chain, points in self.parts:
            m = camera_from_base @ chain.fk({n: joints.get(n, 0.0) for n in chain.joint_names})
            p = (m[:3, :3] @ points.T).T + m[:3, 3]
            p = p[p[:, 2] > 0.02]
            if len(p) < 3:
                continue
            uv = (np.asarray(k) @ (p / p[:, 2:3]).T).T[:, :2]
            hull = cv2.convexHull(np.clip(uv, -4 * width, 5 * width).astype(np.int32))
            cv2.fillConvexPoly(out, hull, 255)
        return out
