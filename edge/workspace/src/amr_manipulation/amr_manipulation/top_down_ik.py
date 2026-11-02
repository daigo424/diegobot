import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, angle):
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


def _floats(text, default):
    return [float(v) for v in (text or default).split()]


class TopDownIK:
    """URDFの関節の連なりから、道具点を指定位置へ真下向きに置く関節角度を求める。

    SO101は5関節で一般の6自由度の姿勢を指定できないため、MoveItの汎用IKを使わずここで解く。
    手首のひねり(roll)は真下向きでも道具点の位置を変えないので、呼び出し側が決めた値で固定する。
    """

    def __init__(self, urdf, base, tip, roll_joint, approach_axis=(0.0, 0.0, 1.0), tcp_offset=(0.0, 0.0, 0.0)):
        joints = {j.find('child').get('link'): j for j in ET.fromstring(urdf).findall('joint')}
        chain, link = [], tip
        while link != base:
            if link not in joints:
                raise ValueError(f'{tip} is not a descendant of {base}')
            chain.append(joints[link])
            link = joints[link].find('parent').get('link')
        self.steps = []
        self.joint_names, lower, upper = [], [], []
        for j in reversed(chain):
            origin = j.find('origin')
            fixed = np.eye(4)
            if origin is not None:
                fixed[:3, :3] = _rpy(*_floats(origin.get('rpy'), '0 0 0'))
                fixed[:3, 3] = _floats(origin.get('xyz'), '0 0 0')
            movable = j.get('type') in ('revolute', 'continuous')
            axis = np.array(_floats(j.find('axis').get('xyz'), '1 0 0')) if movable else None
            self.steps.append((fixed, j.get('name') if movable else None, axis))
            if movable:
                self.joint_names.append(j.get('name'))
                limit = j.find('limit')
                lower.append(float(limit.get('lower', -np.pi)) if limit is not None else -np.pi)
                upper.append(float(limit.get('upper', np.pi)) if limit is not None else np.pi)
        if roll_joint not in self.joint_names:
            raise ValueError(f'{roll_joint} is not in the chain from {base} to {tip}')
        self.roll_joint = roll_joint
        self.solved_joints = [n for n in self.joint_names if n != roll_joint]
        self.lower = dict(zip(self.joint_names, lower))
        self.upper = dict(zip(self.joint_names, upper))
        self.approach_axis = np.array(approach_axis, dtype=float)
        # 位置を合わせる点(tipの座標系)。SO101のtipは固定側の指先にあり、物体を挟む位置はそこから可動側へずれる。
        self.tcp = np.append(np.asarray(tcp_offset, dtype=float), 1.0)

    def fk(self, q):
        m = np.eye(4)
        for fixed, name, axis in self.steps:
            m = m @ fixed
            if name is not None:
                r = np.eye(4)
                r[:3, :3] = _axis_angle(axis, q[name])
                m = m @ r
        return m

    def solve(self, position, roll, seeds, margin=0.02, tolerance=0.003, max_tilt=np.radians(3)):
        """tcp_offsetの点をposition(base基準)へ真下向きに置く関節角度を返す。届かなければNone。"""
        position = np.asarray(position, dtype=float)
        lo = np.array([self.lower[n] + margin for n in self.solved_joints])
        hi = np.array([self.upper[n] - margin for n in self.solved_joints])

        def residual(x):
            q = dict(zip(self.solved_joints, x), **{self.roll_joint: roll})
            m = self.fk(q)
            approach = m[:3, :3] @ self.approach_axis
            # 傾き(rad)を位置の誤差と同じ桁にするため、道具の長さ程度(0.1m)を掛ける。
            return np.concatenate([(m @ self.tcp)[:3] - position, 0.1 * approach[:2]])

        best = None
        for seed in seeds:
            x0 = np.clip([seed[n] for n in self.solved_joints], lo, hi)
            sol = least_squares(residual, x0, bounds=(lo, hi))
            q = dict(zip(self.solved_joints, sol.x), **{self.roll_joint: roll})
            m = self.fk(q)
            error = np.linalg.norm((m @ self.tcp)[:3] - position)
            tilt = np.arccos(np.clip(-(m[:3, :3] @ self.approach_axis)[2], -1.0, 1.0))
            if error <= tolerance and tilt <= max_tilt and (best is None or error < best[0]):
                best = (error, q)
        return None if best is None else best[1]
