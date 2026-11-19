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


class KinematicChain:
    """URDFのbaseからtipまでの関節の連なりの順運動学。"""

    def __init__(self, urdf, base, tip):
        joints = {j.find('child').get('link'): j for j in ET.fromstring(urdf).findall('joint')}
        chain, link = [], tip
        while link != base:
            if link not in joints:
                raise ValueError(f'{tip} is not a descendant of {base}')
            chain.append(joints[link])
            link = joints[link].find('parent').get('link')
        self.steps = []
        self.joint_names, self.lower, self.upper = [], {}, {}
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
                name, limit = j.get('name'), j.find('limit')
                self.joint_names.append(name)
                self.lower[name] = float(limit.get('lower', -np.pi)) if limit is not None else -np.pi
                self.upper[name] = float(limit.get('upper', np.pi)) if limit is not None else np.pi

    def fk(self, q):
        m = np.eye(4)
        for fixed, name, axis in self.steps:
            m = m @ fixed
            if name is not None:
                r = np.eye(4)
                r[:3, :3] = _axis_angle(axis, q[name])
                m = m @ r
        return m


class TopDownIK(KinematicChain):
    """道具点を指定位置へ真下向きに置く関節角度を求める。

    SO101は5関節で一般の6自由度の姿勢を指定できないため、MoveItの汎用IKを使わずここで解く。
    手首のひねり(roll)は真下向きでも道具点の位置を変えないので、呼び出し側が決めた値で固定する。
    """

    def __init__(self, urdf, base, tip, roll_joint, approach_axis=(0.0, 0.0, 1.0), tcp_offset=(0.0, 0.0, 0.0)):
        super().__init__(urdf, base, tip)
        if roll_joint not in self.joint_names:
            raise ValueError(f'{roll_joint} is not in the chain from {base} to {tip}')
        self.roll_joint = roll_joint
        self.solved_joints = [n for n in self.joint_names if n != roll_joint]
        self.approach_axis = np.array(approach_axis, dtype=float)
        # 位置を合わせる点(tipの座標系)。SO101のtipは固定側の指先にあり、物体を挟む位置はそこから可動側へずれる。
        self.tcp = np.append(np.asarray(tcp_offset, dtype=float), 1.0)

    def solve(self, position, roll, seeds, margin=0.02, tolerance=0.003, max_tilt=np.radians(3)):
        """tcp_offsetの点をposition(base基準)へ真下向きに置く関節角度を返す。届かなければNone。"""
        position = np.asarray(position, dtype=float)
        lo = np.array([self.lower[n] + margin for n in self.solved_joints])
        hi = np.array([self.upper[n] - margin for n in self.solved_joints])

        def residual(x):
            q = dict(zip(self.solved_joints, x), **{self.roll_joint: roll})
            m = self.fk(q)
            approach = m[:3, :3] @ self.approach_axis
            # 傾き(rad)を位置の誤差と同じ桁にするため、道具の長さ程度の値を掛ける。
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


class Gaze(KinematicChain):
    """カメラの光軸を指定点へ向ける関節角度を求める。動かす関節以外は呼び出し側の値で固定する。
    cameraは光学座標系(zが前)のフレーム。optical=Falseなら、xが前・yが左・zが上のフレームとして扱う。
    """

    def __init__(self, urdf, base, camera, moving_joints, optical=True):
        super().__init__(urdf, base, camera)
        self.moving_joints = list(moving_joints)
        self.optical = optical

    def solve(self, point, fixed, seed, margin=0.02, limits=None):
        """光軸からpointまでのずれ(画像上の正規化座標x, y)を最小にした関節角度と、そのずれを返す。

        limitsは関節名→(下限, 上限)で、URDFの範囲より狭めたい関節(他の部品に当たる等)に使う。
        """
        point = np.append(np.asarray(point, dtype=float), 1.0)
        limits = limits or {}
        lo = np.array([max(self.lower[n] + margin, limits.get(n, (-np.inf, np.inf))[0]) for n in self.moving_joints])
        hi = np.array([min(self.upper[n] - margin, limits.get(n, (-np.inf, np.inf))[1]) for n in self.moving_joints])

        def offset(x):
            q = dict(fixed, **dict(zip(self.moving_joints, x)))
            p = np.linalg.inv(self.fk(q)) @ point
            if not self.optical:
                p = np.array([-p[1], -p[2], p[0]])
            # 点がカメラの後ろにあっても滑らかに前へ戻せるよう、奥行きは正の小さな値で下から抑える。
            return np.array([p[0], p[1]]) / max(p[2], 0.05)

        x0 = np.clip([seed[n] for n in self.moving_joints], lo, hi)
        sol = least_squares(offset, x0, bounds=(lo, hi))
        return dict(fixed, **dict(zip(self.moving_joints, sol.x))), offset(sol.x)


class ViewPose(KinematicChain):
    """物をなるべく横に近い角度から見る、カメラ(光学座標系)の関節角度を求める。

    見下ろす角度を浅い方から順に試し、物の全体が画像に収まり、カメラと道具の先が床から離れている最初の解を返す。
    """

    def __init__(self, urdf, base, camera, tool, fixed_joints):
        super().__init__(urdf, base, camera)
        self.tool = KinematicChain(urdf, base, tool)
        self.fixed = dict(fixed_joints)
        self.moving_joints = [n for n in self.joint_names if n not in self.fixed]

    def solve(self, bottom, height, radius, seeds, image_size, focal, angles, distance=0.3,
              min_distance=0.2, max_distance=0.5, floor_z=0.0, clearance=0.03, fill=0.8, margin=0.02, limits=None):
        """床(floor_z)に置かれた、底面の中心bottom・高さheight・半径radiusの物を見る関節角度と見下ろす角度を返す。
        解けなければ(None, None)。image_sizeは(幅, 高さ)[px]、focalは(fx, fy)[px]、anglesは試す見下ろす角度[rad]。
        limitsは関節名→(下限, 上限)で、URDFの範囲より狭めたい関節(腕を前へ伸ばしすぎると肩が支えきれない等)に使う。"""
        bottom = np.asarray(bottom, dtype=float)
        center = bottom + [0.0, 0.0, height / 2.0]
        limits = limits or {}
        lo = np.array([max(self.lower[n] + margin, limits.get(n, (-np.inf, np.inf))[0]) for n in self.moving_joints])
        hi = np.array([min(self.upper[n] - margin, limits.get(n, (-np.inf, np.inf))[1]) for n in self.moving_joints])
        # 物を外接する円柱の上下の縁の点。全部が画像の枠の内側(fillの割合)に写れば、物の全体が収まる。
        ring = [np.array([np.cos(a), np.sin(a), 0.0]) * radius for a in np.linspace(0, 2 * np.pi, 8, endpoint=False)]
        corners = [bottom + r + [0.0, 0.0, z] for r in ring for z in (0.0, height)]
        half = np.array(image_size, dtype=float) * fill / 2.0

        def joints(x):
            return dict(self.fixed, **dict(zip(self.moving_joints, x)))

        for angle in angles:
            def residual(x):
                m = self.fk(joints(x))
                p = np.linalg.inv(m) @ np.append(center, 1.0)
                v = center - m[:3, 3]
                down = np.arctan2(-v[2], np.hypot(v[0], v[1]))
                return np.array([p[0] / max(p[2], 0.05), p[1] / max(p[2], 0.05),
                                 down - angle, 0.3 * (np.linalg.norm(v) - distance)])

            for seed in seeds:
                x0 = np.clip([seed[n] for n in self.moving_joints], lo, hi)
                sol = least_squares(residual, x0, bounds=(lo, hi))
                q = joints(sol.x)
                if self._usable(q, center, corners, half, focal, angle, min_distance, max_distance,
                                floor_z + clearance):
                    return q, angle
        return None, None

    def _usable(self, q, center, corners, half, focal, angle, min_distance, max_distance, min_z):
        m = self.fk(q)
        v = center - m[:3, 3]
        if not min_distance <= np.linalg.norm(v) <= max_distance:
            return False
        if abs(np.arctan2(-v[2], np.hypot(v[0], v[1])) - angle) > np.radians(5):
            return False
        if m[2, 3] < min_z or self.tool.fk(q)[2, 3] < min_z:
            return False
        inv = np.linalg.inv(m)
        for c in corners:
            p = inv @ np.append(c, 1.0)
            if p[2] <= 0.0 or abs(focal[0] * p[0] / p[2]) > half[0] or abs(focal[1] * p[1] / p[2]) > half[1]:
                return False
        return True
