"""物体のマスクと深度から、床に置かれた物の向き付きの外接直方体(OBB)を求める。"""
import cv2
import numpy as np


def erode_mask(mask, crop_px, ratio=0.04, max_px=6):
    """マスクの縁は背景の深度を拾うため、写った大きさに比例した幅だけ縁を削る(小さい物を削りすぎない)。"""
    k = int(np.clip(round(crop_px * ratio), 1, max_px))
    return cv2.erode(mask.astype(np.uint8), np.ones((2 * k + 1, 2 * k + 1), np.uint8)).astype(bool)


def back_project(mask, depth, fx, fy, cx, cy):
    """マスク内で深度のある画素を、カメラの光学座標系の点(N, 3)にする。"""
    vs, us = np.nonzero(mask & (depth > 0) & np.isfinite(depth))
    z = depth[vs, us]
    return np.column_stack([(us - cx) * z / fx, (vs - cy) * z / fy, z])


def remove_plane(points, floor_z, band=0.02, threshold=0.006, iterations=100, max_tilt=np.radians(15), min_inliers=0.15,
                 rng=None):
    """点群に床の高さ(floor_zからband以内)の水平に近い平面が大きく含まれていれば、その点を除く。
    除いた後の点と、平面の高さ(無ければNone)を返す。積み木の上面のような物の水平な面は残す。"""
    if len(points) < 30:
        return points, None
    rng = rng or np.random.default_rng(0)
    best = None
    for _ in range(iterations):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if np.arccos(min(abs(n[2]), 1.0)) > max_tilt:
            continue
        inliers = np.abs((points - a) @ n) < threshold
        if abs(float(np.median(points[inliers, 2])) - floor_z) > band:
            continue
        if best is None or inliers.sum() > best.sum():
            best = inliers
    if best is None or best.sum() < min_inliers * len(points):
        return points, None
    plane_z = float(np.median(points[best, 2]))
    # 平面より下に出た点も物ではない。
    keep = ~best & (points[:, 2] > plane_z)
    return points[keep], plane_z


def remove_outliers(points, scale=3.0):
    """中央値から水平に遠い点(マスクのはみ出しで拾った奥の背景など)を除く。"""
    if len(points) < 10:
        return points
    d = np.linalg.norm(points[:, :2] - np.median(points[:, :2], axis=0), axis=1)
    mad = np.median(np.abs(d - np.median(d))) + 1e-6
    return points[d <= np.median(d) + scale * 1.4826 * mad]


def fit_obb(points, floor_z, min_points=50, min_elongation=1.3, low=2.0, high=98.0):
    """床に置かれた物の点群(床に水平な座標系)から、OBBの中心・水平面内の向き[rad]・寸法(長辺, 短辺, 高さ)を求める。

    縦は床の法線に固定し、水平面内の向きだけを主成分で決める。点が足りないか、長辺と短辺の差が小さく向きが
    定まらなければ、向きは0として valid=False を返す(中心と寸法はその場合も参考値として返す)。
    下端は隠れて見えないため、床の高さを下端とする。"""
    if len(points) < min_points:
        return None, 0.0, None, False
    xy = points[:, :2]
    mean = xy.mean(axis=0)
    values, vectors = np.linalg.eigh(np.cov((xy - mean).T))
    major = vectors[:, 1]
    yaw = float(np.arctan2(major[1], major[0]))
    # 向きは180°回しても同じ直方体なので、-90°〜90°にそろえる(フレームごとに反転して見えないように)。
    if yaw > np.pi / 2:
        yaw -= np.pi
    elif yaw <= -np.pi / 2:
        yaw += np.pi
    c, s = np.cos(yaw), np.sin(yaw)
    local = (xy - mean) @ np.array([[c, -s], [s, c]])
    lo, hi = np.percentile(local, low, axis=0), np.percentile(local, high, axis=0)
    mid = (lo + hi) / 2.0
    top = float(np.percentile(points[:, 2], high))
    center = np.array([mean[0] + c * mid[0] - s * mid[1], mean[1] + s * mid[0] + c * mid[1], (top + floor_z) / 2.0])
    extents = np.array([hi[0] - lo[0], hi[1] - lo[1], top - floor_z])
    valid = values[1] >= min_elongation ** 2 * max(values[0], 1e-12)
    return center, (yaw if valid else 0.0), extents, bool(valid)
