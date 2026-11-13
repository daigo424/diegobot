import numpy as np

from amr_perception.obb import back_project, erode_mask, fit_obb, remove_outliers, remove_plane

FLOOR = -0.0095


def box_points(center, yaw, size, n=3000, rng=None):
    """床に置いた直方体の表面の点(上面と側面)。"""
    rng = rng or np.random.default_rng(1)
    lx, ly, h = size
    local = rng.uniform(-0.5, 0.5, (n, 3)) * [lx, ly, h]
    face = rng.integers(0, 5, n)
    local[face == 0, 0], local[face == 1, 0] = lx / 2, -lx / 2
    local[face == 2, 1], local[face == 3, 1] = ly / 2, -ly / 2
    local[face == 4, 2] = h / 2
    c, s = np.cos(yaw), np.sin(yaw)
    world = local @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    return world + [center[0], center[1], FLOOR + h / 2]


def floor_points(n=1500, rng=None):
    rng = rng or np.random.default_rng(2)
    xy = rng.uniform(-0.1, 0.1, (n, 2)) + [0.3, 0.0]
    return np.column_stack([xy, np.full(n, FLOOR) + rng.normal(0, 0.001, n)])


def test_fits_a_box_on_the_floor():
    points = np.vstack([box_points((0.3, 0.02), np.radians(30), (0.06, 0.03, 0.08)), floor_points()])
    points, plane_z = remove_plane(points, FLOOR)
    assert abs(plane_z - FLOOR) < 0.003
    center, yaw, extents, valid = fit_obb(remove_outliers(points), FLOOR)
    assert valid
    assert np.allclose(center[:2], (0.3, 0.02), atol=0.005)
    assert abs(np.degrees(yaw) - 30) < 5
    assert np.allclose(extents, (0.06, 0.03, 0.08), atol=0.006)


def test_yaw_stays_within_half_a_turn():
    for deg in (100, -100, 179):
        points = box_points((0.3, 0.0), np.radians(deg), (0.06, 0.03, 0.05))
        _, yaw, _, valid = fit_obb(points, FLOOR)
        assert valid and -np.pi / 2 < yaw <= np.pi / 2
        assert abs(np.cos(yaw - np.radians(deg))) > 0.99


def test_round_object_has_no_orientation():
    rng = np.random.default_rng(3)
    a = rng.uniform(0, 2 * np.pi, 2000)
    z = rng.uniform(FLOOR, FLOOR + 0.08, 2000)
    points = np.column_stack([0.3 + 0.03 * np.cos(a), 0.03 * np.sin(a), z])
    center, yaw, extents, valid = fit_obb(points, FLOOR)
    assert not valid and yaw == 0.0
    assert np.allclose(center[:2], (0.3, 0.0), atol=0.005)


def test_too_few_points_gives_nothing():
    center, yaw, extents, valid = fit_obb(np.zeros((10, 3)), FLOOR)
    assert center is None and not valid


def test_far_background_points_are_dropped():
    points = np.vstack([box_points((0.3, 0.0), 0.0, (0.06, 0.03, 0.05)), [[0.9, 0.5, 0.0]] * 20])
    kept = remove_outliers(points)
    assert kept[:, 0].max() < 0.4


def test_back_project_uses_only_masked_pixels_with_depth():
    mask = np.zeros((4, 4), bool)
    mask[1:3, 1:3] = True
    depth = np.full((4, 4), 0.5, np.float32)
    depth[1, 1] = 0.0
    points = back_project(mask, depth, 100.0, 100.0, 2.0, 2.0)
    assert points.shape == (3, 3) and np.allclose(points[:, 2], 0.5)


def test_erosion_scales_with_size():
    mask = np.zeros((100, 100), bool)
    mask[20:80, 20:80] = True
    assert erode_mask(mask, 60).sum() < mask.sum()
    assert erode_mask(mask, 20).sum() > erode_mask(mask, 200).sum()


def test_flat_top_of_an_object_is_not_taken_for_the_floor():
    rng = np.random.default_rng(4)
    top = np.column_stack([rng.uniform(0.27, 0.33, 1500), rng.uniform(-0.015, 0.015, 1500), np.full(1500, FLOOR + 0.03)])
    side = np.column_stack([np.full(500, 0.27), rng.uniform(-0.015, 0.015, 500), rng.uniform(FLOOR, FLOOR + 0.03, 500)])
    points, plane_z = remove_plane(np.vstack([top, side]), FLOOR)
    assert plane_z is None and len(points) == 2000
