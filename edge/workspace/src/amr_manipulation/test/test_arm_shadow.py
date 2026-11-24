import numpy as np

from amr_manipulation.arm_shadow import ArmShadow

# 原点から上へ伸びる1関節の棒(箱)。関節を回すと棒が横に倒れる。
URDF = '''<robot name="r">
  <link name="base"/><link name="bar"><visual><origin xyz="0 0 0.2"/><geometry><box size="0.04 0.04 0.4"/></geometry></visual></link>
  <joint name="j" type="revolute"><parent link="base"/><child link="bar"/><axis xyz="0 1 0"/><limit lower="-2" upper="2"/></joint>
</robot>'''


def camera_from_base():
    """baseの前1m・高さ0.2mから後ろ(-x)を見るカメラ(光学座標系: z前、x右、y下)。"""
    m = np.eye(4)
    m[:3, :3] = np.array([[0, 1, 0], [0, 0, -1], [-1, 0, 0]])
    m[:3, 3] = -m[:3, :3] @ np.array([1.0, 0.0, 0.2])
    return m


def test_mask_covers_the_link_where_it_is():
    shadow = ArmShadow(URDF, 'base', ['bar'])
    k = np.array([[300.0, 0, 160], [0, 300.0, 120], [0, 0, 1]])
    upright = shadow.mask({'j': 0.0}, camera_from_base(), k, (320, 240))
    assert upright[120, 160] == 255 and upright[120, 20] == 0
    tilted = shadow.mask({'j': 1.5}, camera_from_base(), k, (320, 240))
    assert tilted.sum() < upright.sum()


def test_unknown_mesh_is_ignored():
    urdf = URDF.replace('<box size="0.04 0.04 0.4"/>', '<mesh filename="package://none/x.stl"/>')
    shadow = ArmShadow(urdf, 'base', ['bar'], resolve=lambda url: '/nonexistent.stl')
    assert shadow.parts == []
