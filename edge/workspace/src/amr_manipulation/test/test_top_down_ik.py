import numpy as np
import pytest

from amr_manipulation.top_down_ik import TopDownIK

# 鉛直軸の付け根と、水平軸3本の腕と、ひねりの軸を持つ簡単な腕。道具点はひねりの軸上にある。
ARM = '''<robot name="arm">
  <link name="base"/><link name="l1"/><link name="l2"/><link name="l3"/><link name="l4"/><link name="l5"/><link name="tip"/>
  <joint name="pan" type="revolute"><parent link="base"/><child link="l1"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/><limit lower="-2" upper="2"/></joint>
  <joint name="lift" type="revolute"><parent link="l1"/><child link="l2"/>
    <axis xyz="0 1 0"/><limit lower="-2" upper="2"/></joint>
  <joint name="elbow" type="revolute"><parent link="l2"/><child link="l3"/>
    <origin xyz="0 0 0.12"/><axis xyz="0 1 0"/><limit lower="-2.5" upper="2.5"/></joint>
  <joint name="wrist" type="revolute"><parent link="l3"/><child link="l4"/>
    <origin xyz="0.12 0 0"/><axis xyz="0 1 0"/><limit lower="-2" upper="2"/></joint>
  <joint name="roll" type="revolute"><parent link="l4"/><child link="l5"/>
    <origin xyz="0.03 0 0" rpy="0 1.5708 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="tool" type="fixed"><parent link="l5"/><child link="tip"/><origin xyz="0 0 0.08"/></joint>
</robot>'''

SEEDS = [{'pan': 0.0, 'lift': 0.5, 'elbow': 1.0, 'wrist': 0.5, 'roll': 0.0}]


@pytest.fixture
def ik():
    return TopDownIK(ARM, 'base', 'tip', 'roll')


def test_chain_order(ik):
    assert ik.joint_names == ['pan', 'lift', 'elbow', 'wrist', 'roll']
    assert ik.solved_joints == ['pan', 'lift', 'elbow', 'wrist']


@pytest.mark.parametrize('target', [(0.15, 0.0, 0.05), (0.12, 0.08, 0.02), (0.18, -0.05, 0.1)])
def test_reaches_target_pointing_down(ik, target):
    q = ik.solve(target, 0.3, SEEDS)
    assert q is not None
    m = ik.fk(q)
    assert np.linalg.norm(m[:3, 3] - target) < 0.003
    assert (m[:3, :3] @ ik.approach_axis)[2] < -np.cos(np.radians(3))
    assert q['roll'] == 0.3


def test_out_of_reach(ik):
    assert ik.solve((0.6, 0.0, 0.05), 0.0, SEEDS) is None


def test_respects_joint_limits(ik):
    q = ik.solve((0.15, 0.0, 0.05), 0.0, SEEDS)
    for name in ik.solved_joints:
        assert ik.lower[name] <= q[name] <= ik.upper[name]
