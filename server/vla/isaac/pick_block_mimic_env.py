"""Isaac Lab Mimic environment of the pick-block task: end-effector poses to and from joint-angle actions.

A 5-joint arm cannot reach every 6-D pose, so the inverse kinematics is a damped least-squares fit that favours the
fingertip position. Import only after the simulation app has started.
"""

import os
import sys
import xml.etree.ElementTree as ET

import isaaclab.utils.math as PoseUtils
import torch
from isaaclab.envs import ManagerBasedRLMimicEnv

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pick_block_env import REPO  # noqa: E402
from pick_block_mimic import EEF  # noqa: E402

URDF = REPO / '.claude/report/isaac-scene-inputs/robot.urdf'
ARM = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
# Trading 1 rad of orientation for 5 cm keeps the fingertips on target when the full pose is out of reach.
ORIENTATION_WEIGHT = 0.05
IK_ITERATIONS = 30
IK_DAMPING = 0.01


def rpy_matrix(r, p, y):
    """URDF roll-pitch-yaw: rotations about the fixed x, y, then z axes."""
    def turn(angle, i, j):
        m = torch.eye(3)
        c, s = torch.cos(torch.tensor(angle)), torch.sin(torch.tensor(angle))
        m[i, i], m[i, j], m[j, i], m[j, j] = c, -s, s, c
        return m
    return turn(y, 0, 1) @ turn(p, 2, 0) @ turn(r, 1, 2)


class ArmKinematics:
    """Forward and inverse kinematics from base_footprint to gripper_frame_link, parsed from the URDF."""

    def __init__(self, urdf, device):
        joints = {j.find('child').get('link'): j for j in ET.parse(urdf).getroot().findall('joint')}
        chain, link = [], 'gripper_frame_link'
        while link in joints:
            chain.append(joints[link])
            link = joints[link].find('parent').get('link')
        self.steps = []
        for j in reversed(chain):
            o = j.find('origin')
            origin = torch.eye(4)
            origin[:3, :3] = rpy_matrix(*[float(v) for v in o.get('rpy', '0 0 0').split()])
            origin[:3, 3] = torch.tensor([float(v) for v in o.get('xyz', '0 0 0').split()])
            moving = j.get('type') != 'fixed'
            axis = torch.tensor([float(v) for v in j.find('axis').get('xyz').split()]) if moving else None
            self.steps.append((origin.to(device), ARM.index(j.get('name')) if moving else None,
                               axis.to(device) if moving else None))
        self.limits = torch.zeros(len(ARM), 2, device=device)
        for j in chain:
            if j.get('type') != 'fixed':
                limit = j.find('limit')
                self.limits[ARM.index(j.get('name'))] = torch.tensor(
                    [float(limit.get('lower')), float(limit.get('upper'))])

    def forward(self, q):
        """(N, 5) joint angles -> (N, 4, 4) gripper_frame_link poses in base_footprint."""
        pose = torch.eye(4, device=q.device).repeat(len(q), 1, 1)
        for origin, index, axis in self.steps:
            pose = pose @ origin
            if index is not None:
                turn = torch.eye(4, device=q.device).repeat(len(q), 1, 1)
                turn[:, :3, :3] = PoseUtils.matrix_from_quat(
                    PoseUtils.quat_from_angle_axis(q[:, index], axis.expand(len(q), 3)))
                pose = pose @ turn
        return pose

    def _error(self, q, target):
        pose = self.forward(q)
        pos = target[:, :3, 3] - pose[:, :3, 3]
        rot = PoseUtils.axis_angle_from_quat(PoseUtils.quat_from_matrix(target[:, :3, :3] @ pose[:, :3, :3].mT))
        return torch.cat([pos, ORIENTATION_WEIGHT * rot], dim=1)

    def inverse(self, target, seed):
        """Joint angles that bring the fingertips to ``target`` (N, 4, 4), starting from ``seed`` (N, 5)."""
        q = seed.clone()
        eye = torch.eye(6, device=q.device)
        for _ in range(IK_ITERATIONS):
            err = self._error(q, target)
            jac = torch.stack([(err - self._error(q + 1e-4 * torch.eye(5, device=q.device)[i], target)) / 1e-4
                               for i in range(5)], dim=2)
            step = jac.mT @ torch.linalg.solve(jac @ jac.mT + IK_DAMPING ** 2 * eye, err.unsqueeze(2))
            q = torch.max(torch.min(q + step.squeeze(2), self.limits[:, 1]), self.limits[:, 0])
        return q


class PickBlockMimicEnv(ManagerBasedRLMimicEnv):
    def __init__(self, cfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self.kinematics = ArmKinematics(URDF, self.device)
        self.arm_ids = self.scene['robot'].find_joints(ARM, preserve_order=True)[0]

    def get_robot_eef_pose(self, eef_name, env_ids=None):
        if env_ids is None:
            env_ids = slice(None)
        pos = self.obs_buf['policy']['eef_pos'][env_ids]
        quat = self.obs_buf['policy']['eef_quat'][env_ids]
        return PoseUtils.make_pose(pos, PoseUtils.matrix_from_quat(quat))

    def target_eef_pose_to_action(self, target_eef_pose_dict, gripper_action_dict, noise=None, env_id=0):
        (target,) = target_eef_pose_dict.values()
        (gripper,) = gripper_action_dict.values()
        target = target.clone().unsqueeze(0)
        if noise is not None:
            target[:, :3, 3] += noise * torch.randn_like(target[:, :3, 3])
        seed = self.scene['robot'].data.joint_pos.torch[env_id:env_id + 1, self.arm_ids]
        q = self.kinematics.inverse(target, seed)
        return torch.cat([q[0], gripper], dim=0).unsqueeze(0)

    def action_to_target_eef_pose(self, action):
        return {EEF: self.kinematics.forward(action[:, :5])}

    def actions_to_gripper_actions(self, actions):
        return {EEF: actions[:, 5:6]}

    def get_subtask_term_signals(self, env_ids=None):
        if env_ids is None:
            env_ids = slice(None)
        return {'grasp': self.obs_buf['subtask_terms']['grasp'][env_ids]}
