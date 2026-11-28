"""Manager-based Isaac Lab task of diegobot picking up the block, in the form Isaac Lab Mimic records and replays.

The robot base is fixed at each env origin, so env-local coordinates are base_footprint coordinates.
Import only after the simulation app has started (``launch_simulation``).
"""

import math
import os
import sys
from pathlib import Path

import gymnasium as gym
import isaaclab.envs.mdp as mdp
import torch
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.mdp.recorders.recorders_cfg import ActionStateRecorderManagerCfg
from isaaclab.managers import DatasetExportMode
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg, OffsetCfg
from isaaclab.utils import configclass
from isaaclab_physx.physics import PhysxCfg
from isaaclab_tasks.contrib.stack.mdp.observations import ee_frame_pos, ee_frame_quat
from isaaclab_tasks.contrib.stack.mdp.stack_events import randomize_object_pose

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from diegobot_scene import (  # noqa: E402
    ARM_JOINTS, BASE, BLOCK_SIZE, GRIPPER_LINK, DiegobotSceneCfg, dress_scene, floor_texture, robot_cfg, yaw_quat)

REPO = Path(__file__).resolve().parents[3]
ROBOT_USD = REPO / 'data/vla/isaac/diegobot/diegobot.usd/robot/robot.usda'
TEXTURES = REPO / 'data/vla/isaac/textures/nvidia'
TASK_ID = 'Diegobot-PickBlock-Joint-v0'
ARM = SceneEntityCfg('robot', joint_names=ARM_JOINTS, preserve_order=True)

# The jaw closes to about -0.17 rad on nothing; on the 3 cm block it stops well above that.
GRASP_GRIPPER_RANGE = (-0.05, 0.8)
GRASP_DISTANCE = 0.04
LIFT_HEIGHT = 0.04


def dress(env, env_ids, floor, tile, yaw):
    dress_scene(env.sim.stage, env.num_envs, floor_texture(TEXTURES, floor), tile, yaw)


def joint_ids(robot, names):
    return robot.find_joints(names, preserve_order=True)[0]


def reset_arm(env, env_ids):
    """Start the arm at ``env.arm_start`` when the teleop script sets it (the leader's pose), else at default."""
    robot = env.scene['robot']
    ids = joint_ids(robot, ARM_JOINTS)
    pos = robot.data.default_joint_pos.torch[env_ids][:, ids].clone()
    start = getattr(env, 'arm_start', None)
    if start is not None:
        pos[:] = start
    robot.write_joint_position_to_sim_index(position=pos, joint_ids=ids, env_ids=env_ids)
    robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(pos), joint_ids=ids, env_ids=env_ids)
    robot.set_joint_position_target_index(target=pos, joint_ids=ids, env_ids=env_ids)


def place_block(env, env_ids, pose_range):
    """Put the block at ``env.block_start`` (x, y, yaw) when a replay sets it, else at a random pose."""
    start = getattr(env, 'block_start', None)
    if start is None:
        randomize_object_pose(env, env_ids, [SceneEntityCfg('block')], pose_range=pose_range)
        return
    if isinstance(env_ids, slice):
        env_ids = torch.arange(env.num_envs, device=env.device)[env_ids]
    x, y, yaw = start
    pose = torch.tensor([[x, y, BLOCK_SIZE[2] / 2, *yaw_quat(yaw)]], device=env.device).repeat(len(env_ids), 1)
    pose[:, :3] += env.scene.env_origins[env_ids]
    block = env.scene['block']
    block.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
    block.write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(env_ids), 6, device=env.device),
                                           env_ids=env_ids)


def record_successes(env_cfg, dataset):
    """Record initial state, actions, states and policy observations, writing only the successful episodes."""
    env_cfg.recorders = ActionStateRecorderManagerCfg()
    env_cfg.recorders.dataset_export_dir_path = os.path.dirname(os.path.abspath(dataset))
    env_cfg.recorders.dataset_filename = os.path.splitext(os.path.basename(dataset))[0]
    env_cfg.recorders.dataset_export_mode = DatasetExportMode.EXPORT_SUCCEEDED_ONLY
    os.makedirs(env_cfg.recorders.dataset_export_dir_path, exist_ok=True)


def save_success(env):
    env.recorder_manager.record_pre_reset([0], force_export_or_skip=False)
    env.recorder_manager.set_success_to_episodes([0], torch.tensor([[True]], dtype=torch.bool, device=env.device))
    env.recorder_manager.export_episodes([0])


def block_pose(env):
    block = env.scene['block']
    return torch.cat((block.data.root_pos_w.torch - env.scene.env_origins, block.data.root_quat_w.torch), dim=1)


def gripper_angle(env):
    robot = env.scene['robot']
    return robot.data.joint_pos.torch[:, joint_ids(robot, ['gripper'])]


def block_grasped(env):
    block = env.scene['block'].data.root_pos_w.torch
    ee = env.scene['ee_frame'].data.target_pos_w.torch[:, 0, :]
    q = gripper_angle(env)[:, 0]
    near = torch.linalg.vector_norm(block - ee, dim=1) < GRASP_DISTANCE
    return near & (q > GRASP_GRIPPER_RANGE[0]) & (q < GRASP_GRIPPER_RANGE[1])


def block_lifted(env):
    z = env.scene['block'].data.root_pos_w.torch[:, 2] - env.scene.env_origins[:, 2]
    return (z > BLOCK_SIZE[2] / 2 + LIFT_HEIGHT) & block_grasped(env)


@configclass
class PickBlockSceneCfg(DiegobotSceneCfg):
    robot = robot_cfg(ROBOT_USD)
    # The grasp point between the fingertips (URDF gripper_frame_link), measured from base_link.
    ee_frame = FrameTransformerCfg(
        prim_path=BASE,
        target_frames=[FrameTransformerCfg.FrameCfg(
            prim_path=GRIPPER_LINK, name='end_effector',
            offset=OffsetCfg(pos=(-0.0079, -0.000218, -0.0981274), rot=(0.0, 1.0, 0.0, 0.0)))],
    )


@configclass
class ActionsCfg:
    # Absolute joint targets [rad], the same as the real recordings' leader angles.
    arm_action = mdp.JointPositionActionCfg(asset_name='robot', joint_names=ARM_JOINTS[:5], scale=1.0,
                                            use_default_offset=False)
    gripper_action = mdp.JointPositionActionCfg(asset_name='robot', joint_names=['gripper'], scale=1.0,
                                                use_default_offset=False)


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        actions = ObsTerm(func=mdp.last_action)
        joint_pos = ObsTerm(func=mdp.joint_pos, params={'asset_cfg': ARM})
        joint_vel = ObsTerm(func=mdp.joint_vel, params={'asset_cfg': ARM})
        eef_pos = ObsTerm(func=ee_frame_pos)
        eef_quat = ObsTerm(func=ee_frame_quat)
        gripper_pos = ObsTerm(func=gripper_angle)
        block = ObsTerm(func=block_pose)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    @configclass
    class SubtaskCfg(ObsGroup):
        grasp = ObsTerm(func=block_grasped)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()
    subtask_terms: SubtaskCfg = SubtaskCfg()


@configclass
class EventCfg:
    dress = EventTerm(func=dress, mode='prestartup', params={'floor': 'Ash_Planks', 'tile': 1.2, 'yaw': 1.5708})
    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode='reset', params={'reset_joint_targets': True})
    reset_arm = EventTerm(func=reset_arm, mode='reset')
    # Block centre in base_footprint and its yaw (0 = long side along x; a half turn gives the same block).
    place_block = EventTerm(
        func=place_block, mode='reset',
        params={'pose_range': {'x': (0.08, 0.22), 'y': (0.04, 0.16), 'z': (BLOCK_SIZE[2] / 2, BLOCK_SIZE[2] / 2),
                               'yaw': (-math.pi / 2, math.pi / 2)}},
    )


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    block_dropping = DoneTerm(func=mdp.root_height_below_minimum,
                              params={'minimum_height': -0.05, 'asset_cfg': SceneEntityCfg('block')})
    success = DoneTerm(func=block_lifted)


@configclass
class PickBlockEnvCfg(ManagerBasedRLEnvCfg):
    scene = PickBlockSceneCfg(num_envs=1, env_spacing=2.0, replicate_physics=False)
    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventCfg()
    terminations = TerminationsCfg()
    commands = None
    rewards = None
    curriculum = None

    def __post_init__(self):
        # 120 Hz physics, 30 Hz control: the real teleop follows the leader at 50 Hz and episodes are kept at 15 fps.
        self.sim.dt = 1.0 / 120.0
        self.decimation = 4
        self.sim.render_interval = self.decimation
        self.episode_length_s = 60.0
        self.sim.physics = PhysxCfg(
            bounce_threshold_velocity=0.01,
            friction_correlation_distance=0.00625,
            # Let block contacts stall the position-driven jaw before it tunnels through the grasp.
            solve_articulation_contact_last=True,
        )
        self.viewer.eye = (-0.35, -0.45, 0.75)
        self.viewer.lookat = (0.25, 0.05, 0.0)

    def without_cameras(self):
        self.scene.head_camera = None
        self.scene.wrist_camera = None
        return self


gym.register(id=TASK_ID, entry_point='isaaclab.envs:ManagerBasedRLEnv',
             kwargs={'env_cfg_entry_point': f'{__name__}:PickBlockEnvCfg'}, disable_env_checker=True)
