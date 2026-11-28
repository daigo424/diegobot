"""Isaac Lab Mimic configuration of the pick-block task; the environment class is in pick_block_mimic_env.py.

Actions stay as absolute joint angles, like the real recordings, so the replayed source episodes behave exactly as
recorded; Mimic's end-effector poses are converted to and from joint angles with the URDF's kinematics.
Importable without the simulation app, as Isaac Lab loads task configs before starting it.
"""

import os
import sys

from isaaclab.envs.mimic_env_cfg import MimicEnvCfg, SubTaskConfig
from isaaclab.utils import configclass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pick_block_env import PickBlockEnvCfg  # noqa: E402

EEF = 'diegobot'


@configclass
class PickBlockMimicEnvCfg(PickBlockEnvCfg, MimicEnvCfg):
    class_type: type | str = 'pick_block_mimic_env:PickBlockMimicEnv'

    def __post_init__(self):
        super().__post_init__()
        # State-only for now: annotation and generation do not need the cameras, which slow every step.
        self.without_cameras()
        self.datagen_config.name = 'pick_block'
        self.datagen_config.generation_guarantee = True
        self.datagen_config.generation_keep_failed = False
        self.datagen_config.generation_num_trials = 10
        # Approach and lift come from the same source episode, so the grasp is carried out the way it was shown.
        self.datagen_config.generation_select_src_per_subtask = False
        self.datagen_config.generation_transform_first_robot_pose = False
        self.datagen_config.generation_interpolate_from_last_target_pose = True
        self.datagen_config.seed = 1
        common = dict(object_ref='block', selection_strategy='nearest_neighbor_object',
                      selection_strategy_kwargs={'nn_k': 3}, action_noise=0.003, num_interpolation_steps=5,
                      num_fixed_steps=0, apply_noise_during_interpolation=False)
        self.subtask_configs[EEF] = [
            # The grasp signal turns on while the jaw is still closing; carry on until it has closed.
            SubTaskConfig(subtask_term_signal='grasp', subtask_term_offset_range=(10, 15),
                          description='Approach and grasp the block', **common),
            SubTaskConfig(subtask_term_signal=None, subtask_term_offset_range=(0, 0),
                          description='Lift the block', **common),
        ]
