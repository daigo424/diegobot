"""Registers the diegobot tasks with gym without loading them, for Isaac Lab scripts' ``--external_callback``.

Put this folder on PYTHONPATH and pass ``--external_callback diegobot_tasks.register``.
"""

import gymnasium as gym

MIMIC_TASK_ID = 'Diegobot-PickBlock-Joint-Mimic-v0'


def register():
    if MIMIC_TASK_ID not in gym.registry:
        gym.register(id=MIMIC_TASK_ID, entry_point='pick_block_mimic_env:PickBlockMimicEnv',
                     kwargs={'env_cfg_entry_point': 'pick_block_mimic:PickBlockMimicEnvCfg'},
                     disable_env_checker=True)
    return []
