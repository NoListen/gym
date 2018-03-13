from gym.envs.mujoco.gather.gather_env import GatherEnv
from gym.envs.mujoco.swimmer import SwimmerEnv


class SwimmerGatherEnv(GatherEnv):

    MODEL_CLASS = SwimmerEnv
    ORI_IND = 2
