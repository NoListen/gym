from gym.envs.mujoco.gather.gather_env import GatherEnv
from gym.envs.mujoco.ant import AntEnv


class AntGatherEnv(GatherEnv):

    MODEL_CLASS = AntEnv
    ORI_IND = 6
