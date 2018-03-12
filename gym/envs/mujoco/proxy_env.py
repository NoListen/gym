import gym
from gym import utils


class ProxyEnv(gym.Env, utils.EzPickle):
    def __init__(self, wrapped_env):
        self._wrapped_env = wrapped_env
        utils.EzPickle.__init__(self)

    @property
    def action_space(self):
        return self._wrapped_env.action_space

    @property
    def observation_space(self):
        return self._wrapped_env.observation_space

    @property
    def metadata(self):
        return self._wrapped_env.metadata

    @property
    def wrapped_env(self):
        return self._wrapped_env

    def _reset(self, **kwargs):
        return self._wrapped_env._reset(**kwargs)

    def _step(self, action):
        return self._wrapped_env._step(action)

    def _render(self, *args, **kwargs):
        return self._wrapped_env._render(*args, **kwargs)

    # @property
    # def horizon(self):
    #     return self._wrapped_env.horizon

    def close(self):
        self._wrapped_env.close()