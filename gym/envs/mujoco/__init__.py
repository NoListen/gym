from gym.envs.mujoco.mujoco_env import MujocoEnv
# ^^^^^ so that user gets the correct error
# message if mujoco is not installed correctly
from gym.envs.mujoco.ant import AntEnv
from gym.envs.mujoco.half_cheetah import HalfCheetahEnv
from gym.envs.mujoco.hopper import HopperEnv
from gym.envs.mujoco.walker2d import Walker2dEnv
from gym.envs.mujoco.humanoid import HumanoidEnv
from gym.envs.mujoco.inverted_pendulum import InvertedPendulumEnv
from gym.envs.mujoco.inverted_double_pendulum import InvertedDoublePendulumEnv
from gym.envs.mujoco.reacher import ReacherEnv
from gym.envs.mujoco.swimmer import SwimmerEnv
from gym.envs.mujoco.humanoidstandup import HumanoidStandupEnv
from gym.envs.mujoco.pusher import PusherEnv
from gym.envs.mujoco.thrower import ThrowerEnv
from gym.envs.mujoco.striker import StrikerEnv
from gym.envs.mujoco.humanoid_y import HumanoidYEnv
from gym.envs.mujoco.humanoid_free import HumanoidFreeEnv
from gym.envs.mujoco.ant_free import AntFreeEnv
from gym.envs.mujoco.maze.ant_maze_env import AntMazeEnv
from gym.envs.mujoco.maze.swimmer_maze_env import SwimmerMazeEnv
from gym.envs.mujoco.gather.ant_gather_env import AntGatherEnv
from gym.envs.mujoco.gather.swimmer_gather_env import SwimmerGatherEnv
