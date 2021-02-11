# -*- encoding: utf-8 -*-
# author : Zhelong Huang
import sys
import os
path = os.path.dirname(__file__)
path = "/".join(path.split("\\")[:-1])
sys.path.append(path)

import gym
from gym import spaces
from robot.test_robot import *

class maplessNaviEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self, wheel_velocity_max : float = 10., laser_length : float = 10., laser_num : int = 5):
        # 动作空间: 左轮速度， 右轮速度
        self.action_space = spaces.Box(
            low=np.array([-wheel_velocity_max, -wheel_velocity_max]),
            high=np.array([wheel_velocity_max, wheel_velocity_max]),
            dtype=np.float32
        )
        # 状态空间: laser1, ..., 5,   distance, alpha
        self.observation_space = spaces.Box(
            low=np.array([0.] * laser_num + [0., 0.]),
            high=np.array([laser_length] * laser_num + [100., np.pi])
        )

        self.np_random, _ = gym.utils.seeding.np_random()
        self.client = p.connect(p.DIRECT)
        p.setTimeStep(1 / 30, self.client)
    
    def step(self, action):
        return super().step(action)
    
    def reset(self):
        return super().reset()
    
    def render(self, mode='human'):
        return super().render(mode=mode)
    
    def seed(self, seed=None):
        return super().seed(seed=seed)
    
    def close(self):
        return super().close()

        