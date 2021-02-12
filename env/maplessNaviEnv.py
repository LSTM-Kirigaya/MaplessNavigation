# -*- encoding: utf-8 -*-
# author : Zhelong Huang
import sys
import os
path = os.path.dirname(__file__)
path = "/".join(path.split("\\")[:-1])
sys.path.append(path)

import gym
from gym import spaces
from robot.utils import *

class maplessNaviEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self, wheel_velocity_max : float = 10.):
        # 读入各项参数
        param_path = os.path.join(os.path.dirname(__file__), "robot_parameters.yaml")
        param_dict = load(open(param_path, "r", encoding="utf-8"), Loader=Loader)
        for key, value in param_dict.items():
            setattr(self, key, value)
    
        # 动作空间: 左轮速度， 右轮速度
        self.action_space = spaces.Box(
            low=np.array([-wheel_velocity_max, -wheel_velocity_max]),
            high=np.array([wheel_velocity_max, wheel_velocity_max]),
            dtype=np.float32
        )
        # 状态空间: laser1, ..., 5,   distance, alpha
        self.observation_space = spaces.Box(
            low=np.array([0.] * self.LASER_NUM + [0., 0.]),
            high=np.array([self.LASER_LENGTH] * self.LASER_NUM + [self.MAX_DISTANCE, np.pi])
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

        