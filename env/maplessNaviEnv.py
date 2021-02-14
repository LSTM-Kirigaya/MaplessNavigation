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
from robot.miniBox import *
from robot.scene import *

class MaplessNaviEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self, scene_name : str = "plane_static_obstacle-A", render : bool = False):
        self.scene_name = scene_name
        self._render = render
        # 读入各项参数
        for file in os.listdir("./robot/config"):
            param_path = os.path.join("./robot/config/", file)
            param_dict = load(open(param_path, "r", encoding="utf-8"), Loader=Loader)
            for key, value in param_dict.items():
                setattr(self, key, value)

        # 动作空间: 左轮速度， 右轮速度
        self.action_space = spaces.Box(
            low=np.array([-self.TARGET_VELOCITY, -self.TARGET_VELOCITY]),
            high=np.array([self.TARGET_VELOCITY, self.TARGET_VELOCITY]),
            dtype=np.float32
        )
        # 状态空间: laser1, ..., 5,   distance, alpha
        self.observation_space = spaces.Box(
            low=np.array([0.] * self.LASER_NUM + [0., 0.]),
            high=np.array([self.LASER_LENGTH + 1] * self.LASER_NUM + [self.MAX_DISTANCE, np.pi])
        )
        # 根据参数选择引擎的连接方式
        self._physics_client_id = p.connect(p.GUI if render else p.DIRECT)
        # 获取注册环境并reset
        self._register_scene = RegisterScenes()
        self.seed()
        self.reset()
    
    def __reward_func(self, state : list):
        if checkCollision(self.robot.robot, debug=False):
            self.collision_num += 1
            Re = self.COLLISION_REWARD
        else:
            Re = 0
        
        Rs = 0

        if state[-2] < self.TARGET_RADIUS:
            Rg = self.REACH_TARGET_REWARD
        else:
            Rg = self.DISTANCE_REWARD_COE * self.__distance(self.robot.curPos(), self.TARGET_POS) / self.__distance(self.DEPART_POS, self.TARGET_POS)

        return Re + Rs + Rg

    def __distance(self, v1, v2):
        return sqrt(sum([(x - y) * (x - y) for x, y in zip(v1, v2)]))
    
    def sample(self):
        return self.np_random.uniform(low=-TARGET_VELOCITY, high=TARGET_VELOCITY, size=(2,))
    
    def step(self, action):
        """
            first set, second step
            then calculate the reward
            return state, reward, done, info
        """
        self.robot.apply_action(action=action)
        p.stepSimulation(physicsClientId=self._physics_client_id)    
        state = self.robot.get_observation(self.TARGET_POS)
        reward = self.__reward_func(state)
        if state[-2] < self.DONE_DISTANCE:
            done = True
        elif self.collision_num > self.DONE_COLLISION:
            done = True
        else:
            done = False
        info = {"distance" : state[-2], "collision_num" : self.collision_num}
        return np.array(state), reward, done, info
    
    def reset(self):
        """
            what you need do here:
            - reset scene items
            - reload robot
        """
        p.resetSimulation(physicsClientId=self._physics_client_id)
        p.setGravity(0., 0., -9.8, physicsClientId=self._physics_client_id)
        p.setRealTimeSimulation(0)

        # print("\033[32m enter \033[0m")
        self.collision_num = 0
        self.robot = Robot(
            basePos=self.DEPART_POS,
            baseOri=p.getQuaternionFromEuler(self.DEPART_EULER),
            physicsClientId=self._physics_client_id
        )
        self.scene = self._register_scene.construct(scene_name=self.scene_name)
        state = self.robot.get_observation(targetPos=self.TARGET_POS)
        return np.array(state)
    
    def render(self, mode='human'):
        pass
    
    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
    
    def close(self):
        if self._physics_client_id >= 0:
            p.disconnect()
        self._physics_client_id = -1
        

if __name__ == "__main__":
    env = MaplessNaviEnv()
    # pprint([attr for attr in dir(env) if attr[:2] != "__"])

    from stable_baselines3.common.env_checker import check_env
    check_env(env)