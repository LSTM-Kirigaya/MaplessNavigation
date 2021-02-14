# -*- encoding: utf-8 -*-
# author : Zhelong Huang
import sys
import os
path = os.path.dirname(__file__)
path = "/".join(path.split("\\")[:-1])
sys.path.append(path)

from env.maplessNaviEnv import MaplessNaviEnv
from stable_baselines3 import DDPG
from time import sleep
import pybullet as p

env = MaplessNaviEnv(scene_name="plane_static_obstacle-A", render=True)
agent = DDPG(
    policy="MlpPolicy",
    env=env,
    learning_rate=0.01,
    buffer_size=1000
)

agent.load("./model/test.pkl")

p.setRealTimeSimulation(1)
obs = env.reset()
while True:
    action, _ = agent.predict(obs)
    obs, reward, done, info = env.step(action=action)
    if done:
        break
