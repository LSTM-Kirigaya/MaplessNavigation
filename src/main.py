# -*- encoding: utf-8 -*-
# author : Zhelong Huang
import sys
import os
path = os.path.dirname(__file__)
path = "/".join(path.split("\\")[:-1])
sys.path.append(path)

from env.maplessNaviEnv import MaplessNaviEnv
from time import sleep
import pybullet as p

env = MaplessNaviEnv(scene_name="plane_static_obstacle-A", render=True)

env.reset()
p.setRealTimeSimulation(1)
while True:
    action = env.sample()
    obs, reward, done, info = env.step(action)
    if done:
        break
