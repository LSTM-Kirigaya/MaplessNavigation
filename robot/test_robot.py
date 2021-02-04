# encoding: utf-8
# 展示和测试文件夹中的urdf文件
# 路径添加
import sys
import os
path = os.path.dirname(__file__)
path = "/".join(path.split("\\")[:-1])
sys.path.append(path)


import pybullet as p
import pybullet_data
from time import sleep

cid = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

r2d2_pos = [1, 1, 1]
robot_pos = [0, 0, 0.5]

plane_id = p.loadURDF("plane.urdf", useMaximalCoordinates=True)
r2d2_id = p.loadURDF("r2d2.urdf", basePosition=r2d2_pos, useMaximalCoordinates=True)
robot_id = p.loadURDF("./miniBox.urdf", basePosition=robot_pos, useMaximalCoordinates=True)

p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(1)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

# TODO: 1. 增加机器人键盘控制与reset按钮
# TODO： 2. 添加并测试摄像头、深度激光探测器

while True:
    pass

p.disconnect(cid)