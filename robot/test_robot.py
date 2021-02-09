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

# 常量
UP = p.B3G_DOWN_ARROW
DOWN = p.B3G_UP_ARROW
LEFT = p.B3G_RIGHT_ARROW
RIGHT = p.B3G_LEFT_ARROW

R2D2_POS = [1., 1., 1.]
ROBOT_POS = [0., 0., 0.5]

LEFT_WHEEL_JOINT_INDEX = 3
RIGHT_WHEEL_JOINT_INDEX = 2
MAX_FORCE = 10.
TARGET_VELOCITY = 5.
MULTIPLY = 2.0


# 连接引擎
cid = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

# 添加资源路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 载入机器人
plane_id = p.loadURDF("plane.urdf", useMaximalCoordinates=True)
r2d2_id = p.loadURDF("r2d2.urdf", basePosition=R2D2_POS, useMaximalCoordinates=True)
robot_id = p.loadURDF("./robot/miniBox.urdf", basePosition=ROBOT_POS, useMaximalCoordinates=True)

R2D2_POS, R2D2_Orientation = p.getBasePositionAndOrientation(r2d2_id)
ROBOT_POS, ROBOT_Orientation = p.getBasePositionAndOrientation(robot_id)

"""
miniBox关节信息如下：
        joint index: 0,
        joint name:  b'base_to_front_follow_wheel_pillar',
        joint type:  0

        joint index: 1,
        joint name:  b'front_pillar_to_front_follow_wheel',
        joint type:  0

        joint index: 2,
        joint name:  b'joint_right_wheel',
        joint type:  0

        joint index: 3,
        joint name:  b'joint_left_wheel',
        joint type:  0
"""

# miniBox的两个随动关节设为禁用
p.setJointMotorControlArray(
    bodyUniqueId=robot_id,
    jointIndices=[0, 1],
    controlMode=p.VELOCITY_CONTROL,
    forces=[0., 0.]
)

p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(1)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

# 获取节点信息
def getJointInfo(robot_id : int):
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        print(f"""
            joint index: {info[0]},
            joint name:  {info[1]},
            joint type:  {info[2]}
        """)

def control_miniBox(key_dict : dict):
    if len(key_dict) == 0:
        p.setJointMotorControlArray(
                bodyUniqueId=robot_id,
                jointIndices=[LEFT_WHEEL_JOINT_INDEX, RIGHT_WHEEL_JOINT_INDEX],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[0, 0],
                forces=[0, 0]
        )
    # 先判断是否按下了上键和左键或者上键和右键，如果是，则让机器人拐弯左前，或者拐弯右前
    if UP in key_dict and LEFT in key_dict:
        p.setJointMotorControlArray(
                bodyUniqueId=robot_id,
                jointIndices=[LEFT_WHEEL_JOINT_INDEX, RIGHT_WHEEL_JOINT_INDEX],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[TARGET_VELOCITY / MULTIPLY, TARGET_VELOCITY],
                forces=[MAX_FORCE, MAX_FORCE]
        )
    elif UP in key_dict and RIGHT in key_dict:
        p.setJointMotorControlArray(
                bodyUniqueId=robot_id,
                jointIndices=[LEFT_WHEEL_JOINT_INDEX, RIGHT_WHEEL_JOINT_INDEX],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[TARGET_VELOCITY, TARGET_VELOCITY / MULTIPLY],
                forces=[MAX_FORCE, MAX_FORCE]
        )
    elif DOWN in key_dict and LEFT in key_dict:
        p.setJointMotorControlArray(
                bodyUniqueId=robot_id,
                jointIndices=[LEFT_WHEEL_JOINT_INDEX, RIGHT_WHEEL_JOINT_INDEX],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[-1. * TARGET_VELOCITY, -1. * TARGET_VELOCITY / MULTIPLY],
                forces=[MAX_FORCE, MAX_FORCE]
        )
    elif DOWN in key_dict and RIGHT in key_dict:
        p.setJointMotorControlArray(
                bodyUniqueId=robot_id,
                jointIndices=[LEFT_WHEEL_JOINT_INDEX, RIGHT_WHEEL_JOINT_INDEX],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[-1. * TARGET_VELOCITY / MULTIPLY, -1. * TARGET_VELOCITY],
                forces=[MAX_FORCE, MAX_FORCE]
        )
    # 前后左右
    elif UP in key_dict:
        p.setJointMotorControlArray(
                bodyUniqueId=robot_id,
                jointIndices=[LEFT_WHEEL_JOINT_INDEX, RIGHT_WHEEL_JOINT_INDEX],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[TARGET_VELOCITY, TARGET_VELOCITY],
                forces=[MAX_FORCE, MAX_FORCE]
        )
    elif DOWN in key_dict:
        p.setJointMotorControlArray(
                bodyUniqueId=robot_id,
                jointIndices=[LEFT_WHEEL_JOINT_INDEX, RIGHT_WHEEL_JOINT_INDEX],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[-1. * TARGET_VELOCITY, -1. * TARGET_VELOCITY],
                forces=[MAX_FORCE, MAX_FORCE]
        )
    elif LEFT in key_dict:
        p.setJointMotorControlArray(
                bodyUniqueId=robot_id,
                jointIndices=[LEFT_WHEEL_JOINT_INDEX, RIGHT_WHEEL_JOINT_INDEX],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[-1. * TARGET_VELOCITY, TARGET_VELOCITY],
                forces=[MAX_FORCE, MAX_FORCE]
        )
    elif RIGHT in key_dict:
        p.setJointMotorControlArray(
                bodyUniqueId=robot_id,
                jointIndices=[LEFT_WHEEL_JOINT_INDEX, RIGHT_WHEEL_JOINT_INDEX],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[TARGET_VELOCITY, -1. * TARGET_VELOCITY],
                forces=[MAX_FORCE, MAX_FORCE]
        )

# TODO： 2. 添加并测试摄像头、深度激光探测器

# 增加控件，主要是重置用的reset按钮和调节全局参数的三个slider
"""
slider控制的参数为：
        MAX_FORCE 差分驱动轮的最大马力                          (>0)
        TARGET_VELOCITY 差分驱动轮的最大速度                    (>0)
        MULTIPLY 转弯时，差分驱动轮的两个轮子的速度之比（大的比上小的） (>1)
"""
MAX_FORCE_param_id = p.addUserDebugParameter("MAX_FORCE", 0, 100, MAX_FORCE)
TARGET_VELOCITY_param_id = p.addUserDebugParameter("TARGET_VELOCITY", 0, 100, TARGET_VELOCITY)
MULTIPLY_param_id = p.addUserDebugParameter("MULTIPLY", 1, 10, MULTIPLY)

reset_btn_id = p.addUserDebugParameter("reset", 1, 0, 0)
previous_btn_value = p.readUserDebugParameter(reset_btn_id)

# 开始测试
# getJointInfo(robot_id)

while True:
    # 读取并执行键盘信息
    key_dict = p.getKeyboardEvents()
    control_miniBox(key_dict)
    # 读取并更新slider中的参数
    MAX_FORCE = p.readUserDebugParameter(MAX_FORCE_param_id)
    TARGET_VELOCITY = p.readUserDebugParameter(TARGET_VELOCITY_param_id)
    MULTIPLY = p.readUserDebugParameter(MULTIPLY_param_id)
    # 判断按钮是否被按下，若是，则重置机器人的位置
    if p.readUserDebugParameter(reset_btn_id) != previous_btn_value:
        p.resetBasePositionAndOrientation(robot_id, ROBOT_POS, ROBOT_Orientation)
        p.resetBasePositionAndOrientation(r2d2_id, R2D2_POS, R2D2_Orientation)
        previous_btn_value = p.readUserDebugParameter(reset_btn_id)

p.disconnect(cid)