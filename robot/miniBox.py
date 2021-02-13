import sys
import os
path = os.path.dirname(__file__)
path = "/".join(path.split("\\")[:-1])
sys.path.append(path)
from robot.utils import *
from functools import partial

class Robot(object):
    def __init__(self, basePos : list = [0., 0., 0.], baseOri : list = [0., 0., 0., 1.], physicsClientId : int = 0):
        self._physics_client_id = physicsClientId
        # 读入各项参数
        param_path = os.path.join(os.path.dirname(__file__), "config\\miniBox_parameters.yaml")
        param_dict = load(open(param_path, "r", encoding="utf-8"), Loader=Loader)
        for key, value in param_dict.items():
            setattr(self, key, value)

        urdf_path = os.path.join(os.path.dirname(__file__), "urdf\\miniBox.urdf")
        self.robot = p.loadURDF(
            fileName=urdf_path,
            basePosition=basePos,
            baseOrientation=baseOri,
            useMaximalCoordinates=self.USE_MAX_COOR,
            physicsClientId=physicsClientId
        )
        
        # 该偏函数用于将输入的速度进行合适的裁剪
        self.clipv = partial(np.clip, a_min=-self.TARGET_VELOCITY, a_max=self.TARGET_VELOCITY)

    def get_bothId(self):
        return self._physics_client_id, self.robot
    
    def apply_action(self, action):     # 施加动作
        if not (isinstance(action, list) or isinstance(action, np.ndarray)):
            assert f"apply_action() only receive list or ndarray, but receive {type(action)}"
        left_v, right_v = action

        left_v = self.clipv(left_v)
        right_v = self.clipv(right_v)

        p.setJointMotorControlArray(
            bodyUniqueId=self.robot,
            jointIndices=[self.LEFT_WHEEL_JOINT_INDEX, self.RIGHT_WHEEL_JOINT_INDEX],
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=[left_v, right_v],
            forces=[self.MAX_FORCE, self.MAX_FORCE]
        )

    def get_observation(self, targetPos):       # 根据目的地的坐标得到机器人目前的状态
        # obversation: laser1, ..., n, distance, alpha
        basePos, baseOri = p.getBasePositionAndOrientation(self.robot)
        _, _, results = rayTest(self.robot, self.LASER_LENGTH, self.LASER_NUM)
        lasers_info = [self.LASER_LENGTH if result[0] == -1 else self.__distance(basePos, result[3]) for index, result in enumerate(results)]
        distance = self.__distance(basePos, targetPos)
        angle = self.__angle(
            v1=self.__get_forward_vector(),
            v2=[y - x for x, y in zip(basePos, targetPos)]
        )
        return lasers_info + [distance, angle]
    
    def curPos(self):
        return p.getBasePositionAndOrientation(self.robot)[0]

    def __get_forward_vector(self):         # 获取机器人朝向的向量
        _, baseOri = p.getBasePositionAndOrientation(self.robot)
        matrix = p.getMatrixFromQuaternion(baseOri)
        return [matrix[0], matrix[3], matrix[6]]

    def __distance(self, v1, v2):
        return sqrt(sum([(x - y) * (x - y) for x, y in zip(v1, v2)]))

    def __angle(self, v1, v2):
        v1 = np.array(v1)
        v2 = np.array(v2)
        cosangle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        return np.arccos(cosangle)

if __name__ == "__main__":
    cid = p.connect(p.DIRECT)
    robot = Robot(basePos=[0., -9., 0.], baseOri=p.getQuaternionFromEuler([0., 0., np.pi / 2.]))
    print(robot.get_observation([0., 9., 0.]))
    p.disconnect(cid)