from robot.test_robot import *

class Robot(object):
    def __init__(self, client : int = 0, basePos : list = [0., 0., 0.], baseOri : list = [0., 0., 0., 1.], useMaxCoor : bool = True,
                 wheel_velocity_max : float = 10., laser_length : float = 10., laser_num : int = 5):
        self.client = client
        

        urdf_path = os.path.join(os.path.dirname(__file__), "miniBox")
        
        self.robot = p.loadURDF(
            fileName=urdf_path,
            basePosition=basePos,
            baseOrientation=baseOri,
            useMaximalCoordinates=useMaxCoor,
            physicsClientId=client
        )

        self.clip = lambda x : np.clip(x, -wheel_velocity_max, wheel_velocity_max)

    def getBothId(self):
        return self.client, self.robot
    
    def apply_action(self, action):
        if not (isinstance(action, list) or isinstance(action, np.ndarray)):
            assert f"apply_action() only receive list or ndarray, but receive {type(action)}"
        left_v, right_v = action
        # 裁剪到范围内
        left_v = self.clip(left_v)
        right_v = self.clip(right_v)
        # 应用到马达
        p.setJointMotorControlArray(
            bodyUniqueId=self.robot,
            jointIndices=[self.LEFT_WHEEL_JOINT_INDEX, self.RIGHT_WHEEL_JOINT_INDEX]
        )