from robot.test_robot import *

class Robot(object):
    def __init__(self, client : int = 0, basePos : list = [0., 0., 0.], baseOri : list = [0., 0., 0., 1.], useMaxCoor : bool = True,
                 wheel_velocity_max : float = 10., laser_length : float = 10., laser_num : int = 5):
        self.client = client
        self.wheel_velocity_max = wheel_velocity_max
        self.laser_length = laser_length
        self.laser_num = laser_num

        # 常数
        self.LEFT_WHEEL_JOINT_INDEX = 3
        self.RIGHT_WHEEL_JOINT_INDEX = 2
        self.MAX_FORCE = 10.
        self.TARGET_VELOCITY = 5.
        self.MULTIPLY = 2.0

        self.DEBUG_TEXT_COLOR = [0., 0., 0.]     # debug文本的颜色
        self.DEBUG_TEXT_SIZE = 1.2               # debug文本的大小
        self.MISS_COLOR = [0., 1., 0.]           # 没有命中的激光的颜色
        self.HIT_COLOR = [1., 0., 0.]            # 命中的激光的颜色
        self.RAY_DEBUG_LINE_WIDTH = 2.           # 激光的debug线的宽度

        # 机器人参数
        self.BASE_THICKNESS = 0.2                # 底盘厚度
        self.BASE_RADIUS = 0.5                   # 底盘半径
        self.WHEEL_THICKNESS = 0.1               # 轮子厚度
        self.WHEEL_RADIUS = 0.2                  # 轮子半径

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