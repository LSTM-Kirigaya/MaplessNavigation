import sys
import os
path = os.path.dirname(__file__)
path = "/".join(path.split("\\")[:-1])
sys.path.append(path)
from robot.utils import *
from robot.register import Registers

# 基类中载入了一些设置在scene_parameters.yaml中的通用参数
class BaseScene(object):
    def __init__(self, physicsClientId : int = 0):
        self._physics_client_id = physicsClientId
        param_path = os.path.join(os.path.dirname(__file__), "config\\scene_parameters.yaml")
        param_dict = load(open(param_path, "r", encoding="utf-8"), Loader=Loader)
        for key, value in param_dict.items():
            setattr(self, key, value)
        self.is_built = False               # 是否已经调用过construct函数了
        self.load_items = {}                # 所有载入的物件的id
        self.debug_items = {}               # 所有的debug组件
    
    def construct(self):
        """
            use loading function in pybullet to load and assign the entity into the scene
            the function must set necessary variable as the attributes of the class
        """
        pass

# 每一个派生类都是可以直接调用的场景类，不过我更希望用户通过scene_parameters.yaml来更改
# 当然，也可以仿照这几个预设的派生类来自定义场景，注意装饰器可以传入参数，代表环境的注册名，否则就以环境类的类名作为
@Registers.scenes.register("plane_static_obstacle-A")
class Scene1(BaseScene):
    def __init__(self, physicsClientId : int = 0):
        super(Scene1, self).__init__(physicsClientId=physicsClientId)
        
    def construct(self):
        if self.is_built:       # 该函数只能执行一次
            raise Exception(f"plane_static_obstacle-A has been built!")
        self.is_built = True    
        # 设置起点和终点

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.load_items["plane"] = p.loadURDF("plane.urdf", useMaximalCoordinates=self.USE_MAX_COOR, physicsClientId=self._physics_client_id)
        self.load_items["fence"] = addFence(
            center_pos=self.CENTER_POS,
            internal_length=self.INTERNAL_LENGTH,
            internal_width=self.INTERNAL_WIDTH,
            height=self.HEIGHT,
            thickness=self.THICKNESS,
            mass=self.FENCE_MASS,
            rgba=self.FENCE_COLOR,
            physicsClientId=self._physics_client_id
        )

        self.load_items["obstacle1"] = addBox(
            pos=self.CENTER_POS,
            halfExtents=[3., 1, 1.5 / 4. * self.HEIGHT],
            physicsClientId=self._physics_client_id
        )
        self.load_items["obstacle2"] = addBox(
            pos=[self.CENTER_POS[0] + (self.INTERNAL_WIDTH / 2. - 1.), self.CENTER_POS[1], self.CENTER_POS[2]],
            halfExtents=[1., 1., 1.5 / 4. * self.HEIGHT],
            physicsClientId=self._physics_client_id
        )
        self.load_items["obstacle3"] = addBox(
            pos=[self.CENTER_POS[0] - (self.INTERNAL_WIDTH / 2. - 1.), self.CENTER_POS[1], self.CENTER_POS[2]],
            halfExtents=[1., 1., 1.5 / 4. * self.HEIGHT],
            physicsClientId=self._physics_client_id
        )


@Registers.scenes.register("plane_static_obstacle-B")
class Scene2(BaseScene):
    def __init__(self, physicsClientId : int = 0):
        super(Scene2, self).__init__(physicsClientId=physicsClientId)

    def construct(self):
        if self.is_built:
            raise Exception(f"plane_static_obstacle-B has been built!")
        self.is_built = True

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.load_items["plane"] = p.loadURDF("plane.urdf", useMaximalCoordinates=self.USE_MAX_COOR, physicsClientId=self._physics_client_id)
        self.load_items["fence"] = addFence(
            center_pos=self.CENTER_POS,
            internal_length=self.INTERNAL_LENGTH,
            internal_width=self.INTERNAL_WIDTH,
            height=self.HEIGHT,
            thickness=self.THICKNESS,
            mass=self.FENCE_MASS,
            rgba=self.FENCE_COLOR,
            physicsClientId=self._physics_client_id
        )

        self.load_items["obstacle1"] = addBox(
            pos=[-3., -4., 0.],
            halfExtents=[7., 1., 1.5 / 4. * self.HEIGHT],
            physicsClientId=self._physics_client_id
        )

        self.load_items["obstacle2"] = addBox(
            pos=[3., 4., 0.],
            halfExtents=[7., 1., 1.5 / 4. * self.HEIGHT],
            physicsClientId=self._physics_client_id
        )


@Registers.scenes.register("plane_static_obstacle-C")
class Scene3(BaseScene):
    def __init__(self, physicsClientId : int = 0):
        super(Scene3, self).__init__(physicsClientId=physicsClientId)
    
    def construct(self):
        if self.is_built:
            raise Exception(f"plane_static_obstacle-C has been built!")
        self.is_built = True

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.load_items["plane"] = p.loadURDF("plane.urdf", useMaximalCoordinates=self.USE_MAX_COOR, physicsClientId=self._physics_client_id)
        self.load_items["fence"] = addFence(
            center_pos=self.CENTER_POS,
            internal_length=self.INTERNAL_LENGTH,
            internal_width=self.INTERNAL_WIDTH,
            height=self.HEIGHT,
            thickness=self.THICKNESS,
            mass=self.FENCE_MASS,
            rgba=self.FENCE_COLOR,
            physicsClientId=self._physics_client_id
        )
        self.load_items["obstacle1"] = addCylinder(
            pos=self.CENTER_POS,
            raidus=2.,
            length=3.,
            physicsClientId=self._physics_client_id
        )

        self.load_items["obstacle2"] = addSphere(
            pos=[-5., -4., 0.],
            radius=1.5,
            physicsClientId=self._physics_client_id
        )

        self.load_items["obstacle3"] = addSphere(
            pos=[-4., 5., 0.],
            radius=2., 
            physicsClientId=self._physics_client_id
        )

        self.load_items["obstacle4"] = addSphere(
            pos=[3., 6., 0.],
            radius=1.3,
            physicsClientId=self._physics_client_id
        )

        self.load_items["obstacle5"] = addSphere(
            pos=[4., -2., 0.],
            radius=1.2,
            physicsClientId=self._physics_client_id
        )

        self.load_items["obstacle6"] = addSphere(
            pos=[6., -6., 0.],
            radius=1.4,
            physicsClientId=self._physics_client_id
        )



class RegisterScenes(object):
    def __init__(self, physicsClientId : int = 0):
        self._physics_client_id = physicsClientId
        self.scenes_dict = Registers.scenes._dict
    
    def construct(self, scene_name : str):
        if not isinstance(scene_name, str):
            raise ValueError(f"construct only recevie str, but receive {type(scene_name)}")
        if scene_name not in self.scenes_dict:
            raise KeyError(f"{scene_name} is not a registered scene name, all the available scene are {list(self.scenes_dict.keys())}")
        self.scene = self.scenes_dict[scene_name](physicsClientId=self._physics_client_id)
        self.scene.construct()
        return self.scene

if __name__ == "__main__":
    cid = p.connect(p.GUI)
    register_scenes = RegisterScenes()
    scene = register_scenes.construct("plane_static_obstacle-B")
    
    btn_id = p.addUserDebugParameter("reset", 1, 0, 0)
    previous = p.readUserDebugParameter(btn_id)

    while True:
        if previous != p.readUserDebugParameter(btn_id):
            p.resetSimulation()
            scene = register_scenes.construct("plane_static_obstacle-A")
            print(scene)
            previous = p.readUserDebugParameter(btn_id)
    p.disconnect(cid)