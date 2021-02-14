# MaplessNavigation

## Introduction
Reinforcement learning algorithm for mapless navigation, including the simulator based on `PyBullet` and 3-level reinforcement learning model based on `Parl`.

---

## Framework

Framework of the project:

```
MaplessNavigation
|-> env
    |-> maplessNaviEnv.py
|-> robot
    |-> config
        |-> miniBox_parameters.yaml
        |-> scene_parameters.yaml
        |-> task.yaml
    |-> urdf
        |-> miniBox.urdf
    |-> miniBox.py
    |-> register.py
    |-> scene.py
    |-> utils.py
|-> Model.py
|-> Agent.py
|-> Algorithm.py
|-> Train.py
|-> evaluate.py
|-> main.py
```

- `env`: environment for RL train and test,  `MaplessNaviEnv` in `maplessNaviEnv.py` is a gym env, you can use it just like the commom gym such as CartPole. 

- `robot`: urdf and functions to load and operate robots. YAML in `config` define several attributes of the robot, scene and task, you can adjust them.

---

## Robot

Robot used in the project is `miniBox`, which is described in `.\robot\urdf\miniBox.urdf`.

`miniBox` is a differentially driven robot. Preview of the model:

<center>
    <img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);" 
    src="https://i.loli.net/2021/02/10/HjoJK87T52YMOxa.png" width = "80%" alt=""/>
    <br>
</center>

Functions regarding to operating robot and create environmental items are implemented in `.\robot\utils.py`. But you won't call them under normal circumstances.

---

## Scene

We currently provide three scenes in `scene.py`.

![](https://i.loli.net/2021/02/14/oPZv8iIJ2wjGQkn.png)

Each scene is a registered class, you can build you own scene. Basic framework of the class to build the scene class is as follows:

```python
@Registers.scenes.register("scene_name")
class SceneX(BaseScene):
    def __init__(self, physicsClientId : int = 0):
        super(SceneX, self).__init__(physicsClientId=physicsClientId)
        
    def construct(self):
        if self.is_built:       
            raise Exception(f"scene_name has been built!")
        self.is_built = True    

        """
            load items to build your own scene
        """
```

> `"scene_name"` is the registered name you provide, `SceneX` is the name of the class, you can define it casually as long as it doesn't repeat with other class name

You can use loading function `PyBullet` provides or the adding function in `utils.py` to load items in your scene.

---

## Condition to reset the simulator

When distance between miniBox and target point is less than `DONE_DISTANCE` m or total collision times of the episode reach `DONE_COLLISION` times, env will reset itself, where `DONE_DISTANCE` and `DONE_COLLISION` are defined in `task.yaml`.

> The logic will be adjusted in the future

