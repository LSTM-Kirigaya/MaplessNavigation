# -*- encoding: utf-8 -*-
# author : Zhelong Huang
import sys
import os
path = os.path.dirname(__file__)
path = "/".join(path.split("\\")[:-1])
sys.path.append(path)

from Train import *
from env.maplessNaviEnv import *

env = MaplessNaviEnv(scene_name='plane_static_obstacle-A', render=True, evaluate=True)
obs_dim = env.observation_space.shape[0]
act_dim = env.action_space.shape[0]

# create 3-level model
model = Model(act_dim=act_dim)
algorithm = DDPG(
    model=model,
    gamma=param_dict["GAMMA"],
    tau=param_dict["TAU"],
    actor_lr=param_dict["ACTOR_LR"],
    critic_lr=param_dict["CRITIC_LR"]
)
agent = Agent(
    algorithm=algorithm,
    obs_dim=obs_dim,
    act_dim=act_dim
)

# ./model/2021.2.25/s_72001_r_963976
# ./model/2021.2.26/s_592286_r_884427
# ./model/2021.2.26/s_615055_r_977890
# ./model/2021.2.26/s_759057_r_700960
# ./model/2021.2.26/s_778768_r_881551

agent.restore("./model/2021.3.19(plane_static_obstacle-B)/s_504007_r_-7769767")

reward, info = evaluate(env, agent)
print(reward)
print(info)
# p.setRealTimeSimulation(1)
# obs = env.reset()
# while True:
#     action = agent.predict(obs)[0]
#     obs, reward, done, info = env.step(action=action)
#     if done:
#         break
