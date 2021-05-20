import parl
from gym import Env
import numpy as np
from parl.utils import action_mapping, ReplayMemory, logger
from yaml import load, Loader
import os
from time import time

from Model import Model
from Algorithm import DDPG
from Agent import Agent

param_path = os.path.join(os.path.dirname(__file__), "training_parameters.yaml")
param_dict = load(open(param_path, "r", encoding="utf-8"), Loader=Loader)

print(param_dict)

def run_episode(env : Env, agent : parl.Agent, rpm : ReplayMemory, return_time : bool = False):
    if return_time:
        start_tp = time()
        total_sample_time = 0.
        total_learn_time = 0.

    total_reward, steps = 0., 0
    obs = env.reset()
    while True:
        steps += 1
        ls_tp = time()
        if np.random.random() < param_dict["EPSILON"]:
            action = np.random.uniform(-1., 1., size=(2,))
        else:
            batch_obs = np.expand_dims(obs, axis=0)
            action = agent.predict(batch_obs.astype("float32"))
            action = np.squeeze(action)
            # add guassion noise, clip, map to corresponding interval
            action = np.clip(np.random.normal(action, 1.0), -1., 1.)
        if return_time:
            total_sample_time += time() - ls_tp

        action = action_mapping(action, env.action_space.low[0], env.action_space.high[0]) 

        next_obs, reward, done, info = env.step(action)
        # with open("./log/sample.log", "a+", encoding="utf-8") as f:
        #     f.write(str(action) + "|" + str(next_obs))

        rpm.append(obs, action, param_dict["REWARD_SCALE"] * reward, next_obs, done)
        
        # do warm up until rpm size reach MEMORY_WARMUP_SIZE
        if rpm.size() > param_dict["MEMORY_WARMUP_SIZE"]:
            batch_obs, batch_action, batch_reward, batch_next_obs, \
                batch_terminal = rpm.sample_batch(param_dict["BATCH_SIZE"])
            ls_tp = time()
            critic_cost = agent.learn(batch_obs, batch_action, batch_reward,
                                      batch_next_obs, batch_terminal)
            if return_time:
                total_learn_time += time() - ls_tp
    
            obs = next_obs
            total_reward += reward
        
        if done:
            break
    
    if return_time:
        run_time = time() - start_tp
        time_info = {
            "run time" : run_time,
            "total sample time" : total_sample_time,
            "total learn time" : total_learn_time
        }
        return total_reward, steps, time_info
    else:
        return total_reward, steps

def evaluate(env : Env, agent : parl.Agent):       
    obs = env.reset()
    total_reward, steps = 0., 0
    while True:
        steps += 1
        batch_obs = np.expand_dims(obs, axis=0)
        action = agent.predict(batch_obs)
        action = np.clip(action, -1., 1.)
        action = np.squeeze(action)
        action = action_mapping(action, env.action_space.low[0], env.action_space.high[0])

        next_obs, reward, done, info = env.step(action)
        obs = next_obs
        total_reward += reward
        
        if done:
            break

    return total_reward, info               
