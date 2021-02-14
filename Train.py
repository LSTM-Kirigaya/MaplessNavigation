import parl
from gym import Env
import numpy as np
from parl.utils import action_mapping, ReplayMemory, logger
from yaml import load, Loader
import os

from Model import Model
from Algorithm import DDPG
from Agent import Agent

param_path = os.path.join(os.path.dirname(__file__), "training_parameters.yaml")
param_dict = load(open(param_path, "r", encoding="utf-8"), Loader=Loader)

print(param_dict)

def run_episode(env : Env, agent : parl.Agent, rpm : ReplayMemory):
    total_reward, steps = 0., 0
    obs = env.reset()
    while True:
        steps += 1
        batch_obs = np.expand_dims(obs, axis=0)
        action = agent.predict(batch_obs.astype("float32"))
        action = np.squeeze(action)

        # add guassion noise, clip, map to corresponding interval
        action = np.clip(np.random.normal(action, 1.0), -1.0, 1.0)
        action = action_mapping(action, env.action_space.low[0], env.action_space.high[0]) 

        next_obs, reward, done, info = env.step(action)
        rpm.append(obs, action, param_dict["REWARD_SCALE"] * reward, next_obs, done)
        
        # do warm up until rpm size reach MEMORY_WARMUP_SIZE
        if rpm.size() > param_dict["MEMORY_WARMUP_SIZE"]:
            batch_obs, batch_action, batch_reward, batch_next_obs, \
                batch_terminal = rpm.sample_batch(param_dict["BATCH_SIZE"])
            critic_cost = agent.learn(batch_obs, batch_action, batch_reward,
                                      batch_next_obs, batch_terminal)
            obs = next_obs
            total_reward += reward
        
        if done:
            break
    return total_reward, steps

def evaluate(env : Env, agent : parl.Agent):
    eval_reward = []
    for i in range(5):          # evaluate 5 episodes
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
        eval_reward.append(total_reward)
    return np.mean(eval_reward)
