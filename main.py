from Train import *
from env.maplessNaviEnv import *

env = MaplessNaviEnv(scene_name='plane_static_obstacle-A', render=False)
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

# replay memory vessel
rpm = ReplayMemory(int(param_dict["MEMORY_SIZE"]), obs_dim, act_dim)

# persistence training
save_path = "./model.ckpt"
if os.path.exists(save_path):
    agent.restore(save_path=save_path)

test_flag, total_steps = 0, 0
while total_steps < param_dict["TRAIN_TOTAL_STEPS"]:
    train_reward, steps = run_episode(env, agent, rpm)
    total_steps += steps

    # because total_steps are not increased step by step, we need to set a threshold instead of use mod
    if total_steps // param_dict["TEST_EVERY_STEPS"] >= test_flag:
        test_flag += total_steps // param_dict["TEST_EVERY_STEPS"] - test_flag + 1
        evaluate_reward = evaluate(env, agent)
        logger.info("Steps {}, Test reward: {}".format(
            total_steps, evaluate_reward
        ))
        
        # save model
        save_path = "./model/s_{}_r_{}".format(total_steps, int(evaluate_reward))
        agent.save(save_path=save_path)