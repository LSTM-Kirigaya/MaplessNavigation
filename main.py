from Train import *
from env.maplessNaviEnv import *
import datetime

import warnings
warnings.filterwarnings("ignore")
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '0' # 默认值，输出所有信息
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1' # 屏蔽通知信息
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2' # 屏蔽通知信息和警告信息
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' # 屏蔽通知信息、警告信息和报错信息

scene_name = "plane_static_obstacle-A"

# make sure to save all the model files to a dir named today's date
today = datetime.date.today()
today_str = f"{today.year}.{today.month}.{today.day}({scene_name})"
if not os.path.exists(f"./model/{today_str}"):
    os.makedirs(f"./model/{today_str}")       

env = MaplessNaviEnv(scene_name=scene_name, render=False)
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
save_path = ""
if os.path.exists(save_path):
    print(f"\033[33msuccessfully load existing model\033[0m: {save_path}")
    agent.restore(save_path=save_path)

test_flag, total_steps = 0, 0
while total_steps < param_dict["TRAIN_TOTAL_STEPS"]:
    train_reward, steps, time_info = run_episode(env, agent, rpm, return_time=True)
    total_steps += steps
    # because total_steps are not increased step by step, we need to set a threshold instead of use mod
    if total_steps // param_dict["TEST_EVERY_STEPS"] >= test_flag:
        test_flag += total_steps // param_dict["TEST_EVERY_STEPS"] - test_flag + 1
        evaluate_reward, info = evaluate(env, agent)

        log_str = "Steps: {} | Test reward: {} | distance: {} | collision : {}".format(
            total_steps, round(evaluate_reward, 2), round(info["distance"], 2), info["collision_num"]
        )
        logger.info(log_str)
        print(f"\033[34m{time_info}\033[0m")
        with open(f"./model/{today_str}/train.log", "a", encoding="utf-8") as f:
            f.write(log_str + "\n")
        
        # save model
        save_path = "./model/Rc_Rp_Rr"
        # save_path = "./model/{}/s_{}_r_{}".format(today_str, total_steps, int(evaluate_reward))
        agent.save(save_path=save_path)

f.close()
