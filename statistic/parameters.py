import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

ours_actor = 13 * 32 + 32 * 64 + 64 * 128 + 128 * 32 + 32 * 2 + 13 * 2
ours_critic = 13 * 32 + 32 * 64 + 64 * 128 + 15 * 16 * 64 + 64 * 128 + 2 * 32 + 32 * 64 + 64 * 128 + 3 * 128 * 128 + 128 * 1

baseline_actor = 13 * 512 + 512 * 512 + 512 * 512 + 512 * 2
baseline_critic = 15 * 512 + 512 * 512 + 512 * 512 + 512 * 512 + 512 * 1

actor_p = [ours_actor, baseline_actor]
critic_p = [ours_critic, baseline_critic]
all_p = [actor_p[0] + critic_p[0], actor_p[1] + critic_p[1]]

ours_a_r = actor_p[0] / all_p[0]
ours_c_r = critic_p[0] / all_p[0]

base_a_r = actor_p[1] / all_p[1]
base_c_r = critic_p[1] / all_p[1]

f = lambda x : str(round(x, 2)) + "%"

plt.style.use("seaborn")

width = 0.3
index = np.array([1,2])
actor_index = index - width / 2.
critic_index = index + width / 2.
all_index = index + width * 1.5
plt.bar(actor_index, actor_p, width, alpha=0.7, label="#actor parameters")
plt.bar(critic_index, critic_p, width, alpha=0.7, label="#critic parameters")
plt.bar(all_index, all_p, width, alpha=0.7, label="#all parameters")

plt.text(actor_index[0] - width / 5, actor_p[0], f(ours_a_r), fontsize=12)
plt.text(actor_index[1] - width / 5, actor_p[1], f(base_a_r), fontsize=12)
plt.text(critic_index[0] - width / 5, critic_p[0], f(ours_c_r), fontsize=12)
plt.text(critic_index[1] - width / 5, critic_p[1], f(base_c_r), fontsize=12)

plt.ylabel("Values", fontsize=15)
plt.xticks(index + width / 2., ["ours", "baselines"], fontsize=15)
plt.legend(fontsize=15)
plt.show()