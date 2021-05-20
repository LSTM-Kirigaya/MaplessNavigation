import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def read_train_log(train_log_path : list):
    data = {
        "steps" : [],
        "reward" : [],
        "distance" : [],
        "collision" : []
    }
    for model_dir in train_log_path:
        for index, line in enumerate(open(model_dir, "r", encoding="utf-8")):
            if index > 17:
                continue
            meta = line.strip().split("|")
            meta = list(map(lambda x : float(x.strip().split(":")[-1]), meta))
            data["steps"].append(index)
            data["reward"].append(meta[1] / 1000000)
            data["distance"].append(meta[2])
            data["collision"].append(meta[3])
    return pd.DataFrame(data)

def read_traing_log2(train_log_path1 : list, train_log_path2 : list):
    data = {
        "steps" : [],
        "reward" : [],
        "distance" : [],
        "collision" : [],
        "type" : []
    }

    distance, collision = [], []
    for model_dir in train_log_path1:
        for index, line in enumerate(open(model_dir, "r", encoding="utf-8")):
            if index > 17:
                continue
            meta = line.strip().split("|")
            meta = list(map(lambda x : float(x.strip().split(":")[-1]), meta))
            data["steps"].append(index)
            data["reward"].append(meta[1] / 1000000)
            data["distance"].append(meta[2])
            data["collision"].append(meta[3])
            data["type"].append("ours")

            distance.append(meta[2])
            collision.append(meta[3])
    print("Ours distance:", np.mean(distance))
    print("Ours collision:", np.mean(collision))

    distance, collision = [], []
    for model_dir in train_log_path2:
        for index, line in enumerate(open(model_dir, "r", encoding="utf-8")):
            if index > 17:
                continue
            meta = line.strip().split("|")
            meta = list(map(lambda x : float(x.strip().split(":")[-1]), meta))
            data["steps"].append(index)
            data["reward"].append(meta[1] / 1000000)
            data["distance"].append(meta[2])
            data["collision"].append(meta[3])
            data["type"].append("baselines")
            distance.append(meta[2])
            collision.append(meta[3])
    print("Ours distance:", np.mean(distance))
    print("Ours collision:", np.mean(collision))
    
    return pd.DataFrame(data)

files1 = [
    "./model/2021.3.8(plane_static_obstacle-B)/train.log",
    "./model/2021.3.9(plane_static_obstacle-B)/train.log",
    "./model/2021.3.10(plane_static_obstacle-B)/train.log",
    "./model/2021.3.11(plane_static_obstacle-B)/train.log",
    "./model/2021.3.12(plane_static_obstacle-B)/train.log"
]

files2 = [
    "./model/2021.3.13(plane_static_obstacle-B)/train.log",
    "./model/2021.3.14(plane_static_obstacle-B)/train.log",
    "./model/2021.3.15(plane_static_obstacle-B)/train.log",
    "./model/2021.3.16(plane_static_obstacle-B)/train.log",
    "./model/2021.3.17(plane_static_obstacle-B)/train.log",
    "./model/2021.3.18(plane_static_obstacle-B)/train.log",
    "./model/2021.3.19(plane_static_obstacle-B)/train.log"
]

pd_date1 = read_train_log(files1)
pd_date2 = read_train_log(files2)

pd_data = read_traing_log2(files1, files2)

font_style = {
    "fontsize" : 15
}

import seaborn as sns
plt.figure(dpi=100, figsize=[18, 6])
ax = plt.subplot(1,2,1)
sns.lineplot(data=pd_data, x="steps", y="reward", hue="type", ax=ax)
ax.set_xlabel(r"step(×$10^6$)", **font_style)
ax.set_ylabel(r"reward(×$10^6$)", **font_style)
plt.legend(**font_style) 

ax = plt.subplot(1,2,2)
sns.lineplot(data=pd_data, x="steps", y="distance", hue="type", ax=ax)
ax.set_xlabel(r"step(×$10^6$)", **font_style)
ax.set_ylabel(r"distance", **font_style)
plt.legend(**font_style) 

plt.show()