import time
import datetime
import os
import numpy as np

def get_training_gap_time(dir_path : str):
    consumer_time_each_episode = []
    files = [os.path.getctime(dir_path + f"/{file}") for file in os.listdir(dir_path)]
    files.sort()
    for i in range(0, len(files) - 1):
        consumer_time_each_episode.append(files[i + 1] - files[i])
    return consumer_time_each_episode

results = get_training_gap_time("./model/2021.3.16(plane_static_obstacle-B)")
print(np.mean(results) / 60)