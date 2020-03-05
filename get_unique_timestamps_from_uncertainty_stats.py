import argparse
import os
import select
import sys
import numpy as np
import time
from glob import glob
from tqdm import tqdm

parser = argparse.ArgumentParser()
parser.add_argument('--uncertainty_info', type=str, required=True, help="folder containing uncertainty info")
parser.add_argument('--output_file', type=str, default='labeled_timestamps.npy', help="file in which to store numpy array containing timestamps and labels");
parser.add_argument('--condition_threshold', type=int, default=10, help="threshold above which we consider scans locally ambiguous")
parser.add_argument('--scale_threshold', type=float, default=.25, help="threshold above which we consider scans locally ambiguous")
opt = parser.parse_args()
start_time = str(int(time.time()))
print(opt)

stats_file = os.path.join(opt.uncertainty_info, 'local_uncertainty_stats.txt')

timestamps = []
positive = 0
with open(stats_file) as f:
    for line in tqdm(f.readlines()):
        line = line.strip()
        timestamp, vals = line.split(': ')
        # timestamp = s[s.find('stats_') + len('stats_'):s.find('.txt')]
        condition, scale = vals.split(', ')
        condition = float(condition)
        scale = float(scale)

        if condition < opt.condition_threshold and scale < opt.scale_threshold:
            timestamps.append((timestamp, 1, condition, scale))
            positive += 1
        else:
            timestamps.append((timestamp, 0, condition, scale))

print("Positive Examples:", positive)

labeled_timestamps = np.array(timestamps)
np.save(opt.output_file, labeled_timestamps)
