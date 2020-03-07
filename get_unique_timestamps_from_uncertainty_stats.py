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
parser.add_argument('--output_file', type=str, default='labeled_timestamps.npy', help="file in which to store numpy array containing timestamps and labels")
parser.add_argument('--low_condition_threshold', type=int, default=3, help="threshold below which we consider scans locally invariant")
parser.add_argument('--low_scale_threshold', type=float, default=.15, help="threshold below which we consider scans locally invariant")
parser.add_argument('--high_condition_threshold', type=int, default=12, help="threshold above which we consider scans locally ambiguous")
parser.add_argument('--high_scale_threshold', type=float, default=.3, help="threshold above which we consider scans locally ambiguous")
opt = parser.parse_args()
start_time = str(int(time.time()))
print(opt)

stats_file = os.path.join(opt.uncertainty_info, 'local_uncertainty_stats.txt')

positive_timestamps = []
negative_timestamps = []

with open(stats_file) as f:
    for line in tqdm(f.readlines()):
        line = line.strip()
        timestamp, vals = line.split(': ')
        condition, scale = vals.split(', ')
        condition = float(condition)
        scale = float(scale)

        if condition < opt.low_condition_threshold and scale < opt.low_scale_threshold:
            positive_timestamps.append((timestamp, 1, condition, scale))
        elif condition > opt.high_condition_threshold and scale > opt.high_scale_threshold:
            negative_timestamps.append((timestamp, 0, condition, scale))

print("Positive Examples:", len(positive_timestamps))
print("Negative Examples:", len(negative_timestamps))

# ensure same size
positive_timestamps = positive_timestamps[:len(negative_timestamps)]
negative_timestamps = negative_timestamps[:len(positive_timestamps)]

timestamps = np.concatenate((positive_timestamps, negative_timestamps))
print("Total Examples:", len(timestamps))

labeled_timestamps = np.array(timestamps)
np.save(opt.output_file, labeled_timestamps)
