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
parser.add_argument('--condition_threshold', type=int, default=15, help="threshold above which we consider scans locally ambiguous")
parser.add_argument('--scale_threshold', type=int, default=4, help="threshold above which we consider scans locally ambiguous")
opt = parser.parse_args()
start_time = str(int(time.time()))
print(opt)

stats_files = glob(os.path.join(opt.uncertainty_info, 'stats_*.txt'))

timestamps = []
for s in tqdm(stats_files):
    timestamp = s[s.find('stats_') + len('stats_'):s.find('.txt')]
    with open(s) as f:
        condition = float(f.readline().strip())
        scale = float(f.readline().strip())

        if condition < opt.condition_threshold and scale < opt.scale_threshold:
            timestamps.append((timestamp, 1))
        else:
            timestamps.append((timestamp, 0))

labeled_timestamps = np.array(timestamps)
np.save('labeled_timestamps.npy', labeled_timestamps)