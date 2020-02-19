import argparse
import os
import glob
from PIL import Image
from tqdm import tqdm
import time

parser = argparse.ArgumentParser()
parser.add_argument(
    '--uncertainty_info', type=str, help='directory to find uncertainty info')
parser.add_argument(
    '--condition_threshold', type=float, help='threshold for avg condition #')
parser.add_argument(
    '--scale_threshold', type=float, help='threshold for scale')
opt = parser.parse_args()
print(opt)

stats_files = glob.glob(os.path.join(opt.uncertainty_info, 'stats_*.txt'))

for name in tqdm(stats_files):
    timestamp = name[name.find('stats_') + len('stats_'):name.find('.txt')]
    print("Timestamp: ", timestamp)
    with open(name, 'r') as f:
        condition = float(f.readline().strip())
        scale = float(f.readline().strip())

        if opt.condition_threshold and condition < opt.condition_threshold:
            continue
        if opt.scale_threshold and scale < opt.scale_threshold:
            continue

    print("Condition: ", condition)
    print("Scale: ", scale)
    img = Image.open(os.path.join(opt.uncertainty_info, 'cloud_' + timestamp + '.bmp'))
    img.show()
    time.sleep(3)
    img.close()
