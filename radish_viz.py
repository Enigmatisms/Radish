#-*-coding:utf-8-*-
# sort the radish dataset and visualize

import matplotlib.pyplot as plt
from pose import Pose
import numpy as np

def readAndSort(input_name:str):
    with open(input_name, 'r') as file:
        raw_lines = file.readlines()
        split_line = [x[:-1].split(' ') for x in raw_lines]
        all_lines = sorted(split_line, key = lambda x: float(x[1]))
        all_lines = sorted(all_lines, key = lambda x: float(x[0]))
    return all_lines

def timeStampAlign(data:list):
    now_need = ''
    result = []
    for record in data:
        if not now_need:
            now_need = record[1]
            result.append(record)
        else:
            if record[0] != now_need:
                continue
            result.append(record)
            now_need = record[1]
    start_stamps = set()
    end_stamps = set()
    for record in data:
        start_stamps.add(record[0])
        end_stamps.add(record[1])
    print("There are %d start stamps, %d end stamps"%(len(start_stamps), len(end_stamps)))
    return result

if __name__ == "__main__":
    data = readAndSort("test.txt")
    organized = timeStampAlign(data)
    # for term in organized:
    #     print(term)
    all_pose = [np.zeros((2, 1))]
    acc_transform = Pose(0, 0, 0)
    for term in organized:
        local = Pose(float(term[2]), float(term[3]), float(term[-1]))
        acc_transform = acc_transform * local
        all_pose.append(acc_transform.t())
    result = np.concatenate(all_pose, axis = -1)
    plt.scatter(result[0, :], result[1, :], c = 'r', s = 7)
    plt.plot(result[0, :], result[1, :], c = 'k')
    plt.show()
    print("There are (%d) items in original file, and (%d) items after organizing."%(len(data), len(organized)))