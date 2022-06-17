#-*-coding:utf-8-*-
"""
    input trajectory and carmen relation file, output carmen log file
"""

import os
import sys
import numpy as np
from pose import Pose

def buildLookUp(input_name:str) -> dict:
    with open(input_name, "r") as file:
        raw_inputs = file.readlines()
        all_input = [x[:-1].split(' ') for x in raw_inputs]
        result = dict()
        for i, item in enumerate(all_input):
            key = int(item[0][:-6])
            result[key] = Pose(float(item[1]), float(item[2]), float(item[3]))
    return result

def getQueryData(input_name: str) -> dict:
    with open(input_name, "r") as file:
        raw_inputs = [x[:-1].split(' ') for x in file.readlines()]
        all_queries = []
        for item in raw_inputs:
            start_stamp = int(float(item[0]) * 1000)
            end_stamp = int(float(item[1]) * 1000)
            all_queries.append(
                ( start_stamp, end_stamp, Pose(float(item[2]), float(item[3]), float(item[-1])) )
            )
        return all_queries

def evaluate(queries: list, lut: dict, verbose: bool = False):
    half_valid_cnt = invalid_cnt = 0
    l1_trans_err_sum = []
    l2_err_sum = []

    l1_angular_err_sum = []
    l2_angular_err_sum = []
    # 角度处理上需要考虑奇性
    for sq, eq, gt_pose in queries:
        if sq in lut and eq in lut:
            s_pose:Pose = lut[sq]
            e_pose:Pose = lut[eq]
            delta_pose:Pose = s_pose.inverse() * e_pose
            delta:Pose = (delta_pose - gt_pose).cWiseAbs()
            l1_trans_err_sum.append(delta.x + delta.y)
            l2_err_sum.append(delta.normSquared())
            l1_angular_err_sum.append(delta.theta)
            l2_angular_err_sum.append((delta.theta ** 2.))
        else:
            if sq in lut or eq in lut: half_valid_cnt += 1
            else: invalid_cnt += 1

    l1_trans_err_sum = np.array(l1_trans_err_sum)
    l2_err_sum = np.array(l2_err_sum)
    l1_angular_err_sum = np.array(l1_angular_err_sum)
    l2_angular_err_sum = np.array(l2_angular_err_sum)

    if verbose == True:
        avg_l1_t_err = l1_trans_err_sum.mean()
        avg_l2_t_err = l2_err_sum.mean()
        avg_l1_r_err = l1_angular_err_sum.mean()
        avg_l2_r_err = l2_angular_err_sum.mean()
        sqrt_avg_l2_r_err = np.sqrt(avg_l2_r_err)
        print("========== Final error ===========")
        print("Translation error: %f, %f"%(avg_l1_t_err, avg_l2_t_err))
        print("Rotation error: %f, L2 MSE: %f, sqrt: %f"%(avg_l1_r_err, avg_l2_r_err, sqrt_avg_l2_r_err))
        print("Valid query: %d, half valid query: %d, invalid: %d"%(len(l2_err_sum), half_valid_cnt, invalid_cnt))
    return np.stack([l1_trans_err_sum, l2_err_sum, l1_angular_err_sum, l2_angular_err_sum], axis = 0)
    

if __name__ == "__main__":
    map_name = sys.argv[1]
    query = getQueryData("./data/%s.relations.txt"%(map_name))
    result = []
    for i in range(5):
        path = "./data/%s/c_traj_%d.tjc"%(map_name, i)
        if os.path.exists(path) == False:
            break
        lut = buildLookUp(path)
        result.append(evaluate(query, lut))
        print("trajectory %d is processed.\n"%(i))
    final_result = np.hstack(result)
    result_mean = final_result.mean(axis = 1)
    result_std = final_result.std(axis = 1)
    print("========== Final error for %s ==========="%(map_name))
    print("Translation error: %f±%f, %f±%f"%(result_mean[0], result_std[0], result_mean[1], result_std[1]))
    print("Rotation error: %f±%f, L2 MSE: %f±%f"%(result_mean[2], result_std[2], result_mean[3], result_std[3]))
