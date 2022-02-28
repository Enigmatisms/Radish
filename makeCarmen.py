#-*-coding:utf-8-*-
"""
    input trajectory and carmen relation file, output carmen log file
"""

import numpy as np
from pose import Pose

def buildLookUp(input_name:str) -> dict:
    with open(input_name, "r") as file:
        raw_inputs = file.readlines()
        all_input = [x[:-1].split(' ') for x in raw_inputs]
        result = dict()
        for item in all_input:
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

def evaluate(queries: list, lut: dict):
    valid_cnt = half_valid_cnt = invalid_cnt = 0
    l2_err_sum = 0.0
    x_err_sum = y_err_sum = t_err_sum = 0.0
    # 角度处理上需要考虑奇异性
    for sq, eq, gt_pose in queries:
        if sq in lut and eq in lut:
            s_pose:Pose = lut[sq]
            e_pose:Pose = lut[eq]
            delta_pose:Pose = s_pose.inverse() * e_pose
            delta:Pose = (delta_pose - gt_pose).cWiseAbs()
            l2_err_sum += delta.normSquared()
            x_err_sum += delta.x
            y_err_sum += delta.y
            t_err_sum += delta.theta
            valid_cnt += 1
        else:
            if sq in lut or eq in lut: half_valid_cnt += 1
            else: invalid_cnt += 1
    print("========== Final error ===========")
    print("Translation error: %f, %f"%(x_err_sum / valid_cnt, y_err_sum / valid_cnt))
    print("Rotation error: %f, L2 MSE: %f"%(t_err_sum / valid_cnt, np.sqrt(l2_err_sum / valid_cnt)))
    print("Valid query: %d, half valid query: %d, invalid: %d"%(valid_cnt, half_valid_cnt, invalid_cnt))

if __name__ == "__main__":
    lut = buildLookUp("./data/fr079.txt")
    query = getQueryData("./data/fr079.relations.txt")
    evaluate(query, lut)
