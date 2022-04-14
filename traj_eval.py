#!/usr/bin/env python3
#-*-coding:utf-8-*-
"""
    Trajectory evaluator for pure LiDAR-based chain SLAM, cartographer and GMapping
"""

import os
import csv
import argparse
import numpy as np
import matplotlib.pyplot as plt

from sys import argv
from numpy.linalg import svd, det

all_method_names = ("cartographer", "chain SLAM", "GMapping", "bosch")
error_cats = ("traj abs", "traj sqr", "rot abs", "rot sqr")
data_cats = ("mean", "std")

# 这是我写过的最迷惑的列表生成式
__header__ = ["%s %s %s"%(n, error_cat, data_cat) for n in all_method_names for error_cat in error_cats for data_cat in data_cats]

# 输入的轨迹应有shape (轨迹点数，4)，第一维度是时间戳
class TrajEval:
    def __init__(self) -> None:
        pass

    @staticmethod
    def goodAngle(ang):
        if ang > np.pi:
            return ang - 2 * np.pi
        elif ang < -np.pi:
            return ang + 2 * np.pi
        return ang

    @staticmethod
    def readFromFile(path:str, use_flag:bool = False)->np.array:
        with open(path, 'r') as file:
            raw_all = file.readlines()
            result = np.zeros((len(raw_all), 5 if use_flag else 4))
            for i, line in enumerate(raw_all):
                stripped = line.split(' ')
                result[i, 0] = float(stripped[0]) / 1e9
                for j in range(1, 3):
                    result[i, j] = float(stripped[j])
                if stripped[3][-1] == '\n':
                    angle = float(stripped[3][:-1])
                else:
                    angle = float(stripped[3])
                while angle > np.pi:
                    angle -= np.pi * 2
                while angle < -np.pi:
                    angle += np.pi * 2
                result[i, 3] = angle
                if use_flag:
                    result[i, -1] = int(stripped[-1][0])
            if use_flag:
                return result[result[:, -1] > 0.5, :-1]
            return result

    @staticmethod
    def icpPostProcess(data:np.array, gt:np.array):
        query_c, pair_c = np.zeros(2, dtype = float), np.zeros(2, dtype = float)
        qpt = np.zeros((2, 2), dtype = float)
        query_start_p, base_start_p = data[0, 1:-1], gt[0, 1:-1]
        for i, time_stamp in enumerate(data[:, 0]):
            idx, _ = TrajEval.binarySearch(time_stamp, gt[:, 0])
            query_pt = data[i, 1:-1]
            value_pt = gt[idx, 1:-1]
            query_c += query_pt
            pair_c += value_pt
            qpt += (query_pt - query_start_p).reshape(-1, 1) @ (value_pt - base_start_p).reshape(1, -1)
        query_c /= data.shape[0]
        pair_c /= data.shape[0]
        qpt -= data.shape[0] * (query_c - query_start_p).reshape(-1, 1) @ (pair_c - base_start_p).reshape(1, -1)
        trans = np.zeros((3, 3), dtype = float)
        u, s, vh = svd(qpt)
        r = vh.T @ u.T
        rr = vh.T @ np.array([[1., 0.], [0., det(r)]]) @ u.T
        translation = (pair_c.reshape(-1, 1) - rr @ query_c.reshape(-1, 1))[:2].ravel()
        delta_angle = np.arctan2(rr[1, 0], rr[0, 0])
        result = np.zeros_like(data)
        result[:, 0] = data[:, 0] 
        for i in range(data.shape[0]):
            result[i, 1:-1] = data[i, 1:-1].reshape(1, -1) @ rr.T + translation
            result[i, -1] = TrajEval.goodAngle(data[i, -1] + delta_angle)
        return result

    @staticmethod
    def binarySearch(val, target_list):
        s = 0
        e = len(target_list) - 1
        while e > s + 1:
            m = (e + s) >> 1
            v_mid = target_list[m]
            if val < v_mid:
                e = m
            elif val > v_mid:
                s = m
            else:
                return m, target_list[m]
        if abs(target_list[s] - val) < abs(target_list[e] - val):
            return s, target_list[s]
        else:
            return e, target_list[e]

    # 如果只是简单的2D最邻近点，是不行的，轨迹可能交叉，此时的2D最邻近点可能选错，总是会选择更好的那个点，故最靠谱的只能是时间戳上的最邻近
    # 尽量使得不同算法的时间戳是对应的, 输入是 (point_num, 4)
    # 需要计算轨迹误差，隔周
    @staticmethod
    def temperalNearestNeighborEvaluator(src:np.array, dst:np.array):
        # 点数更多的作为被查找对象（GMapping可能就是稀疏轨迹）
        def compare(base:np.array, query:np.array):
            abs_traj_2d = []
            sqr_traj_2d = []
            abs_rot_2d = []
            sqr_rot_2d = []
            for i, time_stamp in enumerate(query[:, 0]):
                idx, _ = TrajEval.binarySearch(time_stamp, base[:, 0])
                query_pt = query[i, 1:]
                value_pt = base[idx, 1:]
                diff = query_pt - value_pt
                if diff[2] > np.pi:
                    diff[2] -= 2 * np.pi
                if diff[2] < -np.pi:
                    diff[2] += 2 * np.pi
                diff_dist = np.linalg.norm(diff[:2])
                abs_traj_2d.append(diff_dist)
                sqr_traj_2d.append(diff_dist ** 2)
                abs_rot_2d.append(abs(diff[2]))
                sqr_rot_2d.append(diff[2] ** 2)
            return abs_traj_2d, sqr_traj_2d, abs_rot_2d, sqr_rot_2d
        if len(dst) >= len(src):
            return compare(dst, src)
        else:
            return compare(src, dst)

        # 简单可视化操作
    @staticmethod
    def visualizeOneTrajectory(traj:np.array, dot_size = 0, color = None, label = None, linewidth = None, dot_color = None):
        plt.plot(traj[:, 1], traj[:, 2], c = color, label = label, linewidth = linewidth)
        if dot_size > 0:
            plt.scatter(traj[:, 1], traj[:, 2], c = dot_color, s = dot_size)

    @staticmethod
    def visualizePerDimError(x_error:list, y_error:list, theta_error:list, make_abs = False):
        plt.subplot(1, 3, 1)
        xs = np.arange(len(x_error))
        transform = np.abs if make_abs else np.array
        plt.subplot(1, 3, 1)
        plt.plot(xs, transform(x_error), label = 'absolute error x-axis')
        plt.subplot(1, 3, 2)
        plt.plot(xs, transform(y_error), label = 'absolute error y-axis')
        plt.subplot(1, 3, 3)
        plt.plot(xs, transform(theta_error), label = 'absolute error theta-axis')

    @staticmethod
    def visualizeTrajError(error2d:list, error3d:list):
        plt.subplot(1, 2, 1)
        xs = np.arange(len(error2d))
        plt.plot(xs, error2d, label = 'absolute error x-axis')
        plt.subplot(1, 2, 2)
        plt.plot(xs, error3d, label = 'absolute error y-axis')

    # 只比较L1 2D轨迹以及旋转误差 / L2 2D轨迹与旋转误差
    @staticmethod
    def mean_std(abs_traj_2d, sqr_traj_2d, abs_rot_2d, sqr_rot_2d, verbose = True):
        abs_traj_2d_mean, abs_traj_2d_std = np.mean(abs_traj_2d), np.std(abs_traj_2d)
        sqr_traj_2d_mean, sqr_traj_2d_std = np.mean(sqr_traj_2d), np.std(sqr_traj_2d)
        abs_rot_2d_mean, abs_rot_2d_std = np.mean(abs_rot_2d), np.std(abs_rot_2d)
        sqr_rot_2d_mean, sqr_rot_2d_std = np.mean(sqr_rot_2d), np.std(sqr_rot_2d)
        if verbose:
            print("Absolute Trajectory Error:\t%.6lf±%.6f"%(abs_traj_2d_mean, abs_traj_2d_std))
            print("Squared Trajectory Error:\t%.6lf±%.6f"%(sqr_traj_2d_mean, sqr_traj_2d_std))
            print("Absolute Rotational Error:\t%.6lf±%.6f"%(abs_rot_2d_mean, abs_rot_2d_std))
            print("Squared Rotational Error:\t%.6lf±%.6f"%(sqr_rot_2d_mean, sqr_rot_2d_std))
        return [abs_traj_2d_mean, abs_traj_2d_std, sqr_traj_2d_mean, sqr_traj_2d_std, abs_rot_2d_mean, abs_rot_2d_std, sqr_rot_2d_mean, sqr_rot_2d_std]

    @staticmethod
    def visualizeWithGT(folder:str, traj_name:str, use_flag:bool = False):
        gt_file_path = folder + "c_gt.tjc"
        traj_path = folder + traj_name + ".tjc"
        gt_data = TrajEval.readFromFile(gt_file_path)
        traj_data = TrajEval.readFromFile(traj_path, use_flag)
        TrajEval.visualizeOneTrajectory(gt_data, 3, label = 'gt trajectory')
        TrajEval.visualizeOneTrajectory(traj_data, 3, label = '%s trajectory'%(traj_name))
        plt.legend()
        plt.grid(axis = 'both')
        # plt.show()

    @staticmethod
    def append2csv(csv_pos, all_info):
        with open(csv_pos, 'a', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(all_info)
        csv_file.close()

    @staticmethod
    def visualizeDifferentMethods(folder:str, traj_id:int, verbose:bool = True, save_fig:bool = True):
        gt_file_path = folder + "c_gt.tjc"
        gt_data = TrajEval.readFromFile(gt_file_path)
        if save_fig:
            plt.figure(dpi = 320)
            plt.rcParams["figure.figsize"] = [10.0, 6.0]
            plt.subplots_adjust(left=0.075, right=0.925, top=0.925, bottom=0.075)
        
        all_methods = ("carto_%d.tjc"%(traj_id), "c_traj_%d.tjc"%(traj_id), "gmap_%d.tjc"%(traj_id), "bosch_%d.tjc"%(traj_id))
        
        csv_path = folder + "eval_result.csv"
        if traj_id == 0:
            with open(csv_path, 'w') as tmp:
                writer = csv.writer(tmp)
                writer.writerow(__header__)
        all_results = []
        for method, name in zip(all_methods, all_method_names):
            path = folder + method
            if not os.path.exists(path):
                all_results.extend([0. for _ in range(8)])
                continue
            data = TrajEval.readFromFile(path)
            data = TrajEval.icpPostProcess(data, gt_data)
            TrajEval.visualizeOneTrajectory(data, 2, label = '%s trajectory'%(name), linewidth=1)
            abs_traj_2d, sqr_traj_2d, abs_rot_2d, sqr_rot_2d = TrajEval.temperalNearestNeighborEvaluator(data, gt_data)
            if verbose:
                print("=============== %s evaluation results ==================="%(name))
            
            all_info = TrajEval.mean_std(abs_traj_2d, sqr_traj_2d, abs_rot_2d, sqr_rot_2d, verbose = verbose)
            all_results.extend(all_info)
        TrajEval.visualizeOneTrajectory(gt_data, 2, label = 'Ground truth', linewidth=1)
        TrajEval.append2csv(folder + "eval_result.csv", all_results)
        plt.legend()
        plt.grid(axis = 'both')
        if save_fig == True:
            fig_folder_path = folder + "eval_figs/"
            if not os.path.exists(fig_folder_path):
                os.mkdir(fig_folder_path)
            plt.savefig(fig_folder_path + "eval_result_%d.png"%(traj_id))
        else:
            plt.show()

def binarySearchTest():
    test_list = [1, 5, 7, 10, 13.5, 16.2, 20, 26, 30, 31, 32, 36]
    queries = [4, 5.9, 6.0, 6.2, 8.5, 8.499, 8.9, -1, 60, 31.3, 20, 26, 26.1, 0.9999, 1.01, 1, 36.000000000001]
    for q in queries:
        idx, result = TrajEval.binarySearch(q, test_list)
        print("Query: %f, result: %d, %f"%(q, idx, result))

def trajectory_compare():
    TrajEval.visualizeDifferentMethods(argv[1], int(argv[2]))

def visualizeICP():
    folder = argv[1]
    traj_id = int(argv[2])
    gt_file_path = folder + "c_gt.tjc"
    gt_data = TrajEval.readFromFile(gt_file_path)
    carto_path = folder + "carto_%d.tjc"%(traj_id)
    carto_data = TrajEval.readFromFile(carto_path)
    align_carto = TrajEval.icpPostProcess(carto_data, gt_data)
    plt.figure(dpi = 320)
    plt.plot(gt_data[:, 1], gt_data[:, 2], label = 'gt')
    plt.scatter(gt_data[:, 1], gt_data[:, 2], s = 1)
    plt.plot(carto_data[:, 1], carto_data[:, 2], label = 'before')
    plt.scatter(carto_data[:, 1], carto_data[:, 2], s = 1)
    plt.plot(align_carto[:, 1], align_carto[:, 2], label = 'after')
    plt.scatter(align_carto[:, 1], align_carto[:, 2], s = 1)
    plt.grid(axis = 'both')
    plt.legend()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--save_fig", default = False, action = "store_true", help = "Save figures during evaluation")
    parser.add_argument("-v", "--verbose", default = False, action = "store_true", help = "Output evaluation info in the terminal")
    parser.add_argument("--path", type = str, help = "Folder which contains all the trajectories")
    parser.add_argument("--traj_num", type = int, default = 0, help = "Trajectory file id")
    args = parser.parse_args()
    TrajEval.visualizeDifferentMethods(args.path, args.traj_num, args.verbose, args.save_fig)
