#!/usr/bin/env python3
#-*-coding:utf-8-*-
"""
    Trajectory evaluator for pure LiDAR-based chain SLAM, cartographer and GMapping
"""

from tabnanny import verbose
from unittest import result
import numpy as np
from numpy.linalg import svd, det
import matplotlib.pyplot as plt
import os
from sys import argv

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
            x_error = []
            y_error = []
            theta_error = []
            error_2d = []
            error_3d = []
            for i, time_stamp in enumerate(query[:, 0]):
                idx, _ = TrajEval.binarySearch(time_stamp, base[:, 0])
                query_pt = query[i, 1:]
                value_pt = base[idx, 1:]
                diff = query_pt - value_pt
                if diff[2] > np.pi:
                    diff[2] -= 2 * np.pi
                if diff[2] < -np.pi:
                    diff[2] += 2 * np.pi
                x_error.append(diff[0])
                y_error.append(diff[1])
                theta_error.append(diff[2])
                error_2d.append(np.linalg.norm(diff[:2]))
                error_3d.append(np.linalg.norm(diff))
            return x_error, y_error, theta_error, error_2d, error_3d
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

    @staticmethod
    def mean_std(x_error, y_error, theta_error, error_2d, error_3d, make_abs = False, verbose = True):
        transform = np.abs if make_abs else np.array
        x_error = transform(x_error)
        y_error = transform(y_error)
        theta_error = list(map(abs, theta_error))
        x_mean, x_std = np.mean(x_error), np.std(x_error)
        y_mean, y_std = np.mean(y_error), np.std(y_error)
        t_mean, t_std = np.mean(theta_error), np.std(theta_error)
        err_mean_2d, err_std_2d = np.mean(error_2d), np.std(error_2d)
        err_mean_3d, err_std_3d = np.mean(error_3d), np.std(error_3d)
        complement = " (absolute)" if make_abs else ""
        if verbose:
            print("Error X:\t%.6lf±%.6f%s"%(x_mean, x_std, complement))
            print("Error Y:\t%.6lf±%.6f%s"%(y_mean, y_std, complement))
            print("Error theta:\t%.6lf±%.6f%s"%(t_mean, t_std, complement))
            print("Error 2D:\t%.6lf±%.6f%s"%(err_mean_2d, err_std_2d, complement))
            print("Error 3D:\t%.6lf±%.6f%s"%(err_mean_3d, err_std_3d, complement))
        return (x_mean, x_std), (y_mean, y_std), (t_mean, t_std), (err_mean_2d, err_std_2d), (err_mean_3d, err_std_3d)

    @staticmethod
    def traverseFolder(folder:str, start_with:str, save_fig:bool = False, make_abs:bool = False, verbose:bool = False, use_flag = False):
        all_stuff = os.listdir(folder)
        gt_file_path = folder + "c_gt.tjc"
        if not os.path.exists(gt_file_path):
            raise RuntimeError("No ground truth file named 'c_gt.tjc' found in folder '%s', which is demanded by evaluator."%(folder))
        gt_data = TrajEval.readFromFile(gt_file_path)
        x_means = []
        y_means = []
        t_means = []
        traj_2d_means = []
        traj_3d_means = []
        i = 0
        for stuff in all_stuff:
            stuff_path = folder + stuff
            if not os.path.isfile(stuff_path):
                continue
            if not stuff.startswith(start_with):
                continue
            if start_with == "gmap_" and stuff == "gmap_gt.txt":
                continue
            traj_data = TrajEval.readFromFile(stuff_path, use_flag)
            x_error, y_error, theta_error, error_2d, error_3d = TrajEval.temperalNearestNeighborEvaluator(traj_data, gt_data)
            # 提供一个可以自动保存部分截图的方法
            if save_fig:
                fig_folder = folder + "eval_figs/"
                if os.path.exists(fig_folder) == False:
                    os.mkdir(fig_folder)
                # TrajEval.visualizePerDimError(x_error, y_error, theta_error)
                # plt.savefig(fig_folder + start_with + "%d_err_dim.jpg"%(i))
                # plt.clf()
                # TrajEval.visualizeTrajError(error_2d, error_3d)
                # plt.savefig(fig_folder + start_with + "%d_err_traj.jpg"%(i))
                # plt.clf()
                TrajEval.visualizeWithGT(folder, stuff[:-4], use_flag)
                plt.savefig(fig_folder + start_with + "%d_gt_traj.png"%(i))
                plt.clf()
                plt.cla()
            if verbose:
                print("============== %s%d error output %s ================"%(start_with, i, stuff))
            (x_mean, _), (y_mean, _), (t_mean, _), (err_mean_2d, _), (err_mean_3d, _) = \
            TrajEval.mean_std(x_error, y_error, theta_error, error_2d, error_3d, make_abs, verbose)
            x_means.append(x_mean)
            y_means.append(y_mean)
            t_means.append(t_mean)
            traj_2d_means.append(err_mean_2d)
            traj_3d_means.append(err_mean_3d)
            i += 1
        print("\n============== Total error output ================")
        TrajEval.mean_std(x_means, y_means, t_means, traj_2d_means, traj_3d_means, make_abs, True)

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
    def visualizeDifferentMethods(folder:str, traj_id:int):
        gt_file_path = folder + "c_gt.tjc"
        gt_data = TrajEval.readFromFile(gt_file_path)
        carto_path = folder + "carto_%d.tjc"%(traj_id)
        carto_data = TrajEval.readFromFile(carto_path)
        carto_data = TrajEval.icpPostProcess(carto_data, gt_data)

        # gmap_path = folder + "gmap_%d.tjc"%(traj_id)
        # gmap_data = TrajEval.readFromFile(gmap_path)

        c_traj_path = folder + "c_traj_%d.tjc"%(traj_id)
        cslam_data = TrajEval.readFromFile(c_traj_path)
        cslam_data = TrajEval.icpPostProcess(cslam_data, gt_data)

        TrajEval.visualizeOneTrajectory(gt_data, 2, label = 'Ground truth')

        TrajEval.visualizeOneTrajectory(carto_data, 7, label = 'cartographer trajectory')
        x_error, y_error, theta_error, error_2d, error_3d = TrajEval.temperalNearestNeighborEvaluator(carto_data, gt_data)
        TrajEval.mean_std(x_error, y_error, theta_error, error_2d, error_3d, False, True)
        # TrajEval.visualizeOneTrajectory(gmap_data, 7, label = 'GMapping trajectory')
        TrajEval.visualizeOneTrajectory(cslam_data, 7, label = 'chain SLAM trajectory')
        print("=====================================")
        x_error, y_error, theta_error, error_2d, error_3d = TrajEval.temperalNearestNeighborEvaluator(cslam_data, gt_data)
        TrajEval.mean_std(x_error, y_error, theta_error, error_2d, error_3d, False, True)
        plt.legend()
        plt.grid(axis = 'both')
        plt.show()

def binarySearchTest():
    test_list = [1, 5, 7, 10, 13.5, 16.2, 20, 26, 30, 31, 32, 36]
    queries = [4, 5.9, 6.0, 6.2, 8.5, 8.499, 8.9, -1, 60, 31.3, 20, 26, 26.1, 0.9999, 1.01, 1, 36.000000000001]
    for q in queries:
        idx, result = TrajEval.binarySearch(q, test_list)
        print("Query: %f, result: %d, %f"%(q, idx, result))

def automatic_runner():
    TrajEval.traverseFolder(argv[1], argv[2], verbose = True, save_fig = True, use_flag = False)

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
    plt.plot(gt_data[:, 1], gt_data[:, 2], label = 'gt')
    plt.scatter(gt_data[:, 1], gt_data[:, 2])
    plt.plot(carto_data[:, 1], carto_data[:, 2], label = 'before')
    plt.scatter(carto_data[:, 1], carto_data[:, 2])
    plt.plot(align_carto[:, 1], align_carto[:, 2], label = 'after')
    plt.scatter(align_carto[:, 1], align_carto[:, 2])
    plt.grid(axis = 'both')
    plt.legend()
    plt.show()


if __name__ == "__main__":
    # TrajEval.traverseFolder(argv[1], argv[2], verbose = True, save_fig = True)
    # automatic_runner()
    # TrajEval.visualizeWithGT(argv[1], "carto_0")
    # TrajEval.readFromFile(argv[1])
    trajectory_compare()
    # visualizeICP()