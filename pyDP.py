# -*- coding: utf8 -*-
# @Time    : 2016/11/30 13:34
# @Author  : Dylan
# @Site    : 
# @File    : pyDP.py
# @Software: PyCharm Community Edition


"""
This is an implementation of Douglas-Peucker Algorithm, which can be used for trajectory compression.
When there are enough points in a trajectory, this algorithm can find the key point in the trajectory.
This tool support high dimension points.
depend on: numpy
"""
import numpy as np


def distance_point2line(a, b, c, distance_index):
    """Calculate the distance from a point to a line, when the line is decided by two points.
    the equation: |vector(ac)×vector(bc)|/vector(ab), a, b decide a line, c is the target point.
    :param a: the first line point
    :param b: the second line point
    :param c: the target point
    :param distance_index: which fields in a point are considered when calculating distance
    :return: the distance to line which is decided by point a and point b
    """
    if len(a) != len(b) or len(a) != len(c):
        print("there are different size between the points!")
        return
    for i in distance_index:
        if i >= len(a):
            print("fields index out of range")
            return
    af = []
    bf = []
    cf = []
    for i in distance_index:
        af.append(a[i])
        bf.append(b[i])
        cf.append(c[i])
    a_f = np.array(af, dtype="float32")
    b_f = np.array(bf, dtype="float32")
    c_f = np.array(cf, dtype="float32")

    ab = a_f - b_f
    bc = b_f - c_f
    ac = a_f - c_f
    return sum((ac - sum(ac * ab) / sum(ab**2) * ab)**2)**0.5


def trajectory_compression(raw_trajectory, threshold, distance_index):
    """
    :param raw_trajectory: the trajectory data, exp: [[x1, y1], [x2, y2], ... , [xn, yn]]
    :param threshold: distance threshold
    :param distance_index: which fields in a point are considered when calculating distance
    :return: compressed trajectory
    """
    if len(raw_trajectory) < 2:
        print("less than 2 points in your trajectory!")
        return
    try:
        threshold = float(threshold)
    except:
        print("can`t trans the threshold(type%s) to float" % type(threshold))
        return
    key_arr = [0 for i in range(len(raw_trajectory))]
    key_arr[0] = 1
    key_arr[len(raw_trajectory)-1] = 1
    find_far_point(raw_trajectory, 0, len(raw_trajectory)-1, distance_index, threshold, key_arr)
    key_points = []
    for i in range(len(key_arr)):
        if key_arr[i] == 1:
            key_points.append(raw_trajectory[i])
    return key_points


def find_far_point(arr, i, j, distance_index, threshold, flag_arr):
    """
    This function is used for finding the farthest points (to line (arr[i], arr[j])) in sub trajectory arr(i, j),
    it is recursive until there are no more points's distance is larger than threshold to the line. if a point is not
    ignored in the trajectory, it's index in flag_arr will be set as 1, otherwise 0.
    :param arr: trajectory data
    :param i: start index
    :param j: end index
    :param distance_index: which fields in a point are considered when calculating distance
    :param threshold: distance threshold
    :param flag_arr: the flag array of useful or ignore for the trajectory points(0:ignore, 1:useful)
    :return: none, but change the flag_arr
    """
    if i > len(arr) or j > len(arr):
        print("index out of range")
        return
    max_distance = float(0)
    flag = -1
    for k in range(i + 1, j):
        d = distance_point2line(arr[i], arr[j], arr[k], distance_index)
        if d > max_distance:
            max_distance = d
            flag = k
    if max_distance > threshold:
        flag_arr[flag] = 1
        find_far_point(arr, i, flag, distance_index, threshold, flag_arr)
        find_far_point(arr, flag, j, distance_index, threshold, flag_arr)

'''
arrs = [[1, 1, 1, 'x'], [2, 2, 2, 'y'], [3, 3, 3, 'z']]
narrs = trajectory_compression(arrs, "0.0007", [0, 1, 2])
print(narrs)
'''
# print(distance_point2line([1, 0], [2.6, 1], [3.5, 6]))

