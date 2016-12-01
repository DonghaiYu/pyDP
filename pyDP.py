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
depend on: numpy, math
"""
import math
import numpy as np


EUCLIDIAN_DISTANCE = 1
MANHATTAN_DISTANCE = 2


def distance(a, b, method):
    """Distance calculation between two points.
    Args:
        a: one of the two points.
        b: another point of the two points.
        method: distance calculation method.
    Returns:
        The distance between a and b based on method.
    :param a: point a
    :param b: point b
    :param method: distance calculation method
    :return: distance between a and b.
    """
    dist = float('inf')
    if len(a) != len(b):
        print("different size between A and B")
        return dist
    a_f = []
    b_f = []
    for i in range(0, len(a)):
        try:
            a_f.append(float(a[i]))
            b_f.append(float(b[i]))
        except TypeError as error:
            print("can't trans the points to float list")
            break
    if method == EUCLIDIAN_DISTANCE:
        ab_sum = 0
        for i in range(len(a_f)):
            ab_sum += pow((a_f[i] - b_f[i]), 2)
        dist = math.sqrt(ab_sum)
    elif method == MANHATTAN_DISTANCE:
        ab_sum = 0
        for i in range(len(a_f)):
            ab_sum += abs(a_f[i] - b_f[i])
        dist = ab_sum
    return dist


def distance_point2line(a, b, c):
    """Calculate the distance from a point to a line, when the line is decided by two points.
    the equation: |vector(ac)×vector(bc)|/vector(ab), a, b decide a line, c is the target point.
    :param a: the first line point
    :param b: the second line point
    :param c: the target point
    :return: the distance to line which is decided by point a and point b
    """
    if len(a) != len(b) or len(a) != len(c):
        print("there are different size between the points!")
        return
    a_f = np.array(a, dtype="float32")
    b_f = np.array(b, dtype="float32")
    c_f = np.array(c, dtype="float32")

    ab = a_f - b_f
    bc = b_f - c_f
    ac = a_f - c_f
    return sum((ac * bc)**2)**0.5 / sum(ab**2)**0.5


def trajectory_compression(raw_trajectory, threshold):
    """
    :param raw_trajectory: the trajectory data, exp: [[x1, y1], [x2, y2], ... , [xn, yn]]
    :param threshold: distance threshold
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
    find_far_point(raw_trajectory, 0, len(raw_trajectory)-1, threshold, key_arr)
    key_points = []
    for i in range(len(key_arr)):
        if key_arr[i] == 1:
            key_points.append(raw_trajectory[i])
    return key_points


def find_far_point(arr, i, j, threshold, flag_arr):
    """
    This function is used for finding the farthest points (to line (arr[i], arr[j])) in sub trajectory arr(i, j),
    it is recursive until there are no more points's distance is larger than threshold to the line. if a point is not
    ignored in the trajectory, it's index in flag_arr will be set as 1, otherwise 0.
    :param arr: trajectory data
    :param i: start index
    :param j: end index
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
        d = distance_point2line(arr[i], arr[j], arr[k])
        print(d)
        if d > max_distance:
            max_distance = d
            flag = k
    if max_distance > threshold:
        flag_arr[flag] = 1
        find_far_point(arr, i, flag, threshold, flag_arr)
        find_far_point(arr, flag, j, threshold, flag_arr)

arrs = [[116.382433, 39.879020],[116.382523, 39.879123],[116.382373, 39.878888],[116.382373, 39.878888],[116.382423, 39.878871]]
narrs = trajectory_compression(arrs, "0.0007", EUCLIDIAN_DISTANCE)
print(narrs)

