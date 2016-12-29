# -*- coding: utf8 -*-
# @Time    : 2016/11/30 13:34
# @Author  : Dylan东海
# @Site    : 
# @File    : pyDP.py
# @Software: PyCharm Community Edition


"""
This is an implementation of Douglas-Peucker Algorithm, which can be used for trajectory compression.
When there are enough points in a trajectory, this algorithm can find the key point in the trajectory.
This tool support high dimension points.
depend on: numpy 1.11.2, python 3.5.2
"""
import numpy as np
import math

EARTH_RADIUS = 6378.137

'''
STANDARD = 0
MARS = 1
'''


def distance_point2line_standard(a, b, c, distance_index):
    """Calculate the distance from a point to a line on the plane, when the line is decided by two points.
    the equation: |vector(ac)-vector(ab)·(vector(ac)·vector(ab)/|vector(ab)|²)|,
    a, b decide a line, c is the target point.
    :param a: the first line point
    :param b: the second line point
    :param c: the target point
    :param distance_index: which fields in a point are considered when calculating distance
    :return: the distance to line which is decided by point a and point b
    """
    if len(a) != len(b) or len(a) != len(c):
        print("there are different sizes between the points!")
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
    a_f = np.array(af, dtype="float")
    b_f = np.array(bf, dtype="float")
    c_f = np.array(cf, dtype="float")

    ab = a_f - b_f
    bc = b_f - c_f
    ac = a_f - c_f
    return sum((ac - sum(ac * ab) / sum(ab**2) * ab)**2)**0.5


def distance_point2line_mars(a, b, c, distance_index):
    """Calculate the distance from a point to a line on the Mars(Gaode) coordinate system, when the line is decided by two points.
    There must be longitude and latitude in each point, and the result is the actual distance from c to the line <a, b>,
    the equation(Heron): (2 * sqrt(p(p-distance(a, b))(p-distance(b, c))(p-distance(a, c)))) / distance(a, b);
                        (p = (distance(a, b) + distance(b, c) + distance(a, c)) / 2)
    a, b decide a line, c is the target point.
    :attention: the result of this method may be smaller than the actual distance,
        but we hope it is not matter when the 3 points are in a small area(e.g the urban area)
    :param a: the first line point
    :param b: the second line point
    :param c: the target point
    :param distance_index: which fields in a point are considered when calculating distance,
            limit on 2(longitude, latitude)
    :return: the distance to line which is decided by point a and point b on earth surface.
    """
    if len(a) != len(b) or len(a) != len(c):
        print("there are different size between the points!")
        return
    if len(a) < 2:
        print("at least 2 dimensions are needed in your point")
        return
    if len(distance_index) != 2:
        print("the longitude and latitude index are needed")
        return
    lng_index = distance_index[0]
    lat_index = distance_index[1]
    ab = mars_distance(a[lat_index], a[lng_index], b[lat_index], b[lng_index])
    bc = mars_distance(b[lat_index], b[lng_index], c[lat_index], c[lng_index])
    ac = mars_distance(a[lat_index], a[lng_index], c[lat_index], c[lng_index])
    p = (ab + bc + ac) / 2
    return 2 * math.sqrt(p * (p - ab) * (p - bc) * (p - ac)) / ab


def rad(d):
    return d * math.pi / 180.0


def mars_distance(lat1, lng1, lat2, lng2):
    """
    :param lat1: latitude of point1
    :param lng1: longitude of point1
    :param lat2: latitude of point2
    :param lng2: longitude of point2
    :return: the distance between point1 and point2 on the Mars(Gaode) coordinate system
    """
    rad_lat1 = rad(lat1)
    rad_lat2 = rad(lat2)
    a = rad_lat1 - rad_lat2
    b = rad(lng1) - rad(lng2)
    s = 2 * math.asin(math.sqrt(math.pow(math.sin(a / 2), 2) +
                                math.cos(rad_lat1) * math.cos(rad_lat2) * math.pow(math.sin(b / 2), 2)))
    s *= EARTH_RADIUS
    return math.fabs(s * 1000)


def trajectory_compression(raw_trajectory, threshold, distance_index, distance_function):
    """
    :param raw_trajectory: the trajectory data, exp: [[x1, y1], [x2, y2], ... , [xn, yn]]
    :param threshold: distance threshold
    :param distance_index: which fields in a point are considered when calculating distance
    :param distance_function: method for calculating distance
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
    find_far_point(raw_trajectory, 0, len(raw_trajectory)-1, distance_index, threshold, key_arr, distance_function)
    key_points = []
    for i in range(len(key_arr)):
        if key_arr[i] == 1:
            key_points.append(raw_trajectory[i])
    return key_points


def find_far_point(arr, i, j, distance_index, threshold, flag_arr, dist_fun):
    """
    This function is used for finding the farthest points (to line (arr[i], arr[j])) in sub trajectory arr(i, j),
    it recurs until there are no more points's distance is larger than threshold to the line. if a point is not
    ignored in the trajectory, it's index in flag_arr will be set as 1, otherwise 0.
    :param arr: trajectory data
    :param i: start index
    :param j: end index
    :param distance_index: which fields in a point are considered when calculating distance
    :param threshold: distance threshold
    :param flag_arr: the flag array of useful or ignore for the trajectory points(0:ignore, 1:useful)
    :param dist_fun: method for calculating distance
    :return: none, but change the flag_arr
    """
    if i > len(arr) or j > len(arr):
        print("index out of range")
        return
    max_distance = float(0)
    flag = -1
    for k in range(i + 1, j):
        d = dist_fun(arr[i], arr[j], arr[k], distance_index)
        if d > max_distance:
            max_distance = d
            flag = k
    if max_distance > threshold:
        flag_arr[flag] = 1
        find_far_point(arr, i, flag, distance_index, threshold, flag_arr, dist_fun)
        find_far_point(arr, flag, j, distance_index, threshold, flag_arr, dist_fun)

'''
arrs = [[1, 1, 1, 'x'], [2, 2, 2, 'y'], [3, 3, 3, 'z']]
arrs2 = [[116.368904, 39.913423], [116.368904, 39.923423], [116.398258, 39.904600]]
narrs = trajectory_compression(arrs, "0.0007", [0, 1, 2], distance_point2line_plane)
narrs2 = trajectory_compression(arrs2, 100, [0, 1], distance_point2line_mars)
print(narrs2)
'''
'''
print(mars_distance(39.923423, 116.368904, 39.922501, 116.387271))
print(distance_point2line_mars([116.368904, 39.913423], [116.398258, 39.904600], [116.368904, 39.923423], [0, 1]))
'''
