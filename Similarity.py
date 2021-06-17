from math import *
from scipy.stats import ks_2samp, zscore
import numpy as np


def Distance(p1, p2):
    dist = 0.0
    elem_type = type(p1)
    if elem_type == float or elem_type == int:
        dist = float(abs(p1 - p2))
    else:
        sumval = 0.0
        for i in range(len(p1)):
            sumval += pow(p1[i] - p2[i], 2)
        dist = pow(sumval, 0.5)
    return dist


def DtwX(s1, s2):
    w = len(s1)
    h = len(s2)
    opValue = 10000

    mat = [([[0, 0, 0, 0] for j in range(w)]) for i in range(h)]

    # print_matrix(mat)

    for x in range(w):
        for y in range(h):
            dist = Distance(s1[x], s2[y])
            mat[y][x] = [dist, 0, 0, 0]

            # print_matrix(mat)

    elem_0_0 = mat[0][0]
    elem_0_0[1] = elem_0_0[0] * 2

    for x in range(1, w):
        mat[0][x][1] = mat[0][x][0] + mat[0][x - 1][1]
        mat[0][x][2] = x - 1
        mat[0][x][3] = 0

    for y in range(1, h):
        mat[y][0][1] = mat[y][0][0] + mat[y - 1][0][1]
        mat[y][0][2] = 0
        mat[y][0][3] = y - 1

    for y in range(1, h):
        for x in range(1, w):
            distlist = [mat[y][x - 1][1], mat[y - 1][x][1], 2 * mat[y - 1][x - 1][1]]
            mindist = min(distlist)
            idx = distlist.index(mindist)
            mat[y][x][1] = mat[y][x][0] + mindist
            if mat[y][x][1] > opValue:
                return mat[y][x][1]
            if idx == 0:
                mat[y][x][2] = x - 1
                mat[y][x][3] = y
            elif idx == 1:
                mat[y][x][2] = x
                mat[y][x][3] = y - 1
            else:
                mat[y][x][2] = x - 1
                mat[y][x][3] = y - 1

    result = mat[h - 1][w - 1]
    retval = result[1]
    path = [(w - 1, h - 1)]
    while True:
        x = result[2]
        y = result[3]
        path.append((x, y))

        result = mat[y][x]
        if x == 0 and y == 0:
            break
    return retval


def DtwTS(s1, s2):
    return DtwX(s1, s2) / (len(s2) + len(s1)) * 2, len(s2) + len(s1)


def DtwSorted(s1, s2):
    total_dtw = 0
    count = 0
    for i in range(len(s1)):
        dtw = DtwX(s1[i], s2[i])
        dtw = dtw # / (len(s1[i])+len(s2[i])) * 2
        count = count + 1
        total_dtw += dtw
    total_dtw = total_dtw / count
    return total_dtw, len(s1[0])


def DtwFD(p, q, zones):
    val_max = np.max(np.array(p), axis=0)[0]
    val_min = np.min(np.array(p), axis=0)[0]
    exp_dis = [0 for i in range(zones)]
    sim_dis = [0 for i in range(zones)]
    exp_dis_count = [0 for i in range(zones)]
    sim_dis_count = [0 for i in range(zones)]

    # statistics
    for i in range(len(p)):
        zone_num = min(max(int((p[i][0] - val_min) // ((val_max - val_min) / 10)), 0), zones - 1)
        exp_dis[zone_num] += p[i][1]
        exp_dis_count[zone_num] += 1
    for i in range(len(q)):
        zone_num = min(max(int((q[i][0] - val_min) // ((val_max - val_min) / 10)), 0), zones - 1)
        sim_dis[zone_num] += q[i][1]
        sim_dis_count[zone_num] += 1

    exp = []
    sim = []
    for i in range(zones):  # normalization
        if exp_dis_count[i] == 0:
            exp_dis_count[i] = 1
        exp_dis[i] /= exp_dis_count[i]
        if sim_dis_count[i] == 0:
            sim_dis_count[i] = 1
        sim_dis[i] /= sim_dis_count[i]
        exp.append([i, exp_dis[i]])
        sim.append([i, sim_dis[i]])
    # exp = zscore(exp)
    # sim = zscore(sim)
    val = DtwX(exp, sim)
    return val / len(sim), len(sim)


# please find here the problem
def SimilarityIndex(s1, s2, indextype):
    val = 0

    if indextype == 'dtw-ts':
        val, length = DtwTS(s1, s2)
    elif indextype == 'ks-data':
        val, length = ks_2samp(s1, s2)
    elif indextype == 'dtw-trajectories':
        val, length = DtwSorted(s1, s2)
    elif indextype == 'dtw-fd':
        val, length = DtwFD(s1, s2, 10)
    return val
