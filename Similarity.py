from math import *
import numpy as np
from scipy.stats import ks_2samp, zscore
import Evaluation1


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


def dtwX(s1, s2):
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


def dtwfd(p, q, zones):
    val_max = max(p[0])
    val_min = min(p[0])
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
        exp_dis[i] /= exp_dis_count[i]
        sim_dis[i] /= sim_dis_count[i]
        exp.append([i, exp_dis[i]])
        sim.append([i, sim_dis[i]])
    exp = zscore(exp)
    sim = zscore(sim)
    val = dtwX(exp, sim)
    return val/len(sim), len(sim)


def dtwDis(p, q, zones):
    val_max = max(p)
    val_min = min(p)
    exp_dis = [0 for i in range(zones)]
    sim_dis = [0 for i in range(zones)]
    exp_dis_count = [0 for i in range(zones)]
    sim_dis_count = [0 for i in range(zones)]

    # statistics
    for i in range(len(p)):
        zone_num = min(max(int((p[i] - val_min) // ((val_max - val_min) / 10)), 0), zones - 1)
        exp_dis_count[zone_num] += 1
        exp_dis[zone_num] += p[i]
    for i in range(len(q)):
        zone_num = min(max(int((q[i] - val_min) // ((val_max - val_min) / 10)), 0), zones - 1)
        sim_dis_count[zone_num] += 1
        sim_dis[zone_num] += q[i]

    for i in range(zones):  # normalization
        if exp_dis_count[i] != 0:
            exp_dis[i] /= exp_dis_count[i]
        if sim_dis_count[i] != 0:
            sim_dis[i] /= sim_dis_count[i]

    exp = []
    sim = []
    for i in range(zones):  # normalization
        exp.append([i, exp_dis[i]])
        sim.append([i, sim_dis[i]])

    val = dtwX(exp, sim)
    return val/len(sim), len(sim)


def dtwTS(s1, s2):
    return dtwX(s1, s2)/len(s2), len(s2)


def dtwSorted(s1, s2):
    s1 = trajectoriesSort(s1)
    s2 = trajectoriesSort(s2)
    total_dtw = 0
    count = 0
    for i in range(len(s1)):
        dtw = dtwX(s1[i], s2[i])
        dtw = dtw / len(s2[i])
        count = count + 1
        total_dtw += dtw
    total_dtw = total_dtw / count
    return total_dtw, len(s2[0])


def CalculateRouteLength(trajectories_list):
    route_length_list = []
    for i in range(len(trajectories_list)):
        length = 0
        for j in range(len(trajectories_list[i]) - 1):
            length = length + np.linalg.norm(
                np.array(trajectories_list[i][j + 1]) - np.array(trajectories_list[i][j]))
        route_length_list.append(length)
    return route_length_list


def trajectoriesSort(ts_list):
    route_length_list = CalculateRouteLength(ts_list)
    ts_list_updated = []
    for i in range(len(ts_list)):
        ts_updated = []
        ts_updated.append(route_length_list[i])
        for j in range(len(ts_list[i])):
            ts_updated.append(ts_list[i][j])
        ts_list_updated.append(ts_updated)

    ts_list_updated.sort()
    for i in range(len(ts_list_updated)):
        ts_list_updated[i].pop(0)
    return ts_list_updated


# ###
def kl_divergence_continuous(p, q):
    return np.sum(np.where(p != 0, p * np.log(p / q), 0))


def kl_divergence_distribution(p, q, zones):  # (0, +inf)
    # return np.sum(np.where(p != 0, p * np.log(p / q), 0))
    val_max = max(p)
    val_min = min(p)
    exp_dis = [0 for i in range(zones)]
    sim_dis = [0 for i in range(zones)]

    # statistics
    for i in range(len(p)):
        zone_num = min(max(int((p[i] - val_min) // ((val_max - val_min) / 10)), 0), zones - 1)
        exp_dis[zone_num] += 1
    for i in range(len(q)):
        zone_num = min(max(int((q[i] - val_min) // ((val_max - val_min) / 10)), 0), zones - 1)
        sim_dis[zone_num] += 1

    for i in range(zones):  # normalization
        exp_dis[i] /= len(p)
        sim_dis[i] /= len(q)

    val = 0
    for i in range(zones):
        if exp_dis[i] == 0:
            continue
        val += exp_dis[i] * log(exp_dis[i] / sim_dis[i])
    return val


def kl_divergence_fd(p, q, zones):  # (0, +inf)
    # return np.sum(np.where(p != 0, p * np.log(p / q), 0))
    val_max = max(p[0])
    val_min = min(p[0])
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
        sim_dis[zone_num] += p[i][1]
        sim_dis_count[zone_num] += 1

    for i in range(zones):  # normalization
        exp_dis[i] /= exp_dis_count[i]
        sim_dis[i] /= sim_dis_count[i]

    val = 0
    for i in range(zones):
        if exp_dis[i] == 0:
            continue
        val += exp_dis[i] * log(exp_dis[i] / sim_dis[i])
    return val


def ksTest(expArray, simArray):
    x, p = ks_2samp(expArray, simArray)
    # result = 1 / (1 - log10(p))
    return p


xlist = []


def SimilarityIndex(s1, s2, indextype):
    val = 0
    fixvalue = 15.82
    if indextype == 'dtw-fd':
        val, length = dtwfd(s1, s2, 10)
        xlist.append(['dtw-fd', length, round(val, 2), round(val / length, 2)])
        val = (fixvalue - val) / fixvalue
    elif indextype == 'dtw-dis':
        val, length = dtwDis(s1, s2, 5)
        xlist.append(['dtw-dis', length, round(val, 2)])
        val = (fixvalue - val) / fixvalue
    elif indextype == 'dtw-ts':
        val, length = dtwTS(s1, s2)
        xlist.append(['dtw-ts', length, round(val, 2)])
        val = (fixvalue - val) / fixvalue
    elif indextype == 'dtw-sort':
        val, length = dtwSorted(s1, s2)
        xlist.append(['dtw-sort', length, round(val, 2)])
        val = (fixvalue - val) / fixvalue

    return val
