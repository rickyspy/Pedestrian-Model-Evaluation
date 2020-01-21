r'''
# Notes
# With the code, we'd like to formulate a framework or benchmark for quantitatively evaluating a pedestrian model
# by comparing the trajectories in simulations and in experiments. Note that an essential condition for the application
# of the evaluation framework is the identical or at least symmetric starting point and destination point for the
# participants in the repeated experiments.
'''

import numpy as np
import pandas as pd
import Similarity1, Radar1, os, copy
from numpy.linalg import norm
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt


# global parameters

# import trajectories
def InputTrajectories(file_folder):
    g = os.walk(file_folder)
    files = []
    for path, dir_list, file_list in g:
        for file in file_list:
            files.append(file)

    trajectoriesListList = []
    for input_file in files:
        ori_data = pd.read_csv(file_folder + "\\" + input_file, header=None, sep="\t")
        trajectories_list = []
        trajectories = []
        for i in range(ori_data.shape[0]):
            trajectories.append((ori_data.iloc[i, 2], ori_data.iloc[i, 3]))
            if i != ori_data.shape[0] - 1 and ori_data.iloc[i, 0] != ori_data.iloc[i + 1, 0]:
                trajectories_list.append(trajectories)
                trajectories = []
            elif i == ori_data.shape[0] - 1:
                trajectories_list.append(trajectories)
        trajectoriesListList.append(trajectories_list)
    return trajectoriesListList


def NormolizedTrajectories(ori_trajectories_list_list, ori_position, dest_position):
    updated_trajectories_list_list = []
    for j in range(len(ori_trajectories_list_list)):
        updated_trajectories_list = []
        for k in range(len(ori_trajectories_list_list[j])):
            updated_trajectories = []
            first_trajectories = ori_trajectories_list_list[j][k][0]
            last_trajectories = ori_trajectories_list_list[j][k][-1]
            angle = AngleRotation(originalPoint, destinationPoint, first_trajectories, last_trajectories)
            for h in range(len(ori_trajectories_list_list[j][k])):  # rotation
                point = ori_trajectories_list_list[j][k][h]
                updated_point = TrajecotoriesNorAdjustment(point, first_trajectories, originalPoint, angle)
                updated_trajectories.append(updated_point)
            if abs(np.max(updated_trajectories, axis=0)[1]) < abs(np.min(updated_trajectories, axis=0)[1]):  # mirror
                for h in range(len(updated_trajectories)):  # mirror
                    updated_trajectories[h] = ((updated_trajectories[h][0], -updated_trajectories[h][1]))
            updated_trajectories_list.append(updated_trajectories)
        updated_trajectories_list_list.append(updated_trajectories_list)
    return updated_trajectories_list_list


def AngleRotation(ori_position_1, dest_position_1, ori_position_2, dest_position_2):
    a = np.array(dest_position_1) - np.array(ori_position_1)
    b = np.array(dest_position_2) - np.array(ori_position_2)
    dot = a[0] * b[0] + a[1] * b[1]
    det = a[0] * b[1] - a[1] * b[0]
    return np.arctan2(det, dot)


def TrajecotoriesNorAdjustment(point, start_point, original_point, angle):
    displacement = np.array(original_point) - np.array(start_point)
    updated_point = np.array(point) + np.array(displacement)
    a = updated_point
    b = original_point
    angle = -angle
    updated_point = (((a[0] - b[0]) * np.cos(angle) - (a[1] - b[1]) * np.sin(angle) + b[0],
                      (a[0] - b[0]) * np.sin(angle) + (a[1] - b[1]) * np.cos(angle) + b[1]))
    return updated_point


def FPSAdjustment(ori_trajectories_list_list_list, orifps, destfps):
    times = orifps / destfps
    dest_trajectories_list_list_list = []
    for i in range(len(ori_trajectories_list_list_list)):
        dest_trajectories_list_list = []
        for j in range(len(ori_trajectories_list_list_list[i])):
            dest_trajectories_list = []
            for k in range(len(ori_trajectories_list_list_list[i][j])):
                dest_trajectories = []
                for ii in range(len(ori_trajectories_list_list_list[i][j][k])):
                    if ii % times == 0:
                        dest_trajectories.append(ori_trajectories_list_list_list[i][j][k][ii])
                dest_trajectories_list.append(dest_trajectories)
            dest_trajectories_list_list.append(dest_trajectories_list)
        dest_trajectories_list_list_list.append(dest_trajectories_list_list)
    return dest_trajectories_list_list_list


# Filter unnecessary trajectories
def FilterTrajectories(tra_list_list, ori_point, dest_point, cutoff_distance):
    updated_tra_list_list = copy.deepcopy(tra_list_list)
    for k in range(len(tra_list_list)):
        for j in range(len(tra_list_list[k])):
            remove_list = []
            for i in range(len(tra_list_list[k][j])):
                if norm(np.array(tra_list_list[k][j][i]) - np.array(ori_point)) < cutoff_distance:
                    remove_list.append(i)
            for i in range(len(tra_list_list[k][j])):
                if norm(np.array(tra_list_list[k][j][i]) - np.array(dest_point)) < cutoff_distance:
                    remove_list.append(i)
            remove_list.reverse()

            for i in range(len(remove_list)):
                updated_tra_list_list[k][j].remove(updated_tra_list_list[k][j][remove_list[i]])
    return updated_tra_list_list


# Fundamental diagram
def CalculateFundamentalDiagram(trajectories_list_list, fps):
    fd_list = []
    for i in range(len(trajectories_list_list)):
        fd = []
        for j in range(len(trajectories_list_list[i][0]) - 1):  # for each time step we have a voronoi diagram
            ped_list = []
            speed_list = []
            density_list = []
            for k in range(len(trajectories_list_list[i])):  # add the individual position
                ped_list.append(list(trajectories_list_list[i][k][j]))
                speed_list.append((np.array(trajectories_list_list[i][k][j + 1])
                                   - np.array(trajectories_list_list[i][k][j])) * fps)
            # calculation
            vor = Voronoi(ped_list)
            # fig = voronoi_plot_2d(vor)
            # plt.show()
            for k in range(len(vor.regions) - 1):
                if not -1 in vor.regions[k]:
                    polygon = [vor.vertices[ii] for ii in vor.regions[k]]
                    area = areaPolygon(polygon)
                    if area != 0:
                        fd.append(np.array([1 / area, norm(speed_list[k])]))
        fd_list.append(fd)
    return fd_list


def areaPolygon(vertices):
    n = len(vertices)
    if n < 3:
        return 0
    area = 0
    for i in range(n - 2):
        # 以第一个坐标点为原点，将多边形分割为n-2个三角形，分别计算每个三角形面积后累加得多边形面积
        area += calculate_triangle_area(vertices[0], vertices[i + 1], vertices[i + 2])
    return abs(area)


def calculate_triangle_area(point_a, point_b, point_c):
    triangle_area = 0.5 * ((point_b[0] - point_a[0]) * (point_c[1] - point_a[1]) -
                           (point_b[1] - point_a[1]) * (point_c[0] - point_a[0]))
    return triangle_area


# Trajectories
# Distribution - Route Length
def CalculateRouteLengthList(trajectories_list_list):
    route_length_list_list = []
    for i in range(len(trajectories_list_list)):
        route_length_list = []
        for j in range(len(trajectories_list_list[i])):
            length = 0
            for k in range(len(trajectories_list_list[i][j]) - 1):
                length = length + norm(
                    np.array(trajectories_list_list[i][j][k + 1]) - np.array(trajectories_list_list[i][j][k]))
            route_length_list.append(length)
        route_length_list_list.append(route_length_list)
    return route_length_list_list


# Distribution - Travel Time
def CalculateTravelTimeList(trajectories_list_list, fps):
    travel_time_list_list = []
    for i in range(len(trajectories_list_list)):
        travel_time_list = []
        for j in range(len(trajectories_list_list[i])):
            travel_time = 0
            for k in range(len(trajectories_list_list[i][j]) - 1):
                travel_time = travel_time + 1
            travel_time = travel_time / fps
            travel_time_list.append(travel_time)
        travel_time_list_list.append(travel_time_list)
    return travel_time_list_list


# Distribution - Speed
def CalculateSpeedList(trajectories_list_list, fps):
    speed_list_list = []
    for i in range(len(trajectories_list_list)):
        speed_list = []
        for j in range(len(trajectories_list_list[i])):
            speed = 0
            count = 0
            for k in range(len(trajectories_list_list[i][j]) - 1):
                speed += (norm(np.array(trajectories_list_list[i][j][k + 1]) -
                               np.array(trajectories_list_list[i][j][k]))) * fps
                count = count + 1
            speed = speed / count
            speed_list.append(speed)
        speed_list_list.append(speed_list)
    return speed_list_list


# Time series - Distance to original point and destination point
def CalculatePointTimeSeries(trajectories_list_list, position, cutoff_distance):
    distance_ts_list = []
    for k in range(len(trajectories_list_list)):
        distance_ts = []
        for j in range(len(trajectories_list_list[k][0])):
            distance = 0
            for i in range(len(trajectories_list_list[k])):
                distance = distance + norm(np.array(trajectories_list_list[k][i][j]) - np.array(position))
            distance = distance / len(trajectories_list_list[k])
            distance_ts.append((j, distance))
        distance_ts_list.append(distance_ts)
    return distance_ts_list


# Time series - Speed
def CaculateSpeedTimeSeries(trajectories_list_list, cutoff_speed, fps):
    speed_ts_list = []
    for k in range(len(trajectories_list_list)):
        speed_ts = []
        for j in range(len(trajectories_list_list[k][0]) - 1):
            speed = 0
            for i in range(len(trajectories_list_list[k])):
                speed += norm(
                    np.array(trajectories_list_list[k][i][j + 1]) - np.array(trajectories_list_list[k][i][j])) * fps
            speed = speed / len(trajectories_list_list[k])
            if speed > cutoff_speed:
                speed_ts.append((j, speed))
        speed_ts_list.append(speed_ts)
    return speed_ts_list


# ### The evalution approach contains four types of methods
# ### Fundamental diagram, trajectories, distribution indexes, time-series indexes
def SimilarityIndexes(expList, simList, indextype):
    index = 0
    index_sum = 0
    number = 0
    index_max = 0

    for i in range(len(expList)):
        for j in range(len(simList)):
            index = Similarity1.SimilarityIndex(expList[i], simList[j], indextype)
            index_sum += index
            number += 1
            index_max = max(index, index_max)



    index = index / number

    return index


# ### Evaluation scores
def Evaluation(oripoint, destpoint, oriexptrajectorieslistlist, orisimtrajectorieslistlist, cutoffdistance, fps):
    exptrajectorieslistlist = NormolizedTrajectories(oriexptrajectorieslistlist, oripoint, destpoint)
    simtrajectorieslistlist = NormolizedTrajectories(orisimtrajectorieslistlist, oripoint, destpoint)
    expModiTrajectoriesListList = FilterTrajectories(exptrajectorieslistlist, oripoint, destpoint, cutoffdistance)
    simModiTrajectoriesListList = FilterTrajectories(simtrajectorieslistlist, oripoint, destpoint, cutoffdistance)

    # fd based data list
    index_fd = SimilarityIndexes(CalculateFundamentalDiagram(oriexptrajectorieslistlist, fps),
                                 CalculateFundamentalDiagram(orisimtrajectorieslistlist, fps), 'dtw-fd')

    # Distribution - Route length
    indexDisRL = SimilarityIndexes(CalculateRouteLengthList(expModiTrajectoriesListList),
                                   CalculateRouteLengthList(simModiTrajectoriesListList), 'dtw-dis')

    # Distribution - Travel Time
    indexDisTT = SimilarityIndexes(CalculateTravelTimeList(expModiTrajectoriesListList, fps),
                                   CalculateTravelTimeList(simModiTrajectoriesListList, fps), 'dtw-dis')

    # Distribution - Speed
    indexDisSpeed = SimilarityIndexes(CalculateSpeedList(expModiTrajectoriesListList, fps),
                                      CalculateSpeedList(simModiTrajectoriesListList, fps), 'dtw-dis')

    # Times series - Original position
    indexTSOriPoint = SimilarityIndexes(CalculatePointTimeSeries(exptrajectorieslistlist, oripoint, cutoffdistance),
                                        CalculatePointTimeSeries(simtrajectorieslistlist, oripoint, cutoffdistance),
                                        "dtw-ts")

    # Times series - Destination position
    indexTSDestPoint = SimilarityIndexes(CalculatePointTimeSeries(exptrajectorieslistlist, destpoint, cutoffdistance),
                                         CalculatePointTimeSeries(simtrajectorieslistlist, destpoint, cutoffdistance),
                                         "dtw-ts")

    # Time series - Speed
    indexTSSpeed = SimilarityIndexes(CaculateSpeedTimeSeries(exptrajectorieslistlist, 0.1, fps),
                                     CaculateSpeedTimeSeries(simtrajectorieslistlist, 0.1, fps), "dtw-ts")

    # Microscopic trajectories
    indexTrajectories = SimilarityIndexes(expModiTrajectoriesListList,
                                          simModiTrajectoriesListList, "dtw-sort")

    scores = [index_fd,  # macroscopic fd
              indexDisRL, indexDisTT, indexDisSpeed,  # static distribution
              indexTSOriPoint, indexTSDestPoint, indexTSSpeed,  # dynamic time series
              indexTrajectories]  # microscopic trajectories
    return scores


### top bottom
if __name__ == "__main__":
    originalPoint = (0, 0)  # starting position
    destinationPoint = (20, 0)  # destination position
    cutoffDistance = 1  # cut-off distance
    orifps = 25  # flames per second
    destfps = 5  # flames per second
    labels = ['EXP', 'BM', 'SFM', 'VO']
    lineStyles = ['ro-', 'ys--', 'b^-.', 'gv:', 'k--']
    folderName = r'C:\Users\xiaoy\Nut\Nutstore\Codes\Pedestrian Dynamics\Code_Voronoi_x1' \
                 r'\PedestrianFlow_Forcebasedmodel\bin\Debug\Evaluation-Test'

    oriTrajectoriesListListList = []
    TrajectoriesListListList = []
    scoresList = []
    for i in range(0, len(labels)):
        TrajectoriesListList = InputTrajectories(folderName + "\\" + labels[i])
        TrajectoriesListListList.append(TrajectoriesListList)
    TrajectoriesListListList = FPSAdjustment(TrajectoriesListListList, orifps, destfps)
    for i in range(0, len(TrajectoriesListListList)):
        scores = Evaluation(originalPoint, destinationPoint, TrajectoriesListListList[0], TrajectoriesListListList[i],
                            cutoffDistance, destfps)
        scoresList.append(scores)
    Radar1.RadarFigure(scoresList, lineStyles, labels)
