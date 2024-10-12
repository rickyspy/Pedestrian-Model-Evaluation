r'''
# Notes
# With the code, we'd like to formulate a framework or benchmark for quantitatively evaluating a pedestrian model
# by comparing the trajectories in simulations and in experiments. Note that an essential condition for the application
# of the evaluation framework is the identical or at least symmetric starting point and destination point for the
# participants in the repeated experiments.
'''

import numpy as np
import pandas as pd
import Similarity, Radar, os, copy
from numpy.linalg import norm
from scipy.spatial import Voronoi, voronoi_plot_2d


# import trajectories
def InputTrajectories(file_folder):
    g = os.walk(file_folder)
    files = []
    for path, dir_list, file_list in g:
        for file in file_list:
            files.append(file)

    trajectories_list_list = []
    for input_file in files:

        # ori_data = pd.read_csv(file_folder + "//" + input_file, header=None, sep="\t|;| ", engine='python')
        ori_data = pd.read_csv(os.path.join(file_folder, input_file), header=None, sep="\t")
        trajectories_list = []
        trajectories = []
        for i in range(ori_data.shape[0]):
            trajectories.append((ori_data.iloc[i, 2], ori_data.iloc[i, 3]))
            if i != ori_data.shape[0] - 1 and ori_data.iloc[i, 0] != ori_data.iloc[i + 1, 0]:
                trajectories_list.append(trajectories)
                trajectories = []
            elif i == ori_data.shape[0] - 1:
                trajectories_list.append(trajectories)
        trajectories_list_list.append(trajectories_list)
    return trajectories_list_list


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
def FilterTrajectories(tra_list_list, cutoff_distance):
    updated_tra_list_list = copy.deepcopy(tra_list_list)
    for k in range(len(tra_list_list)):
        for j in range(len(tra_list_list[k])):
            remove_list = []
            for i in range(len(tra_list_list[k][j])):
                if norm(np.array(tra_list_list[k][j][i]) - np.array(tra_list_list[k][j][0])) < cutoff_distance:
                    remove_list.append(i)
            for i in range(len(tra_list_list[k][j])):
                if norm(np.array(tra_list_list[k][j][i]) - np.array(tra_list_list[k][j][-1])) < cutoff_distance:
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
        temp_timestep = 0

        for x in range(len(trajectories_list_list[i])):
            if len(trajectories_list_list[i][x]) > temp_timestep:
                temp_timestep = len(trajectories_list_list[i][x])

        for j in range(temp_timestep - 1):  # for each time step we have a voronoi diagram
            ped_list = []
            speed_list = []
            for k in range(len(trajectories_list_list[i])):  # add the individual position
                if j < len(trajectories_list_list[i][k]) - 1:
                    ped_list.append(list(trajectories_list_list[i][k][j]))
                    speed_list.append((np.array(trajectories_list_list[i][k][j + 1])
                                       - np.array(trajectories_list_list[i][k][j])) * fps)
            # calculation
            if len(ped_list) > 2:
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

# Microscopic - Speed
def CalculateSpeedData(trajectoies_list_list, fps):
    speed_list_list = []
    for i in range(len(trajectoies_list_list)):
        speed_list = []
        for j in range(1, len(trajectoies_list_list[i])):
            for k in range(1, len(trajectoies_list_list[i][j])):
                coordinate_1 = trajectoies_list_list[i][j][k - 1]
                coordinate_2 = trajectoies_list_list[i][j][k]

                # calculate speed element
                speed = norm(np.array(coordinate_2) - np.array(coordinate_1)) * fps
                speed_list.append(speed)
        speed_list_list.append(speed_list)
    return speed_list_list


# Microscopic - Direction
def CalculateDirectionData(trajectoies_list_list):
    direction_list_list = []
    for i in range(len(trajectoies_list_list)):
        direction_list = []
        for j in range(1, len(trajectoies_list_list[i])):
            for k in range(1, len(trajectoies_list_list[i][j])):
                coordinate_1 = trajectoies_list_list[i][j][k - 1]
                coordinate_2 = trajectoies_list_list[i][j][k]
                coordinate_3 = trajectoies_list_list[i][j][-1]
                z1 = np.array(coordinate_2) - np.array(coordinate_1)
                z2 = np.array(coordinate_3) - np.array(coordinate_1)

                #  direction set
                if norm(z1) == 0 or norm(z2) == 0:
                    continue
                direction_vector_1 = z1 / norm(z1)
                direction_vector_2 = z2 / norm(z2)

                s = np.dot(direction_vector_1, direction_vector_2)

                # e_x, e_y, s = direction_vector_1[0], direction_vector_1[1], 0
                # if e_x > 0:
                #     s = np.arctan(e_y / e_x)
                # elif e_x < 0 and e_y >= 0:
                #     s = np.arctan(e_y / e_x) + np.pi
                # elif e_x < 0 and e_y < 0:
                #     s = np.arctan(e_y / e_x) - np.pi
                # else:
                #     s = np.pi / 2

                direction_list.append(s)
        direction_list_list.append(direction_list)
    return direction_list_list


# Distribution - Route Length
def CalculateRouteLengthList(trajectories_list_list):
    route_length_list_list = []
    for i in range(len(trajectories_list_list)):
        route_length_list = []
        for j in range(len(trajectories_list_list[i])):
            length = 0
            # length = length + norm(np.array(Original_Point) - np.array(trajectories_list_list[i][j][0]))
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


# Time series - Distance to original point and destination point
def CalculateOriPointTimeSeries(trajectories_list_list, cutoff_distance):
    distance_ts_list = []
    for k in range(len(trajectories_list_list)):
        distance_ts = []
        for j in range(np.max(
                [len(trajectories_list_list[k][x]) for x in range(len(trajectories_list_list[k]))])):  ## max time
            distance = 0
            for i in range(len(trajectories_list_list[k])):
                if j >= len(trajectories_list_list[k][i]):
                    continue
                distance += norm(np.array(trajectories_list_list[k][i][j]) - np.array(trajectories_list_list[k][i][0]))
            distance /= len(trajectories_list_list[k])
            if (cutoff_distance < distance):
                distance_ts.append((len(distance_ts), distance))
        distance_ts_list.append(distance_ts)
    return distance_ts_list  # --- --- --- --- ---


def CalculateDestPointTimeSeries(trajectories_list_list, cutoff_distance):
    distance_ts_list = []
    for k in range(len(trajectories_list_list)):
        distance_ts = []
        for j in range(np.max(
                [len(trajectories_list_list[k][x]) for x in range(len(trajectories_list_list[k]))])):  ## max time
            distance = 0
            for i in range(len(trajectories_list_list[k])):
                if j >= len(trajectories_list_list[k][i]):
                    continue
                distance += norm(np.array(trajectories_list_list[k][i][j]) - np.array(trajectories_list_list[k][i][-1]))
            distance /= len(trajectories_list_list[k])
            if (cutoff_distance < distance):
                distance_ts.append((len(distance_ts), distance))
        distance_ts_list.append(distance_ts)
    return distance_ts_list


# ### The evaluation approach contains four types of methods
# ### Fundamental diagram, trajectories, distribution indexes, time-series indexes
def SimilarityIndexes(expList, simList, indextype, indexname):
    index = 0
    index_sum = 0
    number = 0

    for i in range(len(expList)):
        for j in range(len(simList)):
            index = Similarity.SimilarityIndex(expList[i], simList[j], indextype)
            if index != 0:
                index_sum += index
                number += 1
    outdir = './ResultData'
    if not os.path.exists(outdir):
        os.mkdir(outdir)

    pd.DataFrame(expList).to_csv(os.path.join(outdir,'explist-' + indextype + indexname + '.txt'), mode='a', sep=' ')
    pd.DataFrame(simList).to_csv(os.path.join(outdir,'simlist-' + indextype + indexname + '.txt'), mode='a', sep=' ')
    # pd.DataFrame(simList).to_csv(outdir + '/explist-' + indextype + indexname + '.txt', mode='a', sep=' ')
    if number == 0:
        number = 1
    return index_sum / number


def ScoreNormalization(scorelist):
    s_list_1 = []
    s_list_2 = []
    s_list_3 = []
    for i in range(len(scorelist) - 1, -1, -1):
        for j in range(len(scorelist[i])):
            if scorelist[0][j] == 0:
                scorelist[0][j] = 0.1
            s_list_1.append(scorelist[i][j])
            scorelist[i][j] = (scorelist[i][j] - scorelist[0][j]) / scorelist[0][j]
            s_list_2.append(scorelist[i][j])
            scorelist[i][j] = 2 * np.exp(0 - scorelist[i][j]) / (1 + np.exp(0 - scorelist[i][j]))
            s_list_3.append(scorelist[i][j])
        print(s_list_1)
        # print(s_list_2)
        # print(s_list_3)
        s_list_1.clear()
        s_list_2.clear()
        s_list_3.clear()
    return scorelist


# ### Evaluation scores
def Evaluation(ori_exp_trajectories_list_list, ori_sim_trajectories_list_list, cutoff_distance, fps):
    exp_trajectories_list_list = \
        FilterTrajectories(ori_exp_trajectories_list_list, cutoff_distance)
    sim_trajectories_list_list = \
        FilterTrajectories(ori_sim_trajectories_list_list, cutoff_distance)

    # fd based data list
    index_fd = SimilarityIndexes(CalculateFundamentalDiagram(exp_trajectories_list_list, fps),
                                 CalculateFundamentalDiagram(sim_trajectories_list_list, fps),
                                 'dtw-fd', '-fd')

    # microscopic - speed
    index_speed = SimilarityIndexes(CalculateSpeedData(exp_trajectories_list_list, fps),
                                    CalculateSpeedData(sim_trajectories_list_list, fps),
                                    'ks-data', '-speed')

    # microscopic - direction
    index_direction = SimilarityIndexes(CalculateDirectionData(exp_trajectories_list_list),
                                        CalculateDirectionData(sim_trajectories_list_list),
                                        'ks-data', '-direction')

    # Distribution - Route length
    index_Dis_RL = SimilarityIndexes(CalculateRouteLengthList(exp_trajectories_list_list),
                                     CalculateRouteLengthList(sim_trajectories_list_list),
                                     'ks-data', '-RL')

    # Distribution - Travel Time
    index_Dis_TT = SimilarityIndexes(CalculateTravelTimeList(exp_trajectories_list_list, fps),
                                     CalculateTravelTimeList(sim_trajectories_list_list, fps),
                                     'ks-data', '-TT')

    # Times series - Original position
    index_TS_OriPoint = SimilarityIndexes(
        CalculateOriPointTimeSeries(exp_trajectories_list_list, cutoff_distance),
        CalculateOriPointTimeSeries(sim_trajectories_list_list, cutoff_distance),
        "dtw-ts", '-oripoint')

    # Times series - Destination position
    index_TS_DestPoint = SimilarityIndexes(
        CalculateDestPointTimeSeries(exp_trajectories_list_list, cutoff_distance),
        CalculateDestPointTimeSeries(sim_trajectories_list_list, cutoff_distance),
        "dtw-ts", '-destpoint')

    # Microscopic trajectories
    indexTrajectories = SimilarityIndexes(exp_trajectories_list_list,
                                          sim_trajectories_list_list, "dtw-trajectories", '-trajectories')

    scores = [index_fd,  # speed choice
              index_Dis_RL, index_Dis_TT, index_speed,  # static distribution
              index_TS_OriPoint, index_TS_DestPoint, index_direction,  # dynamic time series
              indexTrajectories]  # direction choice

    return scores


### top bottom
if __name__ == "__main__":

    ## 1 Data pre-processing ##
    Labels = ['EXP', 'SFM', 'HM']  # empirical and model result labels
    Ori_Fps = 25  # flames per second in original trajectories
    Adj_Fps = 5  # flames per second in adjustment trajectories
    Folder_Name = r'BaseData'
    Trajectories_List_List_List = []
    for i in range(0, len(Labels)):
        # Trajectories_List_List = InputTrajectories(os.path.join(Folder_Name, Labels[i]))
        Trajectories_List_List_List.append(InputTrajectories(os.path.join(Folder_Name, Labels[i])))
    Trajectories_List_List_List = FPSAdjustment(Trajectories_List_List_List, Ori_Fps, Adj_Fps)

    ## 2 Evaluation ##
    Scores_List = []
    Cutoff_Distance = 1  # cut-off distance
    for i in range(0, len(Trajectories_List_List_List)):
        scores = Evaluation(Trajectories_List_List_List[0], Trajectories_List_List_List[i], Cutoff_Distance, Adj_Fps)
        Scores_List.append(scores)
    Scores_List = ScoreNormalization(Scores_List)

    ## 3 Radar figure ##
    Line_Styles = ['k--', 'b^-.', 'gs--', 'ro-', 'yv:`']
    Radar.RadarFigure(Scores_List, Line_Styles, Labels)
    Radar.SoloRadarFigure(Scores_List, Line_Styles, Labels)
    print("Finished!!")
