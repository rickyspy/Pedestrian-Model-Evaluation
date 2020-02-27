import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


def RadarFigure(scoreslist, linestylelist, modelnamelist):
    # 模板数据和标签
    dataLenth = 8
    angles = np.linspace((1 / 4) * np.pi, (2 + 1 / 4) * np.pi, dataLenth, endpoint=False)
    angles = np.concatenate((angles, [angles[0]]))
    labels = np.array(['TS of \n Starting Position', 'Mircroscopic \n Trajectories Pattern', 'Dis of \n Route Length'
                          , 'Dis of \n Travel Time', 'Dis of \n Speed',
                       'Macroscopic \n Fundamental Diagram', 'TS of \n Speed', 'TS of \n Destination Position'])

    fig = plt.figure(figsize=(3.4, 3.3), dpi=300, linewidth=0.5)
    ax = fig.add_subplot(111, polar=True)
    plt.rcParams.update({'font.size': 6.5})
    plt.grid(linewidth=0.25, linestyle='--')

    ### data
    linelist = []
    for i in range(len(scoreslist)):
        z = [scoreslist[i][4], scoreslist[i][7], scoreslist[i][1], scoreslist[i][2], scoreslist[i][3], scoreslist[i][0],
             scoreslist[i][6], scoreslist[i][5]]
        data = np.array(z)
        data = np.concatenate((data, [data[0]]))
        li, = ax.plot(angles, data, linestylelist[i], linewidth=0.75, markersize=1.5)  # 线样 1
        linelist.append(li)

    ax.set_thetagrids(angles * 180 / np.pi, labels, fontproperties="Calibri")
    plt.ylim(-0.25, 1.25)  # y axis size
    plt.yticks(np.arange(0, 1.5, step=0.5))

    # plt.yticks.grid = True

    # label rotation
    plt.gcf().canvas.draw()
    # angles1 = np.linspace(0.5*np.pi + (1/8) * np.pi, 0.5*np.pi +(2+1/8) * np.pi, dataLenth, endpoint=False)
    angles1 = angles + 0.5 * np.pi
    angles1[np.cos(angles1) < 0] = angles1[np.cos(angles1) < 0] + np.pi
    angles1 = np.rad2deg(angles1)
    labels = []
    for label, angle in zip(ax.get_xticklabels(), angles1):
        x, y = label.get_position()
        lab = ax.text(x, y, label.get_text(), transform=label.get_transform(), ha=label.get_ha(), va=label.get_va())
        lab.set_rotation(angle)
        labels.append(lab)
    ax.set_xticklabels([])
    # plt.subplots_adjust(top=0.68,bottom=0.32,left=0.05,right=0.95)

    # 设置刻度标签的大小
    for tick in ax.xaxis.get_major_ticks():
        tick.label.set_fontsize(25)

    ax.grid(True)

    # 设置legend
    # modelnamelist.remove('EXP')
    plt.legend(handles=linelist, labels=modelnamelist, fontsize=7,
               labelspacing=0.075, borderpad=None, edgecolor='white',  # borderaxespad = None,
               loc=5, bbox_to_anchor=(1.175, -0.06))
    # plt.figure(figsize = (2,2))
    plt.savefig("radar.jpg")
    plt.show()
