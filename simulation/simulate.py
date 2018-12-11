import csv
import os
import numpy as np
from matplotlib import pyplot as plt

from line import Line

result_folder = os.path.join('..', 'results')


def plot_positions(time, position, n_wagon, stations={}):
    plt.figure(1)

    for name, pos in stations.items():
        plt.hlines(pos, time[0], time[-1], linestyles='dashed')
        plt.text(0, pos + 100, name, fontsize=12)

    for i in range(n_wagon):
        plt.plot(time, position[:, i])
    plt.title('Position de plusieurs wagons sur la ligne verte')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    # plt.ylim((0, 3300))
    plt.show()


def main(writer):
    sim_time = 0
    end_time = 60 * 60
    # end_time = 1
    n_wagon = 21
    dt = 0.1
    time = np.arange(0, end_time, dt)
    position = np.zeros((len(time), n_wagon))

    line = Line('verte_full.json')
    stations = {name: dist for name, dist in zip(line.station_names, line.dist)}

    monitor = [400, 475]

    for v in line.speed_lim:
        print(v)
    print('min: ', min(line.speed_lim) * 3.6)
    print('max: ', max(line.speed_lim) * 3.6)
    print('moyenne: ', (sum(line.speed_lim) / len(line.speed_lim)) * 3.6)
    exit()

    try:

        for i, t in enumerate(time):
            line.update(t)

            # d = ['%06.1f' % t]
            now_p = np.zeros((n_wagon))
            # n_wagon_now = len(line.wagons)
            for index, w in enumerate(reversed(line.wagons[-n_wagon:])):
                now_p[-index] = w.p_front

                # if monitor[0] < t and t < monitor[1]:
                #     if w in line.wagons[-4:-1]:
                #         d.append('%06.1f %s' %
                #                  (w.dp, w.state.__class__.__name__[:15]))
                # print(w.state)
                # print(w.p_front)
                # d.append('%02.3f' % w.dp)

            position[i, :] = now_p

            # if monitor[0] < t and t < monitor[1]:
            #     print(', '.join(d))

    except Exception as e:
        plot_positions(time, position, n_wagon, stations)
        raise e
    plot_positions(time, position, n_wagon, stations)


if __name__ == '__main__':
    file_name = 'test1.csv'
    # with open(file_name, 'w', newline='') as csvfile:
    #     writer = csv.writer(csvfile, delimiter=',',
    #                         quotechar='"', quoting=csv.QUOTE_MINIMAL)

    main(2)
