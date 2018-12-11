import json
from math import sqrt

from params import *
from wagon import Wagon


class Line():
    def __init__(self, data_file, max_wagons=None):
        self.data_file = data_file
        self.wagons = []
        self.last_departure = None
        self.max_wagons = max_wagons

        self.init_line()

    def init_line(self):
        # read data file
        with open(self.data_file, encoding='utf-8') as json_file:
            json_data = json.load(json_file)

        # self.dist[i] is the distance from the beginning hangar to the station i
        self.dist = [0]
        # self.n_wagons_stopover[i] is the number of wagons to stop at station i
        self.n_wagons_stopover = []
        # self.speed_lim[i] is the speed limit between station i and i + 1
        self.speed_lim = []
        A = 1 / (2 * accel_max) + 1 / (2 * deccel_nom)
        B = time_at_rv_speed - delta_rv_speed / accel_max + full_boa_time
        self.station_names = []
        for station in json_data['stations']:
            self.station_names.append(station['name'])
            n_wagons_stopover = station['n_wagons_stopover']
            if n_wagons_stopover is None:
                n_wagons_stopover = w_per_train
            self.n_wagons_stopover.append(n_wagons_stopover)
            next_dist = station['next_dist']
            if next_dist is not None:
                self.dist.append(next_dist + self.dist[-1])

                C = delta_rv_speed**2 / (2 * accel_max) - \
                    time_at_rv_speed * delta_rv_speed - next_dist
                self.speed_lim.append(
                    min((-B + sqrt(B**2 - 4 * A * C)) / (2 * A), vitesse_max))

        self.find_min_interval()

    def find_min_interval(self):
        """Find minimum interval between trains. It corresponds to the
        maximum time for a stopover."""

        wait_time = []
        for i, speed in enumerate(self.speed_lim[:-1]):
            vtr1 = speed
            vtr2 = self.speed_lim[i + 1]
            vrv2 = vtr2 - delta_rv_speed
            wait_time.append(vtr1 / (2 * deccel_nom) + full_boa_time +
                             vrv2 / accel_max - vrv2**2 / (2 * accel_max * vtr2))
        self.min_interval = max(wait_time) * 1.1  # * 3
        print('min interval: ', self.min_interval)

    #######################################################
    #               Position Along the Line               #
    #######################################################

    def get_index(self, p):
        """Return the index of the station preceding p"""
        # if p < self.dist[0] or p > self.dist[-1]:
        #     raise ValueError('Position %f does not exist.' % p)
        for i in range(len(self.dist) - 1):
            if p < self.dist[i + 1]:  # p is before next station
                return i
        return len(self.dist) - 1

    def get_speed_lim(self, p):
        index = self.get_index(p)
        return self.speed_lim[index]

    def get_next_speed_lim(self, p):
        index = self.get_next_station_index(p)
        if index >= len(self.speed_lim):
            return None
        return self.speed_lim[index]

    def get_prev_station_index(self, p):
        return self.get_index(p)

    def get_now_station_index(self, p):
        prev_i = self.get_prev_station_index(p)
        next_i = self.get_next_station_index(p)
        closest_i, closest_dist = min([
            (prev_i, p - self.get_station_pos(prev_i)),
            (next_i, self.get_station_pos(next_i) - p)], key=lambda tup: tup[1])
        if closest_dist < 10:
            return closest_i
        else:
            return None

    def get_next_station_index(self, p):
        return self.get_index(p) + 1

    def get_terminus_index(self):
        return len(self.dist) - 2

    def get_station_pos(self, i):
        return self.dist[i]

    def get_next_station_pos(self, p):
        return self.get_station_pos(self.get_next_station_index(p))

    def get_next_n_wagons_stopover(self, p):
        i = self.get_next_station_index(p)
        return self.n_wagons_stopover[i]

    #######################################################
    #                  Wagon Requests                     #
    #######################################################

    def insert_wagon(self, w, index=0):
        self.wagons.insert(w, index)

    def prepare_next_train(self, t):
        train = []
        for i in range(w_per_train):
            p0 = i * longeur_wagon + longeur_wagon
            train.append(Wagon(self, p0, t))
        for w in reversed(train):
            w.connected = train
            self.wagons.insert(w, 0)

    def get_ahead(self, w):
        #  return the wagon ahead of wagon w
        w_i = self.wagons.index(w)
        if w_i == len(self.wagons) - 1:
            return None
        return self.wagons[w_i + 1]

    def get_behind(self, w):
        #  return the wagon behind of wagon w
        w_i = self.wagons.index(w)
        if w_i == 0:
            return None
        return self.wagons[w_i - 1]

    #######################################################
    #                     Run Time                        #
    #######################################################

    def prepare_train(self, t):
        p0 = self.get_station_pos(0)
        head = Wagon(self, p0, t, state='head')
        last = head
        for n in range(w_per_train - 1):
            p0 -= longeur_wagon
            new = Wagon(self, p0, t, state='tail')
            last.couple_back(new)
            last = new

        self.wagons = last.connected + self.wagons

    def update(self, t):
        for w in reversed(self.wagons):
            w.update(t)
            w.state.apply_throttle(t)

        if len(self.wagons) == 0 or self.wagons[0].p_back > self.get_station_pos(1):
            if self.max_wagons is None or len(self.wagons) < self.max_wagons - 3:
                print('preparing new train')
                self.prepare_train(t)
