from random import random
from numpy.random import geometric
from enum import Enum

from params import *
from state_machine import TrainReadyHangar, Tail


def rk4(f, t0, p0, v0, dt):
    a0 = f(t0, 2)
    tf = t0 + dt
    af = f(tf, 2)

    t_mid = t0 + dt / 2
    a_mid = f(t_mid, 2)

    p = p0 + v0 * dt + (a0 + 2 * a_mid) * dt**2 / 6
    v = v0 + (a0 + 4 * a_mid + af) * dt / 6

    return p, v


###############################################################################
#                                Doors States                                 #
###############################################################################
"""Side doors and boa doors both take time to open and close and can be
 blocked. These states are meant to be used for the wagon doors attributes and
 not the wagon itself.
"""


class DoorsState(Enum):
    Opened = 'opened'
    Closing = 'closing'
    Closed = 'closed'
    Opening = 'opening'
    Blocked = 'blocked'


class Wagon():

    def __init__(self, line, p0, t, state='head'):
        self.line = line
        self.p_front = p0
        self.dp = 0
        self.ddp = 0
        self.connected = [self]
        self._init_doors()
        if state == 'head':
            self.state = TrainReadyHangar(self, t)
        elif state == 'tail':
            self.state = Tail(self, t)
        else:
            raise NotImplementedError

    def _init_doors(self):
        self._side_doors_state = DoorsState.Closed
        self._side_doors_t0 = 0.0
        self._front_boa_state = DoorsState.Closed
        self._front_boa_t0 = 0.0
        self._back_boa_state = DoorsState.Closed
        self._back_boa_t0 = 0.0

    ###########################################
    #          Kinematic Properties           #
    ###########################################
    @property
    def p_front(self):
        """1d position along the track"""
        return self._p

    @p_front.setter
    def p_front(self, value):
        self._p = value

    @property
    def p_back(self):
        return self._p - longeur_wagon

    @property
    def dp(self):
        """1d speed along the track"""
        return self._dp

    @dp.setter
    def dp(self, value):
        self._dp = value

    @property
    def ddp(self):
        """1d accel along the track"""
        return self._ddp

    @ddp.setter
    def ddp(self, value):
        self._ddp = value

    ###########################################
    #                Actions                  #
    ###########################################

    def get_side_doors_blocked_time(self):
        return (side_doors_openning_time + side_doors_closing_time) * \
            geometric(side_doors_deblock_prob)

    def get_boa_blocked_time(self):
        return (boa_openning_time + boa_closing_time) * \
            geometric(boa_deblock_prob)

    def side_doors_closed(self, t):
        if self._side_doors_state is DoorsState.Closing and t - self._side_doors_t0 >= side_doors_closing_time:
            # if random() < side_doors_error_prob:
            #     self._side_doors_state = DoorsState.Blocked
            #     self._side_doors_t0 = t
            #     self._side_doors_blocked_time = self.get_side_doors_blocked_time()
            # else:
            self._side_doors_state = DoorsState.Closed
        elif self._side_doors_state is DoorsState.Blocked and t - self._side_doors_t0 >= self._side_doors_blocked_time:
            self._side_doors_state = DoorsState.Closed
        return self._side_doors_state is DoorsState.Closed

    def back_boa_closed(self, t):
        if self._back_boa_state is DoorsState.Closing and t - self._back_boa_t0 >= boa_closing_time:
            # if random() < boa_error_prob:
            #     self._back_boa_state = DoorsState.Blocked
            #     self._back_boa_t0 = t
            #     self._back_boa_blocked_time = self.get_boa_blocked_time()
            # else:
            self._back_boa_state = DoorsState.Closed
        elif self._back_boa_state is DoorsState.Blocked and t - self._back_boa_t0 >= self._back_boa_blocked_time:
            self._back_boa_state = DoorsState.Closed
        return self._back_boa_state is DoorsState.Closed

    def front_boa_closed(self, t):
        if self._front_boa_state is DoorsState.Closing and t - self._front_boa_t0 >= boa_closing_time:
            # if random() < boa_error_prob:
            #     self._front_boa_state = DoorsState.Blocked
            #     self._front_boa_t0 = t
            #     self._front_boa_blocked_time = self.get_boa_blocked_time()
            # else:
            self._front_boa_state = DoorsState.Closed
        elif self._front_boa_state is DoorsState.Blocked and t - self._front_boa_t0 >= self._front_boa_blocked_time:
            self._front_boa_state = DoorsState.Closed
        return self._front_boa_state is DoorsState.Closed

    def side_doors_blocked(self):
        return self._side_doors_state is DoorsState.Blocked

    def front_boa_blocked(self):
        return self._front_boa_state is DoorsState.Blocked

    def back_boa_blocked(self):
        return self._back_boa_state is DoorsState.Blocked

    def close_side_doors(self, t):
        if self._side_doors_state is DoorsState.Closed or self._side_doors_state is DoorsState.Closing:
            return
        self._side_doors_state = DoorsState.Closing
        self._side_doors_t0 = t

    def open_side_doors(self, t):
        if self._side_doors_state is DoorsState.Opened or self._side_doors_state is DoorsState.Opening:
            return
        self._side_doors_state = DoorsState.Opening
        self._side_doors_t0 = t

    def close_front_boa(self, t):
        if self._front_boa_state is DoorsState.Closed or self._front_boa_state is DoorsState.Closing:
            return
        self._front_boa_state = DoorsState.Closing
        self._front_boa_t0 = t

    def open_front_boa(self, t):
        if self._front_boa_state is DoorsState.Opened or self._front_boa_state is DoorsState.Opening:
            return
        self._front_boa_state = DoorsState.Opening
        self._front_boa_t0 = t

    def close_back_boa(self, t):
        if self._back_boa_state is DoorsState.Closed or self._back_boa_state is DoorsState.Closing:
            return
        self._back_boa_state = DoorsState.Closing
        self._back_boa_t0 = t

    def open_back_boa(self, t):
        if self._back_boa_state is DoorsState.Opened or self._back_boa_state is DoorsState.Opening:
            return
        self._back_boa_state = DoorsState.Opening
        self._back_boa_t0 = t

    def couple_front(self, w):
        w.couple_back(self)

    def couple_back(self, w):
        train = w.connected
        train.extend(self.connected)
        for wagon in train:
            wagon.connected = train

    def decouple_front(self):
        if len(self.connected) == 1:
            raise Exception('Tried to decouple but only one wagon in train.')
        i = self.connected.index(self)
        if i == len(self.connected) - 1:
            raise Exception('Tried to decouple head wagon front its front.')
        back = self.connected[:i + 1]
        front = self.connected[i + 1:]
        for wagon in back:
            wagon.connected = back
        for wagon in front:
            wagon.connected = front

    ###########################################
    #          Operating Properties           #
    ###########################################

    def is_train_head(self):
        return self is self.get_train_head()

    def get_train_head(self):
        return self.connected[-1]

    def is_stopover_head(self):
        stopover_head_i = self.line.get_next_n_wagons_stopover(self.p_front) - 1
        if stopover_head_i >= len(self.connected):
            return Exception('Cannot stop %d wagons at station %d.' % (stopover_head_i + 1, self.get_next_station_index()))
        return self is self.connected[stopover_head_i]

    def get_ahead(self):
        return self.line.get_ahead(self)

    def get_behind(self):
        return self.line.get_behind(self)

    def get_speed_lim(self):
        return self.line.get_speed_lim(self.p_front)

    def get_next_speed_lim(self):
        return self.line.get_next_speed_lim(self.p_front)

    def get_next_station_index(self):
        return self.line.get_next_station_index(self.p_front)

    def get_next_station_pos(self):
        return self.line.get_next_station_pos(self.p_front)

    def get_now_station_index(self):
        return self.line.get_now_station_index(self.p_front)

    def is_terminus(self):
        return self.get_now_station_index() == self.line.get_terminus_index()

    ###########################################
    #            Run Time Methods             #
    ###########################################

    def update(self, t):
        self.state = self.state.update(t)
        self.state.apply_throttle(t)

        # update physics
        # if self.state,
        # self.p_front, self.dp = rk4()
