"""
This module (along with its imports) represents the brain of the wagons.
It uses state machines which can act on the controls of the wagons via
actions applied at creation of the state. The state updates itself via its
update method which returns the state after the update.
"""
import trajectory
from params import *


class State:
    def __init__(self, wagon, t, **kwargs):
        self.wagon = wagon
        self.line = wagon.line
        self.t0 = t
        for key, value in kwargs.items():
            setattr(self, key, value)
        self.action()

    def action(self):
        """Happening once, automatically called by __init__"""
        pass

    def update(self, t):
        """Called for every updates. Returns the State after update. (self if unchanged)"""
        raise NotImplementedError

    def apply_throttle(self, t):
        self.wagon.p_front = self.wagon.trajectory(t)
        self.wagon.dp = self.wagon.trajectory(t, 1)
        self.wagon.ddp = self.wagon.trajectory(t, 2)

    def is_collision_course(self):
        behind = self.wagon
        ahead = self.wagon.get_ahead()
        if ahead is None:
            return False

        a10 = behind.ddp
        v10 = behind.dp
        p10 = behind.p_front
        a20 = ahead.ddp
        v20 = ahead.dp
        p20 = ahead.p_back - breaking_marjin
        return trajectory.is_collision_course(a10, v10, p10, a20, v20, p20, max_jerk, deccel_max)


class EmergencyStop(State):
    """Emergency stop. Vehicle sets brakes and closes all doors.
    This state is only used for the head wagon, others are in Tail state.
    """

    def action(self):
        print('Emergency stop! Time: %03.2f, position: %05.2f' %
              (self.t0, self.wagon.p_front))
        print('ahead is: ', self.wagon.get_ahead().state.__class__.__name__)
        p0 = self.wagon.p_front
        v0 = self.wagon.dp
        a0 = self.wagon.ddp
        self.wagon.trajectory = trajectory.FastestStop(
            max_jerk, deccel_max, p0, v0, a0, self.t0)

    def update(self, t):
        return self


###############################################################################
#                             Beginning of Line                               #
###############################################################################
"""Beginnig of the line
Trains are first at the hangar, then go to the first station, embark people
and leave if no train ahead or the train ahead is far enough.
"""


class TrainReadyHangar(State):
    """N wagons coupled and ready in end of the line hangar.
    This state is only used for the head wagon, others are in Tail state.
    """

    def action(self):
        self.wagon.trajectory = trajectory.Trajectory(
            p0=self.wagon.p_front, v0=0, a0=0, t0=self.t0)

    def update(self, t):
        ahead = self.wagon.get_ahead()
        if ahead is None:
            return GoToFirstStation(self.wagon, t)
        ahead_gone = ahead.p_back > self.line.get_station_pos(0)
        min_time_elapsed = t - self.t0 > min_wait_time_first_train
        if ahead_gone and min_time_elapsed:
            return GoToFirstStation(self.wagon, t)
        return self


class GoToFirstStation(State):
    """Train leaving hangar for first station for embarking.
    This state is only used for the head wagon, others are in Tail state
    """

    def action(self):
        self.wagon.open_back_boa(self.t0)
        for w in self.wagon.connected[1:-1]:
            w.open_front_boa(self.t0)
            w.open_back_boa(self.t0)
        self.wagon.connected[0].open_front_boa(self.t0)
        next_station = self.wagon.get_now_station_index() + 1
        self.station_pos = self.line.get_station_pos(next_station)
        max_v = self.wagon.get_speed_lim()
        a0 = self.wagon.ddp
        v0 = self.wagon.dp
        p0 = self.wagon.p_front
        self.wagon.trajectory = trajectory.GoToPV(max_jerk, accel_max, max_v,
                                                  v=0, p=self.station_pos,
                                                  p0=p0, v0=v0, a0=a0, t0=self.t0)

    def update(self, t):
        if self.is_collision_course():
            return EmergencyStop(self.wagon, t)
        speed_ok = self.wagon.dp < open_max_speed
        pos_ok = self.station_pos - self.wagon.p_front < max_dist_open_doors
        if speed_ok and pos_ok:
            return EmbarkingFirstStation(self.wagon, t)
        return self


class EmbarkingFirstStation(State):
    """Embarking and to train expected to rendez vous with.
    """

    def action(self):
        self.wagon.trajectory = trajectory.Trajectory(p0=self.wagon.p_front, t0=self.t0)
        for w in self.wagon.connected:
            w.open_side_doors(self.t0)

    def update(self, t):
        embarking_rdy = t - self.t0 > min_time_side_doors_open
        line_rdy = self.line.last_departure is None or t - \
            self.line.last_departure > self.line.min_interval - side_doors_closing_time
        if embarking_rdy and line_rdy:
            return ClosingDoorsFirstStation(self.wagon, t)
        return self


class ClosingDoorsFirstStation(State):
    def action(self):
        for w in self.wagon.connected:
            w.close_side_doors(self.t0)

    def update(self, t):
        if all([w.side_doors_closed(t) for w in self.wagon.connected]):
            self.line.last_departure = t
            return LeaveFirstStation(self.wagon, t)
        return self


class LeaveFirstStation(State):
    def action(self):
        station = self.wagon.get_now_station_index()
        max_v = self.line.speed_lim[station]
        p0 = self.wagon.p_front
        v0 = self.wagon.dp
        a0 = self.wagon.ddp
        self.wagon.trajectory = trajectory.SetVelocity(
            max_jerk, accel_max, max_v, p0, v0, a0, self.t0)

    def update(self, t):
        # if self.wagon.dp >= self.wagon.get_speed_lim():
        #     return Transit(self.wagon, t)
        if t > self.wagon.trajectory.get_end_time() + 5:
            return Transit(self.wagon, t)
        return self


###############################################################################
#                                 Transit                                     #
###############################################################################
"""Transit
 A wagon in the "never stopping" mode is "in transit". A wagon becomes in
 transit when it couples with the front of another train of when it leaves the
 first station. The front wagon is responsible for the trajectory. It therefor
 has to check for collisions or delays. If it is in the N last wagon of the
 train it checks for the next station to stop to.
"""


class Transit(State):

    def action(self):
        self.set_trajectory(self.t0)

    def set_trajectory(self, t):
        self.next_station = self.wagon.get_next_station_index()
        # max_v = self.wagon.get_speed_lim()
        p0 = self.wagon.p_front
        v0 = self.wagon.dp
        a0 = self.wagon.ddp
        p = self.wagon.get_next_station_pos() - dist_before_station_set_velocity
        v = self.wagon.get_next_speed_lim()
        # print('updating trajectory v0 = %02.3f, v = %02.3f, p0 = %05.1f, p = %05.1f' %
        #       (v0, v, p0, p))
        # print('Next station: ', self.next_station)
        try:
            self.wagon.trajectory = trajectory.AjustVelocity(max_freestanding_jerk, max_freestanding_accel,
                                                             v=v, p=p, p0=p0, v0=v0, a0=a0, t0=t)
        except NotImplementedError as e:
            print('in transit')
            print(self.wagon.p_front)
            raise e

    def stopover(self, t):
        if not self.wagon.is_stopover_head():
            return False    # Not stopover head
        behind = self.wagon.connected[0].get_behind()
        if behind is not None:
            if isinstance(behind.state, Decoupling):
                return False    # Behind just decoupled
            next_station = self.wagon.get_next_station_index()
            if isinstance(behind.state, StopingAtNextStation) and behind.get_next_station_index() == next_station:
                return False    # Behind already stops at next station
        station_pos = self.wagon.get_next_station_pos()
        stop_time = t + boa_closing_time + decoupling_time
        p0 = self.wagon.trajectory(stop_time)
        v0 = self.wagon.trajectory(stop_time, 1)
        a0 = self.wagon.trajectory(stop_time, 2)
        dist, _ = trajectory.get_breaking_dist_time(
            max_jerk, deccel_nom, v0=v0, a0=a0)
        if p0 + dist >= station_pos:
            return True     # Time to close doors
        else:
            return False

    def update(self, t):
        if not self.wagon.is_train_head():
            return Tail(self.wagon, t)
        ahead = self.wagon.get_ahead()
        if ahead is not None and not (isinstance(ahead.state, RendezVousMeet) or isinstance(ahead.state, Coupling)):
            if self.is_collision_course():
                return EmergencyStop(self.wagon, t)
        if self.stopover(t) or self.wagon.get_next_station_index() == self.line.get_terminus_index():
            return StopingAtNextStation(self.wagon, t)

        if self.next_station != self.wagon.get_next_station_index():
            self.set_trajectory(t)

        return self


class Delayed(State):
    def action(self):
        raise NotImplementedError('Delayed state not implemented')

    def update(self, t):
        return self


class Tail(State):
    def action(self):
        self.wagon.trajectory = None

    def decouple(self, t):
        if not self.wagon.is_stopover_head():
            return False    # Not stopover head
        next_station = self.wagon.get_next_station_index()
        if next_station == self.line.get_terminus_index():
            return False
        behind = self.wagon.connected[0].get_behind()
        if behind is not None:
            if isinstance(behind.state, Decoupling):
                return False    # Behind just decoupled
            if isinstance(behind.state, StopingAtNextStation) and behind.get_next_station_index() == next_station:
                return False    # Behind already stops at next station
        station_pos = self.wagon.get_next_station_pos()
        stop_time = t + boa_closing_time + decoupling_time
        train_head = self.wagon.get_train_head()
        p0 = train_head.trajectory(stop_time)
        v0 = train_head.trajectory(stop_time, 1)
        a0 = train_head.trajectory(stop_time, 2)
        dist, _ = trajectory.get_breaking_dist_time(
            max_jerk, deccel_nom, v0=v0, a0=a0)
        if p0 + dist >= station_pos:
            return True     # Time to close doors
        else:
            return False

    def update(self, t):
        if self.wagon.get_next_station_index() in [0, 1]:
            return self
        if self.decouple(t):
            return ClosingBoaDoors(self.wagon, t)

        return self

    def apply_throttle(self, t):
        head = self.wagon.get_train_head()
        offset = len(self.wagon.connected[self.wagon.connected.index(
            self.wagon) + 1:]) * longeur_wagon
        self.wagon.p_front = head.p_front - offset
        self.wagon.dp = head.dp
        self.wagon.ddp = head.ddp


###############################################################################
#                                  Stopover                                   #
###############################################################################
"""Stopover
 A wagon in transit among the N last wagon of the train eventually decouples
 from the train (if any wagons continue) and stops at a station to embark
 people.
"""


class ClosingBoaDoors(Tail):
    def action(self):
        ahead = self.wagon.get_ahead()
        self.wagon.close_front_boa(self.t0)
        ahead.close_back_boa(self.t0)

    def update(self, t):
        ahead = self.wagon.get_ahead()
        if self.wagon.front_boa_closed(t) and ahead.back_boa_closed(t):
            return Decoupling(self.wagon, t)
        if self.wagon.front_boa_blocked() or ahead.back_boa_blocked():
            raise NotImplementedError('Boa doors blocked handling not implemented.')
        return self


class Decoupling(State):

    def action(self):
        self.wagon.decouple_front()
        p0 = self.wagon.p_front
        v0 = self.wagon.dp
        a0 = 0
        self.wagon.trajectory = trajectory.Trajectory(p0=p0, v0=v0, a0=a0, t0=self.t0)

    def update(self, t):
        if t - self.t0 >= decoupling_time:
            return StopingAtNextStation(self.wagon, t)
        return self


class StopingAtNextStation(State):
    """Wagon(s) stopping at a station.
    This state is only used for the (now) head wagon, others are in Tail state."""

    def action(self):
        self.station_pos = self.wagon.get_next_station_pos()
        # max_v = self.wagon.get_speed_lim()
        a0 = self.wagon.ddp
        v0 = self.wagon.dp
        p0 = self.wagon.p_front
        self.wagon.trajectory = trajectory.AjustVelocity(max_jerk, deccel_nom, v=0,
                                                         p=self.station_pos, p0=p0, v0=v0, a0=a0, t0=self.t0)

    def update(self, t):
        if not self.wagon.is_train_head():
            return Tail(self.wagon, t)
        ahead = self.wagon.get_ahead()
        if ahead is not None and isinstance(ahead.get_train_head().state, EmergencyStop):
            return EmergencyStop(self.wagon, t)
        speed_ok = self.wagon.dp < open_max_speed
        pos_ok = self.wagon.p_front - self.station_pos < max_dist_open_doors
        if speed_ok and pos_ok:
            if self.wagon.is_terminus():
                return Terminus(self.wagon, t)
                # raise NotImplementedError('Terminus not implemented yet!')
            return Embarking(self.wagon, t)
        return self


class Embarking(State):
    def action(self):
        self.station = self.wagon.get_now_station_index()
        for w in self.wagon.connected:
            w.open_side_doors(self.t0)

    def update(self, t):
        if self.ready_to_leave(t):
            return ClosingSideDoors(self.wagon, t)
        return self

    def ready_to_leave(self, t):
        behind = self.wagon.connected[0].get_behind()
        if behind is None:
            return False
        if behind.p_front < self.line.get_station_pos(self.station - 1) + 10:
            return False
        v_lim = self.line.speed_lim[self.station]
        v_rdv = v_lim - delta_rv_speed
        accel_dist, accel_time = trajectory.get_accel_dist_time(
            max_jerk, accel_max, v_rdv)

        t_rdv = behind.trajectory.real_roots(p=self.wagon.p_back + accel_dist)[0]
        close_door_time = t_rdv - time_at_rv_speed - accel_time - \
            side_doors_closing_time - side_doors_standby_time
        if t >= close_door_time:
            if abs(behind.trajectory(t_rdv, 1) - v_lim) > wrong_velocity_threshold:
                raise Exception('Wrong velocity for rendez vous (%f m/s, should be %f m/s). Station: %i' %
                                (behind.trajectory(t_rdv, 1), v_lim, self.station))
            return True
        else:
            return False


###############################################################################
#                                Rendez-Vous                                  #
###############################################################################
"""When a wagon has been to a station long enough and a train arrives, it
 closes the side doors and accelerate to a rendez-vous speed and couples with
 the front of the train.
"""


class ClosingSideDoors(State):
    def action(self):
        for w in self.wagon.connected:
            w.close_side_doors(self.t0)

    def update(self, t):
        if all([w.side_doors_closed(t) for w in self.wagon.connected]):
            return RendezVousAccel(self.wagon, t)
        if any([w.side_doors_blocked() for w in self.wagon.connected]):
            return Delayed(self.wagon, t)
        return self


class RendezVousAccel(State):

    def action(self):
        station = self.wagon.get_now_station_index()
        if station is None:
            raise Exception('Wagon seems not to be at a station.')
        behind = self.wagon.connected[0].get_behind()
        v_lim = self.line.speed_lim[station]
        v_rdv = v_lim - delta_rv_speed
        accel_dist, accel_time = trajectory.get_accel_dist_time(
            max_jerk, accel_max, v_rdv)

        meet_point = self.wagon.p_back + accel_dist + v_rdv * time_at_rv_speed
        t_rdv = behind.trajectory.real_roots(p=meet_point)[0]
        if abs(behind.trajectory(t_rdv, 1) - v_lim) > wrong_velocity_threshold:
            raise Exception('Wrong velocity for rendez vous. Station: %i' % self.station)

        leave_time = t_rdv - time_at_rv_speed - accel_time
        self.wagon.trajectory = trajectory.Trajectory(p0=self.wagon.p_front, v0=0, a0=0, t0=self.t0).extend(
            trajectory.SetVelocity(max_jerk, accel_max, v_rdv, p0=self.wagon.p_front, t0=leave_time))

    def update(self, t):
        if self.is_collision_course():
            return EmergencyStop(self.wagon, t)
        if t > self.wagon.trajectory.get_end_time():
            return RendezVousMeet(self.wagon, t)
        return self


class RendezVousMeet(State):
    def action(self):
        self.behind = self.wagon.connected[0].get_behind()
        v_lim = self.wagon.get_speed_lim()
        v_rdv = v_lim - delta_rv_speed
        accel_dist, accel_time = trajectory.get_coupling_dist_time(
            max_jerk, v_lim, v_rdv)
        p_front = self.behind.p_front
        p_back = self.wagon.connected[0].p_back
        t_accel = ((accel_dist + p_back - p_front - v_lim *
                    accel_time) / delta_rv_speed)
        p_meet = accel_dist + v_rdv * t_accel + self.wagon.p_front
        p0 = self.wagon.p_front
        v0 = self.wagon.dp
        # a0 = self.wagon.ddp
        a0 = 0
        self.wagon.trajectory = trajectory.AjustVelocity(
            max_jerk, accel_max, v_lim, p_meet, p0, v0, a0, self.t0)

    def update(self, t):
        if self.is_collision_course():
            return EmergencyStop(self.wagon, t)
        if self.wagon.p_back - self.behind.p_front <= min_dist_couple:
            return Coupling(self.wagon, t)
        return self


class Coupling(State):

    def update(self, t):
        if self.is_collision_course():
            return EmergencyStop(self.wagon, t)
        if t - self.t0 > coupling_time:
            self.wagon.connected[0].open_back_boa(t)
            behind = self.wagon.connected[0].get_behind()
            behind.open_front_boa(t)
            self.wagon.couple_back(behind)
            return Transit(self.wagon, t)
        return self


###############################################################################
#                                End of Line                                  #
###############################################################################
"""
"""


class Terminus(State):

    def action(self):
        for w in self.wagon.connected:
            w.open_side_doors(self.t0)

    def update(self, t):
        if self.ready_to_leave(t):
            return ClosingSideDoorsTerminus(self.wagon, t)
        return self

    def ready_to_leave(self, t):
        return False


class ClosingSideDoorsTerminus(State):
    def action(self):
        raise NotImplementedError

    def update(self, t):
        raise NotImplementedError
