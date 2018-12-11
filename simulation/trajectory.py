from numpy import poly1d, roots, empty, isreal
from scipy.optimize import fsolve
from math import isclose, sqrt


def rk4(f, t0, p0, v0, dt):
    a0 = f(t0, 2)
    tf = t0 + dt
    af = f(tf, 2)

    t_mid = t0 + dt / 2
    a_mid = f(t_mid, 2)

    p = p0 + v0 * dt + (a0 + 2 * a_mid) * dt**2 / 6
    v = v0 + (a0 + 4 * a_mid + af) * dt / 6

    return p, v


def get_breaking_dist_time(max_jerk, max_decel, v0, a0=0):
    """Get breaking distance and time from initial velocity and acceleration.
    Negative jerk is first applied until max deceleration is reached. Then
    positive jerk is applied to reach a null velocity with zero acceleration.
    """
    if v0 < 0.01:
        return 0.0, 0.0
    # dejerk_time = (max_decel - a0) / max_jerk
    # dejerk_dist = -max_jerk * dejerk_time**3 / 6 + a0 * dejerk_time**2 / 2 + v0 * dejerk_time
    # v0_decel = -max_jerk * dejerk_time**2 / 2 + a0 * dejerk_time + v0
    # jerk_time = max_decel / max_jerk
    # v0_jerk = max_decel * jerk_time - max_jerk * jerk_time ** 2 / 2
    # jerk_dist = max_jerk * jerk_time**3 / 6 - max_decel * jerk_time**2 / 2 + v0_jerk * jerk_time
    # decel_time = (v0_decel - v0_jerk) / max_decel
    # decel_dist = -max_decel * dejerk_time**2 / 2 + v0_decel * decel_time

    # if decel_time < 0:
    #     raise NotImplementedError
    # return dejerk_dist + decel_dist + jerk_dist, dejerk_time + decel_time + jerk_time

    traj = FastestStop(max_jerk, max_decel, p0=0, v0=v0, a0=a0, t0=0)
    end_time = traj.get_end_time()
    return traj(end_time), end_time


def get_accel_dist_time(max_jerk, max_a, v, v0=0):
    """Get accelerating distance and time.
    Negative jerk is first applied until max deceleration is reached. Then
    positive jerk is applied to reach a null velocity with zero acceleration.
    """
    trajectory = SetVelocity(max_jerk, max_a, v)
    time = trajectory.get_end_time()
    dist = trajectory(time)
    return dist, time


def get_coupling_dist_time(max_jerk, v, v_rdv):
    time = 2 * sqrt((v - v_rdv) / max_jerk)
    return v * time, time


def is_collision_course(a10, v10, p10, a20, v20, p20, max_jerk, max_decel):
    if v10 < v20:
        return False
    br_dist_1, _ = get_breaking_dist_time(max_jerk, max_decel, v10, a10)
    p11 = p10 + br_dist_1
    if p11 < p20:
        return False
    br_dist_2, _ = get_breaking_dist_time(max_jerk, max_decel, v20, a20)
    if p11 < p20 + br_dist_2:
        return False
    print('p11: ', p11, 'p21: ', p20 + br_dist_2)
    print('a10: ', a10, 'v10: ', v10, 'p10: ', p10)
    print('a20: ', a20, 'v20: ', v20, 'p20: ', p20)
    return True


class Trajectory():   # Polynomial defined by parts
    def __init__(self, p0=0, v0=0, a0=0, t0=0):
        self.parts = [(t0, poly1d([a0 / 2, v0, p0]))]

    # def append_polynomial(self, coefs, t0):
    #     self.parts.append((t0, poly1d(coefs)))
    #     self.parts = sorted(self.parts, key=lambda tup: tup[0])

    # def append_trajectory(self, trajectory):
    #     raise NotImplementedError

    def get_end_time(self):
        return self.parts[-1][0]

    def __call__(self, t, d=0):
        if len(self.parts) == 1:
            return self.parts[0][1].deriv(d)(t - self.parts[-1][0])
        for i in range(len(self.parts) - 1):
            if t < self.parts[i + 1][0]:  # t before next part
                return self.parts[i][1].deriv(d)(t - self.parts[i][0])
        return self.parts[-1][1].deriv(d)(t - self.parts[-1][0])

    def extend(self, f):
        if not isinstance(f, Trajectory):
            raise TypeError('Expected a Trajectory')
        if f.parts[0][0] <= self.parts[-1][0]:
            raise ValueError(
                'Extended trajectory starts before current trajectory ended.')
        self.parts.extend(f.parts)
        return self

    def real_roots(self, p=0, d=0):
        ts = []
        for i, tup in enumerate(self.parts):
            t0 = tup[0]
            poly = tup[1]
            roots = (poly.deriv(d) - p).roots
            real_roots = [root for root in roots if isreal(root)]
            for root in sorted(real_roots):
                t = root + t0
                if t < t0:
                    continue
                if i < len(self.parts) - 1 and t > self.parts[i + 1][0]:
                    continue
                ts.append(t)
        return ts


class FastestStop(Trajectory):
    def __init__(self, max_jerk, max_decel, p0=0, v0=0, a0=0, t0=0):
        """
        P1: Jerk down
        P2: Deccel max
        P3: Jerk up
        """
        t1 = (a0 + max_decel) / max_jerk
        t3 = max_decel / max_jerk
        t2 = (max_jerk * t3**2 / 2 - max_decel * t3 -
              max_jerk * t1**2 / 2 + a0 * t1 + v0) / max_decel
        if t2 < 0:
            # print('breaking does not saturate')
            t2 = sqrt((a0 / max_jerk)**2 / 2 + v0 / max_jerk)
            t1 = t2 + a0 / max_jerk
            a20 = -max_jerk * t1 + a0
            v20 = -max_jerk * t1**2 / 2 + a0 * t1 + v0
            p20 = -max_jerk * t1**3 / 6 + a0 * t1**2 / 2 + v0 * t1 + p0
            p30 = max_jerk * t2**3 / 6 + a20 * t2**2 / 2 + v20 * t2 + p20
            t01 = t0
            t02 = t01 + t1
            t03 = t02 + t2
            self.parts = [
                (t01, poly1d([-max_jerk / 6, a0 / 2, v0, p0])),
                (t02, poly1d([max_jerk / 6, a20 / 2, v20, p20])),
                (t03, poly1d([p30])),
            ]

        else:
            # print('breaking saturates')
            v20 = -max_jerk * t1**2 / 2 + a0 * t1 + v0
            v30 = -max_decel * t2 + v20
            p20 = -max_jerk * t1**3 / 6 + a0 * t1**2 / 2 + v0 * t1 + p0
            p30 = -max_decel * t2**2 / 2 + v20 * t2 + p20
            p40 = max_jerk * t3**3 / 6 - max_decel * t3**2 / 2 + v30 * t3 + p30

            t01 = t0
            t02 = t01 + t1
            t03 = t02 + t2
            t04 = t03 + t3
            self.parts = [
                (t01, poly1d([-max_jerk / 6, a0 / 2, v0, p0])),
                (t02, poly1d([-max_decel / 2, v20, p20])),
                (t03, poly1d([max_jerk / 6, -max_decel / 2, v30, p30])),
                (t04, poly1d([p40])),
            ]


class SetAccel(Trajectory):
    def __init__(self, max_jerk, a, p0=0, v0=0, a0=0, t0=0):
        if a < a0:
            max_jerk = -abs(max_jerk)
        self.parts = [(t0, poly1d([max_jerk / 6, a0 / 2, v0, p0]))]
        jerk_time = (a - a0) / max_jerk
        c2 = a / 2
        c1 = self.parts[0][1].deriv(1)(jerk_time)
        c0 = self.parts[0][1](jerk_time)
        self.parts.append((t0 + jerk_time, poly1d([c2, c1, c0])))


class SetVelocity(Trajectory):
    def __init__(self, max_jerk, max_a, v, p0=0, v0=0, a0=0, t0=0):
        if v < v0:
            max_jerk = - abs(max_jerk)
            max_a = - abs(max_a)

        self.parts = [(t0, poly1d([max_jerk / 6, a0 / 2, v0, p0]))]

        jerk_time = (max_a - a0) / max_jerk

        p0_a_const = self.parts[0][1](jerk_time)
        v0_a_const = self.parts[0][1].deriv(1)(jerk_time)
        a0_a_const = max_a
        self.parts.append(
            (t0 + jerk_time, poly1d([a0_a_const / 2, v0_a_const, p0_a_const])))

        dejerk_time = max_a / max_jerk
        a_const_time = (v + max_jerk * dejerk_time**2 / 2 -
                        v0_a_const) / max_a - dejerk_time
        if a_const_time > 0:
            p0_dejerk = self.parts[1][1](a_const_time)
            v0_dejerk = self.parts[1][1].deriv(1)(a_const_time)
            a0_dejerk = self.parts[1][1].deriv(2)(a_const_time)
            self.parts.append((t0 + jerk_time + a_const_time,
                               poly1d([-max_jerk / 6, a0_dejerk / 2, v0_dejerk, p0_dejerk])))
            self.parts.append((t0 + jerk_time + a_const_time + dejerk_time,
                               poly1d([self.parts[2][1].deriv(1)(dejerk_time),
                                       self.parts[2][1](dejerk_time)])))
        else:   # Cannot saturate accel
            self.parts = [(t0, poly1d([max_jerk / 6, a0 / 2, v0, p0]))]

            tc = max(roots([max_jerk, 3 * a0 / 2, a0**2 / (2 * max_jerk) + v0 - v]))
            p0_dejerk = self.parts[0][1](tc)
            v0_dejerk = self.parts[0][1].deriv(1)(tc)
            a0_dejerk = self.parts[0][1].deriv(2)(tc)

            self.parts.append(
                (t0 + tc, poly1d([-max_jerk / 6, a0_dejerk / 2, v0_dejerk, p0_dejerk])))

            dejerk_time = a0_dejerk / max_jerk
            self.parts.append((t0 + tc + dejerk_time,
                               poly1d([self.parts[1][1].deriv(1)(dejerk_time),
                                       self.parts[1][1](dejerk_time)])))


class AjustVelocity(Trajectory):
    """AjustVelocity
    Keep current velocity then ajusts velocity to reach v at point p
    with zero accel.
    """

    def __init__(self, max_jerk, max_a, v, p, p0=0, v0=0, a0=0, t0=0):
        if abs(a0) > 0.5:
            raise NotImplementedError('a0 = %f' % a0)
        self.parts = [(t0, poly1d([v0, p0]))]
        if v < v0:
            max_a = -abs(max_a)
            max_jerk = - abs(max_jerk)

        if abs(v - v0) > max_a**2 / max_jerk:
            # print('saturates')
            # Acceleration saturates
            t1 = t3 = max_a / max_jerk
            t2 = -t3 + (v - v0) / max_a
            v20 = max_jerk * t1**2 / 2 + v0
            v30 = max_a * t2 + v20
            d1 = max_jerk * t1**3 / 6 + v0 * t1
            d2 = max_a * t2**2 / 2 + v20 * t2
            d3 = -max_jerk * t3**3 / 6 + max_a * t3**2 / 2 + v30 * t3
            p0_accel = p - d3 - d2 - d1
            p10 = p0_accel
            p20 = p10 + d1
            p30 = p20 + d2
            p40 = p30 + d3
            t0_accel = (p0_accel - p0) / v0 + t0
            t01 = t0_accel
            t02 = t01 + t1
            t03 = t02 + t2
            t04 = t03 + t3
            self.parts.extend([
                (t01, poly1d([max_jerk / 6, 0, v0, p10])),
                (t02, poly1d([max_a / 2, v20, p20])),
                (t03, poly1d([-max_jerk / 6, max_a / 2, v30, p30])),
                (t04, poly1d([v, p40])),
            ])
        else:
            # print('no saturation')
            t1 = t2 = sqrt((v - v0) / max_jerk)
            a20 = max_jerk * t1
            v20 = max_jerk * t1**2 / 2 + v0
            d1 = max_jerk * t1**3 / 6 + v0 * t1
            d2 = -max_jerk * t2**3 / 6 + a20 * t2**2 / 2 + v20 * t2
            p0_accel = p - d2 - d1
            p10 = p0_accel
            p20 = p10 + d1
            p30 = p20 + d2
            t0_accel = (p0_accel - p0) / v0 + t0
            t01 = t0_accel
            t02 = t01 + t1
            t03 = t02 + t2
            self.parts.extend([
                (t01, poly1d([max_jerk / 6, 0, v0, p10])),
                (t02, poly1d([-max_jerk / 6, a20 / 2, v20, p20])),
                (t03, poly1d([v, p30])),
            ])


class StopAtPosition(Trajectory):
    """Continue at current speed, then slow down to stop at wanted position.
    """

    def __init__(self, max_jerk, max_a, p, p0=0, v0=None, a0=0, t0=0):
        max_jerk = abs(max_jerk)
        max_a = abs(max_a)
        self.parts = []

        if p < p0:
            raise NotImplementedError(
                'Backwards StopAtPosition not implemented. p has to be greater than p0.')

        speed_lower_lim = 0.03
        if v0 is None or v0 < speed_lower_lim:
            raise ValueError('v0 cannot be smaller than %f m/s for StopAtPosition.')

        if isclose(a0, 0.0, abs_tol=0.00001):
            t0_v_const = t0
            p0_v_const = p0
            v0_v_const = v0
        else:   # Stop accelerating/decelerating
            dejerk_time = abs(a0 / max_jerk)
            if a0 > 0:
                self.parts.append((t0, poly1d([-max_jerk / 6, a0 / 2, v0, p0])))
            else:
                self.parts.append((t0, poly1d([max_jerk / 6, a0 / 2, v0, p0])))
            t0_v_const = t0 + dejerk_time
            p0_v_const = self.parts[0][1](dejerk_time)
            v0_v_const = self.parts[0][1].deriv(1)(dejerk_time)

        self.parts.append((t0_v_const, poly1d([v0_v_const, p0_v_const])))

        dejerk_time = max_a / max_jerk
        dejerk_distance = -max_jerk / 6 * dejerk_time**3 + v0_v_const * dejerk_time


class GoToPV(Trajectory):
    def __init__(self, max_jerk, max_a, max_v, v, p, p0=0, v0=0, a0=0, t0=0):
        """
        P1: Jerk up
        P2: Accel max
        P3: Jerk down
        P4: Velocity max
        P5: Jerk down
        P6: Deccel max
        P7: Jerk up
        """
        t01 = t0

        # try saturate speed
        t1 = (max_a - a0) / max_jerk
        t02 = t01 + t1
        p20 = max_jerk * t1**3 / 6 + a0 * t1 ** 2 / 2 + v0 * t1 + p0
        v20 = max_jerk * t1**2 / 2 + a0 * t1 + v0
        t3 = max_a / max_jerk

        v30 = max_v + max_jerk * t3**2 / 2 - max_a * t3
        t2 = (v30 - v20) / max_a
        if t2 < 0:
            accel_sat = False
            racines = roots([max_jerk, 2 * a0, a0**2 / (2 * max_jerk) + v0 - max_v])
            t1 = max(racines)
            t3 = t1 + a0 / max_jerk
            p30 = max_jerk * t1**3 / 6 + a0 * t1**2 / 2 + v0 * t1 + p0
            v30 = max_jerk * t1**2 / 2 + a0 * t1 + v0
            a30 = max_jerk * t1 + a0
            t03 = t01 + t1
            t04 = t03 + t3
        else:
            accel_sat = True
            p30 = max_a * t2**2 / 2 + v20 * t2 + p20
            a30 = max_a
            t03 = t02 + t2
            t04 = t03 + t3
        p40 = -max_jerk * t3**3 / 6 + max_a * t3**2 / 2 + v30 * t3 + p30
        # v40 = -max_jerk * t3**2 / 2 + max_a*t3 + v30
        t5 = max_a / max_jerk
        v60 = -max_jerk * t5**2 / 2 + max_v
        t7 = max_a / max_jerk
        v70 = v - max_jerk * t7**2 / 2 + max_a * t7
        t6 = (v60 - v70) / max_a
        if t6 < 0:
            decel_sat = False
            t5 = sqrt((max_v - v) / max_jerk)
            t7 = t5
            v70 = -max_jerk * t5**2 / 2 + max_v
            a70 = -max_jerk * t5
            t4 = (p - max_jerk * t7**3 / 6 + a70 * t7**2 / 2 - v70 *
                  t7 + max_jerk * t5**3 / 6 - max_v * t5 - p40) / max_v
            p50 = max_v * t4 + p40
            p70 = -max_jerk * t5**3 / 6 + max_v * t5 + p50
            t05 = t04 + t4
            t07 = t05 + t7
        else:
            decel_sat = True
            a70 = - max_a
            t4 = (p - max_jerk * t7**3 / 6 + max_a * t7**2 / 2 - v70 * t7 + max_a *
                  t6**2 / 2 - v60 * t6 + max_jerk * t5**3 / 6 - max_v * t5 - p40) / max_v
            p50 = max_v * t4 + p40
            p60 = -max_jerk * t5**3 / 6 + max_v * t5 + p50
            p70 = -max_a * t6**2 / 2 + v60 * t6 + p60
            t05 = t04 + t4
            t06 = t05 + t5
            t07 = t06 + t6
        if t4 < 0:
            v_sat = False
            accel_sat = True
            decel_sat = True
            t1 = (max_a - a0) / max_jerk
            v20 = max_jerk * t1**2 / 2 + a0 * t1 + v0
            p20 = max_jerk * t1**3 / 6 + a0 * t1**2 / 2 + v0 * t1 + p0
            t3 = 2 * max_a / max_jerk
            a30 = max_a
            a60 = -max_a
            t7 = max_a / max_jerk
            a70 = -max_a

            def f(T):
                nonlocal p30, p60, p70, v30, v60, v70
                t2 = T[0]
                t6 = T[1]
                F = empty((2))
                v30 = max_a * t2 + v20
                v60 = -max_jerk * t3**2 / 2 + max_a * t3 + v30
                v70 = -max_a * t6 + v60
                F[0] = max_jerk * t7**2 / 2 - max_a * t7 + v70 - v
                p30 = max_a * t2**2 / 2 + v20 * t2 + p20
                p60 = -max_jerk * t3**3 / 6 + max_a * t3**2 / 2 + v30 * t3 + p30
                p70 = -max_a * t6**2 / 2 + v60 * t6 + p60
                F[1] = max_jerk * t7**3 / 6 - max_a * t7**2 / 2 + v70 * t7 + p70 - p
                return F
            t2, t6 = fsolve(f, [1.0, 1.0])
            print('t2: ', t2, 't6: ', t6)
            t02 = t01 + t1
            t03 = t02 + t2
            t06 = t03 + t3
            t07 = t06 + t6

            if t2 < 0:  # try saturate only decel
                accel_sat = False
                t7 = max_a / max_jerk
                a70 = -max_a
                a60 = -max_a

                def f(T):
                    nonlocal a30, v30, v60, p30, p60, p70, t3
                    F = empty((2))
                    t1 = T[0]
                    t6 = T[1]
                    t3 = t1 + (a0 + max_a) / max_jerk
                    a30 = max_jerk * t1 + a0
                    v30 = max_jerk * t1**2 / 2 + a0 * t1 + v0
                    v60 = -max_jerk * t3**2 / 2 + a30 * t3 + v30
                    v70 = - max_a * t6 + v60
                    F[0] = max_jerk * t7**2 / 2 - max_a * t7 + v70 - v
                    p30 = max_jerk * t1**3 / 6 + a0 * t1**2 / 2 + v0 * t1 + p0
                    p60 = -max_jerk * t3**3 / 6 + a30 * t3**2 / 2 + v30 * t3 + p30
                    p70 = - max_a * t6**2 / 2 + v60 * t6 + p60
                    F[1] = max_jerk * t7**3 / 6 - max_a * t7**2 / 2 + v70 * t7 + p70 - p
                    return F
                t1, t6 = fsolve(f, [1.0, 1.0])
                print('t1: ', t1, 't6: ', t6)
                t03 = t01 + t1
                t06 = t03 + t3
                t07 = t06 + t6

                if t6 < 0:  # Both accel and decel not saturated
                    accel_sat = False
                    decel_sat = False

                    def f(T):
                        nonlocal p30, p70, v30, v70, a30, a70
                        F = empty((3))
                        a30 = max_jerk * T[0] + a0
                        a70 = -max_jerk * T[1] + a30
                        F[0] = max_jerk * T[2] + a70  # a71 = 0
                        v30 = max_jerk * T[0]**2 / 2 + a0 * T[0] + v0
                        v70 = -max_jerk * T[1]**2 / 2 + a30 * T[1] + v30
                        F[1] = max_jerk * T[2]**2 / 2 + a70 * T[2] + v70 - v  # v71 = v
                        p30 = max_jerk * T[0]**3 / 6 + a0 * T[0]**2 / 2 + v0 * T[0] + p0
                        p70 = -max_jerk * T[1]**3 / 6 + a30 * \
                            T[1]**2 / 2 + v30 * T[1] + p30
                        F[2] = max_jerk * T[2]**3 / 6 + a70 * T[2]**2 / \
                            2 + v70 * T[2] + p70 - p  # p71 = p
                        print(F)
                        return F
                    t1, t3, t7 = fsolve(f, [1.0, 1.0, 1.0])
                    if any(t < 0 for t in [t1, t2, t3]):
                        raise ValueError('Could not solve.')
                    t03 = t01 + t1
                    t07 = t03 + t3
            elif t6 < 0:  # try saturate only accel
                decel_sat = False
                t1 = (max_a - a0) / max_jerk
                a30 = max_a
                v20 = max_jerk * t1**2 / 2 + a0 * t1 + v0

                def f(T):
                    nonlocal t7, a70, v30, v70, p20, p30, p70
                    F = empty((2))
                    t2 = T[0]
                    t3 = T[1]
                    t7 = t3 - max_a / max_jerk
                    a70 = -max_jerk * t3 + max_a
                    v30 = max_a * t2 + v20
                    v70 = -max_jerk * t3**2 / 2 + max_a * t3 + v30
                    F[0] = max_jerk * t7**2 / 2 + a70 * t7 + v70 - v
                    p20 = max_jerk * t1**3 / 6 + a0 * t1**2 / 2 + v0 * t1 + p0
                    p30 = max_a * t2**2 / 2 + v20 * t2 + p20
                    p70 = -max_jerk * t3**3 / 6 + max_a * t3**2 / 2 + v30 * t3 + p30
                    F[1] = max_jerk * t7**3 / 6 + a70 * t7**2 / 2 + v70 * t7 + p70 - p
                    return F
                t2, t3 = fsolve(f, [1.0, 1.0])
                print('t1: ', t1, 't3: ', t3)
                t02 = t01 + t1
                t03 = t02 + t2
                t07 = t03 + t3
            else:
                accel_sat = True
                decel_sat = True

        else:
            v_sat = True

        t08 = t07 + t7

        # print('Accel sat: ', accel_sat)
        # print('Velocity sat: ', v_sat)
        # print('Decel sat: ', decel_sat)

        self.parts = [(t01, poly1d([max_jerk / 6, a0 / 2, v0, p0]))]
        if accel_sat:
            self.parts.append((t02, poly1d([max_a / 2, v20, p20])))
        self.parts.append((t03, poly1d([-max_jerk / 6, a30 / 2, v30, p30])))
        if v_sat:
            self.parts.extend([
                (t04, poly1d([max_v, p40])),
                (t05, poly1d([-max_jerk / 6, 0.0, max_v, p50]))])
        if decel_sat:
            self.parts.append((t06, poly1d([-max_a / 2, v60, p60])))
        self.parts.extend([
            (t07, poly1d([max_jerk / 6, a70 / 2, v70, p70])),
            (t08, poly1d([v, p]))])
