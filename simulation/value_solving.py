from math import sqrt
import matplotlib.pyplot as plt
from params import *
import numpy as np


def plot_breaking():
    import trajectory

    t0 = 545
    dt = 0.1

    p0 = 15000
    v0 = 15
    a0 = 1.2

    f = trajectory.FastestStop(max_jerk, deccel_max, p0=p0, v0=v0, a0=a0, t0=t0)

    t_end = f.parts[-1][0]
    t = np.arange(t0, t_end + 5, dt)

    plt.figure(1)
    p = [f(x) for x in t]
    v = [f(x, 1) for x in t]
    a = [f(x, 2) for x in t]
    j = [f(x, 3) for x in t]

    print([tup[0] for tup in f.parts])

    print('end position: ', f(t_end - 0.1))
    print('end velocity: ', f(t_end - 0.1, 1))

    plt.subplot(4, 1, 1)
    plt.plot(t, p)
    plt.ylabel('position (m)')

    plt.subplot(4, 1, 2)
    plt.plot(t, v)
    plt.ylabel('vitesse (m/s)')

    plt.subplot(4, 1, 3)
    plt.plot(t, a)
    plt.ylabel('acceleration (m/s2)')

    plt.subplot(4, 1, 4)
    plt.plot(t, j)
    plt.ylabel('jerk (m/s3)')
    plt.xlabel('temps (s)')
    plt.show()


def plot_breakingz():
    import trajectory

    t0 = 23
    t_end = 27
    n = 5
    dt = 0.01

    v_tr = 20
    p0_tr = 0
    f_tr = trajectory.Trajectory(p0=p0_tr, v0=v_tr, a0=0, t0=0)

    v_rdv = v_tr - delta_rv_speed
    accel_dist, accel_time = trajectory.get_accel_dist_time(
        max_jerk, accel_max, v_rdv)

    p0_rdv = p0_tr + v_tr * accel_time + 10
    meet_point = p0_rdv + accel_dist + v_rdv * time_at_rv_speed
    t_rdv = f_tr.real_roots(p=meet_point)[0]

    leave_time = t_rdv - time_at_rv_speed - accel_time
    f_rdv = trajectory.Trajectory(p0=p0_rdv, v0=0, a0=0, t0=0).extend(
        trajectory.SetVelocity(max_jerk, accel_max, v_rdv, p0=p0_rdv, t0=leave_time))

    start_time = np.linspace(t0, t_end, n)

    ps = []
    vs = []
    ass = []
    js = []
    ts = []
    for st in start_time:
        p0 = f_rdv(st)
        v0 = f_rdv(st, 1)
        a0 = f_rdv(st, 2)
        print('p0: %f06.1, v0: %f02.3, a0: %f01.4' % (p0, v0, a0))
        f = trajectory.FastestStop(max_jerk, deccel_max, p0=p0, v0=v0, a0=a0, t0=st)

        time = np.arange(st, f.get_end_time() + 5, dt)
        ts.append(time)

        ps.append([f(t) for t in time])
        vs.append([f(t, 1) for t in time])
        ass.append([f(t, 2) for t in time])
        js.append([f(t, 3) for t in time])

    plt.figure(1)
    plt.subplot(4, 1, 1)
    for i in range(len(ts)):
        plt.plot(ts[i], ps[i])
    plt.ylabel('position (m)')

    plt.subplot(4, 1, 2)
    for i in range(len(ts)):
        plt.plot(ts[i], vs[i])
    plt.ylabel('vitesse (m/s)')

    plt.subplot(4, 1, 3)
    for i in range(len(ts)):
        plt.plot(ts[i], ass[i])
    plt.ylabel('acceleration (m/s2)')

    plt.subplot(4, 1, 4)
    for i in range(len(ts)):
        plt.plot(ts[i], js[i])
    plt.ylabel('jerk (m/s3)')
    plt.xlabel('temps (s)')
    plt.show()


def plot_rdv():
    import trajectory

    t0 = 0
    dt = 0.1

    v_tr = 15
    p0_tr = 0
    f_tr = trajectory.Trajectory(p0=p0_tr, v0=v_tr, a0=0, t0=t0)

    v_rdv = v_tr - delta_rv_speed
    accel_dist, accel_time = trajectory.get_accel_dist_time(
        max_jerk, accel_max, v_rdv)

    p0_rdv = p0_tr + v_tr * accel_time + 10
    meet_point = p0_rdv + accel_dist + v_rdv * time_at_rv_speed
    t_rdv = f_tr.real_roots(p=meet_point)[0]

    leave_time = t_rdv - time_at_rv_speed - accel_time
    f_rdv = trajectory.Trajectory(p0=p0_rdv, v0=0, a0=0, t0=t0).extend(
        trajectory.SetVelocity(max_jerk, accel_max, v_rdv, p0=p0_rdv, t0=leave_time))

    t_end = f_rdv.parts[-1][0]
    time = np.arange(t0, t_end + 5, dt)

    p_tr = [f_tr(t) for t in time]
    v_tr = [f_tr(t, 1) for t in time]
    a_tr = [f_tr(t, 2) for t in time]
    j_tr = [f_tr(t, 3) for t in time]
    p_rdv = [f_rdv(t) for t in time]
    v_rdv = [f_rdv(t, 1) for t in time]
    a_rdv = [f_rdv(t, 2) for t in time]
    j_rdv = [f_rdv(t, 3) for t in time]

    plt.subplot(4, 1, 1)
    plt.plot(time, p_tr, time, p_rdv)
    plt.ylabel('position (m)')

    plt.subplot(4, 1, 2)
    plt.plot(time, v_tr, time, v_rdv)
    plt.ylabel('vitesse (m/s)')

    plt.subplot(4, 1, 3)
    plt.plot(time, a_tr, time, a_rdv)
    plt.ylabel('acceleration (m/s2)')

    plt.subplot(4, 1, 4)
    plt.plot(time, j_tr, time, j_rdv)
    plt.ylabel('jerk (m/s3)')
    plt.xlabel('temps (s)')
    plt.show()


def plot_breaking_point():
    import trajectory

    t0 = 0
    dt = 0.01

    v_tr0 = 16
    p0_tr = 0
    f_tr = trajectory.Trajectory(p0=p0_tr, v0=v_tr0, a0=0, t0=t0)

    v_rdv = v_tr0 - delta_rv_speed
    accel_dist, accel_time = trajectory.get_accel_dist_time(
        max_jerk, accel_max, v_rdv)

    p0_rdv = p0_tr + v_tr0 * accel_time + 10
    meet_point = p0_rdv + accel_dist + v_rdv * time_at_rv_speed
    t_rdv = f_tr.real_roots(p=meet_point)[0]

    leave_time = t_rdv - time_at_rv_speed - accel_time
    f_rdv = trajectory.Trajectory(p0=p0_rdv, v0=0, a0=0, t0=t0).extend(
        trajectory.SetVelocity(max_jerk, accel_max, v_rdv, p0=p0_rdv, t0=leave_time))

    t_end = f_rdv.parts[-1][0]
    time = np.arange(t0, t_end + 5, dt)

    p_tr = [f_tr(t) for t in time]
    v_tr = [f_tr(t, 1) for t in time]
    a_tr = [f_tr(t, 2) for t in time]
    j_tr = [f_tr(t, 3) for t in time]
    p_rdv = [f_rdv(t) for t in time]
    v_rdv = [f_rdv(t, 1) for t in time]
    a_rdv = [f_rdv(t, 2) for t in time]
    j_rdv = [f_rdv(t, 3) for t in time]

    brkp_tr = [trajectory.get_breaking_dist_time(
        max_jerk, deccel_max, v0=f_tr(t, 1), a0=f_tr(t, 2))[0] + f_tr(t) for t in time]
    # print('transit done')
    # input()
    brkp_rdv = [trajectory.get_breaking_dist_time(
        max_jerk, deccel_nom, v0=f_rdv(t, 1), a0=f_rdv(t, 2))[0] + f_rdv(t) for t in time]
    # print('rdv done')
    # input()

    fig = plt.figure(1)
    st = fig.suptitle(
        'Cinématique de rendez-vous à %d m/s et point d\'arrêt' % v_tr0, fontsize="x-large")
    plt.subplot(5, 1, 1)
    tr, rv = plt.plot(time, p_tr, time, p_rdv)
    plt.ylabel('Position (m)')

    plt.subplot(5, 1, 2)
    plt.plot(time, v_tr, time, v_rdv)
    plt.ylabel('Vitesse (m/s)')

    plt.subplot(5, 1, 3)
    plt.plot(time, a_tr, time, a_rdv)
    plt.ylabel(r'$Accélération (m/s^2)$')

    plt.subplot(5, 1, 4)
    plt.plot(time, j_tr, time, j_rdv)
    plt.ylabel(r'$À-coup (m/s^3)$')

    plt.subplot(5, 1, 5)
    plt.plot(time, brkp_tr, time, brkp_rdv)  #
    plt.ylabel('Point d\'arrêt (m)')
    plt.xlabel('Temps (s)')

    fig.legend((tr, rv), ('Transit', 'Accel'), 'right')  # , 'upper left'
    st.set_y(0.95)
    fig.subplots_adjust(top=0.85)

    plt.show()


def get_transit_velocity(distances):
    one = False
    if not hasattr(distances, '__iter__'):
        one = True
        distances = [distances]

    A = 1 / (2 * accel_max) + 1 / (2 * deccel_nom)
    B = time_at_rv_speed - delta_rv_speed / accel_max + full_boa_time

    vtr = []
    for dist in distances:
        C = delta_rv_speed**2 / (2 * accel_max) - time_at_rv_speed * delta_rv_speed - dist

        vtr1 = (-B - sqrt(B**2 - 4 * A * C)) / (2 * A)
        vtr2 = (-B + sqrt(B**2 - 4 * A * C)) / (2 * A)

        vtr.append(min(vitesse_max, (-B + sqrt(B**2 - 4 * A * C)) / (2 * A)) * 3.6)

    if one:
        return vtr[0]
    return vtr


def plot_tr_velocity():
    distances = np.arange(280, 2500, 10)
    vtr = get_transit_velocity(distances)

    plt.plot(distances, vtr)
    plt.ylabel('vitesse de transit (km/h)')
    plt.xlabel('distance entre les stations (m)')
    plt.show()


def get_wait_time(vtr):
    wait_time = []
    for i, speed in enumerate(vtr[:-1]):
        vtr1 = speed
        vtr2 = vtr[i + 1]
        vrv2 = vtr2 - delta_rv_speed
        wait_time.append(vtr1 / (2 * deccel_nom) + full_boa_time +
                         vrv2 / accel_max - vrv2**2 / (2 * accel_max * vtr2))
    return wait_time


def plot_wait_time():
    distances = range(280, 2500, 10)
    vtr = get_transit_velocity(distances)
    wait_time = get_wait_time(vtr)
    plt.plot(distances[:-1], wait_time)
    plt.ylabel('temps entre les trains (s)')
    plt.xlabel('distance entre les stations (m)')
    plt.show()


def try_JerkAccelToVelocity():
    import trajectory

    print(trajectory.SetVelocity(max_jerk, accel_max, 25, v0=20).get_end_time())

    # from fdbp import JerkAccelToVelocity
    # t0 = 545
    # dt = 0.1
    # t = np.arange(t0, t0 + 20, dt)
    # f = JerkAccelToVelocity(max_jerk, accel_max, 10, p0=15145,
    #                         v0=1, a0=0.8 * accel_max, t0=t0)

    # plt.figure(1)
    # p = [f(x) for x in t]
    # v = [f(x, 1) for x in t]
    # a = [f(x, 2) for x in t]
    # j = [f(x, 3) for x in t]

    # plt.subplot(4, 1, 1)
    # plt.plot(t, p)
    # plt.ylabel('position (m)')

    # plt.subplot(4, 1, 2)
    # plt.plot(t, v)
    # plt.ylabel('vitesse (m/s)')

    # plt.subplot(4, 1, 3)
    # plt.plot(t, a)
    # plt.ylabel('acceleration (m/s2)')

    # plt.subplot(4, 1, 4)
    # plt.plot(t, j)
    # plt.ylabel('jerk (m/s3)')
    # plt.xlabel('temps (s)')
    # plt.show()

    # v_wanted = np.arange(0, 15, 0.1)
    # v_end = []
    # for v in v_wanted:
    #     v_end.append(JerkAccelToVelocity(max_jerk, accel_max, v, p0=15145, v0=0, a0=0, t0=0)(50, 1))

    # plt.figure(2)
    # plt.plot(v_wanted, v_end)
    # plt.show()


def try_AjustVelocity():
    from trajectory import AjustVelocity
    t0 = 545
    dt = 0.1
    p0 = 1236
    v0 = 2
    p_end = p0 + 190
    v_end = 0
    f = AjustVelocity(max_jerk, accel_max, v=v_end, p=p_end, p0=p0,
                      v0=v0, t0=t0)
    t_end = f.parts[-1][0]
    t = np.arange(t0, t_end + 5, dt)

    p = [f(x) for x in t]
    v = [f(x, 1) for x in t]
    a = [f(x, 2) for x in t]
    j = [f(x, 3) for x in t]

    plt.figure(1)
    plt.subplot(4, 1, 1)
    plt.plot(t, p)
    plt.ylabel('position (m)')

    plt.subplot(4, 1, 2)
    plt.plot(t, v)
    plt.ylabel('vitesse (m/s)')

    plt.subplot(4, 1, 3)
    plt.plot(t, a)
    plt.ylabel('acceleration (m/s2)')

    plt.subplot(4, 1, 4)
    plt.plot(t, j)
    plt.ylabel('jerk (m/s3)')
    plt.xlabel('temps (s)')
    plt.show()


def try_GoToPV():
    from trajectory import GoToPV

    t0 = 545
    p0 = 15000
    v0 = 15
    a0 = 0

    p_end = p0 + 90
    v_end = 0
    # max_v = vitesse_max
    max_v = 15

    dt = 0.1
    f = GoToPV(max_jerk=max_jerk, max_a=accel_max, max_v=max_v,
               v=v_end, p=p_end, p0=p0, v0=v0, a0=a0, t0=t0)
    t_end = f.parts[-1][0]
    t = np.arange(t0, t_end + 5, dt)

    plt.figure(1)
    p = [f(x) for x in t]
    v = [f(x, 1) for x in t]
    a = [f(x, 2) for x in t]
    j = [f(x, 3) for x in t]

    print([tup[0] for tup in f.parts])

    print('end position: ', f(t_end - 0.1))
    print('end velocity: ', f(t_end - 0.1, 1))

    plt.subplot(4, 1, 1)
    plt.plot(t, p)
    plt.ylabel('position (m)')

    plt.subplot(4, 1, 2)
    plt.plot(t, v)
    plt.ylabel('vitesse (m/s)')

    plt.subplot(4, 1, 3)
    plt.plot(t, a)
    plt.ylabel('acceleration (m/s2)')

    plt.subplot(4, 1, 4)
    plt.plot(t, j)
    plt.ylabel('jerk (m/s3)')
    plt.xlabel('temps (s)')
    plt.show()


def try_RendezVousAccel():
    from trajectory import Trajectory, SetVelocity, get_accel_dist_time, AjustVelocity

    t0 = 545
    dt = 0.1

    p0s = 15000

    p01 = p0s - 300
    v01 = 12
    v11 = 15

    f1 = AjustVelocity(max_freestanding_jerk, max_freestanding_accel, v=v11, p=p0s, p0=p01,
                       v0=v01, t0=t0)

    v_rdv = v11 - delta_rv_speed
    accel_dist, accel_time = get_accel_dist_time(max_jerk, accel_max, v_rdv)

    meet_point = p0s + accel_dist + v_rdv * time_at_rv_speed
    t_rdv = f1.real_roots(p=meet_point)[0]
    # if abs(behind.trajectory(t_rdv, 1) - v_lim) > wrong_velocity_threshold:
    #     raise Exception('Wrong velocity for rendez vous. Station: %i' % self.station)

    leave_time = t_rdv - time_at_rv_speed - accel_time
    f2 = Trajectory(p0=p0s, v0=0, a0=0, t0=t0).extend(
        SetVelocity(max_jerk, accel_max, v_rdv, p0=p0s, t0=leave_time))

    t_end = max(f1.parts[-1][0], f2.parts[-1][0])
    t = np.arange(t0, t_end + 5, dt)

    p1 = [f1(x) for x in t]
    v1 = [f1(x, 1) for x in t]
    a1 = [f1(x, 2) for x in t]
    j1 = [f1(x, 3) for x in t]

    p2 = [f2(x) for x in t]
    v2 = [f2(x, 1) for x in t]
    a2 = [f2(x, 2) for x in t]
    j2 = [f2(x, 3) for x in t]

    # print([tup[0] for tup in f.parts])

    # print('end position: ', f(t_end - 0.1))
    # print('end velocity: ', f(t_end - 0.1, 1))

    plt.figure(1)
    plt.subplot(4, 1, 1)
    plt.plot(t, p1, t, p2)
    plt.ylabel('position (m)')

    plt.subplot(4, 1, 2)
    plt.plot(t, v1, t, v2)
    plt.ylabel('vitesse (m/s)')

    plt.subplot(4, 1, 3)
    plt.plot(t, a1, t, a2)
    plt.ylabel('acceleration (m/s2)')

    plt.subplot(4, 1, 4)
    plt.plot(t, j1, t, j2)
    plt.ylabel('jerk (m/s3)')
    plt.xlabel('temps (s)')
    plt.show()


if __name__ == '__main__':
    # plot_tr_velocity()
    # plot_wait_time()
    try_JerkAccelToVelocity()
    # try_AjustVelocity()
    # try_GoToPV()
    # plot_breaking_dist()
    # try_RendezVousAccel()
    # plot_breaking_point()
    # plot_rdv()
    # plot_breaking()
    # plot_breakingz()
