from math import sqrt
import matplotlib.pyplot as plt
from params import *
import numpy as np

import trajectory


def plot_pvaj(f, title, t0=None, t_end=None, fig_nb=1):
    if t0 is None:
        t0 = f.parts[0][0] - 5
    if t_end is None:
        t_end = f.parts[-1][0] + 5
    dt = 0.1
    t = np.arange(t0, t_end, dt)

    p = [f(x) for x in t]
    v = [f(x, 1) for x in t]
    a = [f(x, 2) for x in t]
    j = [f(x, 3) for x in t]

    fig = plt.figure(fig_nb)
    st = fig.suptitle(title, fontsize="x-large")
    plt.subplot(4, 1, 1)
    plt.plot(t, p)
    plt.ylabel('Position (m)')

    plt.subplot(4, 1, 2)
    plt.plot(t, v)
    plt.ylabel('Vitesse (m/s)')

    plt.subplot(4, 1, 3)
    plt.plot(t, a)
    plt.ylabel(r'Accélération (m/s$^2$)')

    plt.subplot(4, 1, 4)
    plt.plot(t, j)
    plt.ylabel(r'À-coup (m/s$^3$)')
    plt.xlabel('Temps (s)')
    st.set_y(0.95)
    fig.subplots_adjust(left=0.06, bottom=0.08, right=0.98,
                        top=0.90, hspace=0.05)
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()
    plt.show()


def plot_FastestStop():

    # Avec saturation

    t0 = 0
    p0 = 15000
    v0 = 15
    a0 = 0.5

    f = trajectory.FastestStop(max_jerk, deccel_max, p0=p0, v0=v0, a0=a0, t0=t0)

    plot_pvaj(f, 'Arrêt immédiat avec saturation de la décélération', 1)

    # Sans saturation

    v0 = 5

    f = trajectory.FastestStop(max_jerk, deccel_max, p0=p0, v0=v0, a0=a0, t0=t0)
    plot_pvaj(f, 'Arrêt immédiat sans saturation de la décélération', 1)


def plot_SetVelocity():

    # Avec saturation

    t0 = 0
    p0 = 0
    v0 = 0
    a0 = 0
    v = 15
    f = trajectory.Trajectory(p0=p0, v0=v0, a0=a0, t0=t0 - 1).extend(
        trajectory.SetVelocity(max_jerk, accel_max, v, p0=p0, v0=v0, a0=a0, t0=t0))
    plot_pvaj(f, "Accélération jusqu'à %02.0f m/s avec saturation de l'accélération" % v)

    # Sans saturation
    v = 2
    f = trajectory.Trajectory(p0=p0, v0=v0, a0=a0, t0=t0 - 1).extend(
        trajectory.SetVelocity(max_jerk, accel_max, v, p0=p0, v0=v0, a0=a0, t0=t0))

    plot_pvaj(f, "Accélération jusqu'à %01.0f m/s sans saturation de l'accélération" % v)


def plot_GoToPV():

    # Avec saturation accel, vitesse, decel
    t0 = 0
    p0 = 15000
    v0 = 0
    a0 = 0
    p = p0 + 350
    v = 0
    v_max = 15
    f = trajectory.GoToPV(max_jerk, accel_max, v_max, v, p, p0=p0, v0=v0, a0=a0, t0=t0)
    plot_pvaj(
        f, "Déplacement jusqu'à une vitesse à la position voulu avec saturation de l'accélération, de la vitesse et de la décélération", t0=t0)

    # Avec saturation vitesse, decel
    t0 = 0
    p0 = 15000
    v0 = 13
    a0 = 0
    p = p0 + 250
    v = 0
    v_max = 15
    f = trajectory.GoToPV(max_jerk, accel_max, v_max, v, p, p0=p0, v0=v0, a0=a0, t0=t0)
    plot_pvaj(
        f, "Déplacement jusqu'à une vitesse à la position voulu sans saturation de l'accélération", t0=t0)

    # Avec saturation accel, decel
    t0 = 0
    p0 = 15000
    v0 = 0
    a0 = 0
    p = p0 + 100
    v = 0
    v_max = 15
    f = trajectory.GoToPV(max_jerk, accel_max, v_max, v, p, p0=p0, v0=v0, a0=a0, t0=t0)
    plot_pvaj(
        f, "Déplacement jusqu'à une vitesse à la position voulu sans saturation de la vitesse", t0=t0)

    # Avec saturation accel, vitesse
    t0 = 0
    p0 = 15000
    v0 = 10
    a0 = 0
    p = p0 + 200
    v = 14
    v_max = 15
    f = trajectory.GoToPV(max_jerk, accel_max, v_max, v, p, p0=p0, v0=v0, a0=a0, t0=t0)
    plot_pvaj(
        f, "Déplacement jusqu'à une vitesse à la position voulu sans saturation de la décélération", t0=t0)

    # Sans saturation
    t0 = 0
    p0 = 15000
    v0 = 12
    a0 = 0
    p = p0 + 100
    v = 13
    v_max = 15
    f = trajectory.GoToPV(max_jerk, accel_max, v_max, v, p, p0=p0, v0=v0, a0=a0, t0=t0)
    plot_pvaj(
        f, "Déplacement jusqu'à une vitesse à un point voule sans saturation de l'accélération, de la vitesse ou de la décélération", t0=t0)


def plot_breaking_point():
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
    brkp_rdv = [trajectory.get_breaking_dist_time(
        max_jerk, deccel_nom, v0=f_rdv(t, 1), a0=f_rdv(t, 2))[0] + f_rdv(t) for t in time]

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

    fig.legend((tr, rv), ('Transit', 'Accelération'), 'best')  # , 'upper left'
    st.set_y(0.95)
    fig.subplots_adjust(left=0.06, bottom=0.08, right=0.99,
                        top=0.90, hspace=0.05)
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()
    plt.show()


if __name__ == '__main__':
    # plot_FastestStop()
    # plot_SetVelocity()
    plot_GoToPV()
    # plot_breaking_point()
