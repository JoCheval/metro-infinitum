import vpython as vp
import sys

from vp_line import VPLine
from vp_wagon import VPWagon
sys.path.insert(0, '../simulation')
from line import Line


###############################################################################
#                                  Controls                                   #
###############################################################################
move_factor = 0.05


def keydown(evt):
    s = evt.key
    if 'down' in s or 'right' in s:
        zoom(zoom_in=False)
    elif 'up' in s or 'left' in s:
        zoom(zoom_in=True)
    elif s == 'a':
        vp.scene.camera._followthis = None
        # vp.scene.camera.follow(None)
        vp.scene.camera.pos += vp.vector(-1, 0, 0) * \
            vp.scene.camera.axis.mag * move_factor
    elif s == 'w':
        vp.scene.camera.pos += vp.vector(0, 1, 0) * \
            vp.scene.camera.axis.mag * move_factor
    elif s == 'd':
        vp.scene.camera.pos += vp.vector(1, 0, 0) * \
            vp.scene.camera.axis.mag * move_factor
    elif s == 's':
        vp.scene.camera.pos += vp.vector(0, -1, 0) * \
            vp.scene.camera.axis.mag * move_factor
    else:
        print(s)


def zoom(zoom_in):
    if zoom_in:
        factor = 0.9
    else:
        factor = 1.1

    center = vp.scene.center
    vp.scene.camera.axis *= factor
    vp.scene.center = center
    print(vp.scene.camera.axis, vp.scene.center)


def mousedown():
    hit = vp.scene.mouse.pick
    if hit is not None:
        if isinstance(hit, vp.box):
            vp.scene.camera.follow(hit)


# def click(evt):
#     print(evt.pos)


# def mouseup(evt):
#     pass


def set_controls():
    # vp.scene.userspin = False
    vp.scene.userzoom = False
    vp.scene.autoscale = False

    vp.scene.width = 1400
    vp.scene.height = 700

    # vp.scene.camera.pos = vp.vector(-780, -560, 1350)
    # vp.scene.camera.axis = vp.vector(0, 0, -170)
    # vp.scene.center = vp.scene.camera.pos

    vp.scene.bind('mousedown', mousedown)
    vp.scene.bind('keydown', keydown)


###############################################################################
#                                    Main                                     #
###############################################################################

def main():
    sim_time = 0
    dt = 0.025
    t_end = 15 * 60

    line = Line('../simulation/verte_full.json')
    print('Done initialising line')
    vpline = VPLine('../simulation/verte_full.json')
    print('Done initialising vpline')
    wagons = []
    vpwagons = []

    while sim_time < t_end:
        sim_time += dt
        vp.rate(1 / dt)

        line.update(sim_time)

        for w in line.wagons:
            if w not in wagons:
                wagons.append(w)
                vpwagons.append(VPWagon(vpline, w))
                # vp.scene.camera.follow(vpwagons[0].visual)

        for vpw in vpwagons:
            vpw.update()


if __name__ == '__main__':
    set_controls()
    vp.scene.background = vp.color.white
    main()
