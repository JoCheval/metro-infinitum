import vpython as vp
from wagon import Wagon
from ligne import Ligne


gameTime = 0


class XYZ():
    def __init__(self, pos=None):
        if pos is None:
            pos = vp.vector(0, 0, 0)
        self.x = vp.arrow(pos=pos, axis=vp.vector(1, 0, 0),
                          shaftwidth=0.1, color=vp.color.red)
        self.y = vp.arrow(pos=pos, axis=vp.vector(0, 1, 0),
                          shaftwidth=0.1, color=vp.color.green)
        self.z = vp.arrow(pos=pos, axis=vp.vector(0, 0, 1),
                          shaftwidth=0.1, color=vp.color.blue)


def mousedown(evt):
    global first_cam_pos, first_mouse_pos, mouse_state
    first_mouse_pos = evt.pos
    first_cam_pos = vp.scene.camera.pos


def mousemove(evt):
    global first_mouse_pos, first_cam_pos
    diff = evt.pos - first_mouse_pos
    vp.scene.camera.pos = first_cam_pos - diff
    first_mouse_pos = evt.pos
    first_cam_pos = vp.scene.camera.pos


def keydown(evt):
    s = evt.key
    if 'up' in s:
        zoom(zoom_in=False)
    elif 'down' in s:
        zoom(zoom_in=True)


def zoom(zoom_in):
    if zoom_in:
        factor = 0.9
    else:
        factor = 1.1

    center = vp.scene.center
    vp.scene.camera.axis *= factor
    vp.scene.center = center
    print(vp.scene.camera.axis, vp.scene.center)


def move(mouse_pos):
    global first_mouse_pos, first_cam_pos
    diff = evt.pos - first_mouse_pos
    vp.scene.camera.pos = first_cam_pos - diff

def click(evt):
    print(evt.pos)


def mouseup(evt):
    pass


def set_controls():
    vp.scene.userspin = False
    vp.scene.userzoom = False
    vp.scene.autoscale = False

    vp.scene.camera.pos = vp.vector(-780, -560, 1350)
    vp.scene.camera.axis = vp.vector(0, 0, -170)
    vp.scene.center = vp.scene.camera.pos

    vp.scene.bind('mousedown', mousedown)
    vp.scene.bind('mousemove', mousemove)

    vp.scene.bind('keydown', keydown)
    # vp.scene.bind('mouseup', mouseup)
    # vp.scene.bind('click', click)


def main():
    set_controls()

    origin = XYZ()
    # world = vp.box(size=vp.vector(20000, 20000, 1))
    # print('hello?')
    # vp.scene.caption = """Right button drag to rotate. Scroll wheel to zoom."""

    # side = 4.0
    # thk = 0.3
    # s2 = 2*side - thk
    # s3 = 2*side + thk

    # wallR = vp.box (pos=vp.vector( side, 0, 0), size=vp.vector(thk, s2, s3),  color = vp.color.red)
    # wallL = vp.box (pos=vp.vector(-side, 0, 0), size=vp.vector(thk, s2, s3),  color = vp.color.red)
    # wallB = vp.box (pos=vp.vector(0, -side, 0), size=vp.vector(s3, thk, s3),  color = vp.color.blue)
    # wallT = vp.box (pos=vp.vector(0,  side, 0), size=vp.vector(s3, thk, s3),  color = vp.color.blue)
    # wallBK = vp.box(pos=vp.vector(0, 0, -side), size=vp.vector(s2, s2, thk), color = vp.color.gray(0.7))

    # ball = vp.sphere (color = vp.color.green, radius = 0.4, make_trail=True, retain=200)
    # ball.mass = 1.0
    # ball.p = vp.vector (-0.15, -0.23, +0.27)

    # side = side - thk*0.5 - ball.radius

    # dt = 0.3
    # while True:
    #     vp.rate(200)
    #     ball.pos = ball.pos + (ball.p/ball.mass)*dt
    #     if not (side > ball.pos.x > -side):
    #         ball.p.x = -ball.p.x
    #     if not (side > ball.pos.y > -side):
    #         ball.p.y = -ball.p.y
    #     if not (side > ball.pos.z > -side):
    #         ball.p.z = -ball.p.z

    ligne_verte = Ligne('verte.json')
    # wagon = Wagon(pos=vp.vector(10, 10, 0))
    # evolving = [wagon]
    vp.scene.camera.axis = vp.vector(0, 0, -1)
    vp.scene.fullscreen = True

    exit = False
    dt = 0.01
    sym_time = 0

    while not exit:
        # vp.sleep(dt)
        vp.rate(1/dt)
        sym_time += dt
        ligne_verte.update(sym_time)
        # poll()

    print(vp.scene.camera.pos)
    print(vp.scene.camera.axis)


if __name__ == '__main__':
    main()
