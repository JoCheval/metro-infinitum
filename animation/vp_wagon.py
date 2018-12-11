import sys
import vpython as vp

sys.path.insert(0, '../simulation')

from params import *

wagon_depth = 3


class VPWagon():

    def __init__(self, vpline, wagon, offset='down'):
        self.vpline = vpline
        self.wagon = wagon
        self.offset = offset

        self.init_visual()

    def init_visual(self):
        color = vp.color.cyan
        self.visual = vp.box(pos=self.get_pos(), length=longeur_wagon,
                             height=largeur_wagon, width=hauteur_wagon, color=color)

    def get_pos(self):
        p = self.wagon.p_front - longeur_wagon / 2
        x, y = self.vpline.get_pos(p, self.offset)
        return vp.vector(x, y, wagon_depth)

    def get_axis(self):
        p1 = self.wagon.p_front - longeur_wagon / 2
        x1, y1 = self.vpline.get_pos(p1, self.offset)
        v1 = vp.vector(x1, y1, 0)
        p2 = p1 + 2
        x2, y2 = self.vpline.get_pos(p2, self.offset)
        v2 = vp.vector(x2, y2, 0)
        return (v2 - v1).norm() * longeur_wagon

    def update(self):
        self.visual.pos = self.get_pos()
        self.visual.axis = self.get_axis()
