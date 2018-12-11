from scipy.interpolate import splev, splrep
import numpy as np
import sys
import json
from math import pi, cos
import vpython as vp

sys.path.insert(0, '../simulation')

from params import *


distance_voix = 1
offset_voix = ((largeur_wagon + distance_voix) / 2)
station_depth = -1
largeur_station = largeur_wagon * 2 + offset_voix * 2 + 3 * 2


def deg2rad(deg):
    return deg / 180 * pi


def coord2xy(lattitude, longitude, lattitude0, longitude0):
    xc = earth_radius * deg2rad(longitude0) * cos(deg2rad(lattitude0))
    yc = earth_radius * deg2rad(lattitude0)
    x = earth_radius * deg2rad(longitude) * cos(deg2rad(lattitude0)) - xc
    y = earth_radius * deg2rad(lattitude) - yc
    return (x, y)


class Spline():
    def __init__(self, dist, px, py):
        self.splx = splrep(dist, px)
        self.sply = splrep(dist, py)
        self.dist = dist

    def get_pos(self, t, offset=0):
        if offset == 0:
            return splev(t, self.splx), splev(t, self.sply)
        x = splev(t, self.splx)
        y = splev(t, self.sply)
        dx = splev(t, self.splx, der=1)
        dy = splev(t, self.sply, der=1)
        tangent = np.array([[dx, dy]]) / np.linalg.norm(np.array([[dx, dy]]))
        return x + offset * tangent[0, 1], y + offset * tangent[0, 0]


class VPStation():
    def __init__(self, name, vpline, p):
        color = vp.color.gray(0.5)
        shape = vp.shapes.rectangle(pos=(0, 0), width=1, height=largeur_station)
        dp = 3  # (m)
        pos = np.arange(p - azur_train_lenght - 3, p + 3, dp)
        path = []
        for p in pos:
            x, y = vpline.get_pos(p, 0)
            path.append(vp.vector(x, y, station_depth))
        self.visual = vp.extrusion(path=path, shape=shape,
                                   color=color)


class VPLine():
    def __init__(self, data_file, lat_center=None, long_center=None):
        self.data_file = data_file
        self.lat_center = lat_center
        self.long_center = long_center

        self.init_visual()

    def init_visual(self):
        # read data file
        with open(self.data_file) as json_file:
            json_data = json.load(json_file)

        #  Find center of the map
        if self.long_center is None:
            westmost = 180
            eastmost = -180
            for station in json_data['stations']:
                longitude = station['coord'][1]
                if longitude < westmost:
                    westmost = longitude
                if longitude > eastmost:
                    eastmost = longitude
            self.long_center = (westmost + eastmost) / 2

        if self.lat_center is None:
            northmost = -90
            southmost = 90
            for station in json_data['stations']:
                lattitude = station['coord'][0]
                if lattitude < southmost:
                    southmost = lattitude
                if lattitude > northmost:
                    northmost = lattitude
            self.lat_center = (northmost + southmost) / 2

        # convert x,y and get speed limits

        x = []
        y = []
        dist = [0]

        for station in json_data['stations']:
            coord = station['coord']
            station_x, station_y = coord2xy(coord[0], coord[1],
                                            self.lat_center, self.long_center)
            x.append(station_x)
            y.append(station_y)
            next_dist = station['next_dist']
            if next_dist is not None:
                dist.append(next_dist + dist[-1])

        # Make spline
        self.spline = Spline(dist, x, y)

        # Make stations
        self.stations = []
        for i, station_data in enumerate(json_data['stations']):
            if i in [0, len(json_data['stations']) - 1]:
                continue    # No visual for hangars
            self.stations.append(VPStation(station['name'], self, dist[i]))

        # Make visual
        color = getattr(vp.color, json_data['color'])
        shape = vp.shapes.rectangle(
            pos=(0, 0), width=1, height=largeur_wagon * 2 + distance_voix)
        path = []
        dp = 10  # (m)
        pos = np.arange(0, dist[-1], dp)
        for p in pos:
            x, y = self.get_pos(p)
            path.append(vp.vector(x, y, tunnel_depth))

        # print(path)
        print('Initializing line visual')
        self.visual = vp.extrusion(path=path, shape=shape,
                                   color=color)
        print('Done initializing line visual')

    def get_pos(self, p, offset=None):
        if offset is None:
            offset = 0
        elif offset == 'up':
            offset = offset_voix
        elif offset == 'down':
            offset = - offset_voix
        return self.spline.get_pos(p, offset=offset)
