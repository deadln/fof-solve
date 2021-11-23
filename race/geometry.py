import math

import numpy
import numpy as np


class Line:
    '''Line in 3d(float)'''

    def __init__(self, a: numpy.array, b: numpy.array):
        '''get line from two points'''
        self.p0 = np.array(a)
        self.p1 = np.array(b)

    def get_point_dist(self, other_point: numpy.array) -> float:
        '''get float distance from this line to other point'''
        cp = np.cross(other_point - self.p0, self.p1 - self.p0)
        cp = math.sqrt(np.dot(cp, cp))
        return cp / math.sqrt(np.dot(self.p1 - self.p0, self.p1 - self.p0))

    def pr_point(self, other_point: numpy.array) -> numpy.array:
        vec_proj = self.p1 - self.p0
        vec_proj_wanted_lgt = np.dot(other_point - self.p0, vec_proj) / math.sqrt(np.dot(vec_proj, vec_proj))
        vec_proj_old_lgt = math.sqrt(np.dot(vec_proj, vec_proj))
        vec_proj = vec_proj * (vec_proj_wanted_lgt / vec_proj_old_lgt)
        ans = self.p0 + vec_proj
        return ans


class Surface:
    '''surface in 3d'''

    def __init__(self, a: numpy.array, b: numpy.array, c: numpy.array):
        self.p0 = np.array(a)
        self.p1 = np.array(b)
        self.p2 = np.array(c)
        self.nv = np.cross(self.p1 - self.p0, self.p2 - self.p0)

    def substitute_point(self, a: numpy.array) -> numpy.array:
        '''substitute point to surface equation'''
        return np.dot(a - self.p0, self.nv)

    def get_point_dist(self, other_point: numpy.array):
        '''get distance to point from this surface'''
        return math.fabs(self.substitute_point(other_point) / math.sqrt(np.dot(self.nv, self.nv)))

    def pr_point(self, other_point: numpy.array) -> numpy.array:
        vec_to_plane = self.nv
        nv_lgt = math.sqrt(np.dot(self.nv, self.nv))
        dist = self.get_point_dist(other_point)
        vec_to_plane = vec_to_plane * dist
        vec_to_plane = vec_to_plane / nv_lgt
        if self.substitute_point(other_point) > 0:
            vec_to_plane = vec_to_plane * -1
        return other_point + vec_to_plane