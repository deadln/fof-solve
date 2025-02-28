import math

import numpy
import numpy as np

class Point:
    '''3d Point (float)'''

    def __init__(self, x=0, y=0, z=0):
        '''Defines x and y variables'''
        self.x = x
        self.y = y
        self.z = z

    def move(self, dx, dy, dz):
        '''Determines where x and y move'''
        self.x = self.x + dx
        self.y = self.y + dy
        self.z = self.z + dz

    def __str__(self):
        return "Point({},{},{})".format(self.x, self.y, self.z)

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_z(self):
        return self.z

    def vector_length(self):
        return self.distance(Point(0.0, 0.0, 0.0))

    def distance(self, other):
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

    def add_point(self, other):
        self.x += other.get_x()
        self.y += other.get_y()
        self.z += other.get_z()

    def mul_vector(self, k):
        self.x *= k
        self.y *= k
        self.z *= k

    def normalize_vector(self):
        ln = self.vector_length()
        self.x /= ln
        self.y /= ln
        self.z /= ln

    def sub_point(self, other):
        self.x -= other.get_x()
        self.y -= other.get_y()
        self.z -= other.get_z()

    def from_dict(self, a):
        self.x = a['x']
        self.y = a['y']
        self.z = a['z']

    def get_dict(self):
        return {'x': self.x, 'y': self.y, 'z': self.z}

    def get_array(self):
        return np.array([self.x, self.y, self.z])

    def get_cp(self, other_point):
        return Point(self.y * other_point.z - self.z * other_point.y,
                     self.z * other_point.x - self.x * other_point.z,
                     self.x * other_point.y - self.y * other_point.x)

    def get_cp_len(self, other_point):
        '''lenght of cross product vector of self point and other point'''
        tmp = self.get_cp(other_point)
        return tmp.vector_length()


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

    # def __init__(self, a: numpy.array, b: numpy.array, c: numpy.array):
    #     self.p0 = np.array(a)
    #     self.p1 = np.array(b)
    #     self.p2 = np.array(c)
    #     self.nv = np.cross(self.p1 - self.p0, self.p2 - self.p0)
    def __init__(self, a: numpy.array, nv: numpy.array):
        self.p0 = np.array(a)
        self.nv = nv

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


class Basis:
    def __init__(self, center, z_vect):
        self.center = center
        self.basis = self.get_vector_basis(z_vect)
        self.basis_1 = np.linalg.inv(self.basis.T)
        self.center_1 = np.dot(-self.basis_1, center)

    def get_vector_basis(self, route_vect):
        base_z = route_vect / np.linalg.norm(route_vect)
        vect_z = np.array([0.0, 0.0, 1.0])
        rot_axis = np.cross(route_vect, vect_z)
        rot_axis = rot_axis / np.linalg.norm(rot_axis)
        base_x = rot_axis
        ang90 = math.pi / 2
        ang270 = -math.pi / 2
        rot_matrix1 = np.array([
            [math.cos(ang90) + (1 - math.cos(ang90)) * rot_axis[0] ** 2,
             (1 - math.cos(ang90)) * rot_axis[0] * rot_axis[1] - math.sin(ang90) * rot_axis[2],
             (1 - math.cos(ang90)) * rot_axis[0] * rot_axis[2] + math.sin(ang90) * rot_axis[1]],
            [(1 - math.cos(ang90)) * rot_axis[1] * rot_axis[0] + math.sin(ang90) * rot_axis[2],
             math.cos(ang90) + (1 - math.cos(ang90)) * rot_axis[1] ** 2,
             (1 - math.cos(ang90)) * rot_axis[1] * rot_axis[2] - math.sin(ang90) * rot_axis[0]],
            [(1 - math.cos(ang90)) * rot_axis[2] * rot_axis[0] - math.sin(ang90) * rot_axis[1],
             (1 - math.cos(ang90)) * rot_axis[2] * rot_axis[1] + math.sin(ang90) * rot_axis[0],
             math.cos(ang90) + (1 - math.cos(ang90)) * rot_axis[2] ** 2]
        ])
        rot_matrix2 = np.array([
            [math.cos(ang270) + (1 - math.cos(ang270)) * rot_axis[0] ** 2,
             (1 - math.cos(ang270)) * rot_axis[0] * rot_axis[1] - math.sin(ang270) * rot_axis[2],
             (1 - math.cos(ang270)) * rot_axis[0] * rot_axis[2] + math.sin(ang270) * rot_axis[1]],
            [(1 - math.cos(ang270)) * rot_axis[1] * rot_axis[0] + math.sin(ang270) * rot_axis[2],
             math.cos(ang270) + (1 - math.cos(ang270)) * rot_axis[1] ** 2,
             (1 - math.cos(ang270)) * rot_axis[1] * rot_axis[2] - math.sin(ang270) * rot_axis[0]],
            [(1 - math.cos(ang270)) * rot_axis[2] * rot_axis[0] - math.sin(ang270) * rot_axis[1],
             (1 - math.cos(ang270)) * rot_axis[2] * rot_axis[1] + math.sin(ang270) * rot_axis[0],
             math.cos(ang270) + (1 - math.cos(ang270)) * rot_axis[2] ** 2]
        ])
        v1 = np.dot(base_z, rot_matrix1)
        v2 = np.dot(base_z, rot_matrix2)
        if v1[2] > v2[2]:
            base_y = v1
        else:
            base_y = v2
        return np.array([base_x, base_y, base_z])

    def to_new_basis(self, point):
        return np.dot(self.basis_1, point) + self.center_1

    def to_old_basis(self, point):
        return np.dot(point, self.basis) + self.center

def is_crossing_rectangles(r1, r2, rect_w_1, rect_h_1, rect_w_2, rect_h_2):
    if abs(r1[0] - r2[0]) > rect_w_1/2+rect_w_2/2 or abs(r1[1] - r2[1]) > rect_h_1/2+rect_h_2/2:
        return False
    return True

def turn_vector(vect, axis, angle):
    axis = axis / np.linalg.norm(axis)
    rot_matrix = np.array([
        [math.cos(angle) + (1 - math.cos(angle)) * axis[0] ** 2,
         (1 - math.cos(angle)) * axis[0] * axis[1] - math.sin(angle) * axis[2],
         (1 - math.cos(angle)) * axis[0] * axis[2] + math.sin(angle) * axis[1]],
        [(1 - math.cos(angle)) * axis[1] * axis[0] + math.sin(angle) * axis[2],
         math.cos(angle) + (1 - math.cos(angle)) * axis[1] ** 2,
         (1 - math.cos(angle)) * axis[1] * axis[2] - math.sin(angle) * axis[0]],
        [(1 - math.cos(angle)) * axis[2] * axis[0] - math.sin(angle) * axis[1],
         (1 - math.cos(angle)) * axis[2] * axis[1] + math.sin(angle) * axis[0],
         math.cos(angle) + (1 - math.cos(angle)) * axis[2] ** 2]
    ])
    return np.dot(vect, rot_matrix)
