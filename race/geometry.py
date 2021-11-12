import math


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

    def __init__(self, a, b):
        '''get line from two points'''
        self.p0 = Point(a.x, a.y, a.z)
        self.p1 = Point(b.x, b.y, b.z)

    def get_point_dist(self, other_point):
        '''get float distance from this line to other point'''
        tmp1 = Point(self.p1.x, self.p1.y, self.p1.z)
        tmp2 = Point(other_point.x, other_point.y, other_point.z)
        tmp1.sub_point(self.p0)
        tmp2.sub_point(self.p0)
        sq = tmp1.get_cp_len(tmp2)
        return sq / self.p1.distance(self.p0)

    def pr_point(self, other_point):
        tmp1 = Point(self.p1.x, self.p1.y, self.p1.z)
        tmp2 = Point(other_point.x, other_point.y, other_point.z)
        tmp1.sub_point(self.p0)
        tmp2.sub_point(self.p0)
        nv = tmp1.get_cp(tmp2)
        nvv = tmp1.get_cp(nv)
        nvv.normalize_vector()
        k = self.get_point_dist(other_point)
        nvv.mul_vector(k)
        ans = Point(other_point.x, other_point.y, other_point.z)
        ans.add_point(nvv)
        return ans


class Surface:
    '''surface in 3d'''

    def __init__(self, a, b, c):
        self.p0 = Point(a.x, a.y, a.z)
        self.p1 = Point(b.x, b.y, b.z)
        tmp1 = Point(b.x, b.y, b.z)
        self.p2 = Point(c.x, c.y, c.z)
        tmp2 = Point(c.x, c.y, c.z)

        tmp1.sub_point(self.p0)
        tmp2.sub_point(self.p0)

        self.nv = tmp1.get_cp(tmp2)
        self.nv.normalize_vector()

        self.d = -(self.nv.x * a.x + self.nv.y * a.y + self.nv.z * a.z)

    def substitute_point(self, a):
        '''substitute point to surface equation'''
        return self.nv.x * a.x + self.nv.y * a.y + self.nv.z * a.z + self.d

    def get_point_dist(self, other_point):
        '''get distance to point from this surface'''
        return math.fabs(self.substitute_point(other_point))
