#!/usr/bin/env python3
# coding=utf8

import rospy
import time
import sys
import os
import math
import numpy as np

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState, GlobalPositionTarget
from geographic_msgs.msg import GeoPointStamped
from std_msgs.msg import String

from mavros_msgs.srv import SetMode, CommandBool, CommandVtolTransition, CommandHome

from pygeodesy.geoids import GeoidPGM
_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

from geometry import *

freq = 20  # Герц, частота посылки управляющих команд аппарату
node_name = "offboard_node"
# Количество аппаратов
INSTANCES_NUM = 3

controllers = []
obstacles = {}
priority = {}


class CopterController():
    def __init__(self, instance_num):
        self.state = "disarm"
        self.instance_num = instance_num
        # создаем топики, для публикации управляющих значений:
        self.pub_pt = rospy.Publisher(f"/mavros{instance_num}/setpoint_raw/local", PositionTarget, queue_size=10)
        self.pub_gpt = rospy.Publisher(f"/mavros{instance_num}/setpoint_raw/global", GlobalPositionTarget, queue_size=10)
        # Объект для управления по локальной системе координат
        self.pt = PositionTarget()
        self.pt.coordinate_frame = self.pt.FRAME_LOCAL_NED
        # Объект для управления по глобальной системе координат
        self.gpt = GlobalPositionTarget()
        self.gpt.coordinate_frame = self.gpt.FRAME_GLOBAL_INT

        self.t0 = time.time()
        self.dt = 0

        # params
        self.P_GAIN = 2  # Множитель вектора скорости для приближения к точке
        self.MAX_VELOCITY = 10.0
        self.MIN_VELOCITY = 0.2
        self.APPROACH_VELOCITY = 5.0
        self.APPROACH_RADIUS = 15.0
        self.ARRIVAL_RADIUS = 2.0
        self.ARRIVAL_RADIUS_GLOBAL = 0.0001
        self.waypoint_list = get_waypoints()
        self.AVOID_RADIUS = 25.0  # Радиус обнаружения "валидных" препятствий
        self.MAX_AVOID_SPEED = 5.0  # Максимальная длина вектора уклонения от препятствий
        self.AVOID_P_GAIN = 2.5
        self.TRAJECTORY_CORRECTION = 1.7  # Множитель вектора скорости для корректирования траектории
        self.TRAJECTORY_CORRECTION_Z = 2.1
        self.MAXIMAL_DEVIATION = 8.0
        self.CHECKPOINT_SURFACE_BIAS = 4.0
        self.CRITICAL_DISTANCE = 3


        self.current_waypoint = np.array([0., 0., 0.])
        self.previous_waypoint = np.array([0., 0., 0.])
        self.pose = np.array([])
        self.pose_global = np.array([])
        self.velocity = np.array([0., 0., 0.])
        self.mavros_state = State()
        self.route_line = Line(np.array([0, 0, 0]), np.array([0, 0, 0]))  # Линия от предыдущей точке к следующей
        self.surface = Surface(np.array([0, 0, 0]), np.array([0, 0, 0]))  # Плоскость, перпендикулятрная линии маршрута
        self.subscribe_on_topics()

    def offboard_loop(self):
        self.dt = time.time() - self.t0
        self.set_mode("OFFBOARD")
        # управляем аппаратом
        if self.state == "disarm":
            self.arming(True)
        elif self.state == "takeoff":
            self.takeoff()
        elif self.state == "tookoff":
            self.follow_waypoint_list()
        elif self.state == "arrival":
            error = self.move_to_point(self.current_waypoint)
        self.pub_pt.publish(self.pt)

    # взлет коптера
    def takeoff(self):
        error = self.move_to_point_straight(self.current_waypoint)
        if error < self.ARRIVAL_RADIUS:
            self.state = "tookoff"
            self.previous_waypoint = np.array(self.pose)
            self.current_waypoint = self.waypoint_list.pop(0)
            self.route_line = Line(self.previous_waypoint, self.current_waypoint)
            print(self.instance_num, 'first waypoint', self.current_waypoint)

    def move_to_point(self, point):
        error = self.pose - point

        velocity = np.array([0.0, 0.0, 0.0])
        # Добавление вектора для уклонения от препятствий
        velocity += self.get_avoid_velocity()
        # Добавление вектора для поддержания траектории маршрута
        if np.linalg.norm(velocity) == 0:
            velocity += self.get_correction_velocity()
        # Вектор к точке
        velocity += self.get_point_velocity(velocity)

        self.set_vel(velocity)
        return np.linalg.norm(error)

    # Полёт к точке без коррекций
    def move_to_point_straight(self, point):
        error = self.pose - point

        # Вектор к точке
        velocity = -self.P_GAIN * error
        velocity_norm = np.linalg.norm(velocity)
        if velocity_norm > self.MAX_VELOCITY:
            velocity = velocity / velocity_norm * self.MAX_VELOCITY
        elif velocity_norm < self.MIN_VELOCITY:
            velocity = velocity / velocity_norm * self.MIN_VELOCITY

        self.set_vel(velocity)
        return np.linalg.norm(error)

    def follow_waypoint_list(self):
        error = self.move_to_point(self.current_waypoint)
        if self.is_passed_turn():
            if len(self.waypoint_list) != 0:
                self.previous_waypoint = np.array(self.current_waypoint)
                self.current_waypoint = self.waypoint_list.pop(0)
                self.route_line = Line(self.previous_waypoint, self.current_waypoint)
                print(self.instance_num, "\n\nNEXT\n\n")
                print(self.instance_num, 'cur waypoint', self.current_waypoint)
            else:
                self.state = "arrival"
                print(self.instance_num, "FINISH")

    def subscribe_on_topics(self):
        # локальная система координат, точка отсчета = место включения аппарата
        rospy.Subscriber(f"/mavros{self.instance_num}/local_position/pose", PoseStamped, self.pose_cb)
        rospy.Subscriber(f"/mavros{self.instance_num}/local_position/velocity_local", TwistStamped, self.velocity_cb)
        # глобальная система координат
        rospy.Subscriber(f"/mavros{self.instance_num}/global_position/global", NavSatFix, self.pose_global_cb)
        # состояние
        rospy.Subscriber(f"/mavros{self.instance_num}/state", State, self.state_cb)

    def pose_cb(self, msg):
        pose = msg.pose.position
        self.pose = np.array([pose.x, pose.y, pose.z])
        # print(self.instance_num, 'local', self.pose)

    def velocity_cb(self, msg):
        velocity = msg.twist.linear
        self.velocity = np.array([velocity.x, velocity.y, velocity.z])

    def pose_global_cb(self, msg):
        if len(self.pose_global) == 0 and len(self.pose) != 0:
            self.transform_waypoints(np.array([msg.latitude, msg.longitude, msg.altitude]))
        elif len(self.pose) == 0:
            self.pose_global = np.array([])
            return
        self.pose_global = np.array([msg.latitude, msg.longitude, msg.altitude])
        # Преобразование эллипсоидной высоты в высоту над уровнем моря
        # self.pose_global = np.array([msg.latitude, msg.longitude, msg.altitude - geoid_height(msg.latitude, msg.longitude)])
        # print(self.instance_num, 'global', self.pose_global)

    def state_cb(self, msg):
        self.mavros_state = msg



    def service_proxy(self, path, arg_type, *args, **kwds):
        service = rospy.ServiceProxy(f"/mavros{self.instance_num}/{path}", arg_type)
        ret = service(*args, **kwds)

        # rospy.loginfo(f"{1}: {path} {args}, {kwds} => {ret}")

    def arming(self, to_arm):
        if self.dt < 5:
            self.set_vel(np.array([0., 0., 3.]))
        if self.dt > 1:
            if self.mavros_state is not None and self.mavros_state.armed != to_arm:
                self.service_proxy("cmd/arming", CommandBool, to_arm)
        if self.dt > 5:
            self.state = "takeoff"
            self.current_waypoint = np.array([self.pose[0], self.pose[1], self.pose[2] + 3.])

    def set_mode(self, new_mode):
        if self.mavros_state is not None and self.mavros_state.mode != new_mode:
            self.service_proxy("set_mode", SetMode, custom_mode=new_mode)

    # Управление по скоростям, локальная система координат, направления совпадают с оными в глобальной системе координат
    def set_vel(self, velocity):
        self.pt.type_mask = self.pt.IGNORE_PX | self.pt.IGNORE_PY | self.pt.IGNORE_PZ | self.pt.IGNORE_AFX | self.pt.IGNORE_AFY | self.pt.IGNORE_AFZ | self.pt.IGNORE_YAW | self.pt.IGNORE_YAW_RATE

        # Скорость, направление на восток
        self.pt.velocity.x = velocity[0]
        # Скорость, направление на север
        self.pt.velocity.y = velocity[1]
        # Скорость, направление вверх
        self.pt.velocity.z = velocity[2]

    # Управление по точкам, локальная система координат.
    def set_pos(self, pose):
        self.pt.type_mask = self.pt.IGNORE_VX | self.pt.IGNORE_VY | self.pt.IGNORE_VZ | self.pt.IGNORE_AFX | self.pt.IGNORE_AFY | self.pt.IGNORE_AFZ | self.pt.IGNORE_YAW | self.pt.IGNORE_YAW_RATE
        # Смещение на восток
        self.pt.position.x = pose[0]
        # Смещение на север
        self.pt.position.y = pose[1]
        # Высота, направление вверх
        self.pt.position.z = pose[2]

    # Управление по точкам, глобальная система координат.
    def set_pos_global(self, pose):
        self.gpt.type_mask = self.gpt.IGNORE_VX | self.gpt.IGNORE_VY | self.gpt.IGNORE_VZ | self.gpt.IGNORE_AFX | self.gpt.IGNORE_AFY | self.gpt.IGNORE_AFZ | self.gpt.IGNORE_YAW | self.gpt.IGNORE_YAW_RATE
        # Смещение по широте
        self.gpt.latitude = pose[0]
        # Смещение по долготе
        self.gpt.longitude = pose[1]
        # Высота, направление вверх
        self.gpt.altitude = pose[2]

    # Преобразование точек маршрута из глобальной системы координат в локальную
    def transform_waypoints(self, pose_global):
        for i in range(len(self.waypoint_list)):
            delta = enu_vector(pose_global, self.waypoint_list[i])
            self.waypoint_list[i] = self.pose + delta
        print("TRANSFORMED WAYPOINTS")
        print(self.waypoint_list)

    # Получение вектора для ухода от препятствий
    def get_avoid_velocity(self):
        # Перпендикулярная плоскость
        self.surface = Surface(self.route_line.pr_point(self.pose), self.current_waypoint - self.previous_waypoint)
        valid_obstacles = self.filter_obstacles()
        # Проецируем на плоскости местоположения препятствий
        for i in range(len(valid_obstacles)):
            valid_obstacles[i] = self.surface.pr_point(valid_obstacles[i] + self.pose)
        # новый базис
        basis = Basis(self.route_line.pr_point(self.pose), self.current_waypoint - self.previous_waypoint)
        rect_w = 1.5
        rect_h = 0.8
        # Перевод проекций в новый базис
        for i in range(len(valid_obstacles)):
            valid_obstacles[i] = basis.to_new_basis(valid_obstacles[i])

        res = np.array([0.0, 0.0, 0.0])
        if len(valid_obstacles) == 0:
            return res
        # for i in range(len(valid_obstacles)):
        #     # if valid_obstacles[i][0] > 0 and valid_obstacles[i][1] > 0:
        #     print(valid_obstacles[i])
        # print('\n\n')

        target = self.get_avoid_point(valid_obstacles, rect_w, rect_h, 0.3, 360)
        print(self.instance_num, target)
        if target is None:
            return res
        target = basis.to_old_basis(target)
        res = (target - self.pose) * self.AVOID_P_GAIN

        # valid_obstacles = self.filter_obstacles()
        # for obstacle in valid_obstacles:
        #     res += -(obstacle / np.linalg.norm(obstacle)) * (self.AVOID_RADIUS / np.linalg.norm(obstacle))
        # res = self.surface.pr_point(self.pose + res) - self.pose
        # if np.linalg.norm(res) > self.MAX_AVOID_SPEED:
        #     res = res / np.linalg.norm(res) * self.MAX_AVOID_SPEED

        # print(self.instance_num, res)

        return res

    def get_correction_velocity(self):
        # if np.linalg.norm(self.pose - self.current_waypoint) < 7:
        #     return np.array([0.0, 0.0, 0.0])
        error = self.route_line.get_point_dist(self.pose)
        res = (self.route_line.pr_point(self.pose) - self.pose) * self.TRAJECTORY_CORRECTION
        if res[2] / self.TRAJECTORY_CORRECTION < -2.0:
            res += np.array([0.0, 0.0, res[2] / self.TRAJECTORY_CORRECTION]) * self.TRAJECTORY_CORRECTION_Z
        if error > self.MAXIMAL_DEVIATION:  # or res[2] / self.TRAJECTORY_CORRECTION < -2.0:
            print(self.instance_num, "CRITICAL ROUTE ERROR", self.dt)
            res = res / np.linalg.norm(res) * self.MAX_AVOID_SPEED * 2
        elif np.linalg.norm(res) > self.MAX_AVOID_SPEED:
            res = res / np.linalg.norm(res) * self.MAX_AVOID_SPEED
        # print(self.instance_num, "ROUTE CORRECTION", res)
        return res

    def get_point_velocity(self, velocity):
        # Меры против столкновений
        for i in range(1, INSTANCES_NUM+1):
            if i != self.instance_num and np.linalg.norm(self.pose - controllers[i-1].pose) < self.CRITICAL_DISTANCE \
                and (len(self.waypoint_list) > len(controllers[i-1].waypoint_list) or \
                     np.linalg.norm(self.current_waypoint - self.pose) > np.linalg.norm(controllers[i-1].current_waypoint - controllers[i-1].pose)):
                # print(self.instance_num, "ANTI-COLLISION MEASURES", self.dt)
                return np.array([0.0, 0.0, 0.0])

        max_speed = self.MAX_VELOCITY
        if np.linalg.norm(self.pose - self.current_waypoint) < self.APPROACH_RADIUS or \
                self.current_waypoint[2] < self.previous_waypoint[2] or len(self.filter_obstacles()) > 4:
            max_speed = self.APPROACH_VELOCITY
        if max_speed**2 > np.linalg.norm(velocity)**2:
            speed_to_point = math.sqrt(max_speed**2 - np.linalg.norm(velocity)**2)
        else:
            speed_to_point = self.MIN_VELOCITY
        route_vect = (self.current_waypoint - self.previous_waypoint) / np.linalg.norm((self.current_waypoint - self.previous_waypoint))
        velocity_to_point = route_vect * speed_to_point  # -error / np.linalg.norm(error) * speed_to_point
        # velocity_to_point = -self.P_GAIN * error
        # if np.linalg.norm(velocity_to_point) > self.MAX_VELOCITY:
        #     velocity_to_point = velocity_to_point / np.linalg.norm(velocity_to_point) * self.MAX_VELOCITY
        return velocity_to_point


    def filter_obstacles(self):
        res = []
        for vect in obstacles[self.instance_num]:
            pose = self.pose + vect
            if self.surface.substitute_point(pose) >= 0 and np.linalg.norm(vect) <= self.AVOID_RADIUS or \
                    self.surface.substitute_point(pose) < 0 and self.surface.get_point_dist(pose) <= 5 and \
                    np.linalg.norm(vect) <= 5:
                res.append(np.array(vect))
        return res

    def is_passed_turn(self):
        route_vect = self.current_waypoint - self.previous_waypoint
        route_vect = route_vect / np.linalg.norm(route_vect)
        sur = Surface(self.current_waypoint - route_vect * self.CHECKPOINT_SURFACE_BIAS, route_vect)
        if sur.substitute_point(self.pose) > 0:
            return True
        return False
        # return error < self.ARRIVAL_RADIUS

    def get_avoid_point(self, obstacles, rect_w, rect_h, vect_step_len, step_count):
        search_vect = np.array([.0, .0, .0])
        step_angle = 2 * math.pi / step_count  # Угол поворота вектора поиска
        turn_axis = np.array([.0, .0, 1.0])  # Ось поворота вектора (по направлению движения)
        iter_count = 0
        while np.linalg.norm(search_vect) < self.MAXIMAL_DEVIATION + 2.0:
            search_vect = np.array([.0, iter_count * vect_step_len, .0])  # Вектор для поиска места для уворота
            # print("vect len", np.linalg.norm(search_vect))
            for i in range(step_count):
                cross_flag = False
                for obstacle in obstacles:
                    if is_crossing_rectangles(search_vect, obstacle, rect_w, rect_h):
                        cross_flag = True
                        break
                if not cross_flag:
                    if np.linalg.norm(search_vect) == 0.0:
                        return None
                    return search_vect
                # print(search_vect, "is bad")
                if np.linalg.norm(search_vect) == 0.0:
                    break
                # Поворот вектора
                search_vect = turn_vector(search_vect, turn_axis, step_angle)
            iter_count += 1
        return np.array([self.MAXIMAL_DEVIATION + 2.0, 0.0, 0.0])


def radar_cb(msg):
    for i in range(1, INSTANCES_NUM+1):
        obstacles[i] = []
    msg = str(msg.data).split()
    for i in range(1, len(msg), 4):
        obstacles[int(msg[i])].append(np.array(list(map(float, [msg[i+1], msg[i+2], msg[i+3]]))))
    # if len(obstacles[1]) > 0:
    #     print(obstacles)

def on_shutdown_cb():
    rospy.logfatal("shutdown")


# Получение точек маршрута
def get_waypoints():
    with open("../tasks/avoid/gps_mission_2.rt") as f:
        s = f.read()

    lst = s.split('\n')
    lst = lst[:-1]
    for i in range(len(lst)):
        lst[i] = list(map(float, lst[i].split()))
        # Преобразование высоты над уровнем моря в эллипсоидную высоту
        lst[i][2] += geoid_height(lst[i][0], lst[i][1])
    lst = list(map(np.array, lst))
    return lst


# преобразование дельты gps в дельту xyz по простейшей формуле (из интернета)
def enu_vector(g1, g2):
    n = g2[0] - g1[0]
    e = g2[1] - g1[1]
    u = g2[2] - g1[2]

    refLat = (g1[0]+g2[0])/2

    nm = n * 333400 / 3  # deltaNorth * 40008000 / 360
    em = e * 1001879 * math.cos(math.radians(refLat)) / 9  # deltaEast * 40075160 *cos(refLatitude) / 360

    return [em, nm, u]


def geoid_height(lat, lon):
    """Calculates AMSL to ellipsoid conversion offset.
    Uses EGM96 data with 5' grid and cubic interpolation.
    The value returned can help you convert from meters
    above mean sea level (AMSL) to meters above
    the WGS84 ellipsoid.

    If you want to go from AMSL to ellipsoid height, add the value.

    To go from ellipsoid height to AMSL, subtract this value.
    """
    return _egm96.height(lat, lon)


# ROS/Mavros работают в системе координат ENU(Восток-Север-Вверх), автопилот px4 и протокол сообщений Mavlink используют систему координат NED(Север-Восток-Вниз)
# см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED


def offboard_loop(controllers):
     # цикл управления всеми дронами
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        for i in range(INSTANCES_NUM):
            if len(controllers[i].pose) == 0:
                continue
            controllers[i].offboard_loop()
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node(node_name)
    rospy.loginfo(node_name + " started")

    controllers = [CopterController(i) for i in range(1, INSTANCES_NUM+1)]
    rospy.Subscriber("/ca_radar/rel_obstacles", String, radar_cb)
    for i in range(1, INSTANCES_NUM+1):
        obstacles[i] = []


    rospy.on_shutdown(on_shutdown_cb)

    try:
        offboard_loop(controllers)
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
