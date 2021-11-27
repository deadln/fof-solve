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

from avoid.geometry import *

freq = 20  # Герц, частота посылки управляющих команд аппарату
node_name = "statistics_node"
# Количество аппаратов
INSTANCES_NUM = 3

controllers = []
obstacles = {}
bias = {}

state = "on_land"
waypoints_num = 0
start_time = -1
ground_level = -100
finished_count = 0


class CopterController():
    def __init__(self, instance_num):
        self.state = "on_land"
        self.instance_num = instance_num

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
        self.AVOID_RADIUS = 20.0  # Радиус обнаружения "валидных" препятствий
        self.MAX_AVOID_SPEED = 5.0  # Максимальная длина вектора уклонения от препятствий
        self.AVOID_P_GAIN = 2.4
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
        if self.state == "on_land":
            self.check_takeoff()
        elif self.state == "tookoff":
            self.follow_waypoint_list()
        elif self.state == "finished":
            return

    def check_takeoff(self):
        if ground_level != -100 and self.pose[2] - ground_level > 2:
            self.state = 'tookoff'
            self.previous_waypoint = np.array(self.pose)
            self.current_waypoint = self.waypoint_list.pop(0)
            self.route_line = Line(self.previous_waypoint, self.current_waypoint)


    def follow_waypoint_list(self):
        global finished_count
        # error = self.move_to_point(self.current_waypoint)
        if self.is_passed_turn():
            if len(self.waypoint_list) != 0:
                self.previous_waypoint = np.array(self.current_waypoint)
                self.current_waypoint = self.waypoint_list.pop(0)
                self.route_line = Line(self.previous_waypoint, self.current_waypoint)
            else:
                self.state = "finished"
                print(self.instance_num, "FINISH")
                finished_count += 1

    def subscribe_on_topics(self):
        # локальная система координат, точка отсчета = место включения аппарата
        rospy.Subscriber(f"/mavros{self.instance_num}/local_position/pose", PoseStamped, self.pose_cb)
        rospy.Subscriber(f"/mavros{self.instance_num}/local_position/velocity_local", TwistStamped, self.velocity_cb)
        # глобальная система координат
        rospy.Subscriber(f"/mavros{self.instance_num}/global_position/global", NavSatFix, self.pose_global_cb)
        # состояние
        rospy.Subscriber(f"/mavros{self.instance_num}/state", State, self.state_cb)

    def pose_cb(self, msg):
        global ground_level
        pose = msg.pose.position
        self.pose = np.array([pose.x, pose.y, pose.z])
        if ground_level == -100:
            ground_level = pose.z
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

    def set_mode(self, new_mode):
        if self.mavros_state is not None and self.mavros_state.mode != new_mode:
            self.service_proxy("set_mode", SetMode, custom_mode=new_mode)



    # Преобразование точек маршрута из глобальной системы координат в локальную
    def transform_waypoints(self, pose_global):
        for i in range(len(self.waypoint_list)):
            delta = enu_vector(pose_global, self.waypoint_list[i])
            self.waypoint_list[i] = self.pose + delta
        # print("TRANSFORMED WAYPOINTS")
        # print(self.waypoint_list)


    def is_passed_turn(self):
        route_vect = self.current_waypoint - self.previous_waypoint
        route_vect = route_vect / np.linalg.norm(route_vect)
        sur = Surface(self.current_waypoint - route_vect * self.CHECKPOINT_SURFACE_BIAS, route_vect)
        if sur.substitute_point(self.pose) > 0:
            return True
        return False
        # return error < self.ARRIVAL_RADIUS



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
    global waypoints_num
    with open("tasks/avoid/gps_mission_2.rt") as f:
        s = f.read()

    lst = s.split('\n')
    lst = lst[:-1]
    for i in range(len(lst)):
        lst[i] = list(map(float, lst[i].split()))
        # Преобразование высоты над уровнем моря в эллипсоидную высоту
        lst[i][2] += geoid_height(lst[i][0], lst[i][1])
    lst = list(map(np.array, lst))
    if waypoints_num == 0:
        waypoints_num = len(lst)
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


def calculate_bias():
    global waypoints_num
    global bias
    for i in range(1, INSTANCES_NUM+1):
        if len(controllers[i-1].waypoint_list) >= waypoints_num-1 or controllers[i-1].state != "tookoff":
            continue
        bias[i].append(controllers[i-1].route_line.get_point_dist(controllers[i-1].pose))
        # print(i, bias[i][-1])

def show_MSE_stats():
    global bias
    summ = 0
    for i in range(1, INSTANCES_NUM+1):
        avg = sum(bias[i]) / len(bias[i])
        mse = math.sqrt(sum([(x - avg)**2 for x in bias[i]]) / len(bias[i]))
        print(i, "MSE", mse)
        summ += mse
    print("summ MSE", summ)


def offboard_loop(controllers):
    global start_time
    global state
    global finished_count
    t0 = time.time()
    dt = 0
     # цикл управления всеми дронами
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        dt = time.time() - t0
        for i in range(INSTANCES_NUM):
            if len(controllers[i].pose) == 0:
                continue
            if state == 'on_land' and controllers[i].state == 'tookoff':
                print("FLIGHT")
                state = 'flight'
                start_time = dt
            if state == 'flight':
                calculate_bias()
            if state == 'flight' and finished_count == INSTANCES_NUM:
                print("FLIGHT TIME", dt - start_time)
                state = 'finished'
                show_MSE_stats()
            if state != "finished":
                controllers[i].offboard_loop()
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node(node_name)
    rospy.loginfo(node_name + " started")

    controllers = [CopterController(i) for i in range(1, INSTANCES_NUM+1)]
    rospy.Subscriber("/ca_radar/rel_obstacles", String, radar_cb)
    for i in range(1, INSTANCES_NUM+1):
        obstacles[i] = []
        bias[i] = []


    rospy.on_shutdown(on_shutdown_cb)

    try:
        offboard_loop(controllers)
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
