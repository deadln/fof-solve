#!/usr/bin/env python3
# coding=utf8

import rospy
import time
import sys
import math
import numpy as np

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState, GlobalPositionTarget
from geographic_msgs.msg import GeoPointStamped

from mavros_msgs.srv import SetMode, CommandBool, CommandVtolTransition, CommandHome

from pygeodesy.geoids import GeoidPGM
_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

freq = 20  # Герц, частота посылки управляющих команд аппарату
node_name = "offboard_node"
lz = {}


class CopterController():
    def __init__(self):
        self.state = "disarm"
        # создаем топики, для публикации управляющих значений:
        self.pub_pt = rospy.Publisher(f"/mavros{1}/setpoint_raw/local", PositionTarget, queue_size=10)
        self.pub_gpt = rospy.Publisher(f"/mavros{1}/setpoint_raw/global", GlobalPositionTarget, queue_size=10)
        self.pt = PositionTarget()
        self.pt.coordinate_frame = self.pt.FRAME_LOCAL_NED
        self.gpt = GlobalPositionTarget()
        self.gpt.coordinate_frame = self.gpt.FRAME_GLOBAL_INT

        self.t0 = time.time()
        self.dt = 0

        # params
        self.p_gain = 2
        self.max_velocity = 5
        self.arrival_radius = 0.6
        self.arrival_radius_global = 0.0001
        # self.waypoint_list = [np.array([106., 509., 10.]), np.array([5., 5., 100.]), np.array([100., 100., 100.]),
        #                       np.array([0., 0., 5.])]
        # self.waypoint_list = [np.array([0., 0., 332.]), np.array([1., 1., 332.]),
        #                       np.array([1.0045822, 1.0009613, 232.])]
        self.waypoint_list = get_waypoints()


        self.current_waypoint = np.array([0., 0., 0.])
        self.pose = np.array([])
        self.pose_global = np.array([])
        self.velocity = np.array([0., 0., 0.])
        self.mavros_state = State()
        self.subscribe_on_topics()

    def offboard_loop(self):
        # цикл управления
        rate = rospy.Rate(freq)
        while not rospy.is_shutdown():
            # print('state:', self.state)
            # print('cur waypoint', self.current_waypoint)
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
            # if self.state == "takeoff":
            self.pub_pt.publish(self.pt)
            # else:
            #     self.pub_gpt.publish(self.gpt)

            rate.sleep()

    # взлет коптера
    def takeoff(self):
        error = self.move_to_point(self.current_waypoint)
        if error < self.arrival_radius:
            self.state = "tookoff"
            self.current_waypoint = self.waypoint_list.pop(0)
            print('first waypoint', self.current_waypoint)
            # self.current_waypoint = np.array([self.pose_global[0], self.pose_global[1], self.pose_global[2] + 10.])

    def move_to_point(self, point):
        # print('point', point)
        error = self.pose - point

        velocity = -self.p_gain * error
        velocity_norm = np.linalg.norm(velocity)
        if velocity_norm > self.max_velocity:
            velocity = velocity / velocity_norm * self.max_velocity
        self.set_vel(velocity)
        return np.linalg.norm(error)

    def move_to_point_global(self, point):
        error_h = np.array([self.pose_global[0] - point[0], self.pose_global[1] - point[1]])
        error_v = abs(self.pose_global[2] - point[2])

        self.set_pos_global(point)
        # velocity = -self.p_gain * error
        # velocity_norm = np.linalg.norm(velocity)
        # if velocity_norm > self.max_velocity:
        #     velocity = velocity / velocity_norm * self.max_velocity
        # self.set_vel(velocity)
        return np.linalg.norm(error_h), error_v

    def follow_waypoint_list(self):
        error = self.move_to_point(self.current_waypoint)
        # error_h, error_v = self.move_to_point_global(self.current_waypoint)
        if error < self.arrival_radius:
            if len(self.waypoint_list) != 0:
                self.current_waypoint = self.waypoint_list.pop(0)
                print("\n\nNEXT\n\n")
                print('cur waypoint', self.current_waypoint)
            else:
                self.state = "arrival"

    def subscribe_on_topics(self):
        # локальная система координат, точка отсчета = место включения аппарата
        rospy.Subscriber("/mavros1/local_position/pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("/mavros1/local_position/velocity_local", TwistStamped, self.velocity_cb)
        # глобальная система координат
        rospy.Subscriber("/mavros1/global_position/global", NavSatFix, self.pose_global_cb)
        # состояние
        rospy.Subscriber("/mavros1/state", State, self.state_cb)

    def pose_cb(self, msg):
        pose = msg.pose.position
        self.pose = np.array([pose.x, pose.y, pose.z])
        # print('local', self.pose)

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
        # print('global', self.pose_global)

    def state_cb(self, msg):
        self.mavros_state = msg

    def arming(self, to_arm):
        if self.dt < 10:
            self.set_vel(np.array([0., 0., 3.]))
        if self.dt > 7.5:
            if self.mavros_state is not None and self.mavros_state.armed != to_arm:
                service_proxy("cmd/arming", CommandBool, to_arm)
        if self.dt > 10:
            self.state = "takeoff"
            self.current_waypoint = np.array([self.pose[0], self.pose[1], self.pose[2] + 20.])

    def set_mode(self, new_mode):
        if self.mavros_state is not None and self.mavros_state.mode != new_mode:
            service_proxy("set_mode", SetMode, custom_mode=new_mode)

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

    def set_pos_global(self, pose):
        self.gpt.type_mask = self.gpt.IGNORE_VX | self.gpt.IGNORE_VY | self.gpt.IGNORE_VZ | self.gpt.IGNORE_AFX | self.gpt.IGNORE_AFY | self.gpt.IGNORE_AFZ | self.gpt.IGNORE_YAW | self.gpt.IGNORE_YAW_RATE
        # self.gpt.type_mask = self.gpt.IGNORE_VX | self.gpt.IGNORE_VY | self.gpt.IGNORE_AFX | self.gpt.IGNORE_AFY | self.gpt.IGNORE_AFZ | self.gpt.IGNORE_YAW | self.gpt.IGNORE_YAW_RATE
        # Смещение по широте
        self.gpt.latitude = pose[0]
        # Смещение по долготе
        self.gpt.longitude = pose[1]
        # Высота, направление вверх
        self.gpt.altitude = pose[2]

        # if self.dt % 10 < 5:
        #     self.gpt.velocity.z = 5
        #     print("UP")
        # else:
        #     self.gpt.velocity.z = -5
        #     print("DOWN")


    def transform_waypoints(self, pose_global):
        for i in range(len(self.waypoint_list)):
            delta = enu_vector(pose_global, self.waypoint_list[i])
            self.waypoint_list[i] = self.pose + delta
        print("TRANSFORMED WAYPOINTS")
        print(self.waypoint_list)


def service_proxy(path, arg_type, *args, **kwds):
    service = rospy.ServiceProxy(f"/mavros{1}/{path}", arg_type)
    ret = service(*args, **kwds)

    # rospy.loginfo(f"{1}: {path} {args}, {kwds} => {ret}")


def on_shutdown_cb():
    rospy.logfatal("shutdown")


# Получение точек маршрута
def get_waypoints():
    with open("../tasks/avoid/gps_mission_1.rt") as f:
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


if __name__ == '__main__':
    rospy.init_node(node_name)
    rospy.loginfo(node_name + " started")

    copter_controller = CopterController()

    rospy.on_shutdown(on_shutdown_cb)

    try:
        copter_controller.offboard_loop()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()