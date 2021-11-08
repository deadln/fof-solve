#!/usr/bin/env python3
# coding=utf8

import rospy
import time
import sys
import math

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState, ParamValue
from geographic_msgs.msg import GeoPointStamped

from mavros_msgs.srv import SetMode, CommandBool, CommandVtolTransition, CommandHome, ParamSet, ParamGet

instances_num = 1 #количество аппаратов
freq = 20 #Герц, частота посылки управляющих команд аппарату
node_name = "offboard_node"
data = {}
lz = {}

def subscribe_on_mavros_topics(suff, data_class):
  #подписываемся на Mavros топики всех аппаратов
  for n in range(1, instances_num + 1):
    data[n] = {}
    topic = f"/mavros{n}/{suff}"
    rospy.Subscriber(topic, data_class, topic_cb, callback_args = (n, suff))

def topic_cb(msg, callback_args):
  n, suff = callback_args
  data[n][suff] = msg

def service_proxy(n, path, arg_type, *args, **kwds):
  service = rospy.ServiceProxy(f"/mavros{n}/{path}", arg_type)
  ret = service(*args, **kwds)

  rospy.loginfo(f"{n}: {path} {args}, {kwds} => {ret}")
  return ret

def arming(n, to_arm):
  d = data[n].get("state")
  if d is not None and d.armed != to_arm:
    service_proxy(n, "cmd/arming", CommandBool, to_arm)

def set_mode(n, new_mode):
  d = data[n].get("state")
  if d is not None and d.mode != new_mode:
    service_proxy(n, "set_mode", SetMode, custom_mode=new_mode)

def set_param(n, name, i = 0, r = 0.0):
  return service_proxy(n, "param/set", ParamSet, name, ParamValue(i,r)).success

def get_param(n, name):
  ret = service_proxy(n, "param/get", ParamGet, param_id=name)

  v = None
  if ret.success:
    i = ret.value.integer
    r = ret.value.real
    if i != 0:
      v = i
    elif r != 0.0:
      v = r
    else:
      v = 0

  return v

def subscribe_on_topics():
  # глобальная (GPS) система координат
  # высота задана в элипсоиде WGS-84 и не равна высоте над уровнем моря, см. https://wiki.ros.org/mavros#mavros.2FPlugins.Avoiding_Pitfalls_Related_to_Ellipsoid_Height_and_Height_Above_Mean_Sea_Level
  subscribe_on_mavros_topics("global_position/global", NavSatFix)

  #локальная система координат, точка отсчета = место включения аппарата
  subscribe_on_mavros_topics("local_position/pose", PoseStamped)
  subscribe_on_mavros_topics("local_position/velocity_local", TwistStamped)

  #состояние
  subscribe_on_mavros_topics("state", State)
  subscribe_on_mavros_topics("extended_state", ExtendedState)


def on_shutdown_cb():
  rospy.logfatal("shutdown")

#ROS/Mavros работают в системе координат ENU(Восток-Север-Вверх), автопилот px4 и протокол сообщений Mavlink используют систему координат NED(Север-Восток-Вниз)
#см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED

#Управление по точкам, локальная система координат.
def set_pos(pt, x, y, z):
  pt.type_mask = pt.IGNORE_VX | pt.IGNORE_VY | pt.IGNORE_VZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

  #Смещение на восток
  pt.position.x = x
  #Смещение на север
  pt.position.y = y
  #Высота, направление вверх
  pt.position.z = z

#Управление по скоростям, локальная система координат, направления совпадают с оными в глобальной системе координат
def set_vel(pt, vx, vy, vz):
  pt.type_mask = pt.IGNORE_PX | pt.IGNORE_PY | pt.IGNORE_PZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

  #Скорость, направление на восток
  pt.velocity.x = vx
  #Скорость, направление на север
  pt.velocity.y = vy
  #Скорость, направление вверх
  pt.velocity.z = vz

#взлет коптера
def mc_takeoff(pt, n, dt):
  if dt<10:
    #скорость вверх
    set_vel(pt, 0, 0, 4)

    #армимся и взлетаем с заданной скоростью
    if dt>5:
      arming(n, True)

#пример управления коптерами
def mc_example(pt, n, dt):
  mc_takeoff(pt, n, dt)

  if dt>10 and dt<15:
    #скорость вверх
    set_vel(pt, 0, 0, 1)

  #летим в одном направлении, разносим по высоте
  if dt>15 and dt<20:
    set_vel(pt, 5, 0, (n-2)/2)

  #первый коптер летит по квадрату, остальные следуют с такой же горизонтальнй скоростью как первый
  if dt>20 and dt<30:
    if n == 1:
      if dt>20:
        set_vel(pt, 0, -5, 0)

      if dt>24:
        set_vel(pt, -5, 0, 0)

      if dt>27:
        set_vel(pt, 0, 5, 0)
    else:
      v1 = data[1]["local_position/velocity_local"].twist.linear
      set_vel(pt, v1.x, v1.y, 0)

  #направляем каждого в свою точку
  if dt>30 and dt<35:
    set_pos(pt, 0, (n-2)*3, 10)

  #снижаем на землю
  if dt>35:
    set_vel(pt, 0, 0, -1)

def offboard_loop():
  pub_pt = {}
  #создаем топики, для публикации управляющих значений
  for n in range(1, instances_num + 1):
    pub_pt[n] = rospy.Publisher(f"/mavros{n}/setpoint_raw/local", PositionTarget, queue_size=10)

  pt = PositionTarget()
  #см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
  pt.coordinate_frame = pt.FRAME_LOCAL_NED

  t0 = time.time()

  #цикл управления
  rate = rospy.Rate(freq)
  while not rospy.is_shutdown():
    dt = time.time() - t0

    #управляем каждым аппаратом централизованно
    for n in range(1, instances_num + 1):
      set_mode(n, "OFFBOARD")
      mc_example(pt, n, dt)
      pub_pt[n].publish(pt)

    rate.sleep()

if __name__ == '__main__':
  if len(sys.argv) > 1:
    instances_num = int(sys.argv[1])

  rospy.init_node(node_name)
  rospy.loginfo(node_name + " started")

  subscribe_on_topics()

  rospy.on_shutdown(on_shutdown_cb)

  try:
    offboard_loop()
  except rospy.ROSInterruptException:
    pass

  rospy.spin()
