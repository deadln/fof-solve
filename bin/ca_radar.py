#!/usr/bin/env python3
# coding=utf8

import rospy
import argparse
import airsim
import time

from std_msgs.msg import String

freq = 20

def arguments():
  global args

  parser = argparse.ArgumentParser()

  parser.add_argument("model", help="model name")
  parser.add_argument("num", type=int, help="models number")

  parser.add_argument("model2", help="obstacle model name")
  parser.add_argument("num2", type=int, help="obstacles number")

  parser.add_argument("radius", type=float, help="radar radius")

  args = parser.parse_args()

def loop():
  global args

  """
  Формат строки для публикации в топик, значения разделены пробелами:
  ВРЕМЯ_СЕК НОМЕР_АППАРАТА1 e1 n1 u1 НОМЕР_АППАРАТА1 e2 n2 u2 НОМЕР_АППАРАТА2 e3 n3 u3 НОМЕР_АППАРАТА2 e4 n4 u4 ...
  где
   ВРЕМЯ_СЕК - текущее время в секундах
   НОМЕР_АППАРАТА - номер аппарата
   e,n,u - относительный вектор от аппарата до препятствия в системе Восток-Север-Верх
  """
  pub = rospy.Publisher("~rel_obstacles", String, queue_size=10)

  rate = rospy.Rate(freq)
  while not rospy.is_shutdown():
    ps = {}
    for n in range(1,args.num+1):
      m = args.model + str(n)
      p = client.simGetObjectPose(m).position
      if not p.containsNan():
        ps[n] = p


    ps2 = []
    for n2 in range(1,args.num2+1):
      m2 = args.model2 + str(n2)
      p2 = client.simGetObjectPose(m2).position
      if not p2.containsNan():
        ps2.append(p2)

    t = time.time()

    s=f"{t}"
    for n, p in ps.items():
      for p2 in ps2:
        if p.distance_to(p2) < args.radius:
          r = p2-p
          s+=f" {n} {r.y_val} {r.x_val} {-r.z_val}" #NED to ENU

    pub.publish(s)

    rate.sleep()

if __name__ == '__main__':
  global client

  client = airsim.MultirotorClient()
  client.confirmConnection()

  arguments()

  rospy.init_node("ca_radar")

  try:
    loop()
  except rospy.ROSInterruptException:
    pass

  rospy.spin()
