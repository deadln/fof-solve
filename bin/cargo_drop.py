#!/usr/bin/env python3
# coding=utf8

import rospy
import argparse
import airsim
import time
import math

from pathlib import Path
from airsim.types import Vector3r

import formations_gen as fgen

from std_msgs.msg import String

freq = 20

# преобразование дельты gps в дельту xyz по простейшей формуле (из интернета)
def enu_vector(g1, g2):
    n = g2[0] - g1[0]
    e = g2[1] - g1[1]
    u = g2[2] - g1[2]

    refLat = (g1[0]+g2[0])/2

    nm = n * 333400 / 3  # deltaNorth * 40008000 / 360
    em = e * 1001879 * math.cos(math.radians(refLat)) / 9  # deltaEast * 40075160 *cos(refLatitude) / 360

    return [em, nm, u]

def loop():
  global args

  """
  Формат строки публикуемой в топик, значения разделены пробелами:
  ВРЕМЯ_СЕК ШИРОТА ДОЛГОТА ВЫСОТА
  где ВРЕМЯ_СЕК - время выдачи, в секундах, ШИРОТА ДОЛГОТА ВЫСОТА - глобальные координаты точки назначения.
  """
  pub = rospy.Publisher("~point", String, queue_size=10)

  rate = rospy.Rate(freq)

  to_publish = False
  while not rospy.is_shutdown():
    p = client.simGetObjectPose(args.model).position
    if not p.containsNan():
      if p.distance_to(p0) < distance:
        to_publish = True

    t = time.time()

    s = f"{t}"
    if to_publish:
      s += " " + g_str

    pub.publish(s)

    rate.sleep()

def arguments():
  global args, p0, g_str, distance

  parser = argparse.ArgumentParser()

  parser.add_argument("model", help="model name")

  parser.add_argument("gps_point", type=Path, help="file with GPS coordinates of point")
  parser.add_argument("distance", type=Path, help="file with distance to start publishing point")

  parser.add_argument("--gps_ref", type=float, nargs=3, help="GPS reference point, center of local coordinate system")

  args = parser.parse_args()

  g = fgen.read_values("gps_point", args.gps_point, 3, row_num = 1)[0]
  g_str = ' '.join(map(str,g))

  distance = fgen.read_values("distance", args.distance, 1, row_num = 1)[0][0]

  enu0 = enu_vector(args.gps_ref, g)

  p0 = Vector3r(enu0[1], enu0[0], -enu0[2]) #ned

if __name__ == '__main__':
  global client

  client = airsim.MultirotorClient()
  client.confirmConnection()

  arguments()

  rospy.init_node("cargo_drop")

  try:
    loop()
  except rospy.ROSInterruptException:
    pass

  rospy.spin()
