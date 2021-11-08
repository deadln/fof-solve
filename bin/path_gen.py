#!/usr/bin/env python3
# coding=utf8

import rospy
import argparse
import math
from collections import deque
import airsim
from airsim.types import Vector3r, Pose

from pathlib import Path

from std_msgs.msg import String

import formations_gen as fgen

freq = 10
poses_dt=0.5

walls = {}
to_plot=False

def plot_data(cs, wall_ps):
  client.simPlotTransforms([Pose()], scale=50.0, thickness=3.0, is_persistent=True)

  points = []
  for c in cs:
    for i in range(len(c)//3):
      j=3*i
      points.append(Vector3r(c[j+1], c[j], -c[j+2])) #ned

  client.simPlotLineStrip(points, color_rgba=[1.0, 1.0, 0.0, 1.0], thickness=10.0, is_persistent=True)

  if args.walls:
    for wall in wall_ps:
      for window in wall:
        window = window + [window[0]]
        points = []
        for corner in window:
          points.append(Vector3r(corner[1], corner[0], -corner[2])) #ned

        client.simPlotLineStrip(points, color_rgba=[1.0, 0.0, 0.0, 1.0], thickness=15.0, is_persistent=True)

def arguments():
  global args, borders, centrals, brd_i

  parser = argparse.ArgumentParser()
  parser.add_argument("model", help="model name")
  parser.add_argument("num", type=int, help="models number")

  """
  Каждая строка файла центральных линий содержит координаты произвольного количества 3D точек, разделенных пробелами:
  x1 y1 z1 x2 y2 z2 ... xN yN zN
  Каждая центральная линия должна упираться в стену с окнами, последняя - в глухую стену.
  """
  parser.add_argument("centrals", type=Path, help="file with centrals")

  """
  Ширина границы, пересекаемую аппаратами, которая ортогональна конечному отрезку центральной линии и проходит через её конечную точку
  """
  parser.add_argument("width", type=float, help="width of wall (border)")

  """
  Каталог с файлами стен, каждый файл с произвольным именем содержит строки из 4 чисел, разделенных пробелами.
  Каждая строка содержит относительные координаты центра окна (от конечной точки центральной линии) и размеры этого окна в системе отсчета Право-Верх,если смотреть по ходу движения
  x y w h
  """
  parser.add_argument("--walls", type=Path, help="directory with walls files")

  """
  Имена стен (имя файла стены без расширения), соответствующие окончаниям центральных линий в количестве на одну меньше (глухая стена не описыается), чем центральных линий
  Если имена стен не указаны, берется отсортированный список всех файлов каталога или список чисел начиная с 1.
  """
  parser.add_argument("--names", nargs='+', help="walls names for sequence")

  args = parser.parse_args()

  centrals = fgen.read_values("centrals", args.centrals, 3, num_blocks = 2)

  if not args.names:
    if args.walls:
      args.names = [ p.name.split('.')[0] for p in sorted(args.walls.glob('*')) ]
      if len(args.names) == 0:
        print("empty walls directory")
        exit()
    else:
      args.names = [ str(i) for i in range(1,len(centrals))]

  if len(centrals) != len(args.names) + 1:
    print("number of centrals must be equal to number of names plus one")
    exit()

  fgen.args = args

  vs={}
  if args.walls:
    load_dir_files(args.walls, 4, walls, vs, args.names)

  borders, wall_ps = borders_and_windows(centrals, args.width, vs, args.names)
  brd_i = 0

  if to_plot:
    plot_data(centrals, wall_ps)

  for i in range(len(centrals)):
    centrals[i] = ' '.join(map(str,centrals[i]))

def load_dir_files(dir_p, num_in_row, str_dict, val_dict, names):
  for fp in dir_p.iterdir():
    vs = fgen.read_values(fp.name, fp, num_in_row)

    svs=[]
    for i in range(len(vs)):
      svs.append( ' '.join(map(str,vs[i])) )

    n = fp.name.split('.')[0]
    str_dict[n] = ' '.join(svs)
    val_dict[n] = vs

  if len(str_dict) == 0:
    print("no data files in directory")
    exit()

  for n in names:
    if n not in str_dict:
      print("no such name")
      exit()

def borders_and_windows(cs, width, wall_values, wall_names):
  bs = []
  wall_ps = []

  for i, c in enumerate(cs):
    cv = (c[-3]-c[-6], c[-2]-c[-5])
    len_cv = math.sqrt(cv[0]*cv[0] + cv[1]*cv[1])
    eorto_cv = (-cv[1]/len_cv, cv[0]/len_cv)

    brd = [0, 0, 0, 0]
    for j in range(2):
      d = eorto_cv[j]*width/2
      brd[j] = c[j-3] + d
      brd[j+2] = c[j-3] - d

    bs.append(brd)

    if args.walls and i<len(wall_names):
      window_ps=[]
      for window in wall_values[wall_names[i]]:
        r, u, w, h = window
        rel_vs = ( (r-w/2, u-h/2), (r-w/2, u+h/2), (r+w/2, u+h/2), (r+w/2, u-h/2) )

        ps = []
        for rel_v in rel_vs:
          ps.append( (c[-3] - eorto_cv[0]*rel_v[0], c[-2] - eorto_cv[1]*rel_v[0], c[-1] + rel_v[1]) )

        window_ps.append(ps)

      wall_ps.append(window_ps)

  return bs, wall_ps

def check_intersection(a, b, m):
  global brd_i

  res = False
  if brd_i < len(borders):
    brd = borders[brd_i]
    c = (brd[0], brd[1])
    d = (brd[2], brd[3])
    if fgen.intersect_on_dir(a, b, c, d):
      brd_i+=1

      #print(f"{m} intersected {brd}")
      res = True

  return res

def loop():
  global args

  pub={}
  """
  Формат строки для публикации в топик, значения разделены пробелами
  central:
  НОМЕР_СООБЩЕНИЯ ИМЯ_СТЕНЫ x1 y1 z1 x2 y2 z2 ... xN yN zN

  Имя означает стену, на которой заканчивается эта центральная линия.

  walls:
  НОМЕР_СООБЩЕНИЯ ИМЯ_СТЕНЫ x1 y1 w1 h1 x2 y2 w2 h2 ... xN yN wN hN

  Если центральная линия последняя, в central вместо имени используется символ '|', а в walls ничего не публикуется.
  """

  ns = ["central"]
  if args.walls:
    ns.append("walls")

  for n in ns:
    pub[n] = rospy.Publisher("~" + n, String, queue_size=10)

  plen = int(freq*poses_dt)
  plane_poses = deque([], plen)
  i = 1

  rate = rospy.Rate(freq)
  while not rospy.is_shutdown():
    if i>plen:
      prev_plane_pos = plane_poses[0]
    else:
      prev_plane_pos = {}

    for n in range(1,args.num+1):
      m = args.model + str(n)
      fgen.set_plane_pos(client, m)

      if m in prev_plane_pos:
        to_next_brd = check_intersection(prev_plane_pos[m], fgen.plane_pos[m], m)
      else:
        to_next_brd = False
      prev_plane_pos[m] = fgen.plane_pos[m]

      if to_next_brd:
        break

    plane_poses.append(prev_plane_pos)

    if brd_i < len(borders):
      if brd_i < len(args.names):
        p_name = args.names[brd_i]
        if args.walls:
          pub['walls'].publish(str(i) + ' ' + p_name + ' ' + walls[p_name])
      else:
        p_name = '|'

      pub['central'].publish(str(i) + ' ' + p_name + ' ' + centrals[brd_i])
      i+=1

    rate.sleep()

if __name__ == '__main__':
  global client

  client = airsim.MultirotorClient()
  client.confirmConnection()

  arguments()

  rospy.init_node("path_generator")

  try:
    loop()
  except rospy.ROSInterruptException:
    pass

  rospy.spin()
