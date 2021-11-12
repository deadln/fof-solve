#!/usr/bin/env python3
# coding=utf8

import rospy
import time
import sys
import math
import random

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from geographic_msgs.msg import GeoPointStamped
from std_msgs.msg import String

from mavros_msgs.srv import SetMode, CommandBool, CommandVtolTransition, CommandHome

from geometry import *

instances_num = 6  # количество аппаратов
freq = 20  # Герц, частота посылки управляющих команд аппарату
node_name = "offboard_node"
# Словарь с топиками дронов. Ключ - номер дрона от 1 до n. Значением является последнее полученное от топика сообщение.
# Доступ: data[n].get('topic_name')
data = {}
# Словарь, хранящие сообщения из топиков центральных линий и стен
current_track_data = {}
# Списки центральных линий и стен
centrals = []
walls = []

current_obstacle = {}  # Словарь с текущими препятствиями для отдельных аппаратов
lz = {}  # Словарь с местами "посадки" для дронов, пролетевших трассу
telemetry_correction = {}  # Словарь корректировки телеметрии
telemetries = {}
drone_departion_time = -1
turn_points = {}
turn_lines = {}
turn_point_counter = 0

DELAY_BETWEEN_DRONES = 2
TURN_EPS = 2  # Окрестность, при вхождении в которую поворот считается пройденным
LINE_EPS = 0.3  # Окрестность линии, при вхождению в которую включается управление скоростями
TARGET_POINT_BIAS = -0.7  # Величина смещения точки цели полёта
TARGET_SURFACE_BIAS = 0.6  # Величина смещения плоскости стены
SPEED = 8  # Скорость сближения с отверстием
INF = 9999999999999
COLLISION_DISTANCE = 1.2
CORRECTION_SPEED = 1
TURN_POINT_BIAS = 7
TURN_POINT_DISTANCE = 2
POINTS_PER_TURN = 8
FULL_THROTTLE_DISTANCE = 10
TURN_RADIUS = 8
DESCENT_BIAS = 7
DESCENT_DIST = 3


## Вспомогательные функции

# Функция для определения аномальной телеметрии
def is_anomaly(telemetry):
    return -0.1 < telemetry['x'] < 0.1 and -0.1 < telemetry['y'] < 0.1


# Функция знака
def sign(x):
    if x > 0:
        return 1
    if x < 0:
        return -1
    return 0


# Получить из сообщения топика список точек центральной линии
def to_points_list(points_string):
    lst = points_string.split()
    res = {'name': lst[1], 'points': []}
    i = 2
    while i < len(lst):
        res['points'].append({'x': float(lst[i]), 'y': float(lst[i + 1]), 'z': float(lst[i + 2])})
        i += 3
    return res


# Получить из сообщения топика список отверстий стены
def to_holes_list(holes_string):
    lst = holes_string.split()
    res = {'name': lst[1], 'holes': []}
    i = 2
    while i < len(lst):
        res['holes'].append(
            {'x': float(lst[i]), 'y': float(lst[i + 1]), 'w': float(lst[i + 2]), 'h': float(lst[i + 3]), 'drones': 0})
        i += 4
    return res


# Преобразование словаря точки в объект Point
def dict_to_point(voc):
    return Point(voc['x'], voc['y'], voc['z'])


# Получение центральной точки препятствия в координатах симулятора
def obstacle_to_coords(central, hole):  # Координаты точки центральной линии, параметры отверстия
    # Получение точек на стене
    p1 = dict(central[-1])
    p2 = dict(central[-1])
    p2['z'] += 1
    p3 = dict(central[-1])
    norm_vect = get_wall_norm_vect(central)
    norm_vect = rotate_vect_xy(norm_vect)
    for key in p3.keys():
        p3[key] += norm_vect[key]
    # Получение векторов на стене для рассчёта абсолютных координат отверстия
    vx = dict(p3)
    for key in vx.keys():
        vx[key] -= p1[key]
    vy = dict(p2)
    for key in vy.keys():
        vy[key] -= p1[key]
    # Вычисление абсолютных координат отверстия
    res = dict(p1)
    for key in res.keys():
        res[key] += vx[key] * hole['x']
    for key in res.keys():
        res[key] += vy[key] * hole['y']
    return res


def get_norm_vect(p1, p2):
    vect = {'x': p2['x'] - p1['x'], 'y': p2['y'] - p1['y'],
            'z': p2['z'] - p1['z']}
    vect_len = math.sqrt(pow(vect['x'], 2) + pow(vect['y'], 2) + pow(vect['z'], 2))
    vect['x'] = vect['x'] / vect_len
    vect['y'] = vect['y'] / vect_len
    vect['z'] = vect['z'] / vect_len
    return vect


# Получение нормализованного вектора, направленного в сторону стены
def get_wall_norm_vect(cent_line):
    # vect = {'x': cent_line[-1]['x'] - cent_line[-2]['x'], 'y': cent_line[-1]['y'] - cent_line[-2]['y'],
    #         'z': cent_line[-1]['z'] - cent_line[-2]['z']}
    # vect_len = math.sqrt(pow(vect['x'], 2) + pow(vect['y'], 2) + pow(vect['z'], 2))
    # vect['x'] = vect['x'] / vect_len
    # vect['y'] = vect['y'] / vect_len
    # vect['z'] = vect['z'] / vect_len
    return get_norm_vect(cent_line[-2], cent_line[-1])


# Поворот вектора в плоскости XY на 90 вправо (dir >= 0) или влево (dir < 0)
def rotate_vect_xy(vect, dir=1):
    if dir < 0:
        swp = vect['x']
        vect['x'] = vect['y'] * (-1)
        vect['y'] = swp
    else:
        swp = vect['x']
        vect['x'] = vect['y']
        vect['y'] = swp * (-1)
    return vect


# Расстояние между точками
def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2) + math.pow(z1 - z2, 2))


# Получить расстояние до линии, которая проецируется из выбранного дроном отверстия перпендикулярно стене
def get_current_line_distance(n, telemetry):
    return walls[current_obstacle[n]['wall_num']]['holes'][current_obstacle[n]['hole_num']]['line'].get_point_dist(
        Point(telemetry['x'], telemetry['y'], telemetry['z']))


## Функции связанные с полётом и ROS

# Стартовая функция для подписывания на топики
def subscribe_on_topics():
    # глобальная (GPS) система координат
    subscribe_on_mavros_topics("global_position/global", NavSatFix)

    # локальная система координат, точка отсчета = место включения аппарата
    subscribe_on_mavros_topics("local_position/pose", PoseStamped)
    subscribe_on_mavros_topics("local_position/velocity_local", TwistStamped)

    # состояние
    subscribe_on_mavros_topics("state", State)
    subscribe_on_mavros_topics("extended_state", ExtendedState)

    # Трасса профессиональной лиги
    subscribe_on_mavros_topics_track("/path_generator/central")
    subscribe_on_mavros_topics_track("/path_generator/walls")


# Подписываемся на Mavros топики всех аппаратов
def subscribe_on_mavros_topics(suff, data_class):
    for n in range(1, instances_num + 1):
        data[n] = {'wall_num': 0}
        topic = f"/mavros{n}/{suff}"
        rospy.Subscriber(topic, data_class, topic_cb, callback_args=(n, suff))


# Подписываемся на Mavros топики трассы
def subscribe_on_mavros_topics_track(topic):
    rospy.Subscriber(topic, String, topic_cb_track, callback_args=(topic))


# При публикаци нового сообщения дронов
def topic_cb(msg, callback_args):
    n, suff = callback_args
    data[n][suff] = msg


# При публикаци нового сообщения трассы
def topic_cb_track(msg, topic):
    current_track_data[topic] = msg


# При выключении ноды
def on_shutdown_cb():
    rospy.logfatal("shutdown")


# Прокси для сервиса (арминг или переключение полётного режима)
def service_proxy(n, path, arg_type, *args, **kwds):
    service = rospy.ServiceProxy(f"/mavros{n}/{path}", arg_type)
    ret = service(*args, **kwds)

    # rospy.loginfo(f"{n}: {path} {args}, {kwds} => {ret}")  # Дебаг-вывод при обращении к сервисам


# Арминг
def arming(n, to_arm):
    d = data[n].get("state")
    if d is not None and d.armed != to_arm:
        service_proxy(n, "cmd/arming", CommandBool, to_arm)


# Переключение режима
def set_mode(n, new_mode):
    d = data[n].get("state")
    if d is not None and d.mode != new_mode:
        service_proxy(n, "set_mode", SetMode, custom_mode=new_mode)


# Управление по точкам, локальная система координат.
def set_pos(pt, x, y, z):
    pt.type_mask = pt.IGNORE_VX | pt.IGNORE_VY | pt.IGNORE_VZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE
    # pt.type_mask = pt.IGNORE_VX | pt.IGNORE_VY | pt.IGNORE_VZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ

    # Смещение на восток
    pt.position.x = x
    # Смещение на север
    pt.position.y = y
    # Высота, направление вверх
    pt.position.z = z


# Управление по скоростям, локальная система координат, направления совпадают с оными в глобальной системе координат
def set_vel(pt, vx, vy, vz):
    pt.type_mask = pt.IGNORE_PX | pt.IGNORE_PY | pt.IGNORE_PZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

    # Скорость, направление на восток
    pt.velocity.x = vx
    # Скорость, направление на север
    pt.velocity.y = vy
    # Скорость, направление вверх
    pt.velocity.z = vz


# Получить вектор скорости для полёта из точки p1 в точку p2 со скоростью speed
def get_speed_vect(p1, p2, speed):
    vect = dict(p2)
    vect_len = get_distance(p1['x'], p1['y'], p1['z'], p2['x'], p2['y'], p2['z'])
    for key in vect.keys():
        vect[key] -= p1[key]
        vect[key] /= vect_len
        vect[key] *= speed
    return vect


# Взлёт
def mc_takeoff(pt, n, dt):
    if dt < 15:
        # скорость вверх
        set_vel(pt, 0, 0, 4)

        # армимся и взлетаем с заданной скоростью
        if dt > 5:
            arming(n, True)


# Найти отверстие в котором назначено меньше всего дронов
def get_least_count_hole(holes_list):
    min_val = 100
    min_num = 0
    for i in range(len(holes_list)):
        if holes_list[i]['drones'] < min_val:
            min_val = holes_list[i]['drones']
            min_num = i
    holes_list[min_num]['drones'] += 1
    return min_num


def get_turn_point(n, telemetry):
    global turn_point_counter
    if n in turn_lines.keys():
        pr_point = turn_lines[n].pr_point(dict_to_point(telemetry)).get_dict()
        if get_distance(pr_point['x'], pr_point['y'], pr_point['z'], turn_points[n]['x'], turn_points[n]['y'], turn_points[n]['z']) > TURN_RADIUS:
            return turn_points[n]
        return turn_lines[n].pr_point(dict_to_point(telemetry)).get_dict()
    cur_cent_point = centrals[current_obstacle[n]['wall_num']]['points'][current_obstacle[n]['point_num']]
    v1 = get_norm_vect(centrals[current_obstacle[n]['wall_num']]['points'][current_obstacle[n]['point_num']],
                       centrals[current_obstacle[n]['wall_num']]['points'][current_obstacle[n]['point_num'] + 1])
    v2 = get_norm_vect(centrals[current_obstacle[n]['wall_num']]['points'][current_obstacle[n]['point_num']],
                       centrals[current_obstacle[n]['wall_num']]['points'][current_obstacle[n]['point_num'] - 1])
    turn_point = dict(centrals[current_obstacle[n]['wall_num']]['points'][current_obstacle[n]['point_num']])
    v1_p = dict_to_point(v1)
    v2_p = dict_to_point(v2)
    vect_cp = v1_p.get_cp(v2_p).get_dict()
    line_point = {}
    for key in v2.keys():
        turn_point[key] += (v1[key] + v2[key]) * TURN_POINT_BIAS
        if key == 'z' and v2['z'] > 0.1:
            print('DESCENT')
            turn_point[key] = cur_cent_point['z'] - DESCENT_BIAS
        line_point[key] = turn_point[key] + vect_cp[key]
    #     vect_cp[key] *= TURN_POINT_DISTANCE
    #     turn_point[key] -= vect_cp[key] * (POINTS_PER_TURN / 2)
    #     turn_point[key] += vect_cp[key] * turn_point_counter
    # turn_point_counter = (turn_point_counter + 1) % POINTS_PER_TURN
    # turn_lines[n] = turn_point
    print('TURN', dict_to_point(turn_point), dict_to_point(line_point))
    turn_points[n] = turn_point
    turn_lines[n] = Line(dict_to_point(turn_point), dict_to_point(line_point))
    cur_cent_point = centrals[current_obstacle[n]['wall_num']]['points'][current_obstacle[n]['point_num']]
    pr_point = turn_lines[n].pr_point(dict_to_point(telemetry)).get_dict()
    if get_distance(pr_point['x'], pr_point['y'], pr_point['z'], turn_points[n]['x'], turn_points[n]['y'],
                    turn_points[n]['z']) > TURN_RADIUS:
        return turn_points[n]
    return turn_lines[n].pr_point(dict_to_point(telemetry)).get_dict()


def get_telemetry(n):
    telemetry = data[n].get('local_position/pose')
    if telemetry is None:
        return None
    telemetry = {'x': telemetry.pose.position.x, 'y': telemetry.pose.position.y, 'z': telemetry.pose.position.z}
    return telemetry


# Основная функция полётных команд
def mc_race(pt, n, dt, target, telemetry):  # Повторяется с частотой freq
    global drone_departion_time

    # Меры по избежанию столкновений
    close_drones = []
    for i in range(1, instances_num + 1):
        if telemetry is not None and telemetries[i] is not None and i != n and get_distance(telemetry['x'],
                                                                                            telemetry['y'],
                                                                                            telemetry['z'],
                                                                                            telemetries[i]['x'],
                                                                                            telemetries[i]['y'],
                                                                                            telemetries[i][
                                                                                                'z']) < COLLISION_DISTANCE:
            close_drones.append(telemetries[i])
    if len(close_drones) > 0:
        correction_vector = {'x': 0, 'y': 0, 'z': 0}
        for drone in close_drones:
            drone_vect = get_speed_vect(drone, telemetry, CORRECTION_SPEED)
            for key in correction_vector.keys():
                correction_vector[key] += drone_vect[key]
        set_vel(pt, correction_vector['x'], correction_vector['y'], correction_vector['z'])
        return

    if current_obstacle[n]['state'] == 'takeoff':
        # скорость вверх
        set_vel(pt, 0, 0, 0.4)
    if current_obstacle[n]['state'] == 'takeoff' and \
            (drone_departion_time == -1 or dt - drone_departion_time > DELAY_BETWEEN_DRONES) and \
            (telemetry is not None and telemetry['z'] >= 0.3) or \
            current_obstacle[n]['state'] == 'flight' or current_obstacle[n]['state'] == 'landing':
        if current_obstacle[n]['state'] == 'takeoff':
            current_obstacle[n]['state'] = 'flight'
            drone_departion_time = dt
        # Летим в точку target
        if target['mode'] == 'pos':
            # set_pos(pt, target['x'] - telemetry_correction[n]['x'], target['y'] - telemetry_correction[n]['y'],
            #         target['z'])
            set_pos(pt, target['x'], target['y'], target['z'])
        # Или устанавливаем вектор скорости target
        elif target['mode'] == 'vel':
            set_vel(pt, target['x'], target['y'], target['z'])


def is_good_hole(hole):
    if (hole['w'] >= 1.5) and (hole['h'] >= 1):
        return True
    else:
        return False

def is_in_projection(n, telemetry):
    hole = walls[current_obstacle[n]['wall_num']]['holes'][current_obstacle[n]['hole_num']]
    vect_to_line = hole['line'].pr_point(dict_to_point(telemetry)).get_dict()
    for key in vect_to_line.keys():
        vect_to_line[key] -= telemetry[key]
    flat_vect = {'x': math.hypot(vect_to_line['x'], vect_to_line['y']), 'y': abs(vect_to_line['z'])}
    if flat_vect['x'] < hole['w'] / 2 - LINE_EPS and flat_vect['y'] < hole['h'] / 2 - LINE_EPS:
        return True
    return False

# Получить место для посадки
def get_lz(n):
    # Точка отчёта - последняя точка последней центральной линии
    land_zone = dict(centrals[-1]['points'][-1])
    norm_vect = get_wall_norm_vect(centrals[-1]['points'])  # Вектор в глухую стену
    # Сдвигаем точку до другого края посадочной площадки
    land_zone['x'] -= norm_vect['x'] * 3
    land_zone['y'] -= norm_vect['y'] * 3
    # Поворачиваем вектор на 90 градусов влево
    swp = norm_vect['x']
    norm_vect['x'] = norm_vect['y'] * (-1)
    norm_vect['y'] = swp
    # Сдвигаемся в левый нижний угол посадочной площадки
    land_zone['x'] += norm_vect['x'] * 7
    land_zone['y'] += norm_vect['y'] * 7
    lz_num = len(lz)  # Номер посадочного места
    norm_vect = get_wall_norm_vect(centrals[-1]['points'])
    # Отсчитываем посадочное место в сторону глухой стены
    land_zone['x'] -= norm_vect['x'] * (lz_num // 8) * 2
    land_zone['y'] -= norm_vect['y'] * (lz_num // 8) * 2
    # Поворачиваем вектор на 90 градусов вправо
    swp = norm_vect['x']
    norm_vect['x'] = norm_vect['y']
    norm_vect['y'] = swp * (-1)
    # Отсчитываем посадочное место вправо
    land_zone['x'] += norm_vect['x'] * (lz_num % 8) * 2
    land_zone['y'] += norm_vect['y'] * (lz_num % 8) * 2
    land_zone['z'] = 1
    return land_zone


# Получить полётную цель
def set_target(n, telemetry):
    target = {'x': 0, 'y': 0, 'z': 0}
    try:
        # Признак того, что трасса пройдена, и нужно снижаться на посадку
        if centrals[current_obstacle[n]['wall_num']]['name'] == '|':
            current_obstacle[n]['state'] = 'landing'

        # Если дрон должен заходить на посадку
        if current_obstacle[n]['state'] == 'landing':
            if n not in lz.keys():
                print(f'DRONE {n} IS LANDING')
                lz[n] = get_lz(n)
            target = lz[n]
            target['mode'] = 'pos'
            target['tag'] = 'landing'
        # Если точка последняя, значит надо лететь в отверстие в стене
        elif current_obstacle[n]['point_num'] == len(centrals[current_obstacle[n]['wall_num']]['points']) - 1:
            # Если отверстие не назначено, то назначить наименее занятое
            if current_obstacle[n]['hole_num'] == -1:
                current_obstacle[n]['hole_num'] = get_least_count_hole(walls[current_obstacle[n]['wall_num']]['holes'])
            # Получаем абсолютные координаты отверстия с помощью последней точки центральной линии и локальных
            # координат отверстия
            target = obstacle_to_coords(centrals[current_obstacle[n]['wall_num']]['points'],
                                        walls[current_obstacle[n]['wall_num']]['holes'][
                                            current_obstacle[n]['hole_num']])
            # Смещаем цель полёта относительно центра отверстия немного назад
            wall_vect = get_wall_norm_vect(centrals[current_obstacle[n]['wall_num']]['points'])
            for key in target.keys():
                target[key] += wall_vect[key] * TARGET_POINT_BIAS
            # if get_current_line_distance(n, telemetry) < min(
            #         walls[current_obstacle[n]['wall_num']]['holes'][current_obstacle[n]['hole_num']]['w'],
            #         walls[current_obstacle[n]['wall_num']]['holes'][current_obstacle[n]['hole_num']][
            #             'h']) / 2 - LINE_EPS and \
        #     vect_to_line = walls[current_obstacle[n]['wall_num']]['holes'][current_obstacle[n]['hole_num']]['line'].get_point_dist(
        # Point(telemetry['x'], telemetry['y'], telemetry['z'])).pr_point(dict_to_point(telemetry)).get_dict()
        #     for key in vect_to_line.keys():
        #         vect_to_line[key] -= telemetry[key]
            #if walls[current_obstacle[n]['wall_num']]['surface'].get_point_dist(dict_to_point(telemetry)) < DESCENT_DIST + TARGET_SURFACE_BIAS and \
            #        telemetry['z'] > target['z'] + walls[current_obstacle[n]['wall_num']]['holes'][current_obstacle[n]['hole_num']]['h'] / 2:
            #    print(f'{n}: DESCENTING')
            #    target = {'x': 0, 'y': 0, 'z': -1}
            #    target['mode'] = 'vel'
            if is_in_projection(n, telemetry) and \
                    walls[current_obstacle[n]['wall_num']]['surface'].get_point_dist(
                        Point(telemetry['x'], telemetry['y'], telemetry['z'])) < FULL_THROTTLE_DISTANCE:
                # Смещаем цель полёта вперёд, за стену
                for key in target.keys():
                    target[key] -= wall_vect[key] * TARGET_POINT_BIAS * 2
                # Назначаем полётной целью вектор скорости, направленный в точку за стеной
                target = get_speed_vect({'x': telemetry['x'], 'y': telemetry['y'],
                                         'z': telemetry['z']}, target, SPEED)
                target['mode'] = 'vel'

            # Если мы далеко от отверстия, то аккуратно приближаемся
            else:
                target['mode'] = 'pos'
                target['tag'] = 'approaching'
        # В противном случае сначала надо достигнуть точки центральной линии по пути
        else:
            target = get_turn_point(n, telemetry)
            # target = centrals[current_obstacle[n]['wall_num']]['points'][current_obstacle[n]['point_num']]
            target['mode'] = 'pos'
            target['tag'] = 'turn'
    # Исключение, которое срабатывает когда дрон пролетел отверстие, а следующая стена ещё не была опубликована
    except IndexError:
        if telemetry is not None:
            print(f'{n}:', telemetry)
        # print(f'{n}: INDEX ERROR')
        target = {'x': 0, 'y': 0, 'z': 0, 'mode': 'vel'}  # Мера для стабилизации дрона
    return target


# Функция цикла автономного полёта
def offboard_loop():  # Запускается один раз
    # создаем топики, для публикации управляющих значений, а также полётных целей (для дебага)
    pub_pt = {}
    targets = {}
    telems = {}
    for n in range(1, instances_num + 1):
        pub_pt[n] = rospy.Publisher(f"/mavros{n}/setpoint_raw/local", PositionTarget, queue_size=10)
        targets[n] = rospy.Publisher(f"/mavros{n}/target", String, queue_size=10)
        telems[n] = rospy.Publisher(f"/mavros{n}/telemetry", String, queue_size=10)

    pt = PositionTarget()  # Объект, посредством которого можно задать желаемое положение дрона и желаемые вектора скорости
    # см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
    pt.coordinate_frame = pt.FRAME_LOCAL_NED

    t0 = time.time()

    # Инициализируем словарь с данными о назначенных препятствиях для каждого дрона
    for n in range(1, instances_num + 1):
        current_obstacle[n] = {}
        current_obstacle[n]['wall_num'] = 0  # Номер текущей стены
        current_obstacle[n]['point_num'] = 1  # Номер текущей точки
        current_obstacle[n]['hole_num'] = -1  # Номер назначенного отверстия
        current_obstacle[n]['state'] = 'takeoff'  # Признак того что нужно заходить на посадку
        # current_obstacle[n]['landing']

    # цикл управления
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        dt = time.time() - t0
        # Чтение топика трассы
        central = current_track_data.get('/path_generator/central')
        wall = current_track_data.get('/path_generator/walls')
        # Обновление списков стен и центральных линий
        if central is not None and wall is not None:
            central = to_points_list(str(central.data))
            wall = to_holes_list(str(wall.data))
            if len(centrals) == 0 or centrals[-1]['name'] != central['name']:
                if central['name'] == '|':
                    print('THE FINAL WALL HAS BEEN APPEARED')
                centrals.append(central)
            if wall is not None and len(walls) == 0 or walls[-1]['name'] != wall['name']:
                holes = []
                for hole in wall['holes']:
                    holes.append(obstacle_to_coords(central['points'], hole))
                # Нахождение трёх точек для плоскости стены
                p1 = dict_to_point(central['points'][-1])
                p2 = dict_to_point(central['points'][-1])
                p2.add_point(Point(0, 0, 1))
                p3 = dict_to_point(central['points'][-1])
                norm_vect = get_wall_norm_vect(central['points'])
                norm_vect = rotate_vect_xy(norm_vect)
                p3.add_point(dict_to_point(norm_vect))
                # Смещение точек плоскости немного вперёд
                norm_vect = get_wall_norm_vect(central['points'])
                for key in norm_vect.keys():
                    norm_vect[key] *= TARGET_SURFACE_BIAS  # Величина смещения плоскости
                norm_vect = dict_to_point(norm_vect)
                p1.add_point(norm_vect)
                p2.add_point(norm_vect)
                p3.add_point(norm_vect)
                # Создание плоскости и назначение признака того что дрон преодолел плоскость
                wall['surface'] = Surface(p1, p2, p3)
                wall['surface_sign'] = sign(wall['surface'].substitute_point(dict_to_point(central['points'][-1])))
                # Построение прямой, перпендикулярной стене, для каждой точки центра отверстия, а также удаление слишком
                # маленьких отверстий
                for i in range(len(wall['holes'])):
                    if is_good_hole(wall['holes'][i]):
                        p1 = dict_to_point(obstacle_to_coords(central['points'], wall['holes'][i]))
                        p2 = dict_to_point(obstacle_to_coords(central['points'], wall['holes'][i]))
                        p2.add_point(dict_to_point(get_wall_norm_vect(central['points'])))
                        wall['holes'][i]['line'] = Line(p1, p2)
                    else:
                        wall['holes'][i]['drones'] = INF
                print('NEW WALL', wall)
                walls.append(wall)
        else:
            continue
        for n in range(1, instances_num + 1):
            telemetries[n] = get_telemetry(n)

        # if len(lz) == instances_num:
        #     if telemetries[n] is None:
        #         exit(0)
        #     flag = True
        #     for n in range(1, instances_num + 1):
        #         if get_distance(telemetry['x'], telemetry['y'], telemetry['z'], lz[n]['x'], lz[n]['y'],
        #                         lz[n]['z']) > 0.3:
        #             flag = False
        #             break
        #         if flag:
        #             exit(0)
        # управляем каждым аппаратом централизованно
        for n in range(1, instances_num + 1):
            # В ЭТОМ ЦИКЛЕ МЫ БУДЕМ ПОЛУЧАТЬ ДАННЫЕ О ТРАССЕ И ЗАДАВАТЬ ПОЛЁТНЫЕ ЦЕЛИ
            pt = PositionTarget()  # Объект, посредством которого можно задать желаемое положение дрона и желаемые
            # вектора скорости, см. также описание mavlink сообщения
            # https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
            pt.coordinate_frame = pt.FRAME_LOCAL_NED

            try:
                # telemetry = data[n].get('local_position/pose')  # Получение текущих координат дрона
                telemetry = telemetries[n]  # Получение текущих координат дрона
                if telemetry is not None:
                    telems[n].publish(str(telemetry['x']) + ' ' + str(telemetry['y']) + ' ' + str(
                        telemetry['z']))
                    if not is_anomaly(telemetry):
                        set_mode(n, "OFFBOARD")  # Переключение в режим полёта по программе
                        arming(n, True)
                else:
                    telems[n].publish(str(telemetry))
                # Обнаружение аномальной телеметрии
                if telemetry is None:
                    # print(f'{n}: TELEMETRY IS MISSING')
                    raise IndexError

                target = set_target(n, telemetry)
                pos = Point(telemetry['x'], telemetry['y'], telemetry['z'])

                # Если назначена посадка
                if current_obstacle[n]['state'] == 'landing':
                    pass
                # Если пересечена плоскость стены (знак при подстановке точки в уравнение плоскости не совпадает со
                # знаком точки, подставленной перед стеной)
                elif current_obstacle[n]['point_num'] == len(
                        centrals[current_obstacle[n]['wall_num']]['points']) - 1 and \
                        walls[current_obstacle[n]['wall_num']]['surface_sign'] != sign(
                    walls[current_obstacle[n]['wall_num']]['surface'].substitute_point(pos)):
                    print(f'{n}:NEXT WALL')
                    # Инскремент счётчика стены и сброс номеров точек и отверстий
                    current_obstacle[n]['wall_num'] += 1
                    current_obstacle[n]['point_num'] = 1
                    current_obstacle[n]['hole_num'] = -1
                    target = set_target(n, telemetry)  # Назначение новой стены
                # Если достигнута окрестность центра поворота
                elif current_obstacle[n]['point_num'] < len(centrals[current_obstacle[n]['wall_num']]['points']) - 1 and \
                    turn_lines[n].get_point_dist(dict_to_point(telemetry)) < TURN_EPS:
                        # get_distance(telemetry['x'], telemetry['y'], telemetry['z'],
                        #              target['x'], target['y'], target['z']) < TURN_EPS:
                    print(f'{n}:NEXT POINT')
                    turn_lines.pop(n, None)
                    current_obstacle[n]['point_num'] += 1
                    target = set_target(n, telemetry)
            # Исключение, которое срабатывает когда дрон пролетел отверстие, а следующая стена ещё не была опубликована
            except IndexError:
                # print(f'{n}: INDEX ERROR')
                # Меры для стабилизации дрона
                if telemetry is None:
                    target = {'x': 0, 'y': 0, 'z': 0, 'mode': 'vel'}
                else:
                    print(f'{n}:', telemetry)
                    target = {'x': telemetry['x'], 'y': telemetry['y'],
                              'z': telemetry['z'],
                              'mode': 'pos'}
            if target is not None:
                # Отправка полётной цели и публикация данных в топиках
                mc_race(pt, n, dt, target, telemetry)
                pub_pt[n].publish(pt)
                targets[n].publish(str(target))

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node(node_name)
    rospy.loginfo(node_name + " land_zoneed")
    if len(sys.argv) > 1:
        instances_num = int(sys.argv[1])
    subscribe_on_topics()

    rospy.on_shutdown(on_shutdown_cb)

try:
    offboard_loop()
except rospy.ROSInterruptException:
    pass

rospy.spin()
