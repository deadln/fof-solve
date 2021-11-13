#!/usr/bin/env python3
# coding=utf8

import airsim
import os
import time
import numpy as np
from PIL import Image, ImageOps
import multiprocessing.dummy as mp


# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()



# set first group params
N_1 = 6
VELOCITY = 7
T_SLEEP_SNAKE = 1.5
T_BETWEEN_GROUPS = 0
WAIT_FOR_LANDING = 5
FOLLOW_NAME = 'iris1'
BOT_NAME = 'zbot'
Z_VAL = -5

# set second group params
N_2 = 10
DELTA_Y, DELTA_Z = 0.5, 0.5
K = 1.5

X_0, Y_0, Z_0 = 510, -70, 101.6 + 17.35

time.sleep(WAIT_FOR_LANDING)

# set route
route_X = [510, 540, 500, 460, 490, 530, 560, 520]
route_Y = [-70, -30, 0, 30, 70, 40, 80, 110]
route_Z = [115, 120, 105, 115, 105, 115, 131, 121]

nums = list(range(N_1))
names = list(map(lambda x: "{}_{}".format(BOT_NAME, x + 1), nums))
POSES_0 = list(map(lambda x: "{}_{}".format(BOT_NAME, x + 1), nums))

def create_path(i):
    path = []
    for x, y, z in zip(route_X, route_Y, route_Z):
        path.append(airsim.Vector3r(x - i * 2 - X_0, y - Y_0, -z + Z_0))
    return path

# enable first group
di = {}
for i, n in zip(nums, names):
    client.enableApiControl(True, n)
    client.armDisarm(True, n)
    di[i] = client.takeoffAsync(1, vehicle_name=n)
for i in nums:
    di[i].join()

while client.simGetVehiclePose(FOLLOW_NAME).position.z_val > Z_VAL:
    time.sleep(0.5)

# move first group
di = {}
for i in range(0, N_1):
    client.moveOnPathAsync(create_path(i), velocity=VELOCITY, vehicle_name='{}_{}'.format(BOT_NAME, i + 1))
    time.sleep(T_SLEEP_SNAKE)

time.sleep(T_BETWEEN_GROUPS)

nums = list(range(N_1, N_1 + N_2))
names = list(map(lambda x: "{}_{}".format(BOT_NAME, x + 1), nums))
POSES_0 = list(map(lambda x: "{}_{}".format(BOT_NAME, x + 1), nums))

def create_path(i):
    global X_0, Y_0, Z_0

    path = []
    for x, y, z in zip(route_X, route_Y, route_Z):
        Y_0 = Y_0 + DELTA_Y if i == N_1 + 1 else Y_0
        Y_0 = Y_0 - DELTA_Y if i == N_1 + 2 else Y_0
        Z_0 = Z_0 - DELTA_Z if i == N_1 + 3 else Z_0
        Z_0 = Z_0 + DELTA_Z if i == N_1 + 4 else Z_0

        Y_0 = Y_0 + DELTA_Y * K if i == N_1 + 1 + N_2 / 2 else Y_0
        Y_0 = Y_0 - DELTA_Y * K if i == N_1 + 2 + N_2 / 2 else Y_0
        Z_0 = Z_0 - DELTA_Z * K if i == N_1 + 3 + N_2 / 2 else Z_0
        Z_0 = Z_0 + DELTA_Z * K if i == N_1 + 4 + N_2 / 2 else Z_0

        path.append(airsim.Vector3r(x - i * 2 - X_0, y - Y_0, -z + Z_0))
    return path


# enable second group
di = {}
for i, n in zip(nums, names):
    client.enableApiControl(True, n)
    client.armDisarm(True, n)
    di[i] = client.takeoffAsync(1, vehicle_name=n)
for i in nums:
    di[i].join()

# move second group
for i in range(N_1, N_1 + N_2):
    client.moveOnPathAsync(create_path(i), velocity=VELOCITY, vehicle_name='{}_{}'.format(BOT_NAME, i + 1))


