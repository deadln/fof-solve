#!/usr/bin/env python3
# coding=utf8

import airsim

cl = airsim.MultirotorClient()
cl.confirmConnection()
cl.enableApiControl(True)
client = cl.client

vehicle_name = "iris1"

burnt_mass = client.call('getBurntMass', vehicle_name)
print("BurntMass = ", burnt_mass)

mass_consumption = client.call('getMassConsumption', vehicle_name)
print("MassConsumption = ", mass_consumption)

client.call_async('dropCargo', vehicle_name).join()

burnt_mass = client.call('getBurntMass', vehicle_name)
print("BurntMass = ", burnt_mass)
