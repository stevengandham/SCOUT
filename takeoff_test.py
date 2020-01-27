# Author: Luke McDaniel

# this file is a python script
# it will arm the drone and take off to a height of 1m

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# -- Define arm and takeoff function
def arm_and_takeoff(altitude):
	#check if copter can be armed
	while not vehicle.is_armable:
		time.sleep(1)
	# arm motors
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True
	# wait for vehicle to arm
	while not vehicle.armed: time.sleep(1)

	# will cause copter to take off to height of 1m
	vehicle.simple_takeoff(altitude)

	while True:
		v_alt = vehicle.location.global_relative_frame.alt
		if v_alt >= altitude - 1.0:
			break
		time.sleep(1)

# -- Main Program
arm_and_takeoff(1)
time.sleep(10)
vehicle.mode = VehicleMode("RTL")