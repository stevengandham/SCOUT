# Authors: Luke McDaniel and Lance Litten

# main program for controlling drone for autonomous flight

# import statements
import time
import signal
import serial
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from velocity_vector import set_velocity_body, set_yaw_body

# set up usb connection with PX4 flight controller
vehicle = connect('/dev/ttyACM0', baud = 115200)
# set up serial connection with other raspberry pi
#lidar = serial.Serial('/dev/ttyS0, baud = 9600)

# catch SIGINT so that copter disarms when ctrl+C is pressed
# ctrl+C can be used to shut drone motors down incase of crash
def keyboardInterruptHandler(vehicle, signal, frame):
        vehicle.armed = False
        print "vehicle disarmed with interrupt"
        exit(0)

# will take off to 2m
def arm_and_takeoff(vehicle):
        # check if copter can be armed
        while not vehicle.is_armable:
                time.sleep(1)
        # vehicle must be in guided mode for navigation
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        # wait for vehicle to arm
        while not vehicle.armed:
                time.sleep(1)
        print "armed and ready for takeoff"
        # will cause copter to take off to 2m
        vehicle.simple_takeoff(2)
        print "simple takeoff ran"
        time.sleep(1)

# will land the drone slowly straight down
def land_and_disarm(vehicle):
        print "try landing"
        # put vehicle into LAND mode
        vehicle.mode = VehicleMode("LAND")
        # set body velocity so motors slowly spin down
        set_velocity_body(vehicle, 0, 0, 0.1)
        time.sleep(3)
        print "vehicle landed"
        # disarm vehicle
        vehicle.armed = False
        while vehicle.armed:
                time.sleep(1)
        # close out vehicle connection
        vehicle.close()


# navigation functions #

# parameters: distance you want to fly in meters #

def fly_right(vehicle, distance):
        print "start flying right"
        set_velocity_body(vehicle, 0, 0.5, 0)
        time.sleep(distance * 2)
        set_velocity_body(vehicle, 0, -0.1, 0)
        i = 0
        while i < 3:
                set_velocity_body(vehicle, 0, 0, 0)
                time.sleep(1)
                i=i+1
        print "finished flying right"

def fly_left(distance):
        print "start flying left"
        set_velocity_body(vehicle, 0, -0.5, 0)
        time.sleep(distance * 2)
        set_velocity_body(vehicle, 0, 0.1, 0)
        set_velocity_body(vehicle, 0, 0, 0)
        print "finished flying left"

def fly_forward(distance):
        print "start flying forward"
        set_velocity_body(vehicle, 0.5, 0, 0)
        time.sleep(distance * 2)
        set_velocity_body(vehicle, -0.1, 0, 0)
        set_velocity_body(vehicle, 0, 0, 0)
        print "finished flying forward"

def fly_backward(distance):
        print "start flying backward"
        set_velocity_body(vehicle, -0.5, 0, 0)
        time.sleep(distance * 2)
        set_velocity_body(vehicle, 0.1, 0, 0)
        set_velocity_body(vehicle, 0, 0, 0)
        print "finished flying backward"

# parameters: none #

# rotates the drone clockwise 90 degrees to the right #
def turn_right(void):
        print "start right rotation"
        set_yaw_body(vehicle, 90, 1)
        print "finished right rotation"

# rotates the drone counter-clockwise 90 degrees to the left #
def turn_left(void):
        print "start left rotation"
        set_yaw_body(vehicle, 90, -1)
        print "finished left rotation"

#------- main function --------#

# takeoff to 2m high
arm_and_takeoff(vehicle)


#----- testing -----#
time.sleep(4)
set_velocity_body(vehicle, 0, 0.1, 0)
time.sleep(1)
set_velocity_body(vehicle, 0, 0, 0)
#fly_right(vehicle, 2)
#time.sleep(3)

land_and_disarm(vehicle)

# get bearings with request bearings from other pi

# if safe after initial takeoff start data collection


