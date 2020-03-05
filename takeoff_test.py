# Authors: Luke McDaniel and Lance Litten

# this file is a python script
# it will arm the drone and take off to a height of 1m

import time
import signal
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from velocity_vector import set_velocity_body, set_yaw_body
#import Tkinter as tk
vehicle=connect('/dev/ttyACM0',baud=115200)
gndspeed=-.7
# -- Catch SIGINT so that copter disarms when program exits
def keyboardInterruptHandler(signal, frame):
        vehicle.armed = False
        print "vehichle disarmed"
        exit(0)

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
        print "armed ready take off"
        # will cause copter to take off to height of 1m
        vehicle.simple_takeoff(altitude)
        print "simple takeoff"
#       while True:
#               v_alt = vehicle.location.global_relative_frame.alt
#               print "%(first)f" %{"first":v_alt}
#               if abs(v_alt) >= altitude *.3:
#                       break
#               time.sleep(1)
        time.sleep(1.5)


def key(event):

    if event.char == event.keysym: #-- standard keys

        if event.keysym == 'r':

            print "try landing"
            vehicle.mode = VehicleMode("LAND")
            set_velocity_body(vehicle,0,0,0.1)
            time.sleep(3)
            print "landed successfully"
            vehicle.armed = False
            vehicle.close()




    else: #-- non standard keys

        if event.keysym == 'Up':

            set_velocity_body(vehicle, gnd_speed, 0, 0)

        elif event.keysym == 'Down':

            set_velocity_body(vehicle,-gnd_speed, 0, 0)

        elif event.keysym == 'Left':

            set_yaw_body(vehicle,1,-1)

        elif event.keysym == 'Right':

            set_yaw_body(vehicle,1,1)


# -- Main Program
signal.signal(signal.SIGINT, keyboardInterruptHandler)
#init_alt = vehicle.location.global_relative_frame.alt
#print "initial altitude: %(first)f" %{"first":float(init_alt)}
arm_and_takeoff(1)
#print "altitude reached"
#root = tk.Tk()

#print(">> Control the drone with the arrow keys. Press r for RTL mode")

#root.bind_all('<Key>', key)

#root.mainloop()
time.sleep(5)
#print "rotate"
#set_velocity_body(vehicle,0,-1,0)
#time.sleep(1)
#set_velocity_body(vehicle,0,1,0)
#time.sleep(1);
#set_velocity_body(vehicle,0,0,0)
#set_yaw_body(vehicle,90,-1)
#time.sleep(1)
#print "rotate"
#set_yaw_body(vehicle,90,-1)
#set_velocity_body(vehicle,0,-0.1,0)
#time.sleep(1)
print "try landing"
vehicle.mode = VehicleMode("LAND")
set_velocity_body(vehicle,0,0,0.1)
time.sleep(3)
print "landed successfully"
vehicle.armed = False
vehicle.close()


