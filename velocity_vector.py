# Authors: Luke McDaniel and Lance Litten

# Navigation function that send flight commands to PX4 with dronekit

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

# function that sets the body velocity of the drone in m/s
# positive vx moves the drone forward, negative vx moves it backwards
# positive vy moves the drone to the right, negative vy moves it left
# positive vx moves the drone DOWN, negative vz moves it UP
def set_velocity_body(vehicle,vx,vy,vz):

        msg=vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0,0,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111000111, #consider only velocities
                0,0,0, #position
                vx,vy,vz, #velocity
                0,0,0, #acceleration
                0,0)
        vehicle.send_mavlink(msg)
        vehicle.flush()

# function that will rotate the drone to change its heading
# drone will always turn clockwise by heading degrees passed to it
def set_yaw_body(vehicle,heading,cw):

        msg=vehicle.message_factory.command_long_encode(
                0,0,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                heading,
                0,
                cw, #clockwise=1 cc=-1
                1, #setting to relative offset instead of absolute
                0,0,0)
        # heading specifies number of degrees clockwizse from current heading
        vehicle.send_mavlink(msg)
        vehicle.flush()

