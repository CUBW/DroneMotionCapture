from pickletools import float8
from pymavlink import mavutil
import math
import tkinter as tk
import pygame
import transformations as tft
from pyquaternion import Quaternion
import time

def connect(port):
    print("Attempting to connect on port %d" % port)
    connection = mavutil.mavlink_connection('udpin:0.0.0.0:%d' % port)
    connection.wait_heartbeat() #wait until we hear a heartbeat from the copter
    print("Connection success")
    print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
    return connection

def takeoff(connection, takeoff_alt):
    #set copter to guided mode
    connection.mav.set_mode_send(connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
    #arm throttle
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    msg = connection.recv_match(type = 'COMMAND_ACK', blocking = True)
    print(msg) #"result: 0" if it executed without error. If you get result: 4, you probably need to set the copter to guided mode.
    #send takeoff command to target altitude.
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, takeoff_alt)
    msg = connection.recv_match(type = 'COMMAND_ACK', blocking = True)
    print(msg)
    #loop until copter reaches takeoff altitude
    while 1:
        # Wait for the next LOCAL_POSITION_NED message
        msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        # Check if altitude is within a threshold of the target altitude
        if abs(msg.z * -1 - takeoff_alt) < 1.0:
            print("Reached target altitude")
            break

def land(connection):
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
#send local frame coordinates and have copter fly over that spot.

def send_waypoint_local(connection, x, y, z):
    connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message
                    (10, connection.target_system, connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                     int(0b010111111000), x, y, z,
                      0, 0, 0, 0, 0, 0, 0, 0))
    msg = connection.recv_match(type = 'COMMAND_ACK', blocking = True)
    print(msg)
    
def tell_drone_where_it_is(connection,time, x, y, z, quaternion):
    connection.mav.send(mavutil.mavlink.ATT_POS_MOCAP(time, quaternion, x, y, z, float("NaN")))
    msg = connection.recv_match(type = 'COMMAND_ACK', blocking = True)
    print(msg)

drone_connection = connect(14550)

# Mavproxy:
# Arm
# Takeoff

tell_drone_where_it_is(drone_connection, time, x, y, z, quaternion)
send_waypoint_local(drone_connection, 10., 10., 10.)
for i in range(10000):
    get_mocap_position()
    tell_drone_where_it_is(drone_connection, time, )
