from pickletools import float8
from pymavlink import mavutil
import math
import tkinter as tk
import pygame
import transformations as tft
from pyquaternion import Quaternion
import time





def euler_to_quaternion(roll, pitch, yaw):
    # Convert angles to radians
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    # Create a Quaternion from Euler angles
    quaternion = Quaternion(axis=[1, 0, 0], angle=roll) * Quaternion(axis=[0, 1, 0], angle=pitch) * Quaternion(axis=[0, 0, 1], angle=yaw)
    
    return quaternion

def send_manual_control(connection, x, y, z, r, buttons=0):
    """
    Send a MANUAL_CONTROL command to the drone.

    Parameters:
    - connection: The MAVLink connection object.
    - x: The x-axis value (pitch), normalized from -1000 to 1000.
    - y: The y-axis value (roll), normalized from -1000 to 1000.
    - z: The z-axis value (throttle), normalized from 0 to 1000.
    - r: The r-axis value (yaw), normalized from -1000 to 1000.
    - buttons: Bitmask of buttons pressed (default to 0).
    """

    # Ensure the values are within the acceptable range
    x = int(max(-1000, min(1000, x)))
    y = int(max(-1000, min(1000, y)))
    z = int(max(-1000, min(1000, z)))
    r = int(max(-1000, min(1000, r)))
    
    print(x,y,z,r)

    # Send the MANUAL_CONTROL message
    connection.mav.manual_control_send(
        connection.target_system,
        x,   # Pitch
        y,   # Roll
        z,   # Throttle
        r,   # Yaw
        buttons
    )
    
    duration = 10;

    #Hold the attitude for the specified duration
    if duration > 0:
        time.sleep(duration / 1000.0)




#Main code:
#set copter to guided mode

drone_connection = connect(14550) #udp connection to ardupilot

#set copter to guided mode
#drone_connection.mav.set_mode_send(drone_connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4) 

#arm throttle
#drone_connection.mav.command_long_send(drone_connection.target_system, drone_connection.target_component, 
#                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

#msg = drone_connection.recv_match(type = 'COMMAND_ACK', blocking = True)
#print(msg) #"result: 0" if it executed without error. If you get result: 4, you probably need to set the copter to guided mode.

#takeoff(drone_connection, 3)

# Initialize Pygame and Joystick
pygame.init()
pygame.joystick.init()

# Check for joystick count
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joystick detected")
    exit()

# Initialize the Joystick
joystick = pygame.joystick.Joystick(0)

joystick.init()

try:
    while True:
        pygame.event.pump()
        
        # Joystick axes Normalized to -1000 to 1000
        left_joystick_x = int(joystick.get_axis(0)*1000)
        left_joystick_y = int(joystick.get_axis(1)*1000)
        right_joystick_x = int(joystick.get_axis(2)*1000)
        right_joystick_y = int(joystick.get_axis(3)*1000)

        print("yaw ",left_joystick_x)
        print("pitch ",right_joystick_y)
        print("roll ",right_joystick_x*-1)
        print("throttle", left_joystick_y*-1)
        buttons = 0
        send_manual_control(drone_connection, 0, 0, left_joystick_y*-1, 0, buttons)

        duration = 10;

        #Hold the attitude for the specified duration
        if duration > 0:
            time.sleep(duration / 1000.0)

        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN: 
                if event.button == 0:
                    print("Square button pressed")
                elif event.button == 1:
                    print("X button pressed")
                    drone_connection.mav.command_long_send(drone_connection.target_system, drone_connection.target_component, 
                                                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
                    msg = drone_connection.recv_match(type = 'COMMAND_ACK', blocking = True)
                    print(msg) #"result: 0" if it executed without error. If you get result: 4, you probably need to set the copter to guided mode.
                elif event.button == 2:
                    print("O button pressed")
                    drone_connection.mav.command_long_send(
                        drone_connection.target_system, 
                        drone_connection.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
                        0,  # Confirmation
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        0,  # Stabilize mode number, adjust if different for your FC
                        0, 0, 0, 0, 0  # Unused parameters
                    )
                    # Wait for ACK message
                    msg = drone_connection.recv_match(type='COMMAND_ACK', blocking=True)
                    print(msg)  # "result: 0" indicates successful execution.
                elif event.button == 3:
                    print("Triangle button pressed")
                    # Send command to arm the drone
                    drone_connection.mav.command_long_send(
                        drone_connection.target_system, 
                        drone_connection.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                        0,  # Confirmation
                        1,  # Arm
                        0, 0, 0, 0, 0, 0  # Unused parameters
                    )
                    # Wait for ACK message
                    msg = drone_connection.recv_match(type='COMMAND_ACK', blocking=True)
                    print(msg)  # "result: 0" indicates successful execution. "result: 4" suggests a possible requirement to set the copter to guided mode.
                
            


except KeyboardInterrupt:
    print("Exiting...")
    land(drone_connection)
finally:
    pygame.quit()