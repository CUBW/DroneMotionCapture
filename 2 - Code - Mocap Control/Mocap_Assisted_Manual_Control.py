# Primary script for controlling the drone with the mocap system

# Packages for mocap
import sys
import time
import pygame

from Custom_Drone_Commands import *
from Custom_Mocap_Commands import *   

def get_mode_gimbal_values(joystick):
    left_joystick_x = joystick.get_axis(0) * 1000
    left_joystick_y = joystick.get_axis(1) * 1000
    right_joystick_x = joystick.get_axis(2) * 1000
    right_joystick_y = joystick.get_axis(3) * -1000
    
    if (left_joystick_x < 30 and left_joystick_x > -30):
        left_joystick_x = 0
    if (left_joystick_y < 30 and left_joystick_y > -30):
        left_joystick_y = 0
    if (right_joystick_y < 30 and right_joystick_y > -30):
        right_joystick_y = 0    
    if (right_joystick_x < 30 and right_joystick_x > -30):
        right_joystick_x = 0
    return [int(left_joystick_x),int(left_joystick_y),int(right_joystick_x),int(right_joystick_y*-1)]


###### MAIN ######
init_time = time.time()

#init pygame
pygame.init()
pygame.joystick.init()

# Check for joystick count
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joystick detected")
    exit()

# Initialize the Joystick
joystick = pygame.joystick.Joystick(0)

joystick.get_init()
print("Controller name:", joystick.get_name())

# Mocap connection:
streaming_client = mocap_connect()

is_running = streaming_client.run()

# Drone connection
drone_connection = drone_connect(14551) #udp connection to ardupilot

# Mocap Streaming Thread, will run until you terminate THIS's files terminal.
drone_rigid_body_id = 12
drone_stream_id = 1
mocap_stream = threaded_mocap_streaming("stream1", drone_stream_id, drone_connection, streaming_client, init_time, drone_rigid_body_id) 
mocap_stream.start() 
#FC_postion_stream = threaded_postion_report("stream2", 2, drone_connection, init_time)
#FC_postion_stream.start()
time.sleep(2)
set_drone_gps_global_origin(drone_connection)
time.sleep(2)


x_min = -1.75
x_max = 1.75
y_min = -.75
y_max = .75
z_min = -1.75
z_max = .1

x = 0
y = 0 
z = .5

while True:
    for event in pygame.event.get():

        # If the user clicked the close button, quit the program
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

        # If a button was pressed, print the button number
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == 3:
                print("Takeoff!")
                take_off_height = .5 # meter
                takeoff(drone_connection, take_off_height)
                time.sleep(2)
            if event.button == 0:
                print("Landing")
                set_mode_land(drone_connection)
                disarm(drone_connection)
                time.sleep(2)

    
    pygame.event.pump()
    left_joystick_x,left_joystick_y,right_joystick_x,right_joystick_y = get_mode_gimbal_values(joystick)
    
    if left_joystick_x != 0 or right_joystick_x != 0 or right_joystick_y != 0:
        x = x + (right_joystick_x/10000)
        y = y + (right_joystick_y/10000)
        z = z + (left_joystick_y/10000)

        if x > x_max or x < x_min:
            x = float(max(min(x_max, x), x_min))

        if y > y_max or y < y_min:
            y = float(max(min(y_max, y), y_min))

        if z > z_max or z < z_min:
            z = float(max(min(z_max, z), z_min))

        accuracy = .15
        print("X: ", f'{x:.3}', " Y: ", f'{y:.3}', " Z: ", f'{z:.3}')
        simple_goto_NED_point(drone_connection, x, y, z, init_time, accuracy)
        time.sleep(.1)

    else:
        accuracy = .15
        simple_goto_NED_point(drone_connection, x, y, z, init_time, accuracy)
        time.sleep(.1)
        continue





# #Take off
# take_off_height = .5 # meter
# takeoff(drone_connection, take_off_height)
# time.sleep(2)
# print("moving to postion")
# x1 = -1.5
# z1 = -.75
# y1 = 0
# accuracy = .10
# goto_NED_point(drone_connection, x1, y1, z1, init_time, accuracy)
# print("moving to postion")
# x1 = 1.5
# z1 = -.75
# y1 = 0
# accuracy = .10
# goto_NED_point(drone_connection, x1, y1, z1, init_time, accuracy)
# print("moving to postion")
# x2 = 0
# z2 = -.75
# y2 = 0
# accuracy = .10
# goto_NED_point(drone_connection, x2, y2, z2, init_time, accuracy)
# print("Landing")
# set_mode_land(drone_connection)
# disarm(drone_connection)
# time.sleep(300)
# Do more interesting things:


