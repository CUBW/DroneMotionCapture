# Script to control the Tello drones with a PS5 controller
# 
# Dependencies:
#   pip install pygame
#   pip install DJITelloPy
#
# Requirements:
#   PS5 controller connected to computer (bluetooth works)
#   Computer connected to the Tello's wifi
#
# Notes:
#   Getting a video feed doesn't seem to complex, for an example:
#   https://github.com/damiafuentes/DJITelloPy/blob/796b018e4ac193410ecb4b2df87ce4a36bbd5f6a/examples/manual-control-pygame.py

import pygame
from djitellopy import Tello


##-----------------------------------------------------------##
###                  Initialize Controller                  ###
##-----------------------------------------------------------##
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


##-----------------------------------------------------------##
###                     Connect to Tello                    ###
##-----------------------------------------------------------##
tello = Tello()

tello.connect()

speed = 100 # 10 - 100 cm/s

tello.set_speed(speed)



##-----------------------------------------------------------##
###                      Do the Thing!                      ###
##-----------------------------------------------------------##
try:
    while True:
        pygame.event.pump()
        
        
        ##--------------------------------------------------##
        ###                 Joysticks                      ###
        ##--------------------------------------------------##
        # Joystick axes Normalized to -100 to 100
        left_joystick_x = int(joystick.get_axis(0)*500)
        left_joystick_y = int(joystick.get_axis(1)*500)
        right_joystick_x = int(joystick.get_axis(2)*500)
        right_joystick_y = int(joystick.get_axis(3)*500)

        left_right_velocity = right_joystick_x
        forward_backward_velocity = -right_joystick_y
        yaw_velocity = left_joystick_x
        up_down_velocity = -left_joystick_y

        tello.send_rc_control(left_right_velocity, 
                        forward_backward_velocity, up_down_velocity, yaw_velocity)
        
        
        ##--------------------------------------------------##
        ###                  Buttons                       ###
        ##--------------------------------------------------##
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN: 
                if event.button == 6:
                    print("Three lines button pressed")
                    print("Connecting")
                    tello.connect()

                elif event.button == 0:
                    print("X button pressed")
                    print("Landing")
                    tello.land()
                    
                elif event.button == 3:
                    print("Triangle button pressed")
                    print("Take off initiated")
                    tello.takeoff()
        
except KeyboardInterrupt:
    print("Exiting...")
    tello.land()
finally:
    pygame.quit()


##-----------------------------------------------------------##
###               PS5 Buttons for Reference                 ###
##-----------------------------------------------------------##
# From trial and error:
# axis 0 = left joystick left = -1 and right = 1
# axis 1 = left joystick up = -1 and down = 1
# axis 2 = right joystick left = -1 and right = 1
# axis 3 = right joystick up = -1 and down = 1
# axis 4 = left trigger nothing = -1 and pressed = 1
# axis 5 = right trigger nothing = -1 and pressed = 1

# Circle = button 1
# x = button 0
# triangle = button 3
# square = button 2
# d_pad up = button 11
# d_pad down = button 12
# d_pad left = button 13
# d_pad right = button 14
# right bumper = button 10
# left bumper = button 9
# left joystick click = button 7
# right joystick click = button 8
# three lines = button 6
# flashy looking thing = button 4
# playstation logo = button 5
# track pad click = button 15