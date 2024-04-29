import threading
import pygame
from djitellopy import Tello
import cv2 # For video updates
import numpy as np # For video frame transforms
import time # For video and controller fps control


##-----------------------------------------------------------##
###      Thread: Update Drone with Controller Commands      ###
##-----------------------------------------------------------##
# This function operates on its own thread and continously 
# updates the drone's velocity
def update_drone(drone, controller):
    while True:

        pygame.event.pump()
            
        ##--------------------------------------------------##
        ###                 Joysticks                      ###
        ##--------------------------------------------------##
        # Joystick axes Normalized to -100 to 100
        left_joystick_x = int(controller.get_axis(0)*100)
        left_joystick_y = int(controller.get_axis(1)*100)
        right_joystick_x = int(controller.get_axis(2)*100)
        right_joystick_y = int(controller.get_axis(3)*100)

        left_right_velocity       =  right_joystick_x
        forward_backward_velocity = -right_joystick_y
        yaw_velocity              =  left_joystick_x
        up_down_velocity          = -left_joystick_y

        drone.send_rc_control(left_right_velocity, 
                forward_backward_velocity, up_down_velocity, yaw_velocity)


        ##--------------------------------------------------##
        ###                  Buttons                       ###
        ##--------------------------------------------------##
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN: 
                if event.button == 4:
                    print("Flashy button pressed")
                    try:
                        print("Connecting")
                        drone.connect()
                    except:
                        print("Connecting Failed")
                elif event.button == 6:
                    print("Three lines button pressed")
                    try:
                        print("Throw drone")
                        drone.initiate_throw_takeoff()
                    except:
                        print("Throw failed")
                elif event.button == 0:
                    print("X button pressed")
                    try:                        
                        print("Landing")
                        drone.land()
                    except:
                        print("Landing Failed")                    
                elif event.button == 3:
                    print("Triangle button pressed")
                    try:
                        print("Take off initiated")
                        drone.takeoff()
                    except:
                        print("Take off Failed")

            ##--------------------------------------------------##
            ###      "Do a barrel roll!" - Peppy Hare          ###
            ##--------------------------------------------------##
                elif event.button == 11:
                    try:
                        drone.flip_forward()
                    except:
                        print("Flip Failed")
                elif event.button == 12:
                    try:
                        drone.flip_back()
                    except:
                        print("Flip Failed")
                elif event.button == 13:
                    try:
                        drone.flip_left()
                    except:
                        print("Flip Failed")
                elif event.button == 14:
                    try:
                        drone.flip_right()
                    except:
                        print("Flip Failed")
   
                        


##-----------------------------------------------------------##
###                  Thread: Update Video                   ###
##-----------------------------------------------------------##
# This function operates on its own thread and continously 
# updates the video feed
def update_video(drone, screen):
    while True:
        fps = 45
        
        # Update battery info:
        battery_level = drone.get_battery()
        text = f"Battery: {battery_level}%"

        # Get the frame
        frame_read = drone.get_frame_read()

        frame = frame_read.frame

        cv2.putText(frame, text, (5, 720 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Rotate the camera frame (it is sideways by default)
        frame = np.rot90(frame)
        frame = np.flipud(frame)

        # Convert the frame to a pygame compatible datatype
        frame = pygame.surfarray.make_surface(frame)

        # Clear the old screen
        # screen.fill([0, 0, 0])

        # Fill the screen
        screen.blit(frame, (0, 0))
        pygame.display.update()

        time.sleep(1 / fps)    


##-----------------------------------------------------------##
###                        Fly Time!                        ###
##-----------------------------------------------------------##
if __name__ == '__main__':

    ##-------------------------------------------------------##
    ###                Initialize Controller                ###
    ##-------------------------------------------------------##
    # Initialize Pygame and Joystick
    pygame.init()
    pygame.joystick.init()

    # Check for joystick count
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joystick detected")
        exit()

    # Initialize the Joystick
    controller = pygame.joystick.Joystick(0)

    controller.init()


    ##-------------------------------------------------------##
    ###                   Connect to Tello                  ###
    ##-------------------------------------------------------##
    drone = Tello()

    # Only print super important things:
    drone.LOGGER.setLevel(0)

    # Connect
    drone.connect()

    speed = 100 # 10 - 100 cm/s
    drone.set_speed(speed)  

    # Set video quality


    # Initialize streaming. Start with off in case things weren't
    # shutdown properly
    drone.streamoff()
    drone.streamon()



    ##-------------------------------------------------------##
    ###                  Start up a Screen                  ###
    ##-------------------------------------------------------##
    # Creat pygame window
    pygame.display.set_caption("Tello Video Stream")
    screen = pygame.display.set_mode([960, 720])



    ##-------------------------------------------------------##
    ###                Thread Time! We Fancy                ###
    ##-------------------------------------------------------##
    video_thread = threading.Thread(target=update_video, args=(drone, screen,))
    video_thread.start()
    print("Video Started")

    drone_control_thread = threading.Thread(target=update_drone, args=(drone,controller,))
    drone_control_thread.start()
    print("Controller Started")

    # Keep on going
    video_thread.join()
    drone_control_thread.join()
