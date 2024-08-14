import pygame
import time
import sys




def get_mode_gimbal_values(joystick):
    left_joystick_x = joystick.get_axis(0) * 1000
    left_joystick_y = joystick.get_axis(1) * 1000
    right_joystick_x = joystick.get_axis(2) * 1000
    right_joystick_y = joystick.get_axis(3) * 1000
    
    if (left_joystick_x < 30 and left_joystick_x > -30):
        left_joystick_x = 0
    if (left_joystick_y < 30 and left_joystick_y > -30):
        left_joystick_y = 0
    if (right_joystick_y < 30 and right_joystick_y > -30):
        right_joystick_y = 0    
    if (right_joystick_x < 30 and right_joystick_x > -30):
        right_joystick_x = 0
    return [int(left_joystick_x),int(left_joystick_y),int(right_joystick_x),int(right_joystick_y*-1)]

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

x_min = -1
x_max = 1
y_min = -1
y_max = 1
z_min = -2
z_max = .1

x = 0
y = 0 
z = 0

while True:
    for event in pygame.event.get():

        # If the user clicked the close button, quit the program
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

        # If a button was pressed, print the button number
        if event.type == pygame.JOYBUTTONDOWN:
            print("Button", event.button, "was pressed")
            time.sleep(1)

        # If a button was released, print the button number
        if event.type == pygame.JOYBUTTONUP:
            print("Button", event.button, "was released")
            time.sleep(1)
    
    pygame.event.pump()
    left_joystick_x,left_joystick_y,right_joystick_x,right_joystick_y = get_mode_gimbal_values(joystick)
    
    if left_joystick_x != 0 or right_joystick_x != 0 or right_joystick_y != 0:
        x = x + (right_joystick_x/100000000)
        y = y + (right_joystick_y/100000000)
        z = z + (left_joystick_y/100000000)

        if x > x_max or x < x_min:
            x = float(max(min(x_max, x), x_min))

        if y > y_max or y < y_min:
            y = float(max(min(y_max, y), y_min))

        if z > z_max or z < z_min:
            z = float(max(min(z_max, z), z_min))

        print("X: ", f'{x:.3}', " Y: ", f'{y:.3}', " Z: ", f'{z:.3}')

    else:
        continue


