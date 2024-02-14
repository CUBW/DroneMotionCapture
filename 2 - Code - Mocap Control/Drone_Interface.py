# Primary script for controlling the drone with the mocap system

# Packages for mocap
import sys
import time

from Custom_Drone_Commands import *
from Custom_Mocap_Commands import *   


###### MAIN ######
# Mocap connection:
streaming_client = mocap_connect()

is_running = streaming_client.run()

# Drone connection
drone_connection = drone_connect(14550) #udp connection to ardupilot
set_drone_gps_global_origin(drone_connection)

# Set the time between updating drone position via mocap
msg_interval = 0.075 # seconds

# Initialize the gps
initalize_gps(drone_connection)

# Take off
take_off_height = 1 # meter
takeoff(drone_connection, take_off_height)

# Do more interesting things: