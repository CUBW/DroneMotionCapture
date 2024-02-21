# Primary script for controlling the drone with the mocap system

# Packages for mocap
import sys
import time

from Custom_Drone_Commands import *
from Custom_Mocap_Commands import *   


###### MAIN ######
init_time = time.time()

# Mocap connection:
streaming_client = mocap_connect()

is_running = streaming_client.run()

# Drone connection
drone_connection = drone_connect(14550) #udp connection to ardupilot
set_drone_gps_global_origin(drone_connection)

# Mocap Streaming Thread, will run until you terminate THIS's files terminal.
stream = threaded_mocap_streaming("stream1", 1, drone_connection, streaming_client, init_time) 
stream.start() 
time.sleep(2)


# Take off
take_off_height = 1 # meter
takeoff(drone_connection, streaming_client, init_time, take_off_height)

# Do more interesting things:


