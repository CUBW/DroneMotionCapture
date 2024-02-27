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
mocap_stream = threaded_mocap_streaming("stream1", 1, drone_connection, streaming_client, init_time) 
mocap_stream.start() 
#FC_postion_stream = threaded_postion_report("stream2", 2, drone_connection, init_time)
#FC_postion_stream.start()
time.sleep(2)


# Take off
take_off_height = .5 # meter
x1 = -1
z1 = 0
y1 = -.5
takeoff(drone_connection, streaming_client, init_time, take_off_height)
time.sleep(5)
print("moving to postion")
goto_NED_point(drone_connection, x1, y1, z1, init_time)
print("got there!")
time.sleep(300)
# Do more interesting things:


