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
drone_rigid_body_id = 2
mocap_stream = threaded_mocap_streaming("stream1", 1, drone_connection, streaming_client, init_time, drone_rigid_body_id) 
mocap_stream.start() 
#FC_postion_stream = threaded_postion_report("stream2", 2, drone_connection, init_time)
#FC_postion_stream.start()
time.sleep(2)


#Take off
take_off_height = .5 # meter
takeoff(drone_connection, take_off_height)
time.sleep(8)
print("moving to postion")
x1 = -1
z1 = -.5
y1 = 0
accuracy = .05
goto_NED_point(drone_connection, x1, y1, z1, init_time, accuracy)
time.sleep(8)
print("moving to postion")
x1 = 1
z1 = -.5
y1 = 0
accuracy = .05
goto_NED_point(drone_connection, x1, y1, z1, init_time, accuracy)
time.sleep(8)
print("moving to postion")
x2 = 0
z2 = -.5
y2 = 0
accuracy = .05
goto_NED_point(drone_connection, x2, y2, z2, init_time, accuracy)
time.sleep(8)
print("Landing")
x3 = 0
z3 = 0
y3 = 0
accuracy = .10
goto_NED_point(drone_connection, x3, y3, z3, init_time, accuracy)
time.sleep(8)
disarm(drone_connection)
time.sleep(300)
# Do more interesting things:


