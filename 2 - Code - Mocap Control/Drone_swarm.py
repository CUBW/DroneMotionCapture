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
drone1_port = 14550 
drone2_port = 14551
drone1_connection = drone_connect(drone1_port) #udp connection to ardupilot
drone2_connection = drone_connect(drone2_port)
set_drone_gps_global_origin(drone1_connection)
set_drone_gps_global_origin(drone2_connection)
time.sleep(3)
# Mocap Streaming Thread, will run until you terminate THIS's files terminal.
drone1_rigid_body = 2
drone2_rigid_body = 3
accuracy = .10
drone1_mocap_stream = threaded_mocap_streaming("stream1", 1, drone1_connection, streaming_client, init_time, drone1_rigid_body) 
drone2_mocap_stream = threaded_mocap_streaming("stream2", 2, drone2_connection, streaming_client, init_time, drone2_rigid_body) 
drone1_mocap_stream.start()
drone2_mocap_stream.start() 

time.sleep(3)

take_off_height = .5
set_drone_gps_global_origin(drone1_connection)
set_drone_gps_global_origin(drone2_connection)
drone1_takeoff_thread = TakeoffThread(drone1_connection, take_off_height, accuracy, init_time)
drone2_takeoff_thread = TakeoffThread(drone2_connection, take_off_height, accuracy, init_time)

drone1_takeoff_thread.start()
drone2_takeoff_thread.start()

drone1_takeoff_thread.join()
drone2_takeoff_thread.join()

print("both drones have taken off")
x1 = 0
y1 = 0
z1 = 0

x2 = 1
y2 = 0
z2 = 0

drone1_goto_ned_thread = GotoNEDPointThread(drone1_connection, x1, y1, z1, init_time, accuracy)
drone2_goto_ned_thread = GotoNEDPointThread(drone2_connection, x2, y2, z2, init_time, accuracy)
drone1_goto_ned_thread.start()
drone2_goto_ned_thread.start()

drone1_goto_ned_thread.join()
drone2_goto_ned_thread.join()

disarm(drone1_connection)
disarm(drone2_connection)




time.sleep(2)