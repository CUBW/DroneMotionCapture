import sys
import time
from Custom_Drone_Commands import *
from Custom_Mocap_Commands import *   


###### MAIN ######
init_time = time.time()

num_of_drones = 2

# Mocap connection:
streaming_client = mocap_connect()

is_running = streaming_client.run()


# Drone connection
swarm_ports = [14550, 14551, 14552] 
swarm_connections= []
swarm_mocap_streaming_threads = []

for port in swarm_ports:
    connection = drone_connect(port)
    swarm_connections.append(connection)

time.sleep(3)
# Mocap Streaming Thread, will run until you terminate THIS's files terminal.

swarm_rigid_body_id_list = [1,2,5]

stream_id = 1
for id, connection in zip(swarm_rigid_body_id_list, swarm_connections):
    stream_id_string = "stream" + str(stream_id)
    threaded_mocap_drone_stream = threaded_mocap_streaming(stream_id_string, stream_id, connection, streaming_client, init_time, id)
    swarm_mocap_streaming_threads.append(threaded_mocap_drone_stream)

accuracy = .10

for drone_mocap_thread_stream in swarm_mocap_streaming_threads:
    drone_mocap_thread_stream.start()

time.sleep(2)
for drone_connection in swarm_connections:
    set_drone_gps_global_origin(drone_connection)
time.sleep(3)

take_off_height = .5
drone1_takeoff_thread = TakeoffThread(swarm_connections[0], take_off_height, accuracy, init_time)
drone2_takeoff_thread = TakeoffThread(swarm_connections[1], take_off_height, accuracy, init_time)
drone3_takeoff_thread = TakeoffThread(swarm_connections[2], take_off_height, accuracy, init_time)

drone1_takeoff_thread.start()
drone2_takeoff_thread.start()
drone3_takeoff_thread.start()

drone1_takeoff_thread.join()
drone2_takeoff_thread.join()
drone3_takeoff_thread.join()

time.sleep(1)

print("all drones have taken off")


accuracy = .15

swarm_target_positions = [[-1,.5,-.5],[0,.5,-.5],[1,.5,-.5]]
goto_ned_threads = []

for drone_connection, target_position in zip(swarm_connections, swarm_target_positions):
    x = target_position[0]
    y = target_position[1]
    z = target_position[2]
    goto_NED_point_thread = GotoNEDPointThread(drone_connection, x, y, z, init_time, accuracy)
    goto_ned_threads.append(goto_NED_point_thread)

for thread in goto_ned_threads:
    thread.start()

for thread in goto_ned_threads:
    thread.join()

swarm_target_positions = [[-1,-.5,-.5],[0,-.5,-.5],[1,-.5,-.5]]
goto_ned_threads = []

for drone_connection, target_position in zip(swarm_connections, swarm_target_positions):
    x = target_position[0]
    y = target_position[1]
    z = target_position[2]
    goto_NED_point_thread = GotoNEDPointThread(drone_connection, x, y, z, init_time, accuracy)
    goto_ned_threads.append(goto_NED_point_thread)

for thread in goto_ned_threads:
    thread.start()

for thread in goto_ned_threads:
    thread.join()


swarm_target_positions = [[-1,0,-.5],[0,0,-.5],[1,0,-.5]]
goto_ned_threads = []

for drone_connection, target_position in zip(swarm_connections, swarm_target_positions):
    x = target_position[0]
    y = target_position[1]
    z = target_position[2]
    goto_NED_point_thread = GotoNEDPointThread(drone_connection, x, y, z, init_time, accuracy)
    goto_ned_threads.append(goto_NED_point_thread)

for thread in goto_ned_threads:
    thread.start()

for thread in goto_ned_threads:
    thread.join()

rotating_triangle = RotatingTriangle(45)

iterations = 20

accuracy = .20

while True:
    swarm_target_positions = rotating_triangle.get_triangle_coordinates()

    goto_ned_threads = []

    for drone_connection, target_position in zip(swarm_connections, swarm_target_positions):
        x = target_position[0]
        y = target_position[1]
        z = target_position[2]
        goto_NED_point_thread = GotoNEDPointThread(drone_connection, x, y, z, init_time, accuracy)
        goto_ned_threads.append(goto_NED_point_thread)

    for thread in goto_ned_threads:
        thread.start()

    for thread in goto_ned_threads:
        thread.join()
    iterations = iterations -1

    if iterations == 0:

        break


for drone in swarm_connections:
    set_mode_land(drone)

for drone in swarm_connections:
    disarm(drone)


time.sleep(2)

