# Commands that are written specifically for our drone.
#
# Most of the commands are wrappers for a mavlink "command_long_send"
# For each of these you will finish the command with eight values, 
# the first is the configuration and the final seven are specific to
# each command. You always send all seven parameters but most commands 
# don't use all seven, so you still send you still send 0s for the 
# left overs.
# Unless otherwise specified the commands/messages can be found at:
#   https://mavlink.io/en/messages/common.html

# Packages for drone
from pymavlink import mavutil
import time # Used to sleep while the drone ramps up
import threading #Used to create process to constantly send mocap data
import math
import matplotlib.pyplot as plt

###-------------------------------------------------------------------------------------###
###                               Connect to Drone                                      ###
###-------------------------------------------------------------------------------------###
def drone_connect(port):
    """ Connect to the drone:
    input - udp port
    note: Doesn't matter the IP address, but needs the port and that is defined 
          when you startup mavproxy on the pi, which is currently done through ssh
    returns - drone_connection: object representing the connection to the drone
    """
    print("Attempting to connect on port %d" % port)

    # Create the object that represents the drone and will be used for all communication
    # with the drone.
    drone_connection = mavutil.mavlink_connection('udpin:0.0.0.0:%d' % port) 

    drone_connection.wait_heartbeat() #wait until we hear a heartbeat from the copter

    print("Connection success")
    print("Heartbeat from system (system %u component %u)" 
          % (drone_connection.target_system, drone_connection.target_component))

    return drone_connection 


###-------------------------------------------------------------------------------------###
###                                  Take Off                                           ###
###-------------------------------------------------------------------------------------###
def takeoff(drone, takeoff_alt):
    """ Takeoff
    Inputs:
        drone:            Drone connection object, obtained from drone_connect(port)
        takeoff_alt:      Desired altitue, in positive meters. 
    """
    # The drone's coordinate frame is NED, meaning the ground is zero and the higher you go 
    # the more negative a position you have. For readability use takeoff_alt is positive,
    # but for drone commands it needs to be negative:
    #takeoff_alt *= -1

    # Initialize the GPS origin
    #set_gps_global_origin(drone)

    # Here is the list of modes:
    # 0 = stabilize
    # 1 = acro
    # 2 = alt_hold
    # 3 = auto
    # 4 = guided
    # 5 = loiter
    # 6 = rtl
    # 7 = circle
    # Tell the drone to enter one of these modes:
    flight_mode = 4
    drone.mav.set_mode_send(drone.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, flight_mode) 

    # arm throttle:
    # reminder - the first 0 isn't a parameter, it is the configuration
    # first parameter: 0 = disarm, 1 = arm
    # second parameter: 0 = use safety checks, 21196 = force arm/disarm
    print("Arming Throttle:")
    drone.mav.command_long_send(drone.target_system, drone.target_component, 
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    msg = drone.recv_match(type = 'COMMAND_ACK', blocking = True)
    print(msg) #"result: 0" if it executed without error. If you get result: 4, you probably need to set the copter to guided mode.

    # Give the drone time to start up
    time.sleep(5)

    # Send takeoff command to target altitude.
    # reminder - the first 0 isn't a parameter, it is the configuration
    # first parameter: Pitch (degrees) 
    # 2nd and 3rd: Empty
    # 4: Yaw (degrees). If magnetometer isn't present this is ignored
    # 5th and 6th: Lat Long
    # 7th: Desired altitude (meters), should be negative because we're in NED coordinate frame
    
    ##### Trying the Attitude/Thrust method:
    # print("Setting throttle to 0.9:")

    # print(time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()))
    # time_us = int((time.time()-init_time) * 1.0e6)
    # drone.mav.set_attitude_target_send(time_us, drone.target_system, drone.target_component, 0b00000111, [1,0,0,0], 0, 0, 0, 0.9, [0,0,0])
    
    # receive_mav_message(drone)

    # time.sleep(5)
    
    # print("Setting throttle to 0.5:")
    # print(time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()))
    # time_us = int((time.time()-init_time) * 1.0e6)
    # drone.mav.set_attitude_target_send(time_us, drone.target_system, drone.target_component, 0, [1,0,0,0], 0, 0, 0, .5, [0,0,0])


    print("Attempting Takeoff:")
    print(time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()))
    drone.mav.command_long_send(drone.target_system, drone.target_component, 
                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, takeoff_alt)
    receive_mav_message(drone)
    print("Finished takeoff script")
    time.sleep(.5)
    # flight_mode = 2
    # drone.mav.set_mode_send(drone.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, flight_mode) 
    # Delay until we reach the desired altitude
    # We must update the via mocap repeatedly
    #msg_interval = 0.075 # seconds
   # last_msg_time = time.time()

    # while 1:
    #     current_time = time.time()

    #     if (current_time - last_msg_time) > msg_interval:
    #         last_msg_time = current_time
    #         current_time_us = int(current_time * 1.0e6) # Mavproxy wants micro seconds

    #         # Update the drone's position from the mocap:
    #         # Get info from mocap
    #         [drone_pos, drone_rot] = mocap_connection.rigid_body_dict[1]

    #         # Update drone's current state
    #         update_drone_state(drone, current_time_us, drone_pos, drone_rot)

    #         # Wait for the next LOCAL_POSITION_NED message
    #         msg = drone.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            
    #         print('Current height: ' + str(msg.z*-1.))

    #         # Check if altitude is within a threshold of the target altitude
    #         if msg.z*-1. < takeoff_alt: # z is negative because of drone's NED coordinate frame
    #             print('Takeoff Success') 
    #             break
    # return



###-------------------------------------------------------------------------------------###
###                                  Take Off THread (swarm)                            ###
###-------------------------------------------------------------------------------------###


class TakeoffThread(threading.Thread):
    def __init__(self, drone, takeoff_alt, accuracy, init_time):
        super().__init__()
        self.drone = drone
        self.accuracy = accuracy
        self.takeoff_alt = takeoff_alt
        self._stop_event = threading.Event()  # Event to stop the thread
        self.init_time = init_time
    def run(self):
        # Set flight mode to guided
        flight_mode = 4
        self.drone.mav.set_mode_send(self.drone.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, flight_mode)

        # Arm throttle
        print("Arming Throttle:")
        self.drone.mav.command_long_send(self.drone.target_system, self.drone.target_component, 
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        
        msg = self.drone.recv_match(type = 'COMMAND_ACK', blocking = True)
        print(msg) #"result: 0" if it executed without error. If you get result: 4, you probably need to set the copter to guided mode.

        # Wait for drone to start up
        time.sleep(5)

        # Send takeoff command
        print("Attempting Takeoff:")
        print(time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()))
        self.drone.mav.command_long_send(self.drone.target_system, self.drone.target_component, 
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, self.takeoff_alt)
        time.sleep(5)
        # Wait for takeoff completion
        #position_mask = int(3576) # use position
        #time_us = int((time.time()-self.init_time) * 1.0e6)
        #self.drone.mav.set_position_target_local_ned_send(time_us, self.drone.target_system, self.drone.target_component, 1, position_mask, 0, 0, self.takeoff_alt, 0, 0, 0, 0, 0, 0, 0, 0)
        #while True:
        #    message = self.drone.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        #    if message:
         #       drone_z = message.z
          #      print(f"Z: {drone_z}")
           #     if abs(drone_z - self.takeoff_alt) < self.accuracy:
            #        print("Drone has reached desired altitude")
             #       break

    def stop(self):
        self._stop_event.set()



def goto_NED_point(drone_connection, x, y, z, init_time, accuracy):
    # 3576 position
    # 3527 velocity 
    # 3135 acceleration
    # 3520 position + vel
    # 3072 pos + vel + acc
    # 2559 yaw 
    # 1535 yaw rate
    # For both yaw and yaw rate must include at least pos vel or acc 
    position_mask = int(3576) # use position
    time_us = int((time.time()-init_time) * 1.0e6)
    drone_connection.mav.set_position_target_local_ned_send(time_us, drone_connection.target_system, drone_connection.target_component, 1, position_mask, x, y, z, 0, 0, 0, 0, 0, 0, 0, 0)
    #drone_connection.mav.request_data_stream_send(drone_connection.target_system, drone_connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION,1,1)

    while True:
        message = drone_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if message:
            drone_x = message.x
            drone_y = message.y
            drone_z = message.z 
            print(f"X: {drone_x}, Y: {drone_y}, Z: {drone_z}") 
            if abs(x-drone_x) < accuracy and abs(y-drone_y) < accuracy and abs(z-drone_z) < accuracy:
                print("Drone has reached position")
                break
    return



def simple_goto_NED_point(drone_connection, x, y, z, init_time, accuracy):
    # 3576 position
    # 3527 velocity 
    # 3135 acceleration
    # 3520 position + vel
    # 3072 pos + vel + acc
    # 2559 yaw 
    # 1535 yaw rate
    # For both yaw and yaw rate must include at least pos vel or acc 
    position_mask = int(3576) # use position
    time_us = int((time.time()-init_time) * 1.0e6)
    drone_connection.mav.set_position_target_local_ned_send(time_us, drone_connection.target_system, drone_connection.target_component, 1, position_mask, x, y, z, 0, 0, 0, 0, 0, 0, 0, 0)
    #drone_connection.mav.request_data_stream_send(drone_connection.target_system, drone_connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION,1,1)
    
    return

###-------------------------------------------------------------------------------------###
###                              Thread for goto NED point (swarm)                      ###
###-------------------------------------------------------------------------------------###

class GotoNEDPointThread(threading.Thread):
    def __init__(self, drone_connection, x, y, z, init_time, accuracy):
        super().__init__()
        self.drone_connection = drone_connection
        self.x = x
        self.y = y
        self.z = z
        self.init_time = init_time
        self.accuracy = accuracy
        self._stop_event = threading.Event()  # Event to stop the thread

    def run(self):
        position_mask = int(3576)
        time_us = int((time.time() - self.init_time) * 1.0e6)
        self.drone_connection.mav.set_position_target_local_ned_send(
            time_us, self.drone_connection.target_system, self.drone_connection.target_component, 1, position_mask,
            self.x, self.y, self.z, 0, 0, 0, 0, 0, 0, 0, 0)

        while not self._stop_event.is_set():
            message = self.drone_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            if message:
                drone_x = message.x
                drone_y = message.y
                drone_z = message.z
                #print(f"X: {drone_x}, Y: {drone_y}, Z: {drone_z}")
                if abs(self.x - drone_x) < self.accuracy and abs(self.y - drone_y) < self.accuracy and abs(
                        self.z - drone_z) < self.accuracy:
                    print("Drone has reached position")
                    self.stop()

    def stop(self):
        self._stop_event.set()




def disarm(drone_connection):
    drone_connection.mav.command_long_send(drone_connection.target_system, drone_connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

def set_mode_land (drone_connection):
    flight_mode = 9 
    drone_connection.mav.set_mode_send(drone_connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, flight_mode) 
    

###-------------------------------------------------------------------------------------###
###                              Set GPS Origin                                         ###
###-------------------------------------------------------------------------------------###
def set_drone_gps_global_origin(drone):
    # Set the origin of the GPS, accordinig to mavproxy this does not need to be accurate, 
    # just needs to be initialized: https://ardupilot.org/copter/docs/common-optitrack.html
    # 1st param: Target system
    # 2nd, 3rd, 4th: Lat, long, altitude
    # 5th: Time
    drone.mav.set_gps_global_origin_send(drone.target_system, 400150000, -1052705000, 1624000, 0)
    return


###-------------------------------------------------------------------------------------###
###                                   Land                                              ###
###-------------------------------------------------------------------------------------###
# NOT VERIFIED
def land(drone): 
    drone.mav.command_long_send(drone.target_system, drone.target_component, 
                                mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    return

###-------------------------------------------------------------------------------------###
###                     Update Drone Attitude and Position                              ###
###-------------------------------------------------------------------------------------###
def update_drone_state(drone, connection_time, drone_pos, drone_rot):
    """ Update the drones attitude and position
    Inputs:
        drone: connection to drone, obtained from drone_connect()
        connection_time: time in seconds from when the script began
        drone_pos: list of 3 floats representing the drone position
        drone_rot: list of 4 floats representing the drone's rotation as a quaternion
    NOTE: The position and orientation are in the mocap's coordinate frame, they are
          converted to the drone's coordinate frame here.
    """
    time_us = int(connection_time * 1.0e6)
    drone.mav.att_pos_mocap_send(time_us, (drone_rot[3], drone_rot[0], drone_rot[2], -drone_rot[1]),
                                    drone_pos[0], drone_pos[2], -drone_pos[1])
    return

###-------------------------------------------------------------------------------------###
###                              Mocap Streaming Thread                                 ###
###-------------------------------------------------------------------------------------###
#class definition
class threaded_mocap_streaming(threading.Thread):
    #Constructor for class
    def __init__(self, thread_name, thread_ID, drone_connection, mocap_connection, init_time, rigid_body_id):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.thread_name = thread_name
        self.threadID = thread_ID
        self.drone_connection = drone_connection
        self.mocap_connection = mocap_connection
        self.init_time = init_time
        self.rigid_body_id = rigid_body_id
    # Streaming Loop
    def run(self):
        """ Initialize the drone's GPS, this requires a few updates of the drone's position
        Inputs:
            drone: connection to drone, obtained from drone_connect()
            mocap_connection: connection to mocap, obtained from mocap_connect() in
                            Custom_Mocap_Commands.py
            init_time: Time in seconds when the main script started 
        """
        pause_between_updates = .1 # seconds
        

        # Do it
        while True:

            time.sleep(pause_between_updates)

            # Get info from mocap
            [drone_pos, drone_rot] = self.mocap_connection.rigid_body_dict[self.rigid_body_id]
            #print(f"Current altitude (m): {drone_pos[1]}")

            # Update drone's current state
            update_drone_state(self.drone_connection, time.time()-self.init_time, drone_pos, drone_rot)
        
        return

###-------------------------------------------------------------------------------------###
###                              Flight Controller Position Thread                      ###
###-------------------------------------------------------------------------------------###
#class definition
class threaded_postion_report(threading.Thread):
    #Constructor for class
    def __init__(self, thread_name, thread_ID, drone_connection, init_time):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.thread_name = thread_name
        self.threadID = thread_ID
        self.drone_connection = drone_connection
        self.init_time = init_time
    # get drone postion Loop
    def run(self):
        #Request GLOBAL_POSITION_INT message be sent at regular interval
        self.drone_connection.mav.request_data_stream_send(self.drone_connection.target_system, self.drone_connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION,1,1)
        while True:
            message = self.drone_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if message:
                altitude_amsl = message.alt / 1000.0
                relative_altitude = message.relative_alt /1000.0
                print(f"Altitude above mean sea level: {altitude_amsl} meters, Relative Altitude: {relative_altitude} meters") 
        return

###-------------------------------------------------------------------------------------###
###                                Receive Mav Message                                  ###
###-------------------------------------------------------------------------------------###
def receive_mav_message(drone):
    """ Blocks the thread and waits to receive a message from mavlink. Also prints the
    time stamp when the message was received. Used for debugging purposes
    Inputs:
        drone: connection to drone, obtained from drone_connect()
    """
    print(time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()))
    msg = drone.recv_match(type = 'COMMAND_ACK', blocking = True)
    print(msg)
    return


###-------------------------------------------------------------------------------------###
###                              Flight Controller Position Thread                      ###
###-------------------------------------------------------------------------------------###
#class definition
class threaded_swarm_positions(threading.Thread):
    #Constructor for class
    def __init__(self, thread_name, thread_ID, drone_connection_list, init_time):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.thread_name = thread_name
        self.threadID = thread_ID
        self.drone_connection_list = drone_connection_list
        self.init_time = init_time
    # get drone postion Loop
    def run(self):
        #Request GLOBAL_POSITION_INT message be sent at regular interva

        self.drone_connection.mav.request_data_stream_send(self.drone_connection.target_system, self.drone_connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION,1,1)
        while True:
            message = self.drone_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if message:
                altitude_amsl = message.alt / 1000.0
                relative_altitude = message.relative_alt /1000.0
                print(f"Altitude above mean sea level: {altitude_amsl} meters, Relative Altitude: {relative_altitude} meters") 
        return
    def initialize_swarm_position_vars(num_of_drones):
        if num_of_drones > 0:
            global drone1_x
            global drone1_y
            global drone1_z
            drone1_x = 0
            drone1_y = 0
            drone1_z = 0

        if num_of_drones > 1:
            global drone2_x
            global drone2_y
            global drone2_z
            drone2_x = 0
            drone2_y = 0
            drone2_z = 0 

        if num_of_drones > 2:
            global drone3_x
            global drone3_y
            global drone3_z
            drone3_x = 0
            drone3_y = 0
            drone3_z = 0 

        if num_of_drones > 3:
            global drone4_x
            global drone4_y
            global drone4_z
            drone4_x = 0
            drone4_y = 0
            drone4_z = 0 

        if num_of_drones > 4:
            global drone5_x
            global drone5_y
            global drone5_z
            drone5_x = 0
            drone5_y = 0
            drone5_z = 0 


class RotatingTriangle:
    def __init__(self, angle):
        self.angle = angle
        self.radius = .75

    def get_triangle_coordinates(self):
        # Calculate the coordinates of the initial triangle
        x1 = self.radius * math.cos(0)
        y1 = self.radius * math.sin(0)
        x2 = self.radius * math.cos(2 * math.pi / 3)
        y2 = self.radius * math.sin(2 * math.pi / 3)

        x3 = self.radius * math.cos(4 * math.pi / 3)
        y3 = self.radius * math.sin(4 * math.pi / 3)

        # Rotate the coordinates around the origin
        new_x1 = x1 * math.cos(self.angle) - y1 * math.sin(self.angle)
        new_y1 = x1 * math.sin(self.angle) + y1 * math.cos(self.angle)

        new_x2 = x2 * math.cos(self.angle) - y2 * math.sin(self.angle)
        new_y2 = x2 * math.sin(self.angle) + y2 * math.cos(self.angle)

        new_x3 = x3 * math.cos(self.angle) - y3 * math.sin(self.angle)
        new_y3 = x3 * math.sin(self.angle) + y3 * math.cos(self.angle)

        # Increment the angle for the next call
        self.angle += math.radians(self.angle)  # Increment by 30 degrees (convert to radians)

        return [[new_x1, new_y1, -.5], [new_x2, new_y2, -.5], [new_x3, new_y3, -.5]]

# rotating_triangle = RotatingTriangle(10)
# colors = ['r', 'g', 'b', 'c', 'm']  # Different colors for each triangle position
# for _ in range(5):  # Plot the next 5 positions
#     triangle_coords = rotating_triangle.get_triangle_coordinates()
#     x_coords = [coord[0] for coord in triangle_coords]  # Extract x coordinates
#     y_coords = [coord[1] for coord in triangle_coords]  # Extract y coordinates
#     plt.plot(x_coords + [x_coords[0]], y_coords + [y_coords[0]], color=colors[_])  # Connect first and last points to close the triangle

# # Set plot limits
# plt.xlim(-1.5, 1.5)
# plt.ylim(-1.5, 1.5)

# # Add labels and title
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Rotating Triangle')

# # Show plot
# plt.grid(True)
# plt.gca().set_aspect('equal', adjustable='box')
# plt.show()