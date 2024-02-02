
# Packages for mocap
import sys
import time
from NatNetClient import NatNetClient
import DataDescriptions
import MoCapData

# Packages for drone
from pymavlink import mavutil


def drone_connect(port):
    print("Attempting to connect on port %d" % port)

    connection = mavutil.mavlink_connection('udpin:0.0.0.0:%d' % port) 

    connection.wait_heartbeat() #wait until we hear a heartbeat from the copter

    print("Connection success")
    print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

    return connection 

def takeoff(connection, takeoff_alt):
    #set copter to altitude hold mode
    connection.mav.set_mode_send(connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 2) 

    #arm throttle
    connection.mav.command_long_send(connection.target_system, connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    msg = connection.recv_match(type = 'COMMAND_ACK', blocking = True)
    print(msg) #"result: 0" if it executed without error. If you get result: 4, you probably need to set the copter to guided mode.

    #send takeoff command to target altitude.
    connection.mav.command_long_send(connection.target_system, connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, takeoff_alt)

    msg = connection.recv_match(type = 'COMMAND_ACK', blocking = True)
    print(msg)

    #loop until copter reaches takeoff altitude
    while 1:
        # Wait for the next LOCAL_POSITION_NED message
        msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    
        # Check if altitude is within a threshold of the target altitude
        if abs(msg.z * -1 - takeoff_alt) < 1.0:
            print("Reached target altitude")
            break

def mocap_connect():
    # Mocap Initilization
    streaming_client = NatNetClient()
    streaming_client.set_client_address("127.0.0.1")
    streaming_client.set_server_address("127.0.0.1")
    streaming_client.set_use_multicast(False)
    streaming_client.set_print_level(0)

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()
    if not is_running:
        print("ERROR: Could not start streaming client.")
        try:
            sys.exit(1)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    time.sleep(1)
    if streaming_client.connected() is False:
        print("ERROR: Could not connect properly.  Check that Motive streaming is on.")
        try:
            sys.exit(2)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    print_configuration(streaming_client)
    print("\n")

    print_level = streaming_client.set_print_level(0)

    return streaming_client


def print_configuration(natnet_client):
    natnet_client.refresh_configuration()
    print("Connection Configuration:")
    print("  Client:          %s"% natnet_client.local_ip_address)
    print("  Server:          %s"% natnet_client.server_ip_address)
    print("  Command Port:    %d"% natnet_client.command_port)
    print("  Data Port:       %d"% natnet_client.data_port)

    if natnet_client.use_multicast:
        print("  Using Multicast")
        print("  Multicast Group: %s"% natnet_client.multicast_address)
    else:
        print("  Using Unicast")

    #NatNet Server Info
    application_name = natnet_client.get_application_name()
    nat_net_requested_version = natnet_client.get_nat_net_requested_version()
    nat_net_version_server = natnet_client.get_nat_net_version_server()
    server_version = natnet_client.get_server_version()

    print("  NatNet Server Info")
    print("    Application Name %s" %(application_name))
    print("    NatNetVersion  %d %d %d %d"% (nat_net_version_server[0], nat_net_version_server[1], nat_net_version_server[2], nat_net_version_server[3]))
    print("    ServerVersion  %d %d %d %d"% (server_version[0], server_version[1], server_version[2], server_version[3]))
    print("  NatNet Bitstream Requested")
    print("    NatNetVersion  %d %d %d %d"% (nat_net_requested_version[0], nat_net_requested_version[1],\
       nat_net_requested_version[2], nat_net_requested_version[3]))
    #print("command_socket = %s"%(str(natnet_client.command_socket)))
    #print("data_socket    = %s"%(str(natnet_client.data_socket)))


###### MAIN ######
# Drone Proof of Concept
drone_connection = drone_connect(14550) #udp connection to ardupilot

drone_connection.mav.command_long_send(drone_connection.target_system, drone_connection.target_component, 
mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
msg = drone_connection.recv_match(type = 'COMMAND_ACK', blocking = True)
print(msg) #"result: 0" if it executed without error. If you get result: 4, you probably need to set the copter to guided mode.

streaming_client = mocap_connect()

is_running = streaming_client.run()

msg_interval = 0.075
last_msg_time = time.time()

print(streaming_client.rigid_body_dict[1])
[drone_pos, drone_rot] = streaming_client.rigid_body_dict[1]

# Startup the drone
takeoff(drone_connection, 1)

while is_running:
    current_time = time.time()
    
    if (current_time - last_msg_time) > msg_interval:
        print(streaming_client.rigid_body_dict[1])
        [drone_pos, drone_rot] = streaming_client.rigid_body_dict[1]
        
        time_us = int(current_time * 1.0e6)
        drone_connection.mav.att_pos_mocap_send(time_us, (drone_rot[3], drone_rot[0], drone_rot[2], -drone_rot[1]), drone_pos[0], drone_pos[2], -drone_pos[1])
        
        last_msg_time = current_time