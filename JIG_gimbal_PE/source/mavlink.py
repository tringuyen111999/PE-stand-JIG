from pymavlink import mavutil
import time
import sys
import threading

time_exit = False

def mav_init():
    # global the_connection
    global time_exit
    # the_connection = mavutil.mavlink_connection("/dev/ttyUSB0", baud=115200)
    # the_connection.wait_heartbeat()
    # print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
    global the_connection
    mav_connected = False
    cnt = 0

    while not mav_connected:
        try:
            the_connection = mavutil.mavlink_connection("/dev/ttyUSB0", baud=115200)
            mav_connected = True
        except:
            print("Can not init mavlink with gimbal. Try again...")
            cnt = cnt + 1
            if cnt > 3: 
                return False
            time.sleep(3)

    while not time_exit:
        # request heartbeat from stm32
        heartbeat_send(self=0 ,type=123,autopilot=8, base_mode=0, custom_mode=0, system_status=1)

        msg = the_connection.recv_match(type="HEARTBEAT", blocking=True, timeout=5)

        if msg is not None:
            print("jig connected.")
            return True
    
        else:
            print("jig timeout. Try again...")
            return False
# Once connected, use 'the_connection' to get and send messages
def recv_match(self, condition=None, type=None, blocking=False, timeout=None):
    '''Receive the next MAVLink message that matches the given type and condition
    type:        Message name(s) as a string or list of strings - e.g. 'SYS_STATUS'
    condition:   Condition based on message values - e.g. 'SYS_STATUS.mode==2 and SYS_STATUS.nav_mode==4'
    blocking:    Set to wait until message arrives before method completes. 
    timeout:     ? <!-- timeout for blocking message when the system will return. Is this just a time? -->
    '''
    msg = the_connection.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    return msg


def heartbeat_send(self, type, autopilot, base_mode, custom_mode, system_status, mavlink_version=3, force_mavlink1=False):
    # Send heartbeat from a GCS (types are define as enum in the dialect file). 
    the_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    # Send heartbeat from a MAVLink application. 
    the_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


# mav_init()
# the_connection = mavutil.mavlink_connection("/dev/ttyUSB0", baud=115200)
# heartbeat_send(self=0 ,type=123,autopilot=8, base_mode=0, custom_mode=0, system_status=1)

# while 1:  
#     HB_pi = the_connection.wait_heartbeat()
#     print(HB_pi)
#     print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
#     # time.sleep(1)
#     recv_match(self=0)
#     msg = the_connection.recv_match(blocking=True)
#     print(type(msg))








    
 


