from pymavlink import mavutil
import time
import sys
import threading
time_exit = False
# time_exit = False
def mavlink_wait_message():
    global time_exit
    global a
    global m_is_heartbeat_live
    global jig_system_status
    global param_values
    global m_params_updated
    global jig_system_status,jig_system_base,jig_system_custom

    while True:
        global the_connection

        try:
            msg = the_connection.recv_match(blocking=True)
            print(msg)
            pass
        except:
            time_exit = True
            break

        if not msg:
            continue

        type = msg.get_type()
        if type == "BAD_DATA":
            pass
        # if mavutil.all_printable(msg.data):
        #     sys.stdout.write(msg.data)
        #     sys.stdout.flush()
        elif type == "HEARTBEAT":
            # m_is_heartbeat_live = True
            # jig_system_status = msg.system_status
            # print("HB receive ", jig_system_status)
            jig_system_status = msg.system_status
            jig_system_base = msg.base_mode
            jig_system_custom = msg.custom_mode
            print("HB receive status: ", jig_system_status)
            print("HB receive base: ", jig_system_base)
            print("HB receive custom: ", jig_system_custom)
            break
            # pass

        elif type == "RAW_IMU":
            #print("msg.xmag:{:.2f} msg.ymag:{:.2f} msg.zmag:{:.2f}". format(msg.xacc, msg.yacc, msg.zacc))
            #a = [msg.xacc, msg.yacc, msg.zacc]
            pass

        elif type == "ATTITUDE":
            # print("roll:{:.2f} pitch:{:.2f} yaw:{:.2f}". format(msg.roll / math.pi * 180, msg.pitch / math.pi * 180, msg.yaw / math.pi * 180))
            pass

        # print(msg)
        elif type == "AUTH_KEY":
            # print(msg)
            pass

        # print("mavlink_wait_message running", time_exit)

        if time_exit:
            break
    return jig_system_status,jig_system_base,jig_system_custom

    # else:
    # 	print(msg.get_type())

    print("mavlink_wait_message exited")
def request_message_stream(msg_id, rate, status):
    the_connection.mav.request_data_stream_send(the_connection.target_system,the_connection.target_component,msg_id,rate, status)
def mav_init():
    # global the_connection
    global time_exit
    global the_connection

    # Start a connection listening to a serial port
    mav_connected = False
    # cnt = 0

    while not mav_connected:
        try:
            the_connection = mavutil.mavlink_connection("/dev/ttyS0", baud=115200)
            mav_connected = True
        except:
            print("Can not init mavlink with s4. Try again...")
            # cnt = cnt + 1
            # if cnt > 3: 
                # return False
            time.sleep(0.5)

    while not time_exit:
        # request heartbeat from stm32
        # print("send first heartbeat")
        # mav_heartbeat_send(type=123,autopilot=8,status = 0,control =0,result = 0)
        request_message_stream(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 500, 1)
        msg = the_connection.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if msg is not None:
            print("Mavlink connected.")
            break
            # return True
        else:
            print("Mavlink timeout. Try again...")
            # return False


# Once connected, use 'the_connection' to get and send messages
def recv_match(condition=None, type=None, blocking=False, timeout=None):
    '''Receive the next MAVLink message that matches the given type and condition
    type:        Message name(s) as a string or list of strings - e.g. 'SYS_STATUS'
    condition:   Condition based on message values - e.g. 'SYS_STATUS.mode==2 and SYS_STATUS.nav_mode==4'
    blocking:    Set to wait until message arrives before method completes. 
    timeout:     ? <!-- timeout for blocking message when the system will return. Is this just a time? -->
    '''
    msg = the_connection.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    return msg


def mav_heartbeat_send(type, autopilot,status, control, result, mavlink_version=3, force_mavlink1=False):
    # Send heartbeat from a GCS (types are define as enum in the dialect file). 
    # the_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
    #                                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, system_status=status, base_mode= control, custom_mode=result)
    for i in range(0,2):
        global the_connection
        the_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, system_status=status, base_mode= control, custom_mode=result)

    # print("mavlink_send_hb running", time_exit)
        time.sleep(0.2)
    print("mavlink_send_hb exited")








    
 


