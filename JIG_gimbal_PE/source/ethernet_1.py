import socket
import time
import subprocess
#---------------------------------------------------------------------------------------------------------------
from test_usb_speed import check_speed_read_usb, check_speed_write_usb,find_usb,check_speed_usb
#---------------------------------------------------------------------------------------------------------------
#mavlink pi_s4
from pymavlink import mavutil
from mavlink import recv_match,heartbeat_send,mav_init
#---------------------------------------------------------------------------------------------------------------
time_exit = False
HOST = '192.168.110.16' # Enter IP or Hostname of your server
PORT = 12345 # Pick an open Port (1000+ recommended), must match the server port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST,PORT))
#---------------------------------------------------------------------------------------------------------------
speed_test = False
speed = []
testing = False
time_exit = False
global the_connection
#---------------------------------------------------------------------------------------------------------------
def test_ethernet():
    ethernet_value = False

    #test ethernet
    s.send('start'.encode('utf-8'))
    reply = s.recv(1024)
    if reply == 'testing'.encode('utf-8'):
        proc = subprocess.Popen(["iperf3 -c 192.168.110.16"], stdout=subprocess.PIPE, shell=True)
        time.sleep(0.1)
        (out, err) = proc.communicate()
        out_new = out.decode().split('\n')
        # print(out_new) 
        # process speed data
        for i in range(0,len(out_new)):
            out_speed =out_new[i].strip().split()
            # print(out_speed)
            for n in range(0,len(out_speed)):
                if out_speed[n] == 'Mbits/sec':
                    speed.append(out_speed[n-1])
    if len(speed)>0:
        if float(speed[len(speed)-1]) >= 90 and float(speed[len(speed)-2]) >= 90:
            ethernet_value = True
    return ethernet_value
#---------------------------------------------------------------------------------------------------------------
def main_test():
    result_test = 0
    if test_ethernet() == True and check_speed_usb() == True:
        result_test = 3 # 0x11
    elif test_ethernet() == False and check_speed_usb() == True:
        result_test = 1 # 0x01
    elif test_ethernet() == True and check_speed_usb() == False:
        result_test = 2 # 0x10
    else:
        result_test = 0 # 0x01
    return result_test
#---------------------------------------------------------------------------------------------------------------

#init success is mavlink connected 
status_sys,control,result = 0,0,0
if mav_init() == True:
    # get status of RP4 to standby(status =1) and send back to S4
    print('send 100')
    status_sys,base,custom = 1,0,0
    heartbeat_send(self=0 ,type=123,autopilot=8,system_status=status_sys, base_mode=control, custom_mode= result)
    # wait hearbeat from S4
    msg_receive = recv_match(self=0)
    print(msg_receive)
    type = msg_receive.get_type()
    if type == "HEARTBEAT":
            m_is_heartbeat_live = True
            jig_system_status = msg_receive.system_status
            print("HB receive ", jig_system_status)
            pass
    # Receive heartbeat and process
    # control =1(run) is start test--> status =2 (running)
    if control == 0: # control =1
        print('send 210')
        status_sys,control,result = 2,1,0
        heartbeat_send(self=0 ,type=123,autopilot=8,system_status=status_sys, base_mode=control, custom_mode= result)
        testing_result = 0
        result = main_test()
        print(result)
        if result != 0:
            status =3 # status =3 is done
            
    # control =2(stop)
    elif control == 2:
        if status_sys ==2:
            #running
            pass
        elif status_sys ==3:
            #done
            pass
        print('send 100')
        status_sys,base,custom = 1,0,0
        heartbeat_send(self=0 ,type=123,autopilot=8,system_status=status_sys, base_mode=control, custom_mode= result)

    elif control == 0:
        print('send 100')
        status_sys,base,custom = 1,0,0
        heartbeat_send(self=0 ,type=123,autopilot=8,system_status=status_sys, base_mode=control, custom_mode= result)
        pass
    