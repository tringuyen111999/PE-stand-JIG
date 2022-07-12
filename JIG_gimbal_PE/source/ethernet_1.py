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
HOST = '192.168.110.21' # Enter IP or Hostname of your server
PORT = 12345 # Pick an open Port (1000+ recommended), must match the server port


#---------------------------------------------------------------------------------------------------------------
speed_test = False
speed = []
testing = False
time_exit = False
count_maintest = 0
#---------------------------------------------------------------------------------------------------------------
def test_ethernet():
    ethernet_value = False

    #test ethernet
    s.send('start'.encode('utf-8'))
    reply = s.recv(1024)
    if reply == 'testing'.encode('utf-8'):
        proc = subprocess.Popen(["iperf3 -c 192.168.110.21"], stdout=subprocess.PIPE, shell=True)
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
def listen():
    global msg_receive, jig_system_base,jig_system_custom,jig_system_status
    try:
        msg_receive = recv_match(self=0)
        # print(msg_receive)
        type = msg_receive.get_type()
        if type == "HEARTBEAT":
                m_is_heartbeat_live = True
                jig_system_status = msg_receive.system_status
                jig_system_base = msg_receive.base_mode
                jig_system_custom = msg_receive.custom_mode
                print("HB receive status: ", jig_system_status)
                print("HB receive base: ", jig_system_base)
                print("HB receive custom: ", jig_system_custom)
    except Exception: pass
    return msg_receive
def main_test():
    result_test = 0
    if test_ethernet() == True and check_speed_usb() == True:
        result_test = 3 # 0x11
    elif test_ethernet() == False and check_speed_usb() == True:
        result_test = 1 # 0x01
    elif test_ethernet() == True and check_speed_usb() == False:
        result_test = 2 # 0x10
    else:
        result_test = 0 # 0x00
    return result_test

#---------------------------------------------------------------------------------------------------------------
while 1:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST,PORT))
        ether_init = True
    except Exception:
        print('connect ethernet fail')
        ether_init = False
    if ether_init == True: break
#init success is mavlink connected 
print('ethernet connected')
status_sys,control,result = 0,0,0
while 1:
    try:
        mav_innitial = mav_init()
    except Exception:
        print('connection fail')
    if mav_innitial == True: break

the_connection = mavutil.mavlink_connection("/dev/ttyAMA0", baud=115200)

while mav_innitial == True:
    while 1:
    # get status of RP4 to standby(status =1) and send back to S4
        listen()
        if jig_system_base != 1:
            print('send 100')
            status_sys = 1
            heartbeat_send(type=123,autopilot=8,status=status_sys, control=control, result= result)
            print(status_sys,control,result)
            # wait hear beat from S4
        else: break
        
    while 1:
        print("begin")
        listen()
        # Receive heartbeat and process
        # control =1(run) is start test--> status =2 (running)
        if jig_system_base == 1 and jig_system_status != 3: # control =1
            print('send 210 to RUNNING test')
            status_sys,control,result = 2,1,0
            heartbeat_send(type=123,autopilot=8,status=status_sys, control=control, result= result)
            for count in range(0,3):
                result = main_test()
                print("result: ",result)
            status_sys = 3 # status =3 is done
        # control =2(stop)
        elif jig_system_base == 2:
            print('stop')
            # status_sys,control,result =0,0,0
            # heartbeat_send(type=123,autopilot=8,status=status_sys, control=control, result= result)
            break
        
        while 1:
            print('send result')
            heartbeat_send(type=123,autopilot=8,status=status_sys, control=control, result= result)
            print(status_sys,control,result)
            listen()
            if jig_system_base == 2: break