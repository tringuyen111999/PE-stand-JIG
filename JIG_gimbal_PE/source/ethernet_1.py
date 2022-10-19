import socket
import time
import subprocess
import os
import sys
from threading import Thread
import threading
#---------------------------------------------------------------------------------------------------------------
from test_usb_speed import check_speed_usb
#---------------------------------------------------------------------------------------------------------------
# mavlink pi_s4
# from pymavlink import mavutil
from mavlink import mav_heartbeat_send,mav_init,mavlink_wait_message

#---------------------------------------------------------------------------------------------------------------
speed = []
#---------------------------------------------------------------------------------------------------------------
def ping_ethernet():
    check_IP = False
    for i in range(0,5):
        proc = subprocess.Popen(["ping 192.168.110.21 -c 3"], stdout=subprocess.PIPE, shell=True)
        (out, err) = proc.communicate()
        out_new = out.decode().split('\n')
        for i in out_new:
            print(i,'\n')
            if '64 bytes from 192.168.110.21' in i:
                check_IP = True
                break
        if check_IP == True:break
    return check_IP


def test_ethernet():
    ethernet_value = False
    for i in range(0,10):
        time.sleep(1)
        proc = subprocess.Popen(["iperf3 -c 192.168.110.21"], stdout=subprocess.PIPE, shell=True)
        (out, err) = proc.communicate()
        out_new = out.decode().split('\n')

        # process speed data
        for i in range(0,len(out_new)):
            out_speed =out_new[i].strip().split()
            for n in range(0,len(out_speed)):
                if out_speed[n] == 'Mbits/sec':
                    speed.append(out_speed[n-1])
        print(speed)
        if len(speed)>0:
            if float(speed[len(speed)-1]) >= 90 and float(speed[len(speed)-2]) >= 90:
                ethernet_value = True
                break

    return ethernet_value
#---------------------------------------------------------------------------------------------------------------

HOST = '192.168.110.20' # Enter IP or Hostname of your server
PORT = 1234 # Pick an open Port (1000+ recommended), must match the server port



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


def main():
    global status_sys,control,result
    status_sys,control,result = 0,0,0 
    mav_init()
        # status_ethernet = check_ethernet()
    while True:
        print("send heartbeat")
        status_sys,control,result =1,0,0
        mav_heartbeat_send(type=123,autopilot=8,status=status_sys, control=control, result= result)
        heart_beart_receive = mavlink_wait_message()
        # status_receive =  heart_beart_receive[0]
        control_receive   =  heart_beart_receive[1]

        if control_receive == 1 and status_sys !=3:
            #send back to s4
            status_sys, control =2,1
            mav_heartbeat_send(type=123,autopilot=8,status=status_sys, control=control, result= result)
            print("receive status ==1")
            if ping_ethernet() == True:
                result_test_final = main_test()
                print("result:  ",result_test_final)
                status_sys =3
            break

        
    #send result
    while True:
        print('send result')
        mav_heartbeat_send(type=123,autopilot=8,status=status_sys, control=control, result= result_test_final)
        print(status_sys,control,result_test_final)
        mavlink_wait_message()
        if heart_beart_receive[1] == 2 : break

main()