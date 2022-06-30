import socket
import time
import subprocess

from test_usb_speed import check_speed_read_usb, check_speed_write_usb,find_usb,check_speed_usb
#mavlink pi_s4
from pymavlink import mavutil
from mavlink import recv_match,heartbeat_send
data_send_s4= []
base = 0
custom = 0
status_sys = 0
time_exit = False
#init mavlink
def mav_init():
    # global the_connection
    global time_exit

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


HOST = '192.168.110.16' # Enter IP or Hostname of your server
PORT = 12345 # Pick an open Port (1000+ recommended), must match the server port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST,PORT))

speed_test = False
speed = []
# Lets loop awaiting for your input
while 1:
    #test ethernet
    s.send('start'.encode('utf-8'))
    reply = s.recv(1024)
    if reply == 'testing'.encode('utf-8'):
        proc = subprocess.Popen(["iperf3 -c 192.168.110.16"], stdout=subprocess.PIPE, shell=True)
        time.sleep(0.1)
        (out, err) = proc.communicate()
        out_new = out.decode().split('\n')
        # print(out_new)
        for i in range(0,len(out_new)):
            out_speed =out_new[i].strip().split()
            # print(out_speed)
            for n in range(0,len(out_speed)):
                if out_speed[n] == 'Mbits/sec':
                    speed.append(out_speed[n-1])
        break
print(speed)
if len(speed)>0:
    if float(speed[len(speed)-1]) >= 90 and float(speed[len(speed)-2]) >= 90:
        speed_test = True
        print('pass')
    print(reply)
    data_send_s4= []

    # check usb speed
    if check_speed_usb() == True:
        print('usb pass')
    else:
        print('usb fail')