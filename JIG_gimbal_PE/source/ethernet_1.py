import socket
import time
import subprocess

from test_usb_speed import check_speed_read_usb, check_speed_write_usb,find_usb,check_speed_usb

HOST = '192.168.110.16' # Enter IP or Hostname of your server
PORT = 12345 # Pick an open Port (1000+ recommended), must match the server port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST,PORT))

speed_test = False
speed = []
# Lets loop awaiting for your input
while 1:
    s.send('start'.encode('utf-8'))
    reply = s.recv(1024)
    if reply == 'testing'.encode('utf-8'):
        proc = subprocess.Popen(["iperf3 -c 192.168.110.16"], stdout=subprocess.PIPE, shell=True)
        time.sleep(0.1)
        (out, err) = proc.communicate()
        out_new = out.decode().split('\n')
        print(out_new)
        for i in range(0,len(out_new)):
            out_speed =out_new[i].strip().split()
            print(out_speed)
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