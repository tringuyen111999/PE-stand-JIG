import time
import serial
import subprocess


#khoi tao UART
def serial_init():
    global ser 
    ser = serial.Serial(
            port='/dev/ttyAMA0',
            baudrate = 115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1)  
#check serial
def check_serial():
    serial_status = False
    ser.write('start\n'.encode('utf-8'))
    time.sleep(1)
    data_read = ser.readline()
    time.sleep(1)
    print(data_read)
    if 'good'.encode('utf-8') in data_read:
        serial_status = True
    return serial_status

#kiem tra toc do ethernet
def check_speed_ethernet(ser_check):
    speed_test = False
    speed = []
    #xu ly tin hieu doc duoc
    if ser_check == True:
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
    else: 
        print('serial fail')
    print(speed)
    if len(speed)>0:
        if float(speed[len(speed)-1]) >= 90 and float(speed[len(speed)-2]) >= 90:
            speed_test = True
    return speed_test

###################################
serial_init()
