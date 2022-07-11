
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

serial_init()
while True:
    ser.write('hello a lan\n'.encode('utf-8'))
    time.sleep(1)
    print("sent ms")
    data_read = ser.readline()
    data_read = data_read.decode()
    print(data_read)