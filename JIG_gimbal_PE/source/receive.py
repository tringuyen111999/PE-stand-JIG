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

def receive_data():
    while ser.isOpen() == True :
        data_read = ser.readline()
        print(data_read)
        if 'start'.encode('utf-8') in data_read:
            ser.write('good\n'.encode('utf-8'))
            time.sleep(0.1)
            proc = subprocess.Popen(["iperf3 -s"], stdout=subprocess.PIPE, shell=True)
            
serial_init()
receive_data()
# while 1: 
#     print(ser.readline())
