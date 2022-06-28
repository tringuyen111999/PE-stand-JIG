import time
import serial
from Test_ethernet import serial_init, check_speed_ethernet,check_serial,ser
from test_usb_speed import check_speed_read_usb, check_speed_write_usb,find_usb,check_speed_usb
#button
import RPi.GPIO as GPIO
#mavlink pi_s4
from pymavlink import mavutil
from mavlink import recv_match,heartbeat_send
GPIO.setmode(GPIO.BCM)  # Thiết lập kiểu đánh số chân BCM
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP) # pps
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

# check ethernet
serial_init() 
def main_test():
    while 1:
        #check cong uart console
        while ser.isOpen() == True:
            # check ethernet speed
            print('check Ethernet speed')
            for i in range(0,5):
                time.sleep(2)
                print('check {}'.format(i))
                if check_speed_ethernet(check_serial()) == True: 
                    print('Ethernet pass')
                    data_send_s4.append(True)
                    break
            else:
                print('Ethernet fail')
                data_send_s4.append(False)
            time.sleep(2)

            # check usb speed
            if check_speed_usb() == True:
                data_send_s4.append(True)
            else: 
                data_send_s4.append(False)
            # in ra ket qua gui S4
            print(data_send_s4)
            
            #ket thuc
            if len(data_send_s4) == 2:break
        else:print('serial fail')   



# heartbeat 0 0 0
# status_sys = 0 #status
# base = 0  #control pi
# custom = 0 #kết quả test
# heartbeat_send(self=0 ,type=123,autopilot=8, base_mode=base, custom_mode=custom, system_status=status_sys)
# nhận heartbear s4 và xử lý
#nếu nhận được  0 0 0 -- > connect
if mav_init() == True:
    status_sys,base,custom = 1,0,0
    heartbeat_send(self=0 ,type=123,autopilot=8, base_mode=base, custom_mode=custom, system_status=status_sys)
# nhận từ S4 status =2 , base =1
status_sys,base,custom = 2,1,0

# gửi lại s4 status =2
status_sys,base,custom = 2,1,0 # running
heartbeat_send(self=0 ,type=123,autopilot=8, base_mode=base, custom_mode=custom, system_status=status_sys)

# Test speed ethernet
#test speed usb
main_test()
# trả kết quả
''' 0 : false hết
    1: USB false
    2: Ethernet false
    3: Pass hết
'''
# gửi kết quả qua 
# s4
status_sys,base,custom = 3,1,3

# nhận base =2 (stop), đưa status = 1
status_sys,base,custom = 1,0,3